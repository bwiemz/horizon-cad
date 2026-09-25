#include "horizon/modeling/Faceting.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/modeling/SolidSewer.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

using hz::math::Vec3;
using topo::Edge;
using topo::Face;
using topo::HalfEdge;
using topo::Wire;

namespace {

using UV = std::pair<double, double>;
using Key = std::tuple<double, double, double>;

Key keyOf(const Vec3& p) {
    return {p.x, p.y, p.z};
}

std::vector<const HalfEdge*> loopOf(const Wire* wire) {
    std::vector<const HalfEdge*> loop;
    if (wire == nullptr || wire->halfEdge == nullptr) return loop;
    const HalfEdge* start = wire->halfEdge;
    const HalfEdge* cur = start;
    do {
        if (cur->origin == nullptr || cur->next == nullptr || cur->next->origin == nullptr ||
            cur->edge == nullptr) {
            return {};
        }
        loop.push_back(cur);
        cur = cur->next;
    } while (cur != start && loop.size() < 100000);
    return loop;
}

/// The control net's size.
double sizeOf(const geo::NurbsSurface& s) {
    double size = 0.0;
    for (const auto& row : s.controlPoints()) {
        for (const auto& p : row) size = std::max(size, (p - s.controlPoints()[0][0]).length());
    }
    return size;
}

/// Flat: every control point on the plane through the surface's middle.
bool isFlat(const geo::NurbsSurface& s) {
    const auto mid =
        s.evaluateWithDerivatives(0.5 * (s.uMin() + s.uMax()), 0.5 * (s.vMin() + s.vMax()));
    Vec3 n = mid.du.cross(mid.dv);
    const double size = sizeOf(s);
    if (n.length() <= 1e-300 || size <= 0.0) return false;
    n = n * (1.0 / n.length());
    for (const auto& row : s.controlPoints()) {
        for (const auto& p : row) {
            if (std::abs((p - mid.point).dot(n)) > 1e-12 * size) return false;
        }
    }
    return true;
}

/// The points of an edge from its half-edge's origin to its end: along its
/// curve, equal lengths apart, as many as keep each chord within
/// @p maxAngle of turning; the ends its vertices.
std::vector<Vec3> sampleEdge(const Edge& e, double maxAngle) {
    const Vec3 p = e.halfEdge->origin->point;
    const Vec3 q = e.halfEdge->next->origin->point;
    const geo::NurbsCurve* c = e.curve.get();
    if (c == nullptr || c->degree() <= 1) return {p, q};
    constexpr int kDense = 512;
    const double t0 = c->tMin();
    const double t1 = c->tMax();
    std::vector<double> ts(kDense + 1);
    std::vector<Vec3> dense(kDense + 1);
    for (int i = 0; i <= kDense; ++i) {
        ts[static_cast<size_t>(i)] = t0 + (t1 - t0) * i / kDense;
        dense[static_cast<size_t>(i)] = c->evaluate(ts[static_cast<size_t>(i)]);
    }
    // The curve may run the other way from the half-edge.
    const bool closed =
        (p - q).length() <= 1e-12 * std::max(1.0, (dense.back() - dense.front()).length());
    if (!closed && (dense.back() - p).length() < (dense.front() - p).length()) {
        std::reverse(ts.begin(), ts.end());
        std::reverse(dense.begin(), dense.end());
    }
    double turn = 0.0;
    std::vector<double> length(dense.size(), 0.0);
    for (size_t i = 1; i < dense.size(); ++i) {
        length[i] = length[i - 1] + (dense[i] - dense[i - 1]).length();
        if (i + 1 < dense.size()) {
            const Vec3 a = dense[i] - dense[i - 1];
            const Vec3 b = dense[i + 1] - dense[i];
            const double la = a.length();
            const double lb = b.length();
            if (la > 0.0 && lb > 0.0) {
                turn += std::acos(std::clamp(a.dot(b) / (la * lb), -1.0, 1.0));
            }
        }
    }
    const int n = std::max(1, static_cast<int>(std::ceil(turn / maxAngle - 1e-9)));
    std::vector<Vec3> out{p};
    size_t at = 1;
    for (int k = 1; k < n; ++k) {
        const double target = length.back() * k / n;
        while (at + 1 < length.size() && length[at] < target) ++at;
        const double span = length[at] - length[at - 1];
        const double f = span > 0.0 ? (target - length[at - 1]) / span : 0.0;
        out.push_back(c->evaluate(ts[at - 1] + (ts[at] - ts[at - 1]) * f));
    }
    out.push_back(q);
    return out;
}

/// A surface's (u, v) for points on it, found once each.
class SurfaceMap {
public:
    explicit SurfaceMap(const geo::NurbsSurface& s) : m_s(s), m_size(sizeOf(s)) {}

    const geo::NurbsSurface& surface() const { return m_s; }
    double period(int dir) const {
        return dir == 0 ? m_s.uMax() - m_s.uMin() : m_s.vMax() - m_s.vMin();
    }
    bool closed(int dir) const { return dir == 0 ? m_s.closedU() : m_s.closedV(); }

    /// The (u, v) of @p p, which lies on the surface; nullopt if it does not.
    std::optional<UV> uvOf(const Vec3& p) {
        const auto key = keyOf(p);
        const auto found = m_uv.find(key);
        if (found != m_uv.end()) return found->second;
        const auto miss = [&](const UV& uv) {
            return (m_s.evaluateWithDerivatives(uv.first, uv.second).point - p).length();
        };
        UV best = m_s.locate(p);
        const double tol = 1e-7 * std::max(m_size, 1e-300);
        if (miss(best) > tol) {
            // Not found from the best starts of a grid: from every cell of
            // a finer one.
            constexpr int kStarts = 32;
            for (int i = 0; i < kStarts && miss(best) > tol; ++i) {
                for (int j = 0; j < kStarts; ++j) {
                    const UV uv = m_s.project(p, m_s.uMin() + period(0) * (i + 0.5) / kStarts,
                                              m_s.vMin() + period(1) * (j + 0.5) / kStarts);
                    if (miss(uv) < miss(best)) best = uv;
                }
            }
        }
        std::optional<UV> result;
        if (miss(best) <= 1e-6 * std::max(m_size, 1e-300)) result = best;
        m_uv.emplace(key, result);
        return result;
    }

    /// Which of u (0) and v (1) does not move the point at @p uv (a pole's
    /// u, an apex's), or -1.
    int freeAt(const UV& uv) const {
        const Vec3 p = m_s.evaluateWithDerivatives(uv.first, uv.second).point;
        const double tol = 1e-7 * std::max(m_size, 1e-300);
        const auto still = [&](double u, double v) {
            return (m_s.evaluateWithDerivatives(u, v).point - p).length() <= tol;
        };
        if (still(m_s.uMin() + 0.31 * period(0), uv.second) &&
            still(m_s.uMin() + 0.77 * period(0), uv.second)) {
            return 0;
        }
        if (still(uv.first, m_s.vMin() + 0.31 * period(1)) &&
            still(uv.first, m_s.vMin() + 0.77 * period(1))) {
            return 1;
        }
        return -1;
    }

    /// The point at an unwrapped (u, v): round a seam, or on the edge.
    Vec3 at(UV uv) const {
        for (int dir = 0; dir < 2; ++dir) {
            double& t = dir == 0 ? uv.first : uv.second;
            const double lo = dir == 0 ? m_s.uMin() : m_s.vMin();
            if (closed(dir)) {
                t = lo + std::fmod(t - lo, period(dir));
                if (t < lo) t += period(dir);
            }
        }
        return m_s.evaluateWithDerivatives(uv.first, uv.second).point;
    }

private:
    const geo::NurbsSurface& m_s;
    double m_size;
    std::map<Key, std::optional<UV>> m_uv;
};

/// A point of a face's boundary with its unwrapped (u, v).
struct LoopPoint {
    Vec3 point;
    UV uv;
    int free = -1;  ///< the coordinate that does not move it (a pole's), or -1
};

double& coord(UV& uv, int dir) {
    return dir == 0 ? uv.first : uv.second;
}
double coord(const UV& uv, int dir) {
    return dir == 0 ? uv.first : uv.second;
}

/// One half-edge's points in (u, v), continuous along it; a pole takes the
/// free coordinate of the point beside it. Nullopt when a point is off the
/// surface.
std::optional<std::vector<LoopPoint>> edgeInUV(const std::vector<Vec3>& samples, SurfaceMap& map) {
    std::vector<LoopPoint> out;
    for (const Vec3& p : samples) {
        const auto uv = map.uvOf(p);
        if (!uv) return std::nullopt;
        out.push_back({p, *uv, map.freeAt(*uv)});
    }
    // Continuous round a seam.
    for (size_t i = 1; i < out.size(); ++i) {
        for (int dir = 0; dir < 2; ++dir) {
            if (!map.closed(dir)) continue;
            const double period = map.period(dir);
            double& t = coord(out[i].uv, dir);
            const double prev = coord(out[i - 1].uv, dir);
            t += period * std::round((prev - t) / period);
        }
    }
    // A pole's free coordinate from the point beside it.
    for (size_t i = 0; i < out.size(); ++i) {
        if (out[i].free < 0 || out.size() < 2) continue;
        const size_t beside = i == 0 ? 1 : i - 1;
        if (out[beside].free >= 0) continue;
        coord(out[i].uv, out[i].free) = coord(out[beside].uv, out[i].free);
    }
    return out;
}

void shift(std::vector<LoopPoint>& points, int dir, double by) {
    for (auto& p : points) coord(p.uv, dir) += by;
}

double signedArea(const std::vector<LoopPoint>& loop) {
    double a = 0.0;
    for (size_t i = 0; i < loop.size(); ++i) {
        const UV& p = loop[i].uv;
        const UV& q = loop[(i + 1) % loop.size()].uv;
        a += p.first * q.second - q.first * p.second;
    }
    return 0.5 * a;
}

/// The four sides of a (u, v) rectangle, counterclockwise from its corner
/// (u0, v0): bottom, right, top, left, corners in each. @p reversed: the
/// face's loop runs clockwise in (u, v), against the surface's normal.
struct Rectangle {
    std::vector<LoopPoint> sides[4];
    bool reversed = false;
};

std::optional<Rectangle> asRectangle(std::vector<LoopPoint> loop) {
    const bool reversed = signedArea(loop) < 0.0;
    if (reversed) std::reverse(loop.begin(), loop.end());
    double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    for (const auto& p : loop) {
        u0 = std::min(u0, p.uv.first);
        u1 = std::max(u1, p.uv.first);
        v0 = std::min(v0, p.uv.second);
        v1 = std::max(v1, p.uv.second);
    }
    const double tol = 1e-6 * std::max(u1 - u0, v1 - v0);
    if (!(u1 - u0 > tol) || !(v1 - v0 > tol)) return std::nullopt;
    const auto on = [tol](double a, double b) { return std::abs(a - b) <= tol; };
    // The side a point is on, counterclockwise from the bottom; a corner
    // counts as the side it starts.
    const auto sideOf = [&](const UV& uv) {
        if (on(uv.second, v0) && !on(uv.first, u1)) return 0;
        if (on(uv.first, u1) && !on(uv.second, v1)) return 1;
        if (on(uv.second, v1) && !on(uv.first, u0)) return 2;
        if (on(uv.first, u0) && !on(uv.second, v0)) return 3;
        return -1;
    };
    // Start at the corner (u0, v0).
    size_t start = loop.size();
    for (size_t i = 0; i < loop.size(); ++i) {
        if (on(loop[i].uv.first, u0) && on(loop[i].uv.second, v0)) {
            start = i;
            break;
        }
    }
    if (start == loop.size()) return std::nullopt;
    std::rotate(loop.begin(), loop.begin() + static_cast<std::ptrdiff_t>(start), loop.end());
    Rectangle r;
    int side = 0;
    for (const LoopPoint& p : loop) {
        const int s = sideOf(p.uv);
        if (s < 0) return std::nullopt;  // inside the rectangle: not its edge
        if (s != side) {
            if (s != side + 1) return std::nullopt;  // the sides in order, none skipped
            r.sides[side].push_back(p);              // a corner ends one side, starts the next
            side = s;
        }
        r.sides[side].push_back(p);
    }
    if (side != 3) return std::nullopt;
    r.sides[3].push_back(loop.front());
    r.reversed = reversed;
    for (const auto& s : r.sides) {
        if (s.size() < 2) return std::nullopt;
    }
    return r;
}

/// A face's outer loop in (u, v): each half-edge's points continuous, and
/// each placed where the one before ends; across a pole, where u (or v) is
/// free, one of three places. A placing that closes the loop into a
/// rectangle is the face. A face that is all of its surface (a sphere, its
/// loop a seam there and back) closes both ways round: it is taken the way
/// round the surface's normal, which a STEP reader makes point out.
std::optional<Rectangle> faceInUV(const std::vector<std::vector<Vec3>>& edges, SurfaceMap& map) {
    std::vector<std::vector<LoopPoint>> parts;
    for (const auto& samples : edges) {
        auto part = edgeInUV(samples, map);
        if (!part) return std::nullopt;
        parts.push_back(std::move(*part));
    }
    // The joints at a pole: the choices there.
    std::vector<size_t> poles;
    for (size_t k = 1; k < parts.size(); ++k) {
        if (parts[k].front().free >= 0) poles.push_back(k);
    }
    if (poles.size() > 4) return std::nullopt;
    size_t combinations = 1;
    for (size_t i = 0; i < poles.size(); ++i) combinations *= 3;
    std::optional<Rectangle> against;  // the first against the normal
    for (size_t combination = 0; combination < combinations; ++combination) {
        std::vector<std::vector<LoopPoint>> placed = parts;
        size_t choice = combination;
        size_t pole = 0;
        for (size_t k = 1; k < placed.size(); ++k) {
            const LoopPoint& end = placed[k - 1].back();
            LoopPoint& begin = placed[k].front();
            const int free = begin.free;
            for (int dir = 0; dir < 2; ++dir) {
                if (!map.closed(dir)) continue;
                const double period = map.period(dir);
                double by =
                    period * std::round((coord(end.uv, dir) - coord(begin.uv, dir)) / period);
                if (dir == free && pole < poles.size() && poles[pole] == k) {
                    by += period * (static_cast<double>(choice % 3) - 1.0);
                }
                shift(placed[k], dir, by);
            }
            if (free >= 0 && pole < poles.size() && poles[pole] == k) {
                choice /= 3;
                ++pole;
            }
        }
        // Closed: the last part ends where the first begins (a pole: in its
        // fixed coordinate).
        const LoopPoint& last = placed.back().back();
        const LoopPoint& first = placed.front().front();
        bool closes = true;
        for (int dir = 0; dir < 2; ++dir) {
            if (dir == first.free) continue;
            closes = closes && std::abs(coord(last.uv, dir) - coord(first.uv, dir)) <=
                                   1e-6 * std::max(1.0, map.period(dir));
        }
        if (!closes) continue;
        std::vector<LoopPoint> loop;
        for (auto& part : placed) {
            // Where two parts meet at a point that is not a pole, it is
            // there once; at a pole, at both its places.
            for (size_t i = 0; i < part.size(); ++i) {
                if (i == 0 && !loop.empty() && part[0].free < 0) continue;
                loop.push_back(part[i]);
            }
        }
        if (!loop.empty() && loop.front().free < 0 && loop.size() > 1) loop.pop_back();
        const double area = signedArea(loop);
        if (!(std::abs(area) > 0.0)) continue;
        auto rectangle = asRectangle(loop);
        if (!rectangle) continue;
        if (!rectangle->reversed) return rectangle;
        if (!against) against = std::move(rectangle);
    }
    return against;
}

/// The grid of facets over a rectangle, its boundary the rectangle's own
/// points: a Coons patch in (u, v), its inner points on the surface.
/// How many pieces keep the surface's iso-curve across a rectangle from
/// corner @p from to corner @p to, @p dir running, within @p maxAngle of
/// turning each: the curve through the middle of the other direction (at
/// its edge a sphere's is a pole, all rounding).
size_t piecesAcross(const SurfaceMap& map, UV from, UV to, int dir, double maxAngle) {
    constexpr int kDense = 256;
    std::vector<Vec3> dense;
    for (int i = 0; i <= kDense; ++i) {
        UV uv = from;
        coord(uv, dir) = coord(from, dir) + (coord(to, dir) - coord(from, dir)) * i / kDense;
        coord(uv, 1 - dir) = 0.5 * (coord(from, 1 - dir) + coord(to, 1 - dir));
        dense.push_back(map.at(uv));
    }
    double turn = 0.0;
    for (size_t i = 1; i + 1 < dense.size(); ++i) {
        const Vec3 a = dense[i] - dense[i - 1];
        const Vec3 b = dense[i + 1] - dense[i];
        const double la = a.length();
        const double lb = b.length();
        if (la > 0.0 && lb > 0.0) turn += std::acos(std::clamp(a.dot(b) / (la * lb), -1.0, 1.0));
    }
    return static_cast<size_t>(std::max(1, static_cast<int>(std::ceil(turn / maxAngle - 1e-9))));
}

std::optional<std::vector<std::vector<Vec3>>> gridFacets(const Rectangle& r, SurfaceMap& map,
                                                         double size, double maxAngle) {
    const auto degenerate = [](const std::vector<LoopPoint>& side) {
        for (const auto& p : side) {
            if ((p.point - side.front().point).length() > 0.0) return false;
        }
        return true;
    };
    const auto& bottom = r.sides[0];
    const auto& right = r.sides[1];
    const auto& top = r.sides[2];
    const auto& left = r.sides[3];
    const bool bottomPole = degenerate(bottom);
    const bool topPole = degenerate(top);
    const bool rightPole = degenerate(right);
    const bool leftPole = degenerate(left);
    // Where both sides across are poles (a sphere's top and bottom), the
    // surface's own turning sets the count.
    // Corners (u0, v0) and (u1, v1): the bottom's first, the top's first.
    const UV low = bottom.front().uv;
    const UV high = top.front().uv;
    const size_t nu = bottomPole && topPole ? piecesAcross(map, low, high, 0, maxAngle)
                                            : (bottomPole ? top.size() : bottom.size()) - 1;
    const size_t nv = leftPole && rightPole ? piecesAcross(map, low, high, 1, maxAngle)
                                            : (leftPole ? right.size() : left.size()) - 1;
    if (!bottomPole && !topPole && bottom.size() != top.size()) return std::nullopt;
    if (!leftPole && !rightPole && left.size() != right.size()) return std::nullopt;
    if (nu < 1 || nv < 1) return std::nullopt;

    // Each side by its index, bottom and top left to right, left and right
    // bottom to top (the loop runs top and left the other way); a pole's
    // side spread over its free coordinate, as many points as its opposite.
    const auto sidePoint = [](const std::vector<LoopPoint>& side, bool reversed, bool pole,
                              size_t i, size_t n, int along) {
        if (!pole) return side[reversed ? n - i : i];
        const LoopPoint& from = reversed ? side.back() : side.front();
        const LoopPoint& to = reversed ? side.front() : side.back();
        LoopPoint p = from;
        const double f = static_cast<double>(i) / static_cast<double>(n);
        coord(p.uv, along) =
            coord(from.uv, along) + (coord(to.uv, along) - coord(from.uv, along)) * f;
        return p;
    };
    const auto B = [&](size_t i) { return sidePoint(bottom, false, bottomPole, i, nu, 0); };
    const auto T = [&](size_t i) { return sidePoint(top, true, topPole, i, nu, 0); };
    const auto R = [&](size_t j) { return sidePoint(right, false, rightPole, j, nv, 1); };
    const auto L = [&](size_t j) { return sidePoint(left, true, leftPole, j, nv, 1); };

    std::vector<std::vector<Vec3>> grid(nu + 1, std::vector<Vec3>(nv + 1));
    for (size_t i = 0; i <= nu; ++i) {
        for (size_t j = 0; j <= nv; ++j) {
            if (j == 0) {
                grid[i][j] = B(i).point;
            } else if (j == nv) {
                grid[i][j] = T(i).point;
            } else if (i == 0) {
                grid[i][j] = L(j).point;
            } else if (i == nu) {
                grid[i][j] = R(j).point;
            } else {
                const double s = static_cast<double>(i) / static_cast<double>(nu);
                const double t = static_cast<double>(j) / static_cast<double>(nv);
                UV uv;
                for (int dir = 0; dir < 2; ++dir) {
                    coord(uv, dir) =
                        (1 - t) * coord(B(i).uv, dir) + t * coord(T(i).uv, dir) +
                        (1 - s) * coord(L(j).uv, dir) + s * coord(R(j).uv, dir) -
                        ((1 - s) * (1 - t) * coord(B(0).uv, dir) +
                         s * (1 - t) * coord(B(nu).uv, dir) + (1 - s) * t * coord(T(0).uv, dir) +
                         s * t * coord(T(nu).uv, dir));
                }
                grid[i][j] = map.at(uv);
            }
        }
    }
    std::vector<std::vector<Vec3>> facets;
    for (size_t i = 0; i < nu; ++i) {
        for (size_t j = 0; j < nv; ++j) {
            std::vector<Vec3> quad;
            for (const Vec3& p : {grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]}) {
                if (quad.empty() || (p - quad.back()).length() > 0.0) quad.push_back(p);
            }
            while (quad.size() > 1 && (quad.front() - quad.back()).length() <= 0.0) quad.pop_back();
            if (quad.size() < 3) continue;
            if (quad.size() == 4) {
                const Vec3 n = (quad[1] - quad[0]).cross(quad[2] - quad[0]);
                const double off =
                    n.length() > 0.0 ? std::abs((quad[3] - quad[0]).dot(n)) / n.length() : 0.0;
                if (off > 1e-9 * size) {
                    facets.push_back({quad[0], quad[1], quad[2]});
                    facets.push_back({quad[0], quad[2], quad[3]});
                    continue;
                }
            }
            facets.push_back(std::move(quad));
        }
    }
    // The grid runs counterclockwise in (u, v); the face's loop may not.
    if (r.reversed) {
        for (auto& facet : facets) std::reverse(facet.begin(), facet.end());
    }
    return facets;
}

// ---------------------------------------------------------------------------
// Trimmed faces (Phase 152): any outline, holes too, cut into facets on the
// surface within it.
// ---------------------------------------------------------------------------

/// A face's loop in (u, v), with no pole on it: each half-edge's points
/// placed where the one before ends, round the surface's seams. Nullopt when
/// a point is off the surface, a pole is on it, or the loop does not close
/// in (u, v) (it winds round the surface).
std::optional<std::vector<LoopPoint>> loopInUV(const std::vector<std::vector<Vec3>>& edges,
                                               SurfaceMap& map) {
    std::vector<LoopPoint> loop;
    for (const auto& samples : edges) {
        auto part = edgeInUV(samples, map);
        if (!part || part->empty()) return std::nullopt;
        for (const LoopPoint& p : *part) {
            if (p.free >= 0) return std::nullopt;  // a pole: a rectangle's, if any
        }
        if (!loop.empty()) {
            for (int dir = 0; dir < 2; ++dir) {
                if (!map.closed(dir)) continue;
                const double period = map.period(dir);
                shift(*part, dir,
                      period *
                          std::round((coord(loop.back().uv, dir) - coord(part->front().uv, dir)) /
                                     period));
            }
            loop.insert(loop.end(), part->begin() + 1, part->end());
        } else {
            loop = std::move(*part);
        }
    }
    if (loop.size() < 4) return std::nullopt;
    const LoopPoint& first = loop.front();
    const LoopPoint& last = loop.back();
    const double tol = 1e-9 * std::max({1.0, std::abs(first.uv.first), std::abs(first.uv.second)});
    if (std::abs(first.uv.first - last.uv.first) > tol ||
        std::abs(first.uv.second - last.uv.second) > tol) {
        return std::nullopt;  // winds round the surface: no polygon in (u, v)
    }
    loop.pop_back();
    return loop;
}

/// A vertex of a region being cut into triangles: where it is in (u, v),
/// scaled to about the surface's own lengths, and on the surface.
struct RegionVertex {
    double x = 0.0;
    double y = 0.0;
    LoopPoint at;
};

double cross2(const RegionVertex& a, const RegionVertex& b, const RegionVertex& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/// Whether @p p is in the triangle a, b, c (anticlockwise), or on its edges.
bool inOrOn(const RegionVertex& p, const RegionVertex& a, const RegionVertex& b,
            const RegionVertex& c, double eps) {
    return cross2(a, b, p) >= -eps && cross2(b, c, p) >= -eps && cross2(c, a, p) >= -eps;
}

/// The triangles of a region: its outer loop anticlockwise, its holes
/// clockwise, each hole joined to the outside along a bridge (both ways, so
/// the region is one polygon), then cut by ears. An ear is clipped only if
/// no other point is in it or on its edges: a point left on a triangle's
/// edge would be a vertex in the middle of a neighbour's edge. Nullopt when
/// the polygon will not cut.
struct Clipped {
    std::vector<std::array<std::size_t, 3>> triangles;
    /// The one polygon cut: the outline, holes joined in by their bridges.
    std::vector<std::size_t> polygon;
};

std::optional<Clipped> earClip(std::vector<RegionVertex>& vertices, std::vector<std::size_t> outer,
                               std::vector<std::vector<std::size_t>> holes, double eps,
                               double spacing,
                               const std::function<RegionVertex(double, double)>& onSurface) {
    // Holes, rightmost first, each bridged from its rightmost point to a
    // point of the polygon it can see along +x.
    std::sort(holes.begin(), holes.end(), [&](const auto& a, const auto& b) {
        const auto right = [&](const std::vector<std::size_t>& h) {
            double x = -1e300;
            for (std::size_t i : h) x = std::max(x, vertices[i].x);
            return x;
        };
        return right(a) > right(b);
    });
    std::vector<std::size_t> polygon = std::move(outer);
    // Whether segments p-q and r-t cross, other than where they share an end.
    const auto crosses = [&](const RegionVertex& p, const RegionVertex& q, const RegionVertex& r,
                             const RegionVertex& t) {
        const double d1 = cross2(p, q, r);
        const double d2 = cross2(p, q, t);
        const double d3 = cross2(r, t, p);
        const double d4 = cross2(r, t, q);
        return ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) &&
               ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps));
    };
    for (std::size_t h = 0; h < holes.size(); ++h) {
        const auto& hole = holes[h];
        std::size_t mAt = 0;
        for (std::size_t k = 1; k < hole.size(); ++k) {
            if (vertices[hole[k]].x > vertices[hole[mAt]].x) mAt = k;
        }
        const RegionVertex m = vertices[hole[mAt]];
        // The nearest point of the polygon m can see: the bridge crosses no
        // edge of it, of this hole, or of a hole still to join. (The
        // furthest end of the edge a ray meets can be far: across a seam
        // with no points between its corners.)
        std::vector<std::size_t> order(polygon.size());
        for (std::size_t k = 0; k < order.size(); ++k) order[k] = k;
        std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
            const auto d = [&](std::size_t k) {
                const RegionVertex& v = vertices[polygon[k]];
                return std::hypot(v.x - m.x, v.y - m.y);
            };
            return d(a) < d(b);
        });
        const auto clearOf = [&](const RegionVertex& v, std::size_t vIndex,
                                 const std::vector<std::size_t>& loop, std::size_t skip) {
            for (std::size_t k = 0; k < loop.size(); ++k) {
                const std::size_t a = loop[k];
                const std::size_t b = loop[(k + 1) % loop.size()];
                if (a == vIndex || b == vIndex || a == skip || b == skip) continue;
                if (crosses(m, v, vertices[a], vertices[b])) return false;
            }
            return true;
        };
        std::size_t to = polygon.size();
        for (std::size_t k : order) {
            const std::size_t candidate = polygon[k];
            const RegionVertex& v = vertices[candidate];
            bool seen = clearOf(v, candidate, polygon, polygon.size()) &&
                        clearOf(v, candidate, hole, hole[mAt]);
            for (std::size_t other = h + 1; seen && other < holes.size(); ++other) {
                seen = clearOf(v, candidate, holes[other], polygon.size());
            }
            if (seen) {
                to = candidate;
                break;
            }
        }
        if (to == polygon.size()) return std::nullopt;
        // Splice: ... to, (bridge), m, round the hole, m', (bridge back), to', ...
        // where m' and to' are copies of m and to. The bridge is cut into
        // points on the surface as far apart as the outline's: one straight
        // chord across it would cut through the part.
        const auto at = std::find(polygon.begin(), polygon.end(), to);
        std::vector<std::size_t> spliced(polygon.begin(), at + 1);
        const RegionVertex mCopy = vertices[hole[mAt]];
        const RegionVertex toCopy = vertices[to];
        const double length = std::hypot(mCopy.x - toCopy.x, mCopy.y - toCopy.y);
        const auto pieces =
            spacing > 0.0 ? static_cast<std::size_t>(std::ceil(length / spacing)) : std::size_t{1};
        const auto bridge = [&](bool back) {
            for (std::size_t k = 1; k < pieces; ++k) {
                const double t =
                    static_cast<double>(back ? pieces - k : k) / static_cast<double>(pieces);
                vertices.push_back(onSurface(toCopy.x + (mCopy.x - toCopy.x) * t,
                                             toCopy.y + (mCopy.y - toCopy.y) * t));
                spliced.push_back(vertices.size() - 1);
            }
        };
        bridge(false);
        for (std::size_t k = 0; k < hole.size(); ++k) {
            spliced.push_back(hole[(mAt + k) % hole.size()]);
        }
        vertices.push_back(mCopy);
        spliced.push_back(vertices.size() - 1);
        bridge(true);
        vertices.push_back(toCopy);
        spliced.push_back(vertices.size() - 1);
        spliced.insert(spliced.end(), at + 1, polygon.end());
        polygon = std::move(spliced);
    }

    Clipped clipped;
    clipped.polygon = polygon;
    std::vector<std::array<std::size_t, 3>>& triangles = clipped.triangles;
    std::vector<std::size_t> left = polygon;
    std::size_t guard = 0;
    while (left.size() > 3) {
        bool cut = false;
        for (std::size_t k = 0; k < left.size(); ++k) {
            const std::size_t ia = left[(k + left.size() - 1) % left.size()];
            const std::size_t ib = left[k];
            const std::size_t ic = left[(k + 1) % left.size()];
            const RegionVertex& a = vertices[ia];
            const RegionVertex& b = vertices[ib];
            const RegionVertex& c = vertices[ic];
            if (cross2(a, b, c) <= eps) continue;  // reflex, or straight on
            bool empty = true;
            for (std::size_t other : left) {
                if (other == ia || other == ib || other == ic) continue;
                const RegionVertex& v = vertices[other];
                // A copy at a corner (a bridge's ends) is that corner.
                const auto same = [&](const RegionVertex& w) {
                    return std::abs(v.x - w.x) <= eps && std::abs(v.y - w.y) <= eps;
                };
                if (same(a) || same(b) || same(c)) continue;
                if (inOrOn(v, a, b, c, eps)) {
                    empty = false;
                    break;
                }
            }
            if (!empty) continue;
            triangles.push_back({ia, ib, ic});
            left.erase(left.begin() + static_cast<std::ptrdiff_t>(k));
            cut = true;
            break;
        }
        if (!cut || ++guard > 1000000) return std::nullopt;
    }
    if (cross2(vertices[left[0]], vertices[left[1]], vertices[left[2]]) <= eps) {
        return std::nullopt;
    }
    triangles.push_back({left[0], left[1], left[2]});
    return clipped;
}

/// No point, or no triangle.
constexpr std::size_t kNoPoint = std::numeric_limits<std::size_t>::max();

/// Points inserted into @p triangles, each at @p inside, the mesh kept
/// Delaunay in the plane by flips; an edge of the region's own outline is
/// never flipped. @p boundary holds those edges, by their ends. @p beside
/// holds, for each point, an earlier one next to it (kNoPoint if none):
/// where to look for it when the outline is between it and the one before.
void insertPoints(std::vector<RegionVertex>& vertices,
                  std::vector<std::array<std::size_t, 3>>& triangles,
                  const std::vector<RegionVertex>& inside, const std::vector<std::size_t>& beside,
                  const std::set<std::pair<std::size_t, std::size_t>>& boundary, double eps) {
    const auto key = [](std::size_t a, std::size_t b) {
        return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
    };
    std::map<std::pair<std::size_t, std::size_t>, std::vector<std::size_t>> edgeTriangles;
    const auto index = [&](std::size_t t) {
        const auto& tri = triangles[t];
        for (std::size_t k = 0; k < 3; ++k) {
            edgeTriangles[key(tri[k], tri[(k + 1) % 3])].push_back(t);
        }
    };
    const auto unindex = [&](std::size_t t) {
        const auto& tri = triangles[t];
        for (std::size_t k = 0; k < 3; ++k) {
            auto& list = edgeTriangles[key(tri[k], tri[(k + 1) % 3])];
            list.erase(std::remove(list.begin(), list.end(), t), list.end());
        }
    };
    for (std::size_t t = 0; t < triangles.size(); ++t) index(t);

    // Whether d is inside the circle through a, b, c (anticlockwise).
    const auto inCircle = [&](std::size_t ia, std::size_t ib, std::size_t ic, std::size_t id) {
        const RegionVertex& a = vertices[ia];
        const RegionVertex& b = vertices[ib];
        const RegionVertex& c = vertices[ic];
        const RegionVertex& d = vertices[id];
        const double ax = a.x - d.x, ay = a.y - d.y;
        const double bx = b.x - d.x, by = b.y - d.y;
        const double cx = c.x - d.x, cy = c.y - d.y;
        const double det = (ax * ax + ay * ay) * (bx * cy - cx * by) -
                           (bx * bx + by * by) * (ax * cy - cx * ay) +
                           (cx * cx + cy * cy) * (ax * by - bx * ay);
        return det > eps * eps;
    };
    const auto legalize = [&](std::vector<std::pair<std::size_t, std::size_t>> stack) {
        std::size_t guard = 0;
        while (!stack.empty() && ++guard < 10000000) {
            const std::size_t p = stack.back().first;
            const std::size_t q = stack.back().second;
            stack.pop_back();
            if (boundary.count(key(p, q)) != 0) continue;
            const auto found = edgeTriangles.find(key(p, q));
            if (found == edgeTriangles.end() || found->second.size() != 2) continue;
            const std::size_t t1 = found->second[0];
            const std::size_t t2 = found->second[1];
            const auto opposite = [&](std::size_t t) {
                for (std::size_t v : triangles[t]) {
                    if (v != p && v != q) return v;
                }
                return p;
            };
            const std::size_t c = opposite(t1);
            const std::size_t d = opposite(t2);
            // t1 anticlockwise as (a, b, c) with a, b the shared edge.
            std::size_t a = p;
            std::size_t b = q;
            if (cross2(vertices[a], vertices[b], vertices[c]) < 0.0) std::swap(a, b);
            if (!inCircle(a, b, c, d)) continue;
            // The flip must leave two anticlockwise triangles: the four
            // points a convex quadrilateral.
            if (cross2(vertices[c], vertices[a], vertices[d]) <= eps ||
                cross2(vertices[d], vertices[b], vertices[c]) <= eps) {
                continue;
            }
            unindex(t1);
            unindex(t2);
            triangles[t1] = {c, a, d};
            triangles[t2] = {d, b, c};
            index(t1);
            index(t2);
            stack.push_back({a, d});
            stack.push_back({d, b});
            stack.push_back({b, c});
            stack.push_back({c, a});
        }
    };

    // Which of triangle t's edges a point is on (k, from tri[k] to
    // tri[k + 1]); kWithin when strictly within it; kNoPoint when it is
    // beyond an edge, or on two (at a corner).
    constexpr std::size_t kWithin = 3;
    const auto placeIn = [&](std::size_t t, const RegionVertex& point) {
        const auto& tri = triangles[t];
        std::size_t on = kWithin;
        for (std::size_t k = 0; k < 3; ++k) {
            const double side = cross2(vertices[tri[k]], vertices[tri[(k + 1) % 3]], point);
            if (side > eps) continue;
            if (side < -eps || on != kWithin) return kNoPoint;
            on = k;
        }
        return on;
    };
    // The triangle a point is in or on, walked to from triangle t across an
    // edge the point is beyond; not across the outline, which may only be in
    // the way (a hole, a notch). kNoPoint when every edge it is beyond is.
    const auto walk = [&](std::size_t t, const RegionVertex& point) {
        for (std::size_t steps = 0; steps < triangles.size(); ++steps) {
            const auto& tri = triangles[t];
            bool beyond = false;
            std::size_t across = kNoPoint;
            for (std::size_t k = 0; k < 3 && across == kNoPoint; ++k) {
                const std::size_t a = tri[k];
                const std::size_t b = tri[(k + 1) % 3];
                if (cross2(vertices[a], vertices[b], point) >= -eps) continue;
                beyond = true;
                const auto found = edgeTriangles.find(key(a, b));
                if (found == edgeTriangles.end()) continue;
                for (std::size_t other : found->second) {
                    if (other != t) across = other;
                }
            }
            if (!beyond) return t;
            if (across == kNoPoint) return kNoPoint;
            t = across;
        }
        return kNoPoint;
    };

    // The cut by ears made Delaunay first: its long diagonals, where no
    // point goes in near them, would stay.
    {
        std::vector<std::pair<std::size_t, std::size_t>> all;
        for (const auto& [edge, list] : edgeTriangles) {
            if (list.size() == 2) all.push_back(edge);
        }
        legalize(std::move(all));
    }

    // Each point's triangle, walked to from the last point's (they come in
    // rows, each near the one before), else from the one beside it's, else
    // looked for in every triangle.
    std::vector<std::size_t> home(inside.size(), kNoPoint);
    std::size_t last = 0;
    for (std::size_t n = 0; n < inside.size(); ++n) {
        const RegionVertex& point = inside[n];
        std::size_t within = walk(std::min(last, triangles.size() - 1), point);
        if (within == kNoPoint && beside[n] != kNoPoint && home[beside[n]] != kNoPoint) {
            within = walk(home[beside[n]], point);
        }
        for (std::size_t t = 0; within == kNoPoint && t < triangles.size(); ++t) {
            if (placeIn(t, point) != kNoPoint) within = t;
        }
        // Outside the region, or on a corner: left out.
        const std::size_t on = within == kNoPoint ? kNoPoint : placeIn(within, point);
        if (on == kNoPoint) continue;
        const auto tri = triangles[within];
        const std::size_t v = vertices.size();
        if (on == kWithin) {
            // Its triangle cut in three.
            vertices.push_back(point);
            unindex(within);
            triangles[within] = {tri[0], tri[1], v};
            triangles.push_back({tri[1], tri[2], v});
            triangles.push_back({tri[2], tri[0], v});
            index(within);
            index(triangles.size() - 2);
            index(triangles.size() - 1);
            legalize({{tri[0], tri[1]}, {tri[1], tri[2]}, {tri[2], tri[0]}});
        } else {
            // On the edge from a to b (a corner of the grid's points often
            // is, between two coarser ones): each triangle either side of it
            // cut in two. Not on the outline, which stays as it is.
            const std::size_t a = tri[on];
            const std::size_t b = tri[(on + 1) % 3];
            const std::size_t c = tri[(on + 2) % 3];
            const auto found = edgeTriangles.find(key(a, b));
            if (boundary.count(key(a, b)) != 0 || found == edgeTriangles.end() ||
                found->second.size() != 2) {
                continue;
            }
            const std::size_t other =
                found->second[0] == within ? found->second[1] : found->second[0];
            std::size_t d = a;
            for (std::size_t corner : triangles[other]) {
                if (corner != a && corner != b) d = corner;
            }
            vertices.push_back(point);
            unindex(within);
            unindex(other);
            triangles[within] = {a, v, c};
            triangles[other] = {b, v, d};
            triangles.push_back({v, b, c});
            triangles.push_back({v, a, d});
            index(within);
            index(other);
            index(triangles.size() - 2);
            index(triangles.size() - 1);
            legalize({{b, c}, {c, a}, {a, d}, {d, b}});
        }
        home[n] = within;
        last = within;
    }
}

/// A curved face with any outline, and holes, cut into facets on its
/// surface within it (Phase 152): the region in (u, v), scaled to about the
/// surface's own lengths there, cut into triangles by ears, then points
/// added inside as far apart as the outline's own, kept Delaunay. Its
/// outline points are the edges' own, so the facets meet the faces beside.
/// Nullopt when it cannot be (a pole on it, a hole across a seam).
std::optional<std::vector<std::vector<Vec3>>> trimmedFacets(
    std::vector<LoopPoint> outer, std::vector<std::vector<LoopPoint>> holes, SurfaceMap& map,
    double maxAngle) {
    const bool reversed = signedArea(outer) < 0.0;
    if (reversed) std::reverse(outer.begin(), outer.end());
    double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    for (const LoopPoint& p : outer) {
        u0 = std::min(u0, p.uv.first);
        u1 = std::max(u1, p.uv.first);
        v0 = std::min(v0, p.uv.second);
        v1 = std::max(v1, p.uv.second);
    }
    if (!(u1 > u0) || !(v1 > v0)) return std::nullopt;
    for (auto& hole : holes) {
        // Onto the outer's side of each seam, and clockwise.
        for (int dir = 0; dir < 2; ++dir) {
            if (!map.closed(dir)) continue;
            double mid = 0.0;
            for (const LoopPoint& p : hole) mid += coord(p.uv, dir);
            mid /= static_cast<double>(hole.size());
            const double centre = dir == 0 ? 0.5 * (u0 + u1) : 0.5 * (v0 + v1);
            shift(hole, dir, map.period(dir) * std::round((centre - mid) / map.period(dir)));
        }
        for (const LoopPoint& p : hole) {
            if (p.uv.first < u0 || p.uv.first > u1 || p.uv.second < v0 || p.uv.second > v1) {
                return std::nullopt;  // across the outline: a seam through it
            }
        }
        if (signedArea(hole) > 0.0) std::reverse(hole.begin(), hole.end());
    }

    // Lengths on the surface per unit of u and of v, in the middle.
    const UV mid{0.5 * (u0 + u1), 0.5 * (v0 + v1)};
    const double du = 1e-4 * (u1 - u0);
    const double dv = 1e-4 * (v1 - v0);
    const double su =
        (map.at({mid.first + du, mid.second}) - map.at({mid.first - du, mid.second})).length() /
        (2.0 * du);
    const double sv =
        (map.at({mid.first, mid.second + dv}) - map.at({mid.first, mid.second - dv})).length() /
        (2.0 * dv);
    if (!(su > 0.0) || !(sv > 0.0)) return std::nullopt;

    std::vector<RegionVertex> vertices;
    const auto add = [&](const std::vector<LoopPoint>& loop) {
        std::vector<std::size_t> indices;
        indices.reserve(loop.size());
        for (const LoopPoint& p : loop) {
            vertices.push_back({p.uv.first * su, p.uv.second * sv, p});
            indices.push_back(vertices.size() - 1);
        }
        return indices;
    };
    const std::vector<std::size_t> outerIndices = add(outer);
    std::vector<std::vector<std::size_t>> holeIndices;
    holeIndices.reserve(holes.size());
    for (const auto& hole : holes) holeIndices.push_back(add(hole));
    const double extent = std::max((u1 - u0) * su, (v1 - v0) * sv);
    const double eps = 1e-12 * extent;

    // How far apart the points go: as near as the surface turns by
    // @p maxAngle across the region each way (its straight ways not at all),
    // and no further apart than most of the outline's own. (Its average was
    // stretched by a long straight seam, and the facets fell short.)
    std::vector<double> lengths;
    const auto lengthsOf = [&](const std::vector<std::size_t>& loop) {
        for (std::size_t k = 0; k < loop.size(); ++k) {
            const RegionVertex& a = vertices[loop[k]];
            const RegionVertex& b = vertices[loop[(k + 1) % loop.size()]];
            lengths.push_back(std::hypot(a.x - b.x, a.y - b.y));
        }
    };
    lengthsOf(outerIndices);
    for (const auto& h : holeIndices) lengthsOf(h);
    std::nth_element(lengths.begin(),
                     lengths.begin() + static_cast<std::ptrdiff_t>(lengths.size() / 2),
                     lengths.end());
    double spacing = lengths.empty() ? 0.0 : lengths[lengths.size() / 2];
    const UV low{u0, 0.5 * (v0 + v1)};
    const UV high{u1, 0.5 * (v0 + v1)};
    const UV lowV{0.5 * (u0 + u1), v0};
    const UV highV{0.5 * (u0 + u1), v1};
    const auto turnsU = piecesAcross(map, low, high, 0, maxAngle);
    const auto turnsV = piecesAcross(map, lowV, highV, 1, maxAngle);
    if (turnsU > 1) spacing = std::min(spacing, (u1 - u0) * su / static_cast<double>(turnsU));
    if (turnsV > 1) spacing = std::min(spacing, (v1 - v0) * sv / static_cast<double>(turnsV));

    const auto onSurface = [&](double x, double y) {
        const UV uv{x / su, y / sv};
        return RegionVertex{x, y, {map.at(uv), uv, -1}};
    };
    auto clipped = earClip(vertices, outerIndices, holeIndices, eps, spacing, onSurface);
    if (!clipped) return std::nullopt;
    auto& triangles = clipped->triangles;
    // The polygon's edges, the outline and the bridges to its holes: never
    // flipped, and kept clear of the points added inside.
    std::set<std::pair<std::size_t, std::size_t>> boundary;
    for (std::size_t k = 0; k < clipped->polygon.size(); ++k) {
        const std::size_t a = clipped->polygon[k];
        const std::size_t b = clipped->polygon[(k + 1) % clipped->polygon.size()];
        boundary.insert(a < b ? std::make_pair(a, b) : std::make_pair(b, a));
    }

    // Points inside, as far apart as the outline's own and as far from it:
    // at a grid's corners. A column's are within the region where a line up
    // it has crossed the outline and holes an odd number of times; each is
    // clear of the region's edges near it, found by the grid's cells, each
    // listing the edges that pass near it.
    std::vector<RegionVertex> inside;
    std::vector<std::size_t> beside;
    const double x0 = u0 * su, x1 = u1 * su, y0 = v0 * sv, y1 = v1 * sv;
    const auto nx = spacing > 0.0
                        ? static_cast<std::size_t>(std::min(400.0, std::floor((x1 - x0) / spacing)))
                        : 0;
    const auto ny = spacing > 0.0
                        ? static_cast<std::size_t>(std::min(400.0, std::floor((y1 - y0) / spacing)))
                        : 0;
    if (nx > 1 && ny > 1) {
        const double cw = (x1 - x0) / static_cast<double>(nx);
        const double ch = (y1 - y0) / static_cast<double>(ny);
        const auto cell = [&](double at, double from, double size, std::size_t count) {
            const double k = std::floor((at - from) / size);
            return static_cast<std::size_t>(std::clamp(k, 0.0, static_cast<double>(count - 1)));
        };
        // An edge is listed in the cells round each point along it, a
        // quarter-cell apart: a point within 0.6 spacing (less than a cell)
        // of it finds it in its own cell.
        std::vector<std::vector<std::pair<std::size_t, std::size_t>>> near(nx * ny);
        std::vector<std::size_t> listed(nx * ny, kNoPoint);
        const double step = 0.25 * std::min(cw, ch);
        std::size_t edgeNumber = 0;
        for (const auto& edge : boundary) {
            const RegionVertex& p = vertices[edge.first];
            const RegionVertex& q = vertices[edge.second];
            const auto pieces =
                static_cast<std::size_t>(std::ceil(std::hypot(q.x - p.x, q.y - p.y) / step));
            for (std::size_t k = 0; k <= pieces; ++k) {
                const double t =
                    pieces == 0 ? 0.0 : static_cast<double>(k) / static_cast<double>(pieces);
                const std::size_t ci = cell(p.x + (q.x - p.x) * t, x0, cw, nx);
                const std::size_t cj = cell(p.y + (q.y - p.y) * t, y0, ch, ny);
                for (std::size_t i = ci == 0 ? 0 : ci - 1; i <= std::min(ci + 1, nx - 1); ++i) {
                    for (std::size_t j = cj == 0 ? 0 : cj - 1; j <= std::min(cj + 1, ny - 1); ++j) {
                        if (listed[i * ny + j] == edgeNumber) continue;
                        listed[i * ny + j] = edgeNumber;
                        near[i * ny + j].push_back(edge);
                    }
                }
            }
            ++edgeNumber;
        }
        const auto clear = [&](double x, double y) {
            for (const auto& [a, b] : near[cell(x, x0, cw, nx) * ny + cell(y, y0, ch, ny)]) {
                const RegionVertex& p = vertices[a];
                const RegionVertex& q = vertices[b];
                const double lx = q.x - p.x, ly = q.y - p.y;
                const double len2 = lx * lx + ly * ly;
                const double t =
                    len2 > 0.0 ? std::clamp(((x - p.x) * lx + (y - p.y) * ly) / len2, 0.0, 1.0)
                               : 0.0;
                if (std::hypot(x - (p.x + lx * t), y - (p.y + ly * t)) < 0.6 * spacing) {
                    return false;
                }
            }
            return true;
        };
        // Where a line up at x crosses the outline and the holes, upwards.
        std::vector<double> crossings;
        const auto crossingsAt = [&](double x) {
            crossings.clear();
            const auto of = [&](const std::vector<std::size_t>& loop) {
                for (std::size_t k = 0; k < loop.size(); ++k) {
                    const RegionVertex& p = vertices[loop[k]];
                    const RegionVertex& q = vertices[loop[(k + 1) % loop.size()]];
                    if ((p.x <= x) == (q.x <= x)) continue;
                    crossings.push_back(p.y + (x - p.x) * (q.y - p.y) / (q.x - p.x));
                }
            };
            of(outerIndices);
            for (const auto& h : holeIndices) of(h);
            std::sort(crossings.begin(), crossings.end());
        };
        const auto xAt = [&](std::size_t i) {
            return x0 + (x1 - x0) * static_cast<double>(i) / static_cast<double>(nx);
        };
        const auto yAt = [&](std::size_t j) {
            return y0 + (y1 - y0) * static_cast<double>(j) / static_cast<double>(ny);
        };
        // Each corner's level: the most times both its column and its row
        // halve evenly.
        struct Corner {
            std::size_t i, j;
            int level;
        };
        std::vector<Corner> corners;
        for (std::size_t i = 1; i < nx; ++i) {
            crossingsAt(xAt(i));
            std::size_t below = 0;
            for (std::size_t j = 1; j < ny; ++j) {
                const double y = yAt(j);
                while (below < crossings.size() && crossings[below] < y) ++below;
                if (below % 2 == 0 || !clear(xAt(i), y)) continue;
                corners.push_back({i, j, std::min(std::countr_zero(i), std::countr_zero(j))});
            }
        }
        // Coarse to fine, every 2^k-th corner before those between: each
        // point then goes in among others about as far apart as it, and only
        // a few triangles change. (Column by column, each went in beside the
        // triangles across the region not yet filled, and changed many.)
        std::stable_sort(corners.begin(), corners.end(),
                         [](const Corner& a, const Corner& b) { return a.level > b.level; });
        std::vector<std::size_t> placed(nx * ny, kNoPoint);
        inside.reserve(corners.size());
        beside.reserve(corners.size());
        for (const Corner& c : corners) {
            // Beside it, in before it: a corner of its level or coarser.
            const std::size_t step = std::size_t{1} << c.level;
            std::size_t near = c.i > step ? placed[(c.i - step) * ny + c.j] : kNoPoint;
            if (near == kNoPoint && c.j > step) near = placed[c.i * ny + c.j - step];
            const UV uv{xAt(c.i) / su, yAt(c.j) / sv};
            inside.push_back({xAt(c.i), yAt(c.j), {map.at(uv), uv, -1}});
            beside.push_back(near);
            placed[c.i * ny + c.j] = inside.size() - 1;
        }
    }
    insertPoints(vertices, triangles, inside, beside, boundary, eps);

    std::vector<std::vector<Vec3>> facets;
    facets.reserve(triangles.size());
    for (const auto& tri : triangles) {
        std::vector<Vec3> facet{vertices[tri[0]].at.point, vertices[tri[1]].at.point,
                                vertices[tri[2]].at.point};
        if (reversed) std::reverse(facet.begin(), facet.end());
        facets.push_back(std::move(facet));
    }
    return facets;
}

}  // namespace

FacetedSolid facetCurved(const topo::Solid& exact, double maxAngle) {
    FacetedSolid result;
    maxAngle = std::clamp(maxAngle, 1e-3, math::kPi / 2);
    double size = 0.0;
    for (const auto& v : exact.vertices()) {
        size = std::max(size, (v.point - exact.vertices().front().point).length());
    }

    // Each edge's points once, so the faces either side share them; each
    // chord's curve, by its ends.
    std::unordered_map<const Edge*, std::vector<Vec3>> samples;
    std::map<std::pair<Key, Key>, std::shared_ptr<geo::NurbsCurve>> chordCurves;
    for (const auto& e : exact.edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->origin == nullptr || e.halfEdge->next == nullptr ||
            e.halfEdge->next->origin == nullptr) {
            continue;
        }
        auto points = sampleEdge(e, maxAngle);
        if (e.curve && e.curve->degree() > 1) {
            for (size_t i = 0; i + 1 < points.size(); ++i) {
                auto a = keyOf(points[i]);
                auto b = keyOf(points[i + 1]);
                if (b < a) std::swap(a, b);
                chordCurves[{a, b}] = e.curve;
            }
        }
        samples.emplace(&e, std::move(points));
    }
    const auto along = [&samples](const HalfEdge* he) {
        std::vector<Vec3> s = samples.at(he->edge);
        if (he != he->edge->halfEdge) std::reverse(s.begin(), s.end());
        return s;
    };
    const auto polygon = [&along](const std::vector<const HalfEdge*>& loop) {
        std::vector<Vec3> points;
        for (const HalfEdge* he : loop) {
            const auto s = along(he);
            points.insert(points.end(), s.begin(), s.end() - 1);
        }
        return points;
    };

    std::vector<SolidSewer::InputFace> faces;
    for (const auto& face : exact.faces()) {
        const auto outer = loopOf(face.outerLoop);
        if (outer.empty()) {
            result.error = "a face has no boundary";
            return result;
        }
        std::vector<std::vector<const HalfEdge*>> inner;
        inner.reserve(face.innerLoops.size());
        for (const Wire* w : face.innerLoops) inner.push_back(loopOf(w));
        const auto& surface = face.surface;
        const bool curved = surface != nullptr && !isFlat(*surface);

        if (curved) {
            SurfaceMap map(*surface);
            const auto edgesOf = [&along](const std::vector<const HalfEdge*>& loop) {
                std::vector<std::vector<Vec3>> edges;
                edges.reserve(loop.size());
                for (const HalfEdge* he : loop) edges.push_back(along(he));
                return edges;
            };
            const auto emit = [&](std::vector<std::vector<Vec3>> facets) {
                for (size_t k = 0; k < facets.size(); ++k) {
                    SolidSewer::InputFace f;
                    f.points = std::move(facets[k]);
                    f.topoId = topo::TopologyID::fromTag(face.topoId.tag() +
                                                         "/facet:" + std::to_string(k));
                    f.analyticSurface = surface;
                    faces.push_back(std::move(f));
                }
            };
            const auto edges = edgesOf(outer);
            if (inner.empty()) {
                if (const auto rectangle = faceInUV(edges, map)) {
                    if (auto facets = gridFacets(*rectangle, map, size, maxAngle)) {
                        emit(std::move(*facets));
                        continue;
                    }
                }
            }
            // Any other outline, and holes: cut within its trim (Phase 152).
            if (auto outline = loopInUV(edges, map)) {
                std::vector<std::vector<LoopPoint>> holes;
                bool read = true;
                for (const auto& loop : inner) {
                    auto hole = loopInUV(edgesOf(loop), map);
                    if (!hole) {
                        read = false;
                        break;
                    }
                    holes.push_back(std::move(*hole));
                }
                if (read) {
                    if (auto facets =
                            trimmedFacets(std::move(*outline), std::move(holes), map, maxAngle)) {
                        emit(std::move(*facets));
                        continue;
                    }
                }
            }
        }
        SolidSewer::InputFace f;
        f.points = polygon(outer);
        for (const auto& loop : inner) f.holes.push_back(polygon(loop));
        f.topoId = face.topoId;
        if (curved) {
            // Its outline alone: a loop that meets itself (a seam) is no
            // polygon, and cannot be one face.
            std::map<Key, int> seen;
            for (const Vec3& p : f.points) {
                if (++seen[keyOf(p)] > 1) {
                    result.error =
                        "the curved face " + face.topoId.tag() + " could not be cut into facets";
                    return result;
                }
            }
            f.analyticSurface = surface;
            result.outlined.push_back(face.topoId.tag());
        }
        faces.push_back(std::move(f));
    }

    auto solid = SolidSewer::sew(faces);
    if (solid == nullptr || !solid->checkManifold()) {
        result.error = "the facets did not join into a closed solid";
        return result;
    }
    for (auto& e : solid->edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->next == nullptr) continue;
        auto a = keyOf(e.halfEdge->origin->point);
        auto b = keyOf(e.halfEdge->next->origin->point);
        if (b < a) std::swap(a, b);
        const auto it = chordCurves.find({a, b});
        if (it != chordCurves.end()) e.analyticCurve = it->second;
    }
    result.solid = std::move(solid);
    return result;
}

}  // namespace hz::model
