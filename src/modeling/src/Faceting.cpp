#include "horizon/modeling/Faceting.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
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

        if (curved && inner.empty()) {
            SurfaceMap map(*surface);
            std::vector<std::vector<Vec3>> edges;
            edges.reserve(outer.size());
            for (const HalfEdge* he : outer) edges.push_back(along(he));
            if (const auto rectangle = faceInUV(edges, map)) {
                if (auto facets = gridFacets(*rectangle, map, size, maxAngle)) {
                    for (size_t k = 0; k < facets->size(); ++k) {
                        SolidSewer::InputFace f;
                        f.points = std::move((*facets)[k]);
                        f.topoId = topo::TopologyID::fromTag(face.topoId.tag() +
                                                             "/facet:" + std::to_string(k));
                        f.analyticSurface = surface;
                        faces.push_back(std::move(f));
                    }
                    continue;
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
