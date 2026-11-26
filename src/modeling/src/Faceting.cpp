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
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/math/Vec2.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/SolidSewer.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/LoopUnwrap.h"
#include "horizon/topology/Queries.h"
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

/// How far a polyline through @p points turns: the angles between its
/// successive chords. A point nearer the last one kept than @p shortest is
/// passed over: a chord that short has a direction of rounding alone. Sampled
/// densely across a span of a few last bits (a seam straight up a band in
/// (u, v), its ends at one u but for rounding), such chords turned hundreds
/// of radians, and the span was cut into hundreds of pieces. Where every
/// chord is at least @p shortest, every point is kept and the turn is as it
/// always was, to the last bit.
double turningOf(const std::vector<Vec3>& points, double shortest) {
    std::vector<Vec3> kept;
    kept.reserve(points.size());
    for (const Vec3& p : points) {
        if (kept.empty() || (p - kept.back()).length() >= shortest) kept.push_back(p);
    }
    double turn = 0.0;
    for (std::size_t i = 1; i + 1 < kept.size(); ++i) {
        const Vec3 a = kept[i] - kept[i - 1];
        const Vec3 b = kept[i + 1] - kept[i];
        const double la = a.length();
        const double lb = b.length();
        if (la > 0.0 && lb > 0.0) turn += std::acos(std::clamp(a.dot(b) / (la * lb), -1.0, 1.0));
    }
    return turn;
}

/// The length below which a chord between points of @p points has a direction
/// of rounding alone: a thousand times the last bit of their largest coordinate.
double roundingChord(const std::vector<Vec3>& points) {
    double largest = 0.0;
    for (const Vec3& p : points) {
        largest = std::max({largest, std::abs(p.x), std::abs(p.y), std::abs(p.z)});
    }
    return 1024.0 * std::numeric_limits<double>::epsilon() * largest;
}

/// The points of an edge from its half-edge's origin to its end: along its
/// curve, equal lengths apart, as many as keep each chord within
/// @p maxAngle of turning, and at least @p least chords of a curve that
/// turns at all; the ends its vertices.
std::vector<Vec3> sampleEdge(const Edge& e, double maxAngle, int least = 1) {
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
    std::vector<double> length(dense.size(), 0.0);
    for (size_t i = 1; i < dense.size(); ++i) {
        length[i] = length[i - 1] + (dense[i] - dense[i - 1]).length();
    }
    const double turn = turningOf(dense, roundingChord(dense));
    const int n =
        std::max(turn > 1e-9 ? least : 1, static_cast<int>(std::ceil(turn / maxAngle - 1e-9)));
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
/// How far off its surface, in millimetres, another system's edge may lie
/// and still bound its face: ten micrometres, or on a large surface this
/// much of its size (see SurfaceMap::uvOf).
constexpr double kOffSurface = 1e-2;
constexpr double kOffSurfaceOfSize = 5e-5;

class SurfaceMap {
public:
    explicit SurfaceMap(const geo::NurbsSurface& s) : m_s(s), m_size(sizeOf(s)) {}

    const geo::NurbsSurface& surface() const { return m_s; }
    /// How exactly a point is found on it (uvOf): a ten-millionth of its
    /// size.
    double precision() const { return 1e-7 * std::max(m_size, 1e-300); }
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
        // On it within a millionth of its size, or within ten micrometres,
        // or on a large surface 5e-5 of its size: as near as other systems
        // keep an edge to its surface. In the external corpus most files'
        // edges lie up to 4.6 micrometres off; IronCAD's impeller's, 20
        // micrometres off surfaces 600 mm across, though it says it is
        // accurate to a micrometre. Never a thousandth of its size.
        const double onIt =
            std::max(1e-6 * m_size, std::min(std::max(kOffSurface, kOffSurfaceOfSize * m_size),
                                             1e-3 * std::max(m_size, 1e-300)));
        UV best = m_s.locate(p);
        const double tol = 1e-7 * std::max(m_size, 1e-300);
        if (miss(best) > tol && miss(best) > onIt) {
            // Not found from the best starts of a grid: from every cell of
            // a finer one. (A point already on it, as near as another
            // system's is, is not looked for again: that took a second a
            // face.)
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
        if (miss(best) <= onIt) result = best;
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
/// its edge a sphere's is a pole, all rounding). Its turning is measured on
/// chords no shorter than sixteen times the precision points are found on
/// the surface to (turningOf), or than rounding: a span shorter than that, a
/// straight seam's few last bits of u, is one piece.
size_t piecesAcross(const SurfaceMap& map, UV from, UV to, int dir, double maxAngle) {
    constexpr int kDense = 256;
    std::vector<Vec3> dense;
    for (int i = 0; i <= kDense; ++i) {
        UV uv = from;
        coord(uv, dir) = coord(from, dir) + (coord(to, dir) - coord(from, dir)) * i / kDense;
        coord(uv, 1 - dir) = 0.5 * (coord(from, 1 - dir) + coord(to, 1 - dir));
        dense.push_back(map.at(uv));
    }
    const double turn = turningOf(dense, std::max(16.0 * map.precision(), roundingChord(dense)));
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
/// A boundary loop in (u, v), continuous along it: it closes there, or it
/// winds once round a closed direction of the surface (a circle round a
/// cylinder), and its last point is its first a period on.
struct UVChain {
    std::vector<LoopPoint> points;  ///< its first point again at the end
    int winds = -1;                 ///< the direction it winds round, or -1: it closes
};

/// @p edges, one loop's, in (u, v); nullopt when a point is off the surface
/// or a pole, or it neither closes nor winds once round.
std::optional<UVChain> chainInUV(const std::vector<std::vector<Vec3>>& edges, SurfaceMap& map) {
    UVChain chain;
    auto& loop = chain.points;
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
    if (loop.size() < 3) return std::nullopt;
    const LoopPoint& first = loop.front();
    const LoopPoint& last = loop.back();
    const double tol = 1e-9 * std::max({1.0, std::abs(first.uv.first), std::abs(first.uv.second)});
    const double off[2] = {last.uv.first - first.uv.first, last.uv.second - first.uv.second};
    if (std::abs(off[0]) <= tol && std::abs(off[1]) <= tol) return chain;
    for (int dir = 0; dir < 2; ++dir) {
        if (map.closed(dir) && std::abs(off[1 - dir]) <= tol &&
            std::abs(std::abs(off[dir]) - map.period(dir)) <= tol * map.period(dir)) {
            chain.winds = dir;
            return chain;
        }
    }
    return std::nullopt;
}

/// @p edges, a loop that closes in (u, v), as a polygon there; nullopt when
/// it does not (see chainInUV), or it winds round the surface.
std::optional<std::vector<LoopPoint>> loopInUV(const std::vector<std::vector<Vec3>>& edges,
                                               SurfaceMap& map) {
    auto chain = chainInUV(edges, map);
    if (!chain || chain->winds >= 0 || chain->points.size() < 4) return std::nullopt;
    chain->points.pop_back();
    return std::move(chain->points);
}

/// A band round a closed surface, as a polygon in (u, v) with holes.
struct Band {
    std::vector<LoopPoint> outline;
    std::vector<std::vector<LoopPoint>> holes;
};

/// A face bounded by two loops that each wind once round a closed surface,
/// the other way from each other, with no seam between them: as other
/// systems write a cylinder's side between its two rims. Cut along a seam of
/// its own, from a point of the one straight to the nearest of the other in
/// (u, v), it is one polygon: the one loop, the seam, the other loop, and the
/// seam back a period away. The seam's points are on the surface, the same
/// on both sides, as close as the surface turns by @p maxAngle. The face's
/// other loops, which close, are its holes. It faces as its loops say, or,
/// when they go the same way round, as its surface does. Nullopt when it is
/// not a band.
std::optional<Band> bandInUV(std::vector<UVChain> loops, SurfaceMap& map, double maxAngle) {
    std::vector<std::size_t> winding;
    for (std::size_t k = 0; k < loops.size(); ++k) {
        if (loops[k].winds >= 0) winding.push_back(k);
    }
    if (winding.size() != 2) return std::nullopt;
    UVChain& one = loops[winding[0]];
    UVChain& other = loops[winding[1]];
    const int dir = one.winds;
    if (other.winds != dir) return std::nullopt;
    const double period = map.period(dir);
    const auto way = [dir](const UVChain& c) {
        return coord(c.points.back().uv, dir) - coord(c.points.front().uv, dir);
    };
    // Round it the other way from the one. Solid Edge writes a band's two
    // rims the same way round, both loops forward, which leaves the face on
    // the left of one and the right of the other; the band between them is
    // the face all the same, facing out as its surface does (below).
    const bool sameWay = way(one) * way(other) > 0.0;
    if (sameWay) std::reverse(other.points.begin(), other.points.end());

    // Cut where the seam crosses no hole: at the one's point furthest round
    // from every hole. (From its first point, the seam went through one of
    // the holes round a tube, and the band could not be cut.)
    if (loops.size() > 2) {
        std::vector<std::pair<double, double>> spans;  // each hole's, round the period
        for (std::size_t k = 0; k < loops.size(); ++k) {
            if (loops[k].winds >= 0) continue;
            double lo = 1e300, hi = -1e300;
            for (const LoopPoint& p : loops[k].points) {
                lo = std::min(lo, coord(p.uv, dir));
                hi = std::max(hi, coord(p.uv, dir));
            }
            spans.emplace_back(lo, hi);
        }
        const auto clearance = [&](double t) {
            double nearest = 1e300;
            for (const auto& [lo, hi] : spans) {
                double d = t - lo;
                d -= period * std::floor(d / period);  // round from lo, in [0, period)
                if (d <= hi - lo) return 0.0;          // within it
                nearest = std::min({nearest, d - (hi - lo), period - d});
            }
            return nearest;
        };
        auto& pts = one.points;
        std::size_t cut = 0;
        double widest = -1.0;
        for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
            const double c = clearance(coord(pts[k].uv, dir));
            if (c > widest) {
                widest = c;
                cut = k;
            }
        }
        if (cut > 0) {
            const double across = coord(pts.back().uv, dir) - coord(pts.front().uv, dir);
            std::vector<LoopPoint> rotated(pts.begin() + static_cast<std::ptrdiff_t>(cut),
                                           pts.end());
            for (std::size_t k = 1; k <= cut; ++k) {
                LoopPoint p = pts[k];
                coord(p.uv, dir) += across;
                rotated.push_back(p);
            }
            pts = std::move(rotated);
        }
    }

    // The other loop from its point nearest the one's start round the
    // period, a period on, so it runs back from the one's end to its start.
    const LoopPoint& from = one.points.front();
    auto& pts = other.points;
    pts.pop_back();  // its start, a period on
    std::size_t nearest = 0;
    double best = 1e300;
    for (std::size_t k = 0; k < pts.size(); ++k) {
        double d = coord(pts[k].uv, dir) - coord(from.uv, dir);
        d -= period * std::round(d / period);
        // Across it too, on a surface closed that way as well (a torus).
        double e = coord(pts[k].uv, 1 - dir) - coord(from.uv, 1 - dir);
        if (map.closed(1 - dir)) e -= map.period(1 - dir) * std::round(e / map.period(1 - dir));
        const double far = std::hypot(d, e);
        if (far < best) {
            best = far;
            nearest = k;
        }
    }
    std::vector<LoopPoint> back;
    back.reserve(pts.size() + 1);
    const double step = way(other) < 0.0 ? -period : period;
    for (std::size_t k = 0; k <= pts.size(); ++k) {
        LoopPoint p = pts[(nearest + k) % pts.size()];
        if (nearest + k >= pts.size()) coord(p.uv, dir) += step;  // round past its start
        back.push_back(p);
    }
    // Onto the one's end: it starts a period on from the one's start.
    const double target = coord(one.points.back().uv, dir);
    shift(back, dir, period * std::round((target - coord(back.front().uv, dir)) / period));

    // The seam, from the one's start to the other's end, and back from the
    // other's start to the one's end: the same points a period apart. The
    // outline runs along the one, down the seam's far side, back along the
    // other, and up the seam's near side to the start.
    const auto outlineWith = [&](const std::vector<LoopPoint>& back) {
        const UV a = from.uv;
        const UV b = back.back().uv;
        const std::size_t pieces = std::max(piecesAcross(map, a, b, 1 - dir, maxAngle),
                                            piecesAcross(map, a, b, dir, maxAngle));
        std::vector<LoopPoint> seam;  // from the one's start to the other's end, inside
        for (std::size_t k = 1; k < pieces; ++k) {
            const double t = static_cast<double>(k) / static_cast<double>(pieces);
            const UV uv{a.first + (b.first - a.first) * t, a.second + (b.second - a.second) * t};
            seam.push_back({map.at(uv), uv, -1});
        }
        std::vector<LoopPoint> outline = one.points;  // start to end, a period round
        const double across = coord(one.points.back().uv, dir) - coord(one.points.front().uv, dir);
        for (LoopPoint p : seam) {
            coord(p.uv, dir) += across;
            outline.push_back(p);
        }
        outline.insert(outline.end(), back.begin(), back.end());
        outline.insert(outline.end(), seam.rbegin(), seam.rend());
        return outline;
    };
    Band band;
    band.outline = outlineWith(back);
    if (signedArea(band.outline) < 0.0) {
        // Round the other way in (u, v): on the right of its loops. On a
        // surface closed the other way too (a torus), two loops round it
        // bound two bands, and the face is the other one, on their left.
        if (!sameWay && map.closed(1 - dir)) {
            const double round = map.period(1 - dir);
            shift(back, 1 - dir,
                  coord(back.front().uv, 1 - dir) > coord(from.uv, 1 - dir) ? -round : round);
            band.outline = outlineWith(back);
        }
        // Otherwise its loops are not what its surface says (Solid Edge's):
        // it faces as its surface does, out of the solid, as the reader
        // made every surface face.
        if (signedArea(band.outline) < 0.0) {
            std::reverse(band.outline.begin(), band.outline.end());
        }
    }
    for (std::size_t k = 0; k < loops.size(); ++k) {
        if (k == winding[0] || k == winding[1]) continue;
        loops[k].points.pop_back();
        band.holes.push_back(std::move(loops[k].points));
    }
    return band;
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

/// Whether @p p is in the triangle a, b, c (anticlockwise), or within
/// @p near of its edges: a point found on a surface is as exact as finding
/// it, so points of one iso-line of it (a torus's, EUCLID's) are not quite
/// on a line, and one a hair outside an ear's edge let the ear be cut.
bool inOrOn(const RegionVertex& p, const RegionVertex& a, const RegionVertex& b,
            const RegionVertex& c, double near) {
    const auto inside = [&p, near](const RegionVertex& s, const RegionVertex& t) {
        return cross2(s, t, p) >= -near * std::hypot(t.x - s.x, t.y - s.y);
    };
    return inside(a, b) && inside(b, c) && inside(c, a);
}

/// Whether the points @p loop of @p vertices all lie within @p near of the
/// line through the two furthest apart of them: a region with no width.
bool flatWithin(const std::vector<RegionVertex>& vertices, const std::vector<std::size_t>& loop,
                double near) {
    if (loop.size() < 3) return true;
    const RegionVertex& a = vertices[loop.front()];
    std::size_t far = loop.front();
    double reach = 0.0;
    for (std::size_t i : loop) {
        const double d = std::hypot(vertices[i].x - a.x, vertices[i].y - a.y);
        if (d > reach) {
            reach = d;
            far = i;
        }
    }
    if (!(reach > 0.0)) return true;
    const RegionVertex& b = vertices[far];
    return std::all_of(loop.begin(), loop.end(), [&](std::size_t i) {
        return std::abs(cross2(a, b, vertices[i])) <= 2.0 * near * reach;
    });
}

/// Whether @p b turns left from a to c by more than @p near: it is no
/// corner when it is straight on within that.
bool turnsLeft(const RegionVertex& a, const RegionVertex& b, const RegionVertex& c, double near) {
    return cross2(a, b, c) > near * std::hypot(c.x - a.x, c.y - a.y);
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
                               std::vector<std::vector<std::size_t>> holes, double eps, double near,
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
            if (!turnsLeft(a, b, c, near)) continue;  // reflex, or straight on
            bool empty = true;
            for (std::size_t other : left) {
                if (other == ia || other == ib || other == ic) continue;
                const RegionVertex& v = vertices[other];
                // A copy at a corner (a bridge's ends) is that corner.
                const auto same = [&](const RegionVertex& w) {
                    return std::abs(v.x - w.x) <= eps && std::abs(v.y - w.y) <= eps;
                };
                if (same(a) || same(b) || same(c)) continue;
                if (inOrOn(v, a, b, c, near)) {
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
        if (!cut && flatWithin(vertices, left, near)) {
            // What is left lies along a line within near of it: the last of
            // an edge bent a hair (a Creo strip's rim, 3e-4 mm over 50),
            // whose ears were cut as slivers down to straight. A fan of
            // slivers finishes it, every point of the outline a corner.
            for (std::size_t k = 1; k + 1 < left.size(); ++k) {
                triangles.push_back({left[0], left[k], left[k + 1]});
            }
            return clipped;
        }
        if (!cut || ++guard > 1000000) return std::nullopt;
    }
    if (!turnsLeft(vertices[left[0]], vertices[left[1]], vertices[left[2]], near)) {
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
    // Near enough to an edge to be on it: points found on a surface are as
    // exact as finding them (a ten-millionth of the surface's size, however
    // small the face on it), and no nearer: a corner that small is none.
    const double near = std::max(eps, map.precision());

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
    // The outline's own, not its holes': a tube's wall with a few small
    // holes in it was cut as finely as their edges, into a third of a
    // million triangles, and took 20 s. Near a hole, the cut follows it.
    lengthsOf(outerIndices);
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
    auto clipped = earClip(vertices, outerIndices, holeIndices, eps, near, spacing, onSurface);
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

/// A flat face whose outline strays off its plane, cut into triangles, each
/// flat, their corners the outline's own points: nullopt when it is flat
/// within half what a solid's validity allows (1e-7, or 1e-9 of the part's
/// @p size, times the face's extent), and is one polygon. Creo writes a
/// flat face with a corner 0.75 mm off its plane; others' edges stray by
/// micrometres.
std::optional<std::vector<std::vector<Vec3>>> offPlaneFacets(
    const std::vector<Vec3>& outline, const std::vector<std::vector<Vec3>>& holes, double size) {
    if (outline.size() < 4) return std::nullopt;
    Vec3 normal;
    Vec3 centre;
    math::BoundingBox box;
    for (std::size_t i = 0; i < outline.size(); ++i) {
        const Vec3& a = outline[i];
        const Vec3& b = outline[(i + 1) % outline.size()];
        normal.x += (a.y - b.y) * (a.z + b.z);
        normal.y += (a.z - b.z) * (a.x + b.x);
        normal.z += (a.x - b.x) * (a.y + b.y);
        centre = centre + a;
        box.expand(a);
    }
    if (!(normal.length() > 0.0)) return std::nullopt;
    normal = normal.normalized();
    centre = centre / static_cast<double>(outline.size());
    const Vec3 extent = box.max() - box.min();
    const double tol =
        0.5 * std::max(1e-7, 1e-9 * size) * std::max({1.0, extent.x, extent.y, extent.z});
    const auto off = [&](const Vec3& q) { return std::abs(normal.dot(q - centre)) > tol; };
    bool flat = std::none_of(outline.begin(), outline.end(), off);
    for (const auto& hole : holes) flat = flat && std::none_of(hole.begin(), hole.end(), off);
    if (flat) return std::nullopt;

    // In the plane that fits it best: its outline anticlockwise about its
    // normal (Newell's), its holes clockwise.
    const Vec3 across = std::abs(normal.dot(Vec3::UnitX)) < 0.9 ? Vec3::UnitX : Vec3::UnitY;
    const Vec3 xAxis = (across - normal * normal.dot(across)).normalized();
    const Vec3 yAxis = normal.cross(xAxis);
    std::vector<RegionVertex> vertices;
    const auto add = [&](const std::vector<Vec3>& loop) {
        std::vector<std::size_t> indices;
        for (const Vec3& q : loop) {
            vertices.push_back({(q - centre).dot(xAxis), (q - centre).dot(yAxis), {q, {}, -1}});
            indices.push_back(vertices.size() - 1);
        }
        return indices;
    };
    const std::vector<std::size_t> outer = add(outline);
    std::vector<std::vector<std::size_t>> inner;
    for (const auto& hole : holes) {
        auto indices = add(hole);
        double area = 0.0;
        for (std::size_t k = 0; k < indices.size(); ++k) {
            const RegionVertex& a = vertices[indices[k]];
            const RegionVertex& b = vertices[indices[(k + 1) % indices.size()]];
            area += a.x * b.y - b.x * a.y;
        }
        if (area > 0.0) std::reverse(indices.begin(), indices.end());
        inner.push_back(std::move(indices));
    }
    const double reach = std::max({extent.x, extent.y, extent.z, 1e-300});
    const auto onPlane = [&](double x, double y) {
        const Vec3 q = centre + xAxis * x + yAxis * y;
        return RegionVertex{x, y, {q, {}, -1}};
    };
    auto clipped =
        earClip(vertices, outer, std::move(inner), 1e-12 * reach, tol, 2.0 * reach, onPlane);
    if (!clipped) return std::nullopt;
    std::vector<std::vector<Vec3>> facets;
    facets.reserve(clipped->triangles.size());
    for (const auto& [a, b, c] : clipped->triangles) {
        facets.push_back({vertices[a].at.point, vertices[b].at.point, vertices[c].at.point});
    }
    return facets;
}

namespace {

// ---------------------------------------------------------------------------
// The bridge from exact faces (stage 3): facetExact.
// ---------------------------------------------------------------------------

/// The most chords one curve is cut into, as PrimitiveFactory::segmentsForTolerance stops.
constexpr int kMostSteps = 4096;

/// How far off a side of a face's rectangle in (u, v) a pcurve's sample may be and still lie
/// along it, relative to the larger of 1 and the coordinate's size: a guard on parameter
/// arithmetic (the pcurves of a constructed face are exact lines along their sides), not a
/// geometric tolerance.
constexpr double kOnSide = 1e-9;

/// Points along @p e from its half-edge's origin to its end: its vertices and, along its exact
/// curve, @p samples - 2 between, at equal steps of its parameter; along its NURBS curve where it
/// has no exact one. For telling whether a face's outline lies in a plane.
std::vector<Vec3> outlinePoints(const Edge& e, int samples) {
    std::vector<Vec3> out;
    if (e.halfEdge == nullptr || e.halfEdge->origin == nullptr) return out;
    out.push_back(e.halfEdge->origin->point);
    for (int i = 1; i + 1 < samples; ++i) {
        const double f = static_cast<double>(i) / static_cast<double>(samples - 1);
        if (e.exactCurve) {
            out.push_back(e.exactCurve->evaluate(e.curveStart + (e.curveEnd - e.curveStart) * f));
        } else if (e.curve) {
            out.push_back(
                e.curve->evaluate(e.curve->tMin() + (e.curve->tMax() - e.curve->tMin()) * f));
        }
    }
    if (e.halfEdge->next != nullptr && e.halfEdge->next->origin != nullptr) {
        out.push_back(e.halfEdge->next->origin->point);
    }
    return out;
}

/// Whether @p face's outline, its edges along their curves, lies in one plane within
/// @p tolerance: three of its points not in a line, and every point within it of their plane.
bool flatOutline(const Face& face, double tolerance) {
    std::vector<Vec3> points;
    std::vector<const Wire*> loops{face.outerLoop};
    loops.insert(loops.end(), face.innerLoops.begin(), face.innerLoops.end());
    for (const Wire* wire : loops) {
        for (const HalfEdge* he : loopOf(wire)) {
            const auto along = outlinePoints(*he->edge, topo::kLoopSamples);
            points.insert(points.end(), along.begin(), along.end());
        }
    }
    if (points.size() < 3) return false;
    const Vec3& a = points.front();
    const auto farthest = [&](const std::function<double(const Vec3&)>& from) {
        std::size_t best = 0;
        for (std::size_t i = 1; i < points.size(); ++i) {
            if (from(points[i]) > from(points[best])) best = i;
        }
        return points[best];
    };
    const Vec3 b = farthest([&a](const Vec3& p) { return (p - a).length(); });
    const Vec3 ab = b - a;
    if (!(ab.length() > tolerance)) return false;
    const Vec3 c = farthest([&a, &ab](const Vec3& p) { return ab.cross(p - a).length(); });
    const Vec3 normal = ab.cross(c - a);
    if (!(normal.length() > tolerance * ab.length())) return false;
    const Vec3 n = normal * (1.0 / normal.length());
    return std::all_of(points.begin(), points.end(), [&a, &n, tolerance](const Vec3& p) {
        return std::abs((p - a).dot(n)) <= tolerance;
    });
}

/// The chords a curve from @p t0 to @p t1 is cut into at equal steps of its parameter: as many
/// as keep each within 2 pi / segmentsPerTurn of turning (its whole turning shared among them),
/// and with a chord tolerance as many more as keep each sagging no more than it.
int stepsAlong(const std::function<Vec3(double)>& at, double t0, double t1,
               const FacetResolution& resolution) {
    constexpr int kDense = 512;
    std::vector<Vec3> dense(kDense + 1);
    for (int i = 0; i <= kDense; ++i) {
        dense[static_cast<std::size_t>(i)] = at(t0 + (t1 - t0) * i / kDense);
    }
    const double maxAngle =
        math::kTwoPi / static_cast<double>(std::max(resolution.segmentsPerTurn, 3));
    const double turn = turningOf(dense, roundingChord(dense));
    const int byTurning =
        std::clamp(static_cast<int>(std::ceil(turn / maxAngle - 1e-9)), 1, kMostSteps);
    const double tolerance = resolution.chordTolerance;
    if (!(tolerance > 0.0)) return byTurning;
    // Whether cut into @p n, some chord sags more than the tolerance (each measured at points
    // between its ends).
    const auto sags = [&](int n) {
        constexpr int kWithin = 8;
        for (int k = 0; k < n; ++k) {
            const double a = t0 + (t1 - t0) * k / n;
            const double b = t0 + (t1 - t0) * (k + 1) / n;
            const Vec3 p = at(a);
            const Vec3 chord = at(b) - p;
            const double length2 = chord.dot(chord);
            for (int m = 1; m < kWithin; ++m) {
                const Vec3 q = at(a + (b - a) * m / kWithin);
                const double t =
                    length2 > 0.0 ? std::clamp((q - p).dot(chord) / length2, 0.0, 1.0) : 0.0;
                if ((q - (p + chord * t)).length() > tolerance) return true;
            }
        }
        return false;
    };
    if (!sags(byTurning)) return byTurning;
    int sagging = byTurning;  // the most known to sag
    int enough = byTurning;   // the least known not to, once found
    do {
        sagging = enough;
        enough = std::min(kMostSteps, 2 * enough);
    } while (enough < kMostSteps && sags(enough));
    if (enough == kMostSteps && sags(enough)) return kMostSteps;
    while (enough - sagging > 1) {
        const int middle = sagging + (enough - sagging) / 2;
        if (sags(middle)) {
            sagging = middle;
        } else {
            enough = middle;
        }
    }
    return enough;
}

/// The chords @p e is cut into by its exact curve alone: a circle or an arc by stepsFor at equal
/// angles, a line in one, an ellipse or a NURBS curve by stepsAlong. With no exact curve it is
/// one chord, straight.
int edgeSteps(const Edge& e, const FacetResolution& resolution) {
    if (!e.exactCurve) return 1;
    const double sweep = std::abs(e.curveEnd - e.curveStart);
    switch (e.exactCurve->kind()) {
        case geo::CurveKind::Line:
            return 1;
        case geo::CurveKind::Circle:
            if (const auto* circle = dynamic_cast<const geo::CircleCurve*>(e.exactCurve.get())) {
                return std::min(kMostSteps, resolution.stepsFor(circle->radius(), sweep));
            }
            break;
        default:
            break;
    }
    const geo::Curve3& curve = *e.exactCurve;
    return stepsAlong([&curve](double t) { return curve.evaluate(t); }, e.curveStart, e.curveEnd,
                      resolution);
}

/// The chords across @p surface's own iso-curves along u (@p dir 0) or v (1) over the box
/// [u0, u1] x [v0, v1], where no edge runs that way (a sphere's u, between its poles): its
/// widest circle's stepsFor on a surface turned about an axis, a straight generator's one; the
/// most of stepsAlong over iso-curves across it otherwise.
int ownSteps(const geo::Surface& surface, int dir, double u0, double u1, double v0, double v1,
             const FacetResolution& resolution) {
    const double sweep = dir == 0 ? u1 - u0 : v1 - v0;
    // The most cos(v) is over [v0, v1]: 1 where a whole turn's multiple is within it.
    const auto widestCos = [v0, v1] {
        const double k = std::ceil(v0 / math::kTwoPi);
        if (k * math::kTwoPi <= v1) return 1.0;
        return std::max(std::cos(v0), std::cos(v1));
    };
    switch (surface.kind()) {
        case geo::SurfaceKind::Cylinder:
            if (const auto* s = dynamic_cast<const geo::Cylinder*>(&surface)) {
                return dir == 0 ? resolution.stepsFor(s->radius(), sweep) : 1;
            }
            break;
        case geo::SurfaceKind::Cone:
            if (const auto* s = dynamic_cast<const geo::Cone*>(&surface)) {
                if (dir == 1) return 1;
                const auto radius = [s](double v) {
                    return std::abs(s->radius() + s->axialScale() * v * std::tan(s->halfAngle()));
                };
                return resolution.stepsFor(std::max(radius(v0), radius(v1)), sweep);
            }
            break;
        case geo::SurfaceKind::Sphere:
            if (const auto* s = dynamic_cast<const geo::Sphere*>(&surface)) {
                return resolution.stepsFor(dir == 0 ? s->radius() * widestCos() : s->radius(),
                                           sweep);
            }
            break;
        case geo::SurfaceKind::Torus:
            if (const auto* s = dynamic_cast<const geo::Torus*>(&surface)) {
                return resolution.stepsFor(
                    dir == 0 ? s->majorRadius() + s->minorRadius() * widestCos() : s->minorRadius(),
                    sweep);
            }
            break;
        default:
            break;
    }
    constexpr int kAcross = 5;
    int most = 1;
    for (int i = 0; i < kAcross; ++i) {
        const double f = static_cast<double>(i) / (kAcross - 1);
        const double other = dir == 0 ? v0 + (v1 - v0) * f : u0 + (u1 - u0) * f;
        const auto at = [&surface, dir, other](double t) {
            return dir == 0 ? surface.evaluate(t, other) : surface.evaluate(other, t);
        };
        most = std::max(most, dir == 0 ? stepsAlong(at, u0, u1, resolution)
                                       : stepsAlong(at, v0, v1, resolution));
    }
    return most;
}

/// The sides of a face's rectangle in (u, v), counter-clockwise from its corner (u0, v0).
enum Side : std::size_t { kBottom = 0, kRight = 1, kTop = 2, kLeft = 3 };

/// A curved face as a rectangle of its surface's (u, v): on each side the half-edges along it, or
/// none where the surface is one point all along it (a pole, an apex), which the loop runs along
/// from one edge to the next.
struct ExactRectangle {
    double u0 = 0.0, u1 = 0.0, v0 = 0.0, v1 = 0.0;
    std::array<std::vector<const HalfEdge*>, 4> sides;
    std::array<bool, 4> pole{};
    std::array<Vec3, 4> polePoint;  ///< a pole's side: the vertex the loop runs along it at
    /// Each half-edge's start and end in (u, v), as its loop is unwrapped.
    std::unordered_map<const HalfEdge*, std::pair<math::Vec2, math::Vec2>> ends;
};

/// @p face's outer loop as a rectangle of its exact surface's (u, v), unwrapped as the exact
/// validator unwraps it (topo::unwrapLoop, with the face's sense); nullopt, with @p why, where it
/// has holes, a half-edge has no pcurve, or its outline is no such rectangle.
std::optional<ExactRectangle> rectangleOf(const Face& face, const math::ModelTolerance& tolerance,
                                          std::size_t limit, std::string& why) {
    const geo::Surface& s = *face.exactSurface;
    if (!face.innerLoops.empty()) {
        why = "it has holes, which the bridge cannot cut on a curved surface yet";
        return std::nullopt;
    }
    std::vector<const HalfEdge*> ring;
    if (face.outerLoop == nullptr || !topo::loopHalfEdges(face, *face.outerLoop, limit, ring) ||
        ring.empty()) {
        why = "its loop does not close";
        return std::nullopt;
    }
    std::vector<double> allowances;
    allowances.reserve(ring.size());
    for (const HalfEdge* h : ring) {
        if (!h->pcurve || h->edge == nullptr || h->next == nullptr || h->origin == nullptr) {
            why = "a half-edge of it has no pcurve";
            return std::nullopt;
        }
        allowances.push_back(topo::joinAllowance(*h, tolerance));
    }
    topo::LoopPolygon poly;
    if (!topo::unwrapLoop(ring, topo::kLoopSamples, s, topo::uPeriod(s), topo::vPeriod(s),
                          allowances, poly, topo::loopSense(face, true)) ||
        poly.empty()) {
        why = "its loop winds round its surface";
        return std::nullopt;
    }
    ExactRectangle r;
    r.u0 = r.v0 = std::numeric_limits<double>::infinity();
    r.u1 = r.v1 = -std::numeric_limits<double>::infinity();
    for (const auto& seg : poly) {
        for (const math::Vec2& p : {seg.a, seg.b}) {
            r.u0 = std::min(r.u0, p.x);
            r.u1 = std::max(r.u1, p.x);
            r.v0 = std::min(r.v0, p.y);
            r.v1 = std::max(r.v1, p.y);
        }
    }
    const double gu = kOnSide * std::max({1.0, std::abs(r.u0), std::abs(r.u1)});
    const double gv = kOnSide * std::max({1.0, std::abs(r.v0), std::abs(r.v1)});
    const std::string notRectangle = "its outline is not a rectangle of its surface's (u, v)";
    if (!(r.u1 - r.u0 > gu) || !(r.v1 - r.v0 > gv)) {
        why = notRectangle;
        return std::nullopt;
    }
    // The side all of a run of points lies on, or -1.
    const auto sideOf = [&r, gu, gv](const std::vector<math::Vec2>& points) {
        const auto all = [&points](const std::function<bool(const math::Vec2&)>& on) {
            return std::all_of(points.begin(), points.end(), on);
        };
        if (all([&](const math::Vec2& p) { return std::abs(p.y - r.v0) <= gv; })) return 0;
        if (all([&](const math::Vec2& p) { return std::abs(p.x - r.u1) <= gu; })) return 1;
        if (all([&](const math::Vec2& p) { return std::abs(p.y - r.v1) <= gv; })) return 2;
        if (all([&](const math::Vec2& p) { return std::abs(p.x - r.u0) <= gu; })) return 3;
        return -1;
    };
    const std::size_t per = static_cast<std::size_t>(topo::kLoopSamples - 1);
    std::array<bool, 4> covered{};
    for (std::size_t k = 0; k < ring.size(); ++k) {
        std::vector<math::Vec2> points;
        points.reserve(per + 1);
        for (std::size_t i = 0; i < per; ++i) points.push_back(poly[k * per + i].a);
        points.push_back(poly[k * per + per - 1].b);
        const int side = sideOf(points);
        if (side < 0) {
            why = notRectangle;
            return std::nullopt;
        }
        r.sides[static_cast<std::size_t>(side)].push_back(ring[k]);
        covered[static_cast<std::size_t>(side)] = true;
        r.ends[ring[k]] = {points.front(), points.back()};
        // The way on to the next half-edge: none, or along a pole or an apex.
        const math::Vec2 end = points.back();
        const math::Vec2 next = poly[((k + 1) % ring.size()) * per].a;
        if (std::abs(end.x - next.x) <= gu && std::abs(end.y - next.y) <= gv) continue;
        const int along = sideOf({end, next});
        if (along < 0 || !topo::runsAlongPoint(s, end, next, allowances[k])) {
            why = notRectangle;
            return std::nullopt;
        }
        r.pole[static_cast<std::size_t>(along)] = true;
        r.polePoint[static_cast<std::size_t>(along)] = ring[(k + 1) % ring.size()]->origin->point;
    }
    for (std::size_t side = 0; side < 4; ++side) {
        if (covered[side] == r.pole[side]) {
            why = notRectangle;
            return std::nullopt;
        }
    }
    return r;
}

/// A point of a side of a face's rectangle, and where it is in (u, v).
struct SidePoint {
    Vec3 point;
    math::Vec2 uv;
};

}  // namespace

bool describedByCurves(const topo::Solid& solid) {
    return std::any_of(solid.edges().begin(), solid.edges().end(),
                       [](const Edge& e) { return e.curve && e.curve->degree() > 1; });
}

FacetedSolid facetCurved(const topo::Solid& exact, double maxAngle) {
    FacetedSolid result;
    // A face with no carrier is cut below as flat, its outline: where it is not, there is
    // nothing to cut it on, and its corners (two, on an exact cylinder's side) are no outline of
    // it. Its exact surface is facetExact's to cut.
    double flatness = -1.0;  // the model's resolution, once a face needs it
    for (const auto& face : exact.faces()) {
        if (!face.alive || face.surface != nullptr) continue;
        if (face.exactSurface) {
            if (face.exactSurface->kind() == geo::SurfaceKind::Plane) continue;
            result.error = "face " + face.topoId.tag() + " has no NURBS carrier to facet on";
            return result;
        }
        if (flatness < 0.0) flatness = topo::exactTolerance(exact).linear;
        if (!flatOutline(face, flatness)) {
            result.error = "face " + face.topoId.tag() + " has no surface to facet";
            return result;
        }
    }
    maxAngle = std::clamp(maxAngle, 1e-3, math::kPi / 2);
    double size = 0.0;
    for (const auto& v : exact.vertices()) {
        size = std::max(size, (v.point - exact.vertices().front().point).length());
    }

    // A loop with two corners or fewer (a blade's rim: two gentle curves
    // between the same two corners) is no polygon if its edges are a chord
    // each, the same chord: its curves are two chords or more.
    std::unordered_set<const Edge*> twoCorners;
    for (const auto& face : exact.faces()) {
        std::vector<const Wire*> loops{face.outerLoop};
        loops.insert(loops.end(), face.innerLoops.begin(), face.innerLoops.end());
        for (const Wire* wire : loops) {
            const auto loop = loopOf(wire);
            std::set<const topo::Vertex*> corners;
            for (const HalfEdge* he : loop) corners.insert(he->origin);
            if (corners.size() > 2) continue;
            for (const HalfEdge* he : loop) twoCorners.insert(he->edge);
        }
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
        auto points = sampleEdge(e, maxAngle, twoCorners.count(&e) != 0 ? 2 : 1);
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
            // A band round the surface between two loops that wind round it,
            // with no seam: cut along a seam of its own.
            std::vector<UVChain> chains;
            if (auto chain = chainInUV(edges, map)) chains.push_back(std::move(*chain));
            for (const auto& loop : inner) {
                if (auto chain = chainInUV(edgesOf(loop), map)) chains.push_back(std::move(*chain));
            }
            if (chains.size() == inner.size() + 1) {
                if (auto band = bandInUV(std::move(chains), map, maxAngle)) {
                    if (auto facets = trimmedFacets(std::move(band->outline),
                                                    std::move(band->holes), map, maxAngle)) {
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
        if (!curved) {
            if (auto facets = offPlaneFacets(f.points, f.holes, size)) {
                for (std::size_t k = 0; k < facets->size(); ++k) {
                    SolidSewer::InputFace facet;
                    facet.points = std::move((*facets)[k]);
                    facet.topoId = topo::TopologyID::fromTag(face.topoId.tag() +
                                                             "/facet:" + std::to_string(k));
                    faces.push_back(std::move(facet));
                }
                continue;
            }
        }
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

int FacetResolution::stepsFor(double radius, double sweep) const {
    const int perTurn = chordTolerance > 0.0
                            ? PrimitiveFactory::segmentsForTolerance(radius, chordTolerance)
                            : segmentsPerTurn;
    const double steps = sweep / math::kTwoPi * static_cast<double>(std::max(perTurn, 3));
    return std::max(1, static_cast<int>(std::ceil(steps - 1e-9)));
}

FacetedSolid facetExact(const topo::Solid& exact, const FacetResolution& resolution) {
    FacetedSolid result;
    const math::ModelTolerance tolerance = topo::exactTolerance(exact);
    const double size = topo::exactExtent(exact);
    const std::size_t limit = 2 * exact.edges().size() + 1;
    const auto refuse = [](const Face& face, const std::string& why) {
        FacetedSolid refused;
        refused.error = "face " + face.topoId.tag() + " " + why;
        return refused;
    };

    // Each face as it is cut: flat (one facet, its holes), or a curved face's rectangle.
    struct Plan {
        const Face* face = nullptr;
        std::optional<ExactRectangle> rectangle;  ///< a curved face's
        std::array<int, 2> steps{};               ///< its columns (along u) and rows (along v)
        std::array<int, 2> own{};  ///< where both sides across are poles: its surface's own
    };
    std::vector<Plan> plans;
    for (const auto& face : exact.faces()) {
        if (!face.alive) continue;
        if (face.outerLoop == nullptr || loopOf(face.outerLoop).empty()) {
            return refuse(face, "has no boundary");
        }
        Plan plan;
        plan.face = &face;
        if (face.exactSurface) {
            if (face.exactSurface->kind() != geo::SurfaceKind::Plane) {
                std::string why;
                auto rectangle = rectangleOf(face, tolerance, limit, why);
                if (!rectangle) return refuse(face, "cannot be cut into facets: " + why);
                plan.rectangle = std::move(rectangle);
            }
        } else if (face.surface) {
            if (!isFlat(*face.surface)) return refuse(face, "has no exact surface to facet");
        } else if (!flatOutline(face, tolerance.linear)) {
            return refuse(face, "has no surface to facet");
        }
        plans.push_back(std::move(plan));
    }

    // How many chords each edge is cut into: by its own curve, and then as many as the curved
    // faces it is a side of need (each pair of opposite sides alike).
    std::unordered_map<const Edge*, int> steps;
    for (const auto& e : exact.edges()) {
        if (!e.alive || e.halfEdge == nullptr || e.halfEdge->origin == nullptr) continue;
        int n = edgeSteps(e, resolution);
        const bool curved = e.exactCurve && e.exactCurve->kind() != geo::CurveKind::Line;
        const HalfEdge* twin = e.halfEdge->twin;
        // A closed curve is no chord: three or more.
        if (curved && twin != nullptr && twin->origin == e.halfEdge->origin) n = std::max(n, 3);
        steps[&e] = n;
    }
    // A loop with two corners or fewer (a lens: two arcs between the same two corners) is no
    // polygon if its curves are a chord each, the same chord: two chords or more.
    for (const Plan& plan : plans) {
        std::vector<const Wire*> loops{plan.face->outerLoop};
        loops.insert(loops.end(), plan.face->innerLoops.begin(), plan.face->innerLoops.end());
        for (const Wire* wire : loops) {
            const auto loop = loopOf(wire);
            std::set<const topo::Vertex*> corners;
            for (const HalfEdge* he : loop) corners.insert(he->origin);
            if (corners.size() > 2) continue;
            for (const HalfEdge* he : loop) {
                const Edge& e = *he->edge;
                const auto found = steps.find(&e);
                if (found != steps.end() && e.exactCurve &&
                    e.exactCurve->kind() != geo::CurveKind::Line) {
                    found->second = std::max(found->second, 2);
                }
            }
        }
    }
    for (Plan& plan : plans) {
        const auto& rectangle = plan.rectangle;
        if (!rectangle) continue;
        const ExactRectangle& r = *rectangle;
        const geo::Surface& s = *plan.face->exactSurface;
        if (r.pole[kBottom] && r.pole[kTop]) {
            plan.own[0] = ownSteps(s, 0, r.u0, r.u1, r.v0, r.v1, resolution);
        }
        if (r.pole[kLeft] && r.pole[kRight]) {
            plan.own[1] = ownSteps(s, 1, r.u0, r.u1, r.v0, r.v1, resolution);
        }
    }
    const auto chordsOn = [&steps](const std::vector<const HalfEdge*>& side) {
        int total = 0;
        for (const HalfEdge* h : side) {
            const auto found = steps.find(h->edge);
            total += found == steps.end() ? 1 : found->second;
        }
        return total;
    };
    // Raised until nothing changes; a side of several edges is not cut again, and where it then
    // disagrees with the side across, the face cannot be cut.
    for (std::size_t pass = 0; pass <= exact.edges().size(); ++pass) {
        bool changed = false;
        for (Plan& plan : plans) {
            const auto& rectangle = plan.rectangle;
            if (!rectangle) continue;
            const ExactRectangle& r = *rectangle;
            for (int dir = 0; dir < 2; ++dir) {
                const std::array<Side, 2> across = dir == 0 ? std::array<Side, 2>{kBottom, kTop}
                                                            : std::array<Side, 2>{kLeft, kRight};
                int n = plan.own[static_cast<std::size_t>(dir)];
                for (const Side side : across) {
                    if (!r.pole[side]) n = std::max(n, chordsOn(r.sides[side]));
                }
                n = std::min(n, kMostSteps);
                plan.steps[static_cast<std::size_t>(dir)] = n;
                for (const Side side : across) {
                    if (r.pole[side] || r.sides[side].size() != 1) continue;
                    int& count = steps[r.sides[side].front()->edge];
                    if (count < n) {
                        count = n;
                        changed = true;
                    }
                }
            }
        }
        if (!changed) break;
    }
    for (const Plan& plan : plans) {
        const auto& rectangle = plan.rectangle;
        if (!rectangle) continue;
        const ExactRectangle& r = *rectangle;
        for (const Side side : {kBottom, kRight, kTop, kLeft}) {
            const int dir = side == kBottom || side == kTop ? 0 : 1;
            if (!r.pole[side] &&
                chordsOn(r.sides[side]) != plan.steps[static_cast<std::size_t>(dir)]) {
                return refuse(*plan.face,
                              "cannot be cut into facets: its opposite sides are cut into "
                              "different numbers of chords");
            }
        }
    }

    // Each edge's points once, so the faces either side share them, from its half-edge's origin
    // to its end, at equal steps of its exact curve's parameter; each chord's edge, by its ends.
    struct ChordOf {
        const Edge* edge = nullptr;
        int k = 0;
        int of = 0;
    };
    std::unordered_map<const Edge*, std::vector<Vec3>> samples;
    std::map<std::pair<Key, Key>, ChordOf> chordOf;
    for (const auto& [edge, n] : steps) {
        const HalfEdge* he = edge->halfEdge;
        const topo::Vertex* end = he->twin != nullptr   ? he->twin->origin
                                  : he->next != nullptr ? he->next->origin
                                                        : nullptr;
        if (end == nullptr) continue;
        std::vector<Vec3> points{he->origin->point};
        for (int k = 1; k < n && edge->exactCurve; ++k) {
            const double t = edge->curveStart + (edge->curveEnd - edge->curveStart) *
                                                    static_cast<double>(k) / static_cast<double>(n);
            points.push_back(edge->exactCurve->evaluate(t));
        }
        points.push_back(end->point);
        for (std::size_t k = 0; k + 1 < points.size(); ++k) {
            auto a = keyOf(points[k]);
            auto b = keyOf(points[k + 1]);
            if (b < a) std::swap(a, b);
            chordOf[{a, b}] = {edge, static_cast<int>(k), static_cast<int>(points.size()) - 1};
        }
        samples.emplace(edge, std::move(points));
    }
    const auto along = [&samples](const HalfEdge* he) {
        const auto found = samples.find(he->edge);
        if (found == samples.end()) {
            return std::vector<Vec3>{he->origin->point, he->next->origin->point};
        }
        std::vector<Vec3> s = found->second;
        if (he != he->edge->halfEdge) std::reverse(s.begin(), s.end());
        return s;
    };
    const auto polygon = [&along](const Wire* wire) {
        std::vector<Vec3> points;
        for (const HalfEdge* he : loopOf(wire)) {
            const auto s = along(he);
            points.insert(points.end(), s.begin(), s.end() - 1);
        }
        return points;
    };

    std::vector<SolidSewer::InputFace> faces;
    for (const Plan& plan : plans) {
        const Face& face = *plan.face;
        const auto& rectangle = plan.rectangle;
        if (!rectangle) {
            SolidSewer::InputFace f;
            f.points = polygon(face.outerLoop);
            for (const Wire* w : face.innerLoops) f.holes.push_back(polygon(w));
            f.topoId = face.topoId;
            faces.push_back(std::move(f));
            continue;
        }
        const ExactRectangle& r = *rectangle;
        const geo::Surface& s = *face.exactSurface;
        const auto nu = static_cast<std::size_t>(plan.steps[0]);
        const auto nv = static_cast<std::size_t>(plan.steps[1]);
        // A side's points in order along it, each where it is in (u, v): its half-edges' points,
        // each found on the surface (its coordinate along the side; the other is the side's own),
        // a period round to lie between its half-edge's ends.
        const auto pointsOn = [&](Side side) {
            const int running = side == kBottom || side == kTop ? 0 : 1;
            const double period = running == 0 ? topo::uPeriod(s) : topo::vPeriod(s);
            const double line = side == kBottom ? r.v0
                                : side == kTop  ? r.v1
                                : side == kLeft ? r.u0
                                                : r.u1;
            std::vector<SidePoint> out;
            for (const HalfEdge* h : r.sides[side]) {
                const auto points = along(h);
                const auto& ends = r.ends.at(h);
                const math::Vec2 start = ends.first;
                const math::Vec2 finish = ends.second;
                const std::size_t n = points.size() - 1;
                for (std::size_t k = 0; k <= n; ++k) {
                    math::Vec2 uv = k == 0 ? start : finish;
                    if (k != 0 && k != n) {
                        const double f = static_cast<double>(k) / static_cast<double>(n);
                        const math::Vec2 between = start + (finish - start) * f;
                        const math::Vec2 found = s.invert(points[k]);
                        double c = running == 0 ? found.x : found.y;
                        const double want = running == 0 ? between.x : between.y;
                        if (period > 0.0) c += period * std::round((want - c) / period);
                        uv = running == 0 ? math::Vec2(c, line) : math::Vec2(line, c);
                    }
                    out.push_back({points[k], uv});
                }
            }
            const auto coordinate = [running](const SidePoint& p) {
                return running == 0 ? p.uv.x : p.uv.y;
            };
            std::stable_sort(out.begin(), out.end(),
                             [&coordinate](const SidePoint& a, const SidePoint& b) {
                                 return coordinate(a) < coordinate(b);
                             });
            // Where one half-edge ends the next begins (their shared vertex): that point once.
            out.erase(std::unique(out.begin(), out.end(),
                                  [](const SidePoint& a, const SidePoint& b) {
                                      return (a.point - b.point).length() <= 0.0;
                                  }),
                      out.end());
            return out;
        };
        std::array<std::vector<SidePoint>, 4> sides;
        for (const Side side : {kBottom, kRight, kTop, kLeft}) {
            if (r.pole[side]) continue;
            sides[side] = pointsOn(side);
            const std::size_t want = side == kBottom || side == kTop ? nu : nv;
            if (sides[side].size() != want + 1) {
                return refuse(face,
                              "cannot be cut into facets: a side of it does not run "
                              "corner to corner");
            }
        }
        // A pole's side: its one point, spread over its coordinate in (u, v).
        const auto poleSide = [&r](Side side, std::size_t n) {
            const bool alongU = side == kBottom || side == kTop;
            const Vec3 point = r.polePoint[side];
            std::vector<SidePoint> out;
            for (std::size_t i = 0; i <= n; ++i) {
                const double f = static_cast<double>(i) / static_cast<double>(n);
                out.push_back(
                    {point,
                     alongU ? math::Vec2(r.u0 + (r.u1 - r.u0) * f, side == kBottom ? r.v0 : r.v1)
                            : math::Vec2(side == kLeft ? r.u0 : r.u1, r.v0 + (r.v1 - r.v0) * f)});
            }
            return out;
        };
        for (const Side side : {kBottom, kRight, kTop, kLeft}) {
            if (r.pole[side]) {
                sides[side] = poleSide(side, side == kBottom || side == kTop ? nu : nv);
            }
        }
        const auto& B = sides[kBottom];
        const auto& T = sides[kTop];
        const auto& L = sides[kLeft];
        const auto& R = sides[kRight];
        // A Coons patch in (u, v) from its sides, each point inside on the surface: uniform in
        // (u, v) where its sides are.
        std::vector<std::vector<Vec3>> grid(nu + 1, std::vector<Vec3>(nv + 1));
        for (std::size_t i = 0; i <= nu; ++i) {
            for (std::size_t j = 0; j <= nv; ++j) {
                if (j == 0) {
                    grid[i][j] = B[i].point;
                } else if (j == nv) {
                    grid[i][j] = T[i].point;
                } else if (i == 0) {
                    grid[i][j] = L[j].point;
                } else if (i == nu) {
                    grid[i][j] = R[j].point;
                } else {
                    const double a = static_cast<double>(i) / static_cast<double>(nu);
                    const double b = static_cast<double>(j) / static_cast<double>(nv);
                    const math::Vec2 uv =
                        B[i].uv * (1 - b) + T[i].uv * b + L[j].uv * (1 - a) + R[j].uv * a -
                        (B[0].uv * ((1 - a) * (1 - b)) + B[nu].uv * (a * (1 - b)) +
                         T[0].uv * ((1 - a) * b) + T[nu].uv * (a * b));
                    grid[i][j] = s.evaluate(uv.x, uv.y);
                }
            }
        }
        std::shared_ptr<geo::NurbsSurface> ideal;
        try {
            ideal = s.toNurbs(r.u0, r.u1, r.v0, r.v1);
        } catch (const std::exception& e) {
            return refuse(face, std::string("has no NURBS form over its rectangle: ") + e.what());
        }
        // Row by row from (u0, v0), counter-clockwise in (u, v): about du x dv, outward unless
        // the face is reversed.
        std::vector<std::vector<Vec3>> facets;
        for (std::size_t j = 0; j < nv; ++j) {
            for (std::size_t i = 0; i < nu; ++i) {
                std::vector<Vec3> quad;
                for (const Vec3& p :
                     {grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1]}) {
                    if (quad.empty() || (p - quad.back()).length() > 0.0) quad.push_back(p);
                }
                while (quad.size() > 1 && (quad.front() - quad.back()).length() <= 0.0) {
                    quad.pop_back();
                }
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
        for (std::size_t k = 0; k < facets.size(); ++k) {
            SolidSewer::InputFace f;
            f.points = std::move(facets[k]);
            if (face.reversed) std::reverse(f.points.begin(), f.points.end());
            f.topoId =
                facets.size() == 1
                    ? face.topoId
                    : topo::TopologyID::fromTag(face.topoId.tag() + "/facet:" + std::to_string(k));
            f.analyticSurface = ideal;
            faces.push_back(std::move(f));
        }
    }

    auto solid = SolidSewer::sew(faces);
    if (solid == nullptr || !solid->checkManifold()) {
        result.error = "the facets did not join into a closed solid";
        return result;
    }
    // Each chord named for its edge, and recording its curve, one NURBS for all of them.
    std::unordered_map<const Edge*, std::shared_ptr<geo::NurbsCurve>> ideals;
    for (auto& e : solid->edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->next == nullptr) continue;
        auto a = keyOf(e.halfEdge->origin->point);
        auto b = keyOf(e.halfEdge->next->origin->point);
        if (b < a) std::swap(a, b);
        const auto found = chordOf.find({a, b});
        if (found == chordOf.end()) continue;
        const ChordOf& chord = found->second;
        const Edge& source = *chord.edge;
        if (source.topoId.isValid()) {
            e.topoId = chord.of == 1
                           ? source.topoId
                           : topo::TopologyID::fromTag(source.topoId.tag() +
                                                       "/chord:" + std::to_string(chord.k));
        }
        if (!source.exactCurve || source.exactCurve->kind() == geo::CurveKind::Line) continue;
        auto& ideal = ideals[&source];
        if (!ideal) {
            try {
                ideal = source.exactCurve->toNurbs(std::min(source.curveStart, source.curveEnd),
                                                   std::max(source.curveStart, source.curveEnd));
            } catch (const std::exception& error) {
                result.error = "edge " + source.topoId.tag() +
                               " has no NURBS form over its range: " + error.what();
                return result;
            }
        }
        e.analyticCurve = ideal;
    }
    result.solid = std::move(solid);
    return result;
}

std::vector<ShellMeasure> measureShells(const topo::Solid& solid) {
    const auto corners = [](const topo::Face& face, math::BoundingBox& box) {
        std::vector<const Wire*> loops{face.outerLoop};
        loops.insert(loops.end(), face.innerLoops.begin(), face.innerLoops.end());
        for (const Wire* wire : loops) {
            if (wire == nullptr || wire->halfEdge == nullptr) continue;
            const HalfEdge* he = wire->halfEdge;
            do {
                if (he->origin != nullptr) box.expand(he->origin->point);
                he = he->next;
            } while (he != nullptr && he != wire->halfEdge);
        }
    };
    std::vector<ShellMeasure> measures;
    std::unordered_map<std::string, std::size_t> shellOfFace;
    for (const auto& shell : solid.shells()) {
        ShellMeasure measure;
        if (!shell.faces.empty()) {
            measure.volume = topo::signedVolume(shell);
            for (const Face* face : shell.faces) {
                corners(*face, measure.box);
                shellOfFace[face->topoId.tag()] = measures.size();
            }
        }
        measures.push_back(measure);
    }
    if (!describedByCurves(solid)) return measures;
    const auto cut = facetCurved(solid);
    if (!cut.solid) return measures;  // measured by its corners, as well as can be
    // Each facet is named for its face, `<face>` or `<face>/facet:<k>`.
    std::vector<bool> measured(measures.size(), false);
    for (const auto& shell : cut.solid->shells()) {
        if (shell.faces.empty()) continue;
        std::string tag = shell.faces.front()->topoId.tag();
        if (const auto facet = tag.rfind("/facet:"); facet != std::string::npos) tag.erase(facet);
        const auto it = shellOfFace.find(tag);
        if (it == shellOfFace.end()) continue;
        ShellMeasure& measure = measures[it->second];
        if (!measured[it->second]) {
            measured[it->second] = true;
            measure = ShellMeasure{};
        }
        measure.volume += topo::signedVolume(shell);
        for (const Face* face : shell.faces) corners(*face, measure.box);
    }
    return measures;
}

}  // namespace hz::model
