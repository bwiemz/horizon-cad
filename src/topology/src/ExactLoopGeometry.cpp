#include "ExactLoopGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "horizon/math/RobustPredicates.h"

namespace hz::topo::detail {

namespace {

using math::Vec2;

constexpr int kMaxRefinement = 30;

Vec2 sampleAt(const HalfEdge& h, int i, int samples) {
    const double t = h.pcurveStart + (h.pcurveEnd - h.pcurveStart) * i / (samples - 1);
    return h.pcurve->evaluate(t);
}

/// How far the curve strays from the chord at the chord's parameter midpoint.
double sag(const LoopSegment& s) {
    const Vec2 m = s.curve->evaluate(0.5 * (s.t0 + s.t1)) + s.shift;
    return (m - (s.a + s.b) * 0.5).length();
}

bool properlyCross(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    return math::orient2d(a, b, c) * math::orient2d(a, b, d) < 0 &&
           math::orient2d(c, d, a) * math::orient2d(c, d, b) < 0;
}

bool chordsCross(const LoopSegment& s, const LoopSegment& t) {
    return properlyCross(s.a, s.b, t.a, t.b);
}

void bisect(const LoopSegment& s, LoopSegment halves[2]) {
    const double tm = 0.5 * (s.t0 + s.t1);
    const Vec2 m = s.curve->evaluate(tm) + s.shift;
    halves[0] = {s.curve, s.t0, tm, s.shift, s.a, m};
    halves[1] = {s.curve, tm, s.t1, s.shift, m, s.b};
}

/// Chords that cross are a real crossing only once both are so close to their curves that the
/// curves cannot pass between them; until then bisect and look again. A crossing exactly at a
/// bisection point is two touching pairs and is missed; that needs the curves to meet at a
/// sampled parameter and is not what sagging chords produce. Depth exhausted counts as real.
bool crossesForReal(const LoopSegment& s, const LoopSegment& t, const UvTolerance& tol, int depth) {
    if (!chordsCross(s, t)) return false;
    if (depth >= kMaxRefinement) return true;
    if (sag(s) <= tol(s.a) && sag(t) <= tol(t.a)) return true;
    LoopSegment hs[2];
    LoopSegment ht[2];
    bisect(s, hs);
    bisect(t, ht);
    for (const auto& x : hs) {
        for (const auto& y : ht) {
            if (crossesForReal(x, y, tol, depth + 1)) return true;
        }
    }
    return false;
}

double distanceToSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const double len2 = ab.dot(ab);
    const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    return (p - (a + ab * t)).length();
}

}  // namespace

bool unwrapLoop(const std::vector<const HalfEdge*>& ring, int samples, double periodU,
                double periodV, LoopPolygon& out) {
    out.clear();
    Vec2 prevEnd;
    for (const HalfEdge* h : ring) {
        Vec2 shift;
        const Vec2 start = sampleAt(*h, 0, samples);
        if (!out.empty()) {
            if (periodU > 0.0) shift.x = std::round((prevEnd.x - start.x) / periodU) * periodU;
            if (periodV > 0.0) shift.y = std::round((prevEnd.y - start.y) / periodV) * periodV;
        }
        Vec2 a = start + shift;
        for (int i = 1; i < samples; ++i) {
            const double t0 =
                h->pcurveStart + (h->pcurveEnd - h->pcurveStart) * (i - 1) / (samples - 1);
            const double t1 = h->pcurveStart + (h->pcurveEnd - h->pcurveStart) * i / (samples - 1);
            const Vec2 b = h->pcurve->evaluate(t1) + shift;
            out.push_back({h->pcurve.get(), t0, t1, shift, a, b});
            a = b;
        }
        prevEnd = a;
    }
    if (out.empty()) return true;
    const Vec2 gap = out.front().a - prevEnd;
    const bool windsU = periodU > 0.0 && std::round(gap.x / periodU) != 0.0;
    const bool windsV = periodV > 0.0 && std::round(gap.y / periodV) != 0.0;
    return !windsU && !windsV;
}

double signedArea(const LoopPolygon& loop) {
    if (loop.empty()) return 0.0;
    // About the first point: far from the origin the products would lose their precision.
    const Vec2 o = loop.front().a;
    double twice = 0.0;
    for (const auto& s : loop) {
        const Vec2 p = s.a - o;
        const Vec2 q = s.b - o;
        twice += p.x * q.y - q.x * p.y;
    }
    return twice / 2.0;
}

bool selfIntersects(const LoopPolygon& loop, const UvTolerance& tolerance) {
    const size_t n = loop.size();
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 2; j < n; ++j) {
            if (i == 0 && j == n - 1) continue;  // adjacent across the closing chord
            if (crossesForReal(loop[i], loop[j], tolerance, 0)) return true;
        }
    }
    return false;
}

bool loopsCross(const LoopPolygon& p, const LoopPolygon& q, const UvTolerance& tolerance) {
    for (const auto& s : p) {
        for (const auto& t : q) {
            if (crossesForReal(s, t, tolerance, 0)) return true;
        }
    }
    return false;
}

bool clearlyOutside(const LoopPolygon& outer, const Vec2& point) {
    std::vector<Vec2> poly;
    poly.reserve(outer.size());
    double worstSag = 0.0;
    for (const auto& s : outer) {
        poly.push_back(s.a);
        worstSag = std::max(worstSag, sag(s));
    }
    if (math::classifyPoint(poly, point) != math::Containment::Outside) return false;
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto& s : outer) nearest = std::min(nearest, distanceToSegment(point, s.a, s.b));
    return nearest > worstSag;
}

}  // namespace hz::topo::detail
