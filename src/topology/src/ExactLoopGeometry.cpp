#include "ExactLoopGeometry.h"

#include <algorithm>
#include <vector>

#include "horizon/math/RobustPredicates.h"

namespace hz::topo::detail {

namespace {

using math::Vec2;

constexpr int kMaxRefinement = 30;

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
/// curves cannot pass between them; until then bisect and look again. Depth exhausted counts as
/// real. Blind spot: a crossing is missed whenever one curve's sample point lies exactly on the
/// other's straight chord (the touching pairs are not proper crossings), as at a bisection
/// point; that needs the curves to meet at a sampled parameter, which sagging chords do not do.
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

double signedArea(const LoopPolygon& loop) {
    if (loop.empty()) return 0.0;
    // About the first point: far from the origin the products would lose their precision.
    const Vec2 o = loop.front().a;
    double twice = 0.0;
    for (size_t i = 0; i < loop.size(); ++i) {
        const Vec2 p = loop[i].a - o;
        const Vec2 q = loop[i].b - o;
        twice += p.x * q.y - q.x * p.y;
        // The jump from one pcurve's end to the next one's start: nothing but rounding on most
        // surfaces, a run along the pole or apex on others.
        const Vec2 r = loop[(i + 1) % loop.size()].a - o;
        twice += q.x * r.y - r.x * q.y;
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
    for (const auto& s : outer) poly.push_back(s.a);
    if (math::classifyPoint(poly, point) != math::Containment::Outside) return false;
    // Each chord has its own band: the curve it stands for may lie that far (its sag) from it.
    // A point within a chord's band might be inside the true curve; one margin for the whole
    // loop would let a big arc hide holes beside every other side.
    for (const auto& s : outer) {
        if (distanceToSegment(point, s.a, s.b) <= sag(s)) return false;
    }
    return true;
}

}  // namespace hz::topo::detail
