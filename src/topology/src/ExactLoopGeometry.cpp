#include "ExactLoopGeometry.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
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

namespace {

/// True when @p point lies on the @p side of @p loop's chords and beyond every chord's band.
bool clearly(math::Containment side, const LoopPolygon& loop, const Vec2& point) {
    std::vector<Vec2> poly;
    poly.reserve(loop.size());
    for (const auto& s : loop) poly.push_back(s.a);
    if (math::classifyPoint(poly, point) != side) return false;
    // Each chord has its own band: the curve it stands for may lie that far (its sag) from it.
    // A point within a chord's band might be on the other side of the true curve; one margin for
    // the whole loop would let a big arc hide holes beside every other side.
    for (const auto& s : loop) {
        if (distanceToSegment(point, s.a, s.b) <= sag(s)) return false;
    }
    return true;
}

}  // namespace

bool clearlyOutside(const LoopPolygon& outer, const Vec2& point) {
    return clearly(math::Containment::Outside, outer, point);
}

bool clearlyInside(const LoopPolygon& loop, const Vec2& point) {
    return clearly(math::Containment::Inside, loop, point);
}

// ---------------------------------------------------------------------------------------------
// LoopIndex
// ---------------------------------------------------------------------------------------------

namespace {

bool finitePoint(const Vec2& p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

/// The most buckets an index uses, and how many registrations per item it allows before it
/// uses fewer (long items, such as an outer loop's long sides, span many).
constexpr int kMaxBuckets = 1024;
constexpr std::size_t kRegistrationsPerItem = 16;

/// What a point must be beyond a chord's box grown by its sag before its band is not looked at:
/// distanceToSegment rounds by a few units in the last place of the coordinates it is given,
/// and this is a thousand times more.
double bandMargin(const LoopSegment& s, double sagOfIt) {
    return 1e-12 * (1.0 + std::abs(s.a.x) + std::abs(s.a.y) + std::abs(s.b.x) + std::abs(s.b.y) +
                    std::abs(sagOfIt));
}

}  // namespace

int LoopIndex::Buckets::of(double y) const {
    const double f = std::floor((y - lo) * scale);
    // Monotone in y (a difference and a product by a positive constant never reorder), and
    // whatever it is past the ends, the end bucket: so a box spanning y is in y's bucket.
    if (!(f > 0.0)) return 0;
    if (!(f < static_cast<double>(count - 1))) return count - 1;
    return static_cast<int>(f);
}

void LoopIndex::Buckets::build(double ylo, double yhi, std::size_t items) {
    count = static_cast<int>(std::clamp<std::size_t>(items / 2, 1, kMaxBuckets));
    lo = ylo;
    scale = yhi > ylo ? count / (yhi - ylo) : 0.0;
    members.assign(static_cast<std::size_t>(count), {});
}

void LoopIndex::Buckets::add(int item, double ylo, double yhi) {
    const int last = of(yhi);
    for (int b = of(ylo); b <= last; ++b) members[static_cast<std::size_t>(b)].push_back(item);
}

LoopIndex::LoopIndex(const LoopPolygon& loop) : loop_(loop) {
    const std::size_t n = loop.size();
    if (n == 0) return;
    for (const LoopSegment& s : loop) {
        if (!finitePoint(s.a) || !finitePoint(s.b)) {
            finite_ = false;
            return;
        }
    }
    extent_ = {loop[0].a.x, loop[0].a.y, loop[0].a.x, loop[0].a.y};
    Box all = extent_;
    chordBox_.reserve(n);
    for (const LoopSegment& s : loop) {
        extent_ = {std::min(extent_.x0, s.a.x), std::min(extent_.y0, s.a.y),
                   std::max(extent_.x1, s.a.x), std::max(extent_.y1, s.a.y)};
        const Box b{std::min(s.a.x, s.b.x), std::min(s.a.y, s.b.y), std::max(s.a.x, s.b.x),
                    std::max(s.a.y, s.b.y)};
        chordBox_.push_back(b);
        all = {std::min(all.x0, b.x0), std::min(all.y0, b.y0), std::max(all.x1, b.x1),
               std::max(all.y1, b.y1)};
    }
    // Fewer buckets where items span many: the registrations stay a few per item.
    const auto bucketed = [&](Buckets& into, double ylo, double yhi, const auto& rangeOf) {
        into.build(ylo, yhi, n);
        for (int tries = 0; tries < 8; ++tries) {
            std::size_t total = 0;
            for (std::size_t i = 0; i < n; ++i) {
                const auto [y0, y1] = rangeOf(i);
                total += static_cast<std::size_t>(into.of(y1) - into.of(y0) + 1);
            }
            if (total <= kRegistrationsPerItem * n + kMaxBuckets || into.count == 1) break;
            into.build(ylo, yhi, std::max<std::size_t>(2, static_cast<std::size_t>(into.count)));
        }
        for (std::size_t i = 0; i < n; ++i) {
            const auto [y0, y1] = rangeOf(i);
            into.add(static_cast<int>(i), y0, y1);
        }
    };
    bucketed(chords_, all.y0, all.y1, [&](std::size_t i) {
        return std::pair{chordBox_[i].y0, chordBox_[i].y1};
    });
    bucketed(edges_, extent_.y0, extent_.y1, [&](std::size_t i) {
        const Vec2& a = loop[i].a;
        const Vec2& b = loop[(i + 1) % n].a;
        return std::pair{std::min(a.y, b.y), std::max(a.y, b.y)};
    });
}

void LoopIndex::prepareBands() {
    if (bandsReady_ || !finite_) return;
    bandsReady_ = true;
    const std::size_t n = loop_.size();
    sag_.reserve(n);
    bandBox_.reserve(n);
    double ylo = std::numeric_limits<double>::infinity();
    double yhi = -ylo;
    for (std::size_t i = 0; i < n; ++i) {
        const double s = sag(loop_[i]);
        sag_.push_back(s);
        const double r = s + bandMargin(loop_[i], s);
        const Box& b = chordBox_[i];
        // A sag that is not a number reads as no band, and one that is infinite as all of it:
        // either way the chord is always looked at, so its own comparison decides.
        const Box grown = std::isfinite(r) ? Box{b.x0 - r, b.y0 - r, b.x1 + r, b.y1 + r}
                                           : Box{-std::numeric_limits<double>::infinity(),
                                                 -std::numeric_limits<double>::infinity(),
                                                 std::numeric_limits<double>::infinity(),
                                                 std::numeric_limits<double>::infinity()};
        bandBox_.push_back(grown);
        if (std::isfinite(r)) {
            ylo = std::min(ylo, grown.y0);
            yhi = std::max(yhi, grown.y1);
        }
    }
    if (!(ylo <= yhi)) {
        ylo = 0.0;
        yhi = 0.0;
    }
    bands_.build(ylo, yhi, n);
    for (std::size_t i = 0; i < n; ++i) {
        const Box& g = bandBox_[i];
        if (std::isfinite(g.y0) && std::isfinite(g.y1)) {
            bands_.add(static_cast<int>(i), g.y0, g.y1);
        } else {
            for (auto& m : bands_.members) m.push_back(static_cast<int>(i));
        }
    }
}

math::Containment LoopIndex::classify(const Vec2& point) const {
    const std::size_t n = loop_.size();
    if (n == 0) return math::Containment::Outside;
    // A point beyond the polygon's box is on no edge and wound round by none.
    if (point.x < extent_.x0 || point.x > extent_.x1 || point.y < extent_.y0 ||
        point.y > extent_.y1) {
        return math::Containment::Outside;
    }
    // math::classifyPoint, edge for edge, over the edges whose heights span the point's: the
    // others are not on it and add nothing to its winding.
    int winding = 0;
    for (const int i : edges_.members[static_cast<std::size_t>(edges_.of(point.y))]) {
        const Vec2& a = loop_[static_cast<std::size_t>(i)].a;
        const Vec2& b = loop_[(static_cast<std::size_t>(i) + 1) % n].a;
        const int side = math::orient2d(a, b, point);
        if (side == 0 && point.x >= std::min(a.x, b.x) && point.x <= std::max(a.x, b.x) &&
            point.y >= std::min(a.y, b.y) && point.y <= std::max(a.y, b.y)) {
            return math::Containment::OnBoundary;
        }
        if (a.y <= point.y) {
            if (b.y > point.y && side > 0) ++winding;
        } else if (b.y <= point.y && side < 0) {
            --winding;
        }
    }
    return winding != 0 ? math::Containment::Inside : math::Containment::Outside;
}

bool LoopIndex::inBand(const Vec2& point) const {
    for (const int i : bands_.members[static_cast<std::size_t>(bands_.of(point.y))]) {
        const auto k = static_cast<std::size_t>(i);
        const Box& g = bandBox_[k];
        if (point.x < g.x0 || point.x > g.x1 || point.y < g.y0 || point.y > g.y1) continue;
        if (distanceToSegment(point, loop_[k].a, loop_[k].b) <= sag_[k]) return true;
    }
    return false;
}

bool LoopIndex::clearly(math::Containment side, const Vec2& point) {
    if (!finite_ || !finitePoint(point)) return detail::clearly(side, loop_, point);
    if (classify(point) != side) return false;
    prepareBands();
    return !inBand(point);
}

bool LoopIndex::crosses(const LoopIndex& other, const UvTolerance& tolerance) const {
    if (!finite_ || !other.finite_) return loopsCross(loop_, other.loop_, tolerance);
    if (loop_.empty() || other.loop_.empty()) return false;
    std::vector<int> seen(other.loop_.size(), -1);
    for (std::size_t i = 0; i < loop_.size(); ++i) {
        const Box& s = chordBox_[i];
        const int last = other.chords_.of(s.y1);
        for (int bucket = other.chords_.of(s.y0); bucket <= last; ++bucket) {
            for (const int k : other.chords_.members[static_cast<std::size_t>(bucket)]) {
                const auto kk = static_cast<std::size_t>(k);
                if (seen[kk] == static_cast<int>(i)) continue;
                seen[kk] = static_cast<int>(i);
                const Box& t = other.chordBox_[kk];
                // Chords that cross properly meet at a point inside both their boxes.
                if (s.x0 > t.x1 || t.x0 > s.x1 || s.y0 > t.y1 || t.y0 > s.y1) continue;
                if (crossesForReal(loop_[i], other.loop_[kk], tolerance, 0)) return true;
            }
        }
    }
    return false;
}

bool LoopIndex::selfIntersects(const UvTolerance& tolerance) const {
    if (!finite_) return detail::selfIntersects(loop_, tolerance);
    const std::size_t n = loop_.size();
    std::vector<int> seen(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
        const Box& s = chordBox_[i];
        const int last = chords_.of(s.y1);
        for (int bucket = chords_.of(s.y0); bucket <= last; ++bucket) {
            for (const int k : chords_.members[static_cast<std::size_t>(bucket)]) {
                const auto j = static_cast<std::size_t>(k);
                // The pairs selfIntersects tries: later and not adjacent, nor across the close.
                if (j < i + 2 || (i == 0 && j == n - 1)) continue;
                if (seen[j] == static_cast<int>(i)) continue;
                seen[j] = static_cast<int>(i);
                const Box& t = chordBox_[j];
                if (s.x0 > t.x1 || t.x0 > s.x1 || s.y0 > t.y1 || t.y0 > s.y1) continue;
                if (crossesForReal(loop_[i], loop_[j], tolerance, 0)) return true;
            }
        }
    }
    return false;
}

}  // namespace hz::topo::detail
