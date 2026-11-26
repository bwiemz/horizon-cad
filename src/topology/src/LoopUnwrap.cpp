#include "horizon/topology/LoopUnwrap.h"

#include <algorithm>
#include <cmath>

namespace hz::topo {

namespace {

using math::Vec2;

Vec2 sampleAt(const HalfEdge& h, int i, int samples) {
    return h.pcurve->evaluate(loopSampleParameter(h, i, samples));
}

double periodOf(bool periodic, geo::Interval r) {
    const double p = r.max - r.min;
    return periodic && std::isfinite(p) ? p : 0.0;
}

/// How near a pole run must come to a whole number of periods to be taken for one: a seam's two
/// sides written at one longitude, a run of 0 that is a whole turn round the pole. A guard on
/// parameter arithmetic (the runs are differences of pcurve ends), not a geometric tolerance.
constexpr double kWholeTurn = 1e-9;

/// Lays each run of @p out along a pole or an apex round it the way @p counterClockwise says,
/// within a period, where the changes cancel (the loop still closes) and the loop then runs that
/// way; otherwise leaves @p out as it was. @p joins is the ring's size, each join k running from
/// the end of half-edge k - 1 (the last for k = 0) to the start of half-edge k.
void turnPoleRuns(LoopPolygon& out, std::size_t joins, std::size_t perEdge,
                  const geo::Surface& surface, double periodU, double periodV,
                  const std::vector<double>& joinAllowance, bool counterClockwise) {
    const LoopPolygon before = out;
    for (const bool alongU : {true, false}) {
        const double period = alongU ? periodU : periodV;
        if (!(period > 0.0)) continue;
        std::vector<double> change(joins, 0.0);
        double total = 0.0;
        for (std::size_t k = 0; k < joins; ++k) {
            const LoopSegment& last = k == 0 ? out.back() : out[k * perEdge - 1];
            const LoopSegment& first = out[k * perEdge];
            const Vec2 end = last.b;
            const Vec2 start = first.a;
            const double allowance = joinAllowance[k == 0 ? joins - 1 : k - 1];
            // A run along a pole: the surface is one point along the whole period line through
            // the join, and the join moves along it.
            const Vec2 turn = alongU ? Vec2(period, 0.0) : Vec2(0.0, period);
            if (!runsAlongPoint(surface, end, end + turn, allowance) ||
                !runsAlongPoint(surface, end, start, allowance)) {
                continue;
            }
            // The side of the pole's line the face lies on: where the chords beside the join go.
            const double line = alongU ? 0.5 * (end.y + start.y) : 0.5 * (end.x + start.x);
            const double inBefore = (alongU ? last.a.y : last.a.x) - line;
            const double inAfter = (alongU ? first.b.y : first.b.x) - line;
            if (!(inBefore * inAfter > 0.0)) continue;
            const double side = inBefore > 0.0 ? 1.0 : -1.0;
            // Counter-clockwise, the face is on the run's left: a run along u goes toward +u
            // with the face above it (+v), a run along v toward +v with the face before it (-u).
            const double way = (alongU ? side : -side) * (counterClockwise ? 1.0 : -1.0);
            const double run = alongU ? start.x - end.x : start.y - end.y;
            const double whole = std::round(run / period) * period;
            double laid = 0.0;
            if (std::fabs(run - whole) <= kWholeTurn * period) {
                laid = way * period;  // a whole turn round the pole
            } else {
                laid = way > 0.0 ? run - std::floor(run / period) * period
                                 : run - std::ceil(run / period) * period;
            }
            change[k] = std::round((laid - run) / period) * period;
            total += change[k];
        }
        if (std::round(total / period) != 0.0) continue;  // the loop would no longer close
        // Each join's change moves every chord after it; the closing join (k = 0) takes the rest.
        for (std::size_t k = 1; k < joins; ++k) {
            if (change[k] == 0.0) continue;
            const Vec2 move = alongU ? Vec2(change[k], 0.0) : Vec2(0.0, change[k]);
            for (std::size_t c = k * perEdge; c < out.size(); ++c) {
                out[c].shift += move;
                out[c].a += move;
                out[c].b += move;
            }
        }
    }
    const double area = signedArea(out);
    if (!(area != 0.0 && (area > 0.0) == counterClockwise)) out = before;
}

}  // namespace

LoopSense loopSense(const Face& face, bool outer) {
    return outer != face.reversed ? LoopSense::CounterClockwise : LoopSense::Clockwise;
}

bool runsAlongPoint(const geo::Surface& surface, const Vec2& from, const Vec2& to,
                    double allowance) {
    const math::Vec3 p = surface.evaluate(from.x, from.y);
    const math::Vec3 q = surface.evaluate(to.x, to.y);
    if (!((p - q).length() <= allowance)) return false;
    for (int i = 1; i + 1 < kLoopSamples; ++i) {
        const Vec2 m = from + (to - from) * (static_cast<double>(i) / (kLoopSamples - 1));
        const math::Vec3 x = surface.evaluate(m.x, m.y);
        if (!((x - p).length() <= allowance) || !((x - q).length() <= allowance)) return false;
    }
    return true;
}

double loopSampleParameter(const HalfEdge& h, int i, int samples) {
    return h.pcurveStart + (h.pcurveEnd - h.pcurveStart) * i / (samples - 1);
}

double uPeriod(const geo::Surface& surface) {
    return periodOf(surface.periodicU(), surface.uRange());
}

double vPeriod(const geo::Surface& surface) {
    return periodOf(surface.periodicV(), surface.vRange());
}

bool loopHalfEdges(const Face& face, const Wire& wire, std::size_t limit,
                   std::vector<const HalfEdge*>& ring) {
    ring.clear();
    const HalfEdge* he = wire.halfEdge;
    while (he && he->face == &face && ring.size() <= limit) {
        ring.push_back(he);
        he = he->next;
        if (he == wire.halfEdge) return true;
    }
    return false;
}

double joinAllowance(const HalfEdge& h, const math::ModelTolerance& tolerance) {
    return joinAllowance(h, *h.next, tolerance);
}

double joinAllowance(const HalfEdge& h, const HalfEdge& next,
                     const math::ModelTolerance& tolerance) {
    return tolerance.effective(std::max(h.edge->tolerance, next.origin->tolerance)) +
           tolerance.effective(std::max(next.edge->tolerance, next.origin->tolerance));
}

bool unwrapLoop(const std::vector<const HalfEdge*>& ring, int samples, const geo::Surface& surface,
                double periodU, double periodV, const std::vector<double>& joinAllowance,
                LoopPolygon& out, LoopSense sense) {
    out.clear();
    Vec2 prevEnd;
    for (size_t k = 0; k < ring.size(); ++k) {
        const HalfEdge* h = ring[k];
        Vec2 shift;
        const Vec2 start = sampleAt(*h, 0, samples);
        if (!out.empty()) {
            // A raw jump that is already the same point within the join's allowance (a rounding,
            // or a pole or apex, where a derivative vanishes) is not a wrap; nor is one that runs
            // along a point of the surface, though a pcurve ends a hair off the pole, where the
            // derivative at that end alone would call it one. Each is a move along the singular
            // side; the same measures the join check takes. Which way round it runs is the
            // loop's to say: see the closing check below, and the face's (turnPoleRuns).
            const double allowance = joinAllowance[k - 1];
            const auto d = surface.derivatives(prevEnd.x, prevEnd.y);
            const double dx = prevEnd.x - start.x;
            const double dy = prevEnd.y - start.y;
            const bool wrapU = periodU > 0.0 && std::fabs(dx) * d.du.length() > allowance;
            const bool wrapV = periodV > 0.0 && std::fabs(dy) * d.dv.length() > allowance;
            if ((wrapU || wrapV) && !runsAlongPoint(surface, prevEnd, start, allowance)) {
                if (wrapU) shift.x = std::round(dx / periodU) * periodU;
                if (wrapV) shift.y = std::round(dy / periodV) * periodV;
            }
        }
        Vec2 a = start + shift;
        for (int i = 1; i < samples; ++i) {
            const double t0 = loopSampleParameter(*h, i - 1, samples);
            const double t1 = loopSampleParameter(*h, i, samples);
            const Vec2 b = h->pcurve->evaluate(t1) + shift;
            out.push_back({h->pcurve.get(), t0, t1, shift, a, b});
            a = b;
        }
        prevEnd = a;
    }
    if (out.empty()) return true;
    const auto d = surface.derivatives(prevEnd.x, prevEnd.y);
    const double allowance = joinAllowance[ring.size() - 1];
    const auto winds = [&](double gap, double period, double speed) {
        return period > 0.0 && std::fabs(gap) * speed > allowance &&
               std::round(gap / period) != 0.0 &&
               !runsAlongPoint(surface, prevEnd, out.front().a, allowance);
    };
    // A loop through a pole or apex may take the whole periods it winds at the singular join
    // rather than at a regular one: a cone's loop that wraps at its rim runs, reversed (a mirrored
    // copy's), round the cone at its apex instead. Where the loop closes only after winding, the
    // periods are taken at the last join that is the same point with them added (it runs along a
    // point of the surface), and every chord after it is moved by them; with no such join the
    // loop winds round the surface.
    const std::size_t perEdge = static_cast<std::size_t>(samples - 1);
    const auto takeAtSingularJoin = [&](bool alongU) {
        const Vec2 gap = out.front().a - prevEnd;
        const double period = alongU ? periodU : periodV;
        const double whole = std::round((alongU ? gap.x : gap.y) / period) * period;
        const Vec2 move = alongU ? Vec2(whole, 0.0) : Vec2(0.0, whole);
        for (size_t k = ring.size(); k-- > 1;) {
            const Vec2 end = out[k * perEdge - 1].b;
            const Vec2 start = out[k * perEdge].a + move;
            if (!runsAlongPoint(surface, end, start, joinAllowance[k - 1])) continue;
            for (size_t c = k * perEdge; c < out.size(); ++c) {
                out[c].shift += move;
                out[c].a += move;
                out[c].b += move;
            }
            prevEnd = out.back().b;
            return true;
        }
        return false;
    };
    const Vec2 gap = out.front().a - prevEnd;
    if (winds(gap.x, periodU, d.du.length()) && !takeAtSingularJoin(true)) return false;
    const Vec2 left = out.front().a - prevEnd;
    if (winds(left.y, periodV, d.dv.length()) && !takeAtSingularJoin(false)) return false;
    if (sense != LoopSense::Either) {
        const bool counterClockwise = sense == LoopSense::CounterClockwise;
        const double area = signedArea(out);
        if (!(area != 0.0 && (area > 0.0) == counterClockwise)) {
            turnPoleRuns(out, ring.size(), perEdge, surface, periodU, periodV, joinAllowance,
                         counterClockwise);
        }
    }
    return true;
}

void alignLoop(LoopPolygon& loop, const LoopPolygon& reference, double periodU, double periodV) {
    if (loop.empty() || reference.empty()) return;
    const auto middle = [](const LoopPolygon& l) {
        Vec2 lo = l.front().a;
        Vec2 hi = lo;
        for (const auto& s : l) {
            lo = {std::min(lo.x, s.a.x), std::min(lo.y, s.a.y)};
            hi = {std::max(hi.x, s.a.x), std::max(hi.y, s.a.y)};
        }
        return (lo + hi) * 0.5;
    };
    const Vec2 gap = middle(reference) - middle(loop);
    Vec2 delta;
    if (periodU > 0.0) delta.x = std::round(gap.x / periodU) * periodU;
    if (periodV > 0.0) delta.y = std::round(gap.y / periodV) * periodV;
    for (auto& s : loop) {
        s.shift = s.shift + delta;
        s.a = s.a + delta;
        s.b = s.b + delta;
    }
}

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

}  // namespace hz::topo
