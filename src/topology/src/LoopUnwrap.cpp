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

}  // namespace

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
                LoopPolygon& out) {
    out.clear();
    // Each join from half-edge k - 1 to k, as unwrapped: the jump left once the next pcurve is
    // shifted, and how far a step in u or v moves its point there (zero at a pole or apex).
    struct Join {
        Vec2 jump;
        double speedU = 0.0;
        double speedV = 0.0;
    };
    std::vector<Join> joins(ring.size());
    Vec2 prevEnd;
    for (size_t k = 0; k < ring.size(); ++k) {
        const HalfEdge* h = ring[k];
        Vec2 shift;
        const Vec2 start = sampleAt(*h, 0, samples);
        if (!out.empty()) {
            // A raw jump that is already the same point within the join's allowance (a pole or
            // apex, where a derivative vanishes) is a move along the singular side, not a wrap;
            // the same quantity the join check tests. Which way round it runs is the loop's to
            // say: see the closing check below.
            const double allowance = joinAllowance[k - 1];
            const auto d = surface.derivatives(prevEnd.x, prevEnd.y);
            const double dx = prevEnd.x - start.x;
            const double dy = prevEnd.y - start.y;
            if (periodU > 0.0 && std::fabs(dx) * d.du.length() > allowance) {
                shift.x = std::round(dx / periodU) * periodU;
            }
            if (periodV > 0.0 && std::fabs(dy) * d.dv.length() > allowance) {
                shift.y = std::round(dy / periodV) * periodV;
            }
            joins[k] = {prevEnd - (start + shift), d.du.length(), d.dv.length()};
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
               std::round(gap / period) != 0.0;
    };
    // A loop through a pole or apex may take the whole periods it winds at the singular join
    // rather than at a regular one: a cone's loop that wraps at its rim runs, reversed (a mirrored
    // copy's), round the cone at its apex instead. Where the loop closes only after winding, the
    // periods are taken at the last join that is the same point with them added, and every
    // chord after it is moved by them; with no such join the loop winds round the surface.
    const std::size_t perEdge = static_cast<std::size_t>(samples - 1);
    const auto takeAtSingularJoin = [&](bool alongU) {
        const Vec2 gap = out.front().a - prevEnd;
        const double period = alongU ? periodU : periodV;
        const double whole = std::round((alongU ? gap.x : gap.y) / period) * period;
        for (size_t k = ring.size(); k-- > 1;) {
            const Join& j = joins[k];
            const double jump = (alongU ? j.jump.x : j.jump.y) - whole;
            if (std::fabs(jump) * (alongU ? j.speedU : j.speedV) > joinAllowance[k - 1]) continue;
            const Vec2 move = alongU ? Vec2(whole, 0.0) : Vec2(0.0, whole);
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
    return !(winds(left.y, periodV, d.dv.length()) && !takeAtSingularJoin(false));
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
