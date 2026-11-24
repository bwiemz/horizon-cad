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
    return tolerance.effective(std::max(h.edge->tolerance, h.next->origin->tolerance)) +
           tolerance.effective(std::max(h.next->edge->tolerance, h.next->origin->tolerance));
}

bool unwrapLoop(const std::vector<const HalfEdge*>& ring, int samples, const geo::Surface& surface,
                double periodU, double periodV, const std::vector<double>& joinAllowance,
                LoopPolygon& out) {
    out.clear();
    Vec2 prevEnd;
    for (size_t k = 0; k < ring.size(); ++k) {
        const HalfEdge* h = ring[k];
        Vec2 shift;
        const Vec2 start = sampleAt(*h, 0, samples);
        if (!out.empty()) {
            // A raw jump that is already the same point within the join's allowance (a pole or
            // apex, where a derivative vanishes) is a move along the singular side, not a wrap;
            // the same quantity the join check tests. Which way round it runs is decided by the
            // next pcurve's own window; that is a stage 3 question (pole topology).
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
    const Vec2 gap = out.front().a - prevEnd;
    const auto d = surface.derivatives(prevEnd.x, prevEnd.y);
    const double allowance = joinAllowance[ring.size() - 1];
    const bool windsU = periodU > 0.0 && std::fabs(gap.x) * d.du.length() > allowance &&
                        std::round(gap.x / periodU) != 0.0;
    const bool windsV = periodV > 0.0 && std::fabs(gap.y) * d.dv.length() > allowance &&
                        std::round(gap.y / periodV) != 0.0;
    return !windsU && !windsV;
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

}  // namespace hz::topo
