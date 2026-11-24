#include "horizon/topology/ExactValidator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "ExactLoopGeometry.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/topology/LoopUnwrap.h"

namespace hz::topo {

namespace {

using math::Vec2;
using math::Vec3;

constexpr int kSamples = kLoopSamples;  // a pcurve's two ends and 15 between

/// A number for a detail string, with enough digits to read on parts of any size.
std::string num(double x) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.4g", x);
    return buf;
}

struct Checker {
    const Solid& solid;
    const math::ModelTolerance& tol;
    std::vector<ExactIssue> issues;

    void report(ExactIssueKind kind, const Face* f, const Edge* e, std::string detail) {
        issues.push_back({kind, f, e, std::move(detail)});
    }

    // A pcurve's sample at index i of kSamples, at the unwrapping's own parameters.
    static Vec2 sample(const HalfEdge& h, int i) {
        return h.pcurve->evaluate(loopSampleParameter(h, i, kSamples));
    }

    void checkEdge(const Edge& e) {
        if (!e.exactCurve) {
            report(ExactIssueKind::MissingGeometry, nullptr, &e, "edge has no exact curve");
        } else {
            const HalfEdge* h = e.halfEdge;
            const struct {
                double t;
                const Vertex* v;
            } ends[] = {{e.curveStart, h->origin}, {e.curveEnd, h->twin->origin}};
            for (const auto& end : ends) {
                const double d = (e.exactCurve->evaluate(end.t) - end.v->point).length();
                if (!tol.coincident(d, e.tolerance, end.v->tolerance)) {
                    report(ExactIssueKind::VertexOffCurve, nullptr, &e,
                           "curve end is " + num(d) + " from its vertex");
                }
            }
        }
        for (const HalfEdge* h : {e.halfEdge, e.halfEdge->twin}) {
            if (!h->face || !h->alive) continue;
            if (!h->pcurve) {
                report(ExactIssueKind::MissingGeometry, h->face, &e, "half-edge has no pcurve");
                continue;
            }
            // A face without a surface is reported once, by checkFace.
            if (h->face->exactSurface) checkPcurve(e, *h);
        }
    }

    void checkPcurve(const Edge& e, const HalfEdge& h) {
        const geo::Surface& s = *h.face->exactSurface;
        const Vec2 a = sample(h, 0);
        const Vec2 b = sample(h, kSamples - 1);
        const double da = (s.evaluate(a.x, a.y) - h.origin->point).length();
        const double db = (s.evaluate(b.x, b.y) - h.next->origin->point).length();
        if (!tol.within(da, std::max(e.tolerance, h.origin->tolerance)) ||
            !tol.within(db, std::max(e.tolerance, h.next->origin->tolerance))) {
            report(ExactIssueKind::PcurveOffVertex, h.face, &e,
                   "pcurve ends are " + num(da) + " and " + num(db) + " from their vertices");
        }
        if (!e.exactCurve) return;
        double worst = 0.0;
        std::vector<double> params;
        for (int i = 0; i < kSamples; ++i) {
            const Vec2 uv = sample(h, i);
            const Vec3 p = s.evaluate(uv.x, uv.y);
            // The pcurve and the curve need not share parameters: measure to the nearest point.
            const double t = e.exactCurve->closestParameter(p);
            params.push_back(t);
            worst = std::max(worst, (e.exactCurve->evaluate(t) - p).length());
        }
        // The ends are known: the vertices (checked above) sit at curveStart and curveEnd. Using
        // them, not projections, keeps a closed curve's end samples from projecting to the other
        // end of its range.
        const bool forward = &h == e.halfEdge;
        params.front() = forward ? e.curveStart : e.curveEnd;
        params.back() = forward ? e.curveEnd : e.curveStart;
        if (!tol.within(worst, e.tolerance)) {
            report(ExactIssueKind::PcurveOffCurve, h.face, &e,
                   "pcurve strays " + num(worst) + " from the edge's curve");
        } else if (!advancesWithCurve(e, forward, params)) {
            report(ExactIssueKind::PcurveOffCurve, h.face, &e,
                   "pcurve runs against, or winds differently round, the edge's curve");
        }
    }

    /// True when the projected parameters advance in the half-edge's direction along the curve
    /// (curveStart to curveEnd for the edge's own half-edge, backward for its twin) and cover it
    /// exactly once: the steps sum to |curveEnd - curveStart|. Only order and extent are
    /// compared, not the values, as a pcurve and its curve need not share parameters. On a
    /// periodic curve each step is taken modulo the period. Each step is slack by the model
    /// tolerance in parameter units (tolerance over the curve's speed).
    bool advancesWithCurve(const Edge& e, bool forward, const std::vector<double>& params) const {
        const double dir = ((e.curveEnd >= e.curveStart) ? 1.0 : -1.0) * (forward ? 1.0 : -1.0);
        const geo::Interval range = e.exactCurve->range();
        const double period = range.max - range.min;
        const bool wraps = e.exactCurve->periodic() && std::isfinite(period) && period > 0.0;
        double sum = 0.0;
        double slackSum = 0.0;
        for (size_t i = 0; i + 1 < params.size(); ++i) {
            double step = (params[i + 1] - params[i]) * dir;
            if (wraps) {
                step = std::fmod(step, period);
                if (step > 0.5 * period) step -= period;
                if (step < -0.5 * period) step += period;
            }
            const double speed = e.exactCurve->derivatives(params[i]).d1.length();
            const double slack = speed > 0.0 ? tol.effective(e.tolerance) / speed : 0.0;
            if (step < -slack) return false;
            sum += step;
            slackSum += slack;
        }
        return std::fabs(sum - std::fabs(e.curveEnd - e.curveStart)) <= slackSum;
    }

    // ---- faces and loops ----

    void checkFace(const Face& f) {
        if (!f.exactSurface) {
            report(ExactIssueKind::MissingGeometry, &f, nullptr, "face has no exact surface");
            return;
        }
        std::vector<LoopInfo> loops;
        loops.push_back(checkLoop(f, f.outerLoop, true));
        for (const Wire* w : f.innerLoops) loops.push_back(checkLoop(f, w, false));
        checkLoopRelations(f, loops);
    }

    struct LoopInfo {
        const Wire* wire = nullptr;
        bool usable = false;  // closed, joined and fully carrying pcurves
        LoopPolygon poly;
    };

    /// A parametric tolerance on @p s at a point: the model tolerance over the larger of the
    /// surface's derivative magnitudes there, so it is a distance on the surface whichever way
    /// the point is displaced.
    detail::UvTolerance uvTolerance(const geo::Surface& s) const {
        return [&s, this](const Vec2& p) {
            const auto d = s.derivatives(p.x, p.y);
            const double speed = std::max(d.du.length(), d.dv.length());
            return speed > 0.0 ? tol.linear / speed : std::numeric_limits<double>::infinity();
        };
    }

    /// True when two (u, v) points are the same point of the surface within @p allowance
    /// (a distance): each parameter difference, taken modulo the period on a periodic axis,
    /// times the surface's derivative magnitude along it. Where that derivative vanishes (a
    /// pole) every value of the parameter is the same point.
    static bool sameUv(const geo::Surface& s, const Vec2& a, const Vec2& b, double allowance) {
        const auto d = s.derivatives(a.x, a.y);
        const auto close = [&](double x, double y, double speed, bool periodic, geo::Interval r) {
            double diff = std::fabs(x - y);
            if (periodic) {
                const double period = r.max - r.min;
                diff = std::fmod(diff, period);
                diff = std::min(diff, period - diff);
            }
            return speed <= 0.0 || diff * speed <= allowance;
        };
        return close(a.x, b.x, d.du.length(), s.periodicU(), s.uRange()) &&
               close(a.y, b.y, d.dv.length(), s.periodicV(), s.vRange());
    }

    LoopInfo checkLoop(const Face& f, const Wire* w, bool outer) {
        LoopInfo info;
        info.wire = w;
        if (!w || !w->halfEdge) {
            report(ExactIssueKind::LoopNotClosed, &f, nullptr, "face has no loop");
            return info;
        }
        std::vector<const HalfEdge*> ring;
        if (!loopHalfEdges(f, *w, 2 * solid.edges().size() + 1, ring)) {
            report(ExactIssueKind::LoopNotClosed, &f, w->halfEdge->edge,
                   "the half-edge cycle does not return to its start");
            return info;
        }
        for (const HalfEdge* h : ring) {
            if (!h->pcurve) return info;  // already reported as MissingGeometry
        }
        const geo::Surface& s = *f.exactSurface;
        bool joined = true;
        std::vector<double> allowances;
        for (const HalfEdge* h : ring) {
            const double allowance = joinAllowance(*h, tol);
            allowances.push_back(allowance);
            if (!sameUv(s, sample(*h, kSamples - 1), sample(*h->next, 0), allowance)) {
                joined = false;
                report(ExactIssueKind::LoopNotClosed, &f, h->edge,
                       "pcurve does not join the next one in (u, v)");
            }
        }
        if (!joined) return info;
        // The loop as chords of its pcurves' own (u, v) values, unwrapped so that it is
        // continuous: a periodic surface's seam must not collapse the rectangle, and a loop
        // written inside [0, 2 pi) must not break where it crosses the seam.
        if (!unwrapLoop(ring, kSamples, s, uPeriod(s), vPeriod(s), allowances, info.poly)) {
            report(ExactIssueKind::LoopNotClosed, &f, w->halfEdge->edge,
                   "loop closes only after winding round the surface");
            return info;
        }
        info.usable = true;
        const double area = detail::signedArea(info.poly);
        // Outer loops run counter-clockwise unless the face is reversed; inner loops the
        // other way.
        const bool wantPositive = outer != f.reversed;
        if ((area > 0.0) != wantPositive) {
            report(ExactIssueKind::LoopWrongOrientation, &f, w->halfEdge->edge,
                   "loop has signed area " + num(area) + " in (u, v)");
        }
        if (detail::selfIntersects(info.poly, uvTolerance(s))) {
            report(ExactIssueKind::LoopSelfIntersects, &f, w->halfEdge->edge,
                   "loop crosses itself");
        }
        return info;
    }

    void checkLoopRelations(const Face& f, std::vector<LoopInfo>& loops) {
        const geo::Surface& surface = *f.exactSurface;
        const auto uvTol = uvTolerance(surface);
        // Each loop was unwrapped from its own first pcurve, so a loop may sit a period away
        // from the others: bring them into the window of the first usable one (the outer loop
        // when it is) before comparing, so that a broken outer loop hides nothing between
        // the rest.
        const auto ref =
            std::find_if(loops.begin(), loops.end(), [](const LoopInfo& l) { return l.usable; });
        if (ref == loops.end()) return;
        for (auto j = loops.begin(); j != loops.end(); ++j) {
            if (!j->usable || j == ref) continue;
            alignLoop(j->poly, ref->poly, uPeriod(surface), vPeriod(surface));
        }
        for (size_t i = 0; i < loops.size(); ++i) {
            if (!loops[i].usable) continue;
            for (size_t j = i + 1; j < loops.size(); ++j) {
                if (!loops[j].usable) continue;
                const bool crossing = detail::loopsCross(loops[i].poly, loops[j].poly, uvTol);
                if (crossing) {
                    report(ExactIssueKind::LoopsCross, &f, loops[j].wire->halfEdge->edge,
                           "two loops of the face cross");
                    continue;
                }
                if (i == 0) {
                    const bool outside = std::any_of(
                        loops[j].poly.begin(), loops[j].poly.end(), [&](const LoopSegment& seg) {
                            return detail::clearlyOutside(loops[0].poly, seg.a);
                        });
                    if (outside) {
                        report(ExactIssueKind::HoleOutsideOuterLoop, &f,
                               loops[j].wire->halfEdge->edge,
                               "inner loop lies outside the outer loop");
                    }
                    continue;
                }
                // Two holes that do not cross are apart or nested; the mesher needs them apart.
                if (inside(loops[j].poly, loops[i].poly) || inside(loops[i].poly, loops[j].poly)) {
                    report(ExactIssueKind::HoleInsideHole, &f, loops[j].wire->halfEdge->edge,
                           "inner loops " + std::to_string(i - 1) + " and " +
                               std::to_string(j - 1) + " lie one inside the other");
                }
            }
        }
    }

    /// True when a sample of @p loop lies clearly inside @p around (beyond each of its chords'
    /// sag): for loops that do not cross, the whole of @p loop is then inside.
    static bool inside(const LoopPolygon& loop, const LoopPolygon& around) {
        return std::any_of(loop.begin(), loop.end(), [&](const LoopSegment& seg) {
            return detail::clearlyInside(around, seg.a);
        });
    }
};

}  // namespace

std::vector<ExactIssue> validateExact(const Solid& solid, const math::ModelTolerance& tolerance) {
    Checker c{solid, tolerance, {}};
    for (const Edge& e : solid.edges()) {
        if (e.alive && e.halfEdge && e.halfEdge->twin) c.checkEdge(e);
    }
    for (const Face& f : solid.faces()) {
        if (f.alive) c.checkFace(f);
    }
    std::vector<ExactIssue> out = std::move(c.issues);
    return out;
}

}  // namespace hz::topo
