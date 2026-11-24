#include "horizon/topology/ExactValidator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/RobustPredicates.h"

namespace hz::topo {

namespace {

using math::Vec2;
using math::Vec3;

constexpr int kSamples = 17;  // a pcurve's two ends and 15 between

struct Checker {
    const Solid& solid;
    const math::ModelTolerance& tol;
    std::vector<ExactIssue> issues;

    void report(ExactIssueKind kind, const Face* f, const Edge* e, std::string detail) {
        issues.push_back({kind, f, e, std::move(detail)});
    }

    // A pcurve's sample at index i of kSamples, from pcurveStart to pcurveEnd (either way round).
    static Vec2 sample(const HalfEdge& h, int i) {
        const double t = h.pcurveStart + (h.pcurveEnd - h.pcurveStart) * i / (kSamples - 1);
        return h.pcurve->evaluate(t);
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
                           "curve end is " + std::to_string(d) + " from its vertex");
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
                   "pcurve ends are " + std::to_string(da) + " and " + std::to_string(db) +
                       " from their vertices");
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
        if (!tol.within(worst, e.tolerance)) {
            report(ExactIssueKind::PcurveOffCurve, h.face, &e,
                   "pcurve strays " + std::to_string(worst) + " from the edge's curve");
        } else if (!advancesWithCurve(e, h, params)) {
            report(ExactIssueKind::PcurveOffCurve, h.face, &e,
                   "pcurve runs against the edge's curve");
        }
    }

    /// True when the projected parameters advance in the half-edge's direction along the curve:
    /// forward (curveStart to curveEnd) for the edge's own half-edge, backward for its twin. Only
    /// the order is compared, not the values. On a periodic curve each step is taken modulo the
    /// period, so a closed edge may wrap once; a step is slack by the model tolerance in
    /// parameter units (tolerance over the curve's speed).
    bool advancesWithCurve(const Edge& e, const HalfEdge& h,
                           const std::vector<double>& params) const {
        const double dir =
            ((e.curveEnd >= e.curveStart) ? 1.0 : -1.0) * (&h == e.halfEdge ? 1.0 : -1.0);
        const geo::Interval range = e.exactCurve->range();
        const double period = range.max - range.min;
        const bool wraps = e.exactCurve->periodic() && std::isfinite(period) && period > 0.0;
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
        }
        return true;
    }

    void checkFace(const Face& f) {
        if (!f.exactSurface) {
            report(ExactIssueKind::MissingGeometry, &f, nullptr, "face has no exact surface");
            return;
        }
        checkLoop(f, f.outerLoop, true);
        for (const Wire* w : f.innerLoops) checkLoop(f, w, false);
        checkLoopRelations(f);
    }

    // ---- loops ----

    struct LoopData {
        const Wire* wire = nullptr;
        bool outer = false;
        bool usable = false;  // closed, joined and fully carrying pcurves
        std::vector<Vec2> poly;
    };
    std::vector<LoopData> loops;  // the loops of the face being checked

    /// How far apart two (u, v) points may be and still be the same point of the surface: the
    /// effective model tolerance (plus the vertex's own) turned into parameter units by the
    /// surface's derivative magnitudes at the point. Where a derivative vanishes (a pole) every
    /// value of that parameter is the same point, so any difference is accepted.
    bool sameUv(const geo::Surface& s, const Vec2& a, const Vec2& b, double extra) const {
        const auto d = s.derivatives(a.x, a.y);
        const double eff = tol.effective(extra);
        const auto close = [&](double x, double y, double speed, bool periodic, geo::Interval r) {
            double diff = std::fabs(x - y);
            if (periodic) {
                const double period = r.max - r.min;
                diff = std::fmod(diff, period);
                diff = std::min(diff, period - diff);
            }
            return speed <= 0.0 || diff * speed <= eff;
        };
        return close(a.x, b.x, d.du.length(), s.periodicU(), s.uRange()) &&
               close(a.y, b.y, d.dv.length(), s.periodicV(), s.vRange());
    }

    void checkLoop(const Face& f, const Wire* w, bool outer) {
        LoopData data;
        data.wire = w;
        data.outer = outer;
        if (!w || !w->halfEdge) {
            report(ExactIssueKind::LoopNotClosed, &f, nullptr, "face has no loop");
            loops.push_back(std::move(data));
            return;
        }
        std::vector<const HalfEdge*> ring;
        const size_t limit = 2 * solid.edges().size() + 1;
        const HalfEdge* he = w->halfEdge;
        bool closed = false;
        while (he && he->face == &f && ring.size() <= limit) {
            ring.push_back(he);
            he = he->next;
            if (he == w->halfEdge) {
                closed = true;
                break;
            }
        }
        if (!closed) {
            report(ExactIssueKind::LoopNotClosed, &f, w->halfEdge->edge,
                   "the half-edge cycle does not return to its start");
            loops.push_back(std::move(data));
            return;
        }
        bool carrying = true;
        for (const HalfEdge* h : ring) carrying = carrying && h->pcurve;
        if (!carrying) {  // already reported as MissingGeometry
            loops.push_back(std::move(data));
            return;
        }
        const geo::Surface& s = *f.exactSurface;
        bool joined = true;
        for (const HalfEdge* h : ring) {
            const Vec2 end = sample(*h, kSamples - 1);
            const Vec2 next = sample(*h->next, 0);
            if (!sameUv(s, end, next, h->next->origin->tolerance)) {
                joined = false;
                report(ExactIssueKind::LoopNotClosed, &f, h->edge,
                       "pcurve does not join the next one in (u, v)");
            }
        }
        if (!joined) {
            loops.push_back(std::move(data));
            return;
        }
        // The loop as a polygon of its pcurves' own (u, v) values, unwrapped: a periodic
        // surface's seam must not collapse the rectangle.
        for (const HalfEdge* h : ring) {
            for (int i = 0; i + 1 < kSamples; ++i) data.poly.push_back(sample(*h, i));
        }
        data.usable = true;
        double twice = 0.0;
        for (size_t i = 0; i < data.poly.size(); ++i) {
            const Vec2& p = data.poly[i];
            const Vec2& q = data.poly[(i + 1) % data.poly.size()];
            twice += p.x * q.y - q.x * p.y;
        }
        // Outer loops run counter-clockwise unless the face is reversed; inner loops the
        // other way.
        const bool wantPositive = outer != f.reversed;
        if ((twice > 0.0) != wantPositive) {
            report(ExactIssueKind::LoopWrongOrientation, &f, w->halfEdge->edge,
                   "loop has signed area " + std::to_string(twice / 2.0) + " in (u, v)");
        }
        if (selfIntersects(data.poly)) {
            report(ExactIssueKind::LoopSelfIntersects, &f, w->halfEdge->edge,
                   "loop crosses itself");
        }
        loops.push_back(std::move(data));
    }

    static bool properlyCross(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
        return math::orient2d(a, b, c) * math::orient2d(a, b, d) < 0 &&
               math::orient2d(c, d, a) * math::orient2d(c, d, b) < 0;
    }

    static bool selfIntersects(const std::vector<Vec2>& poly) {
        const size_t n = poly.size();
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = i + 2; j < n; ++j) {
                if (i == 0 && j == n - 1) continue;  // adjacent across the closing segment
                if (properlyCross(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n])) {
                    return true;
                }
            }
        }
        return false;
    }

    static bool loopsCross(const std::vector<Vec2>& p, const std::vector<Vec2>& q) {
        for (size_t i = 0; i < p.size(); ++i) {
            for (size_t j = 0; j < q.size(); ++j) {
                if (properlyCross(p[i], p[(i + 1) % p.size()], q[j], q[(j + 1) % q.size()])) {
                    return true;
                }
            }
        }
        return false;
    }

    void checkLoopRelations(const Face& f) {
        for (size_t i = 0; i < loops.size(); ++i) {
            if (!loops[i].usable) continue;
            for (size_t j = i + 1; j < loops.size(); ++j) {
                if (!loops[j].usable) continue;
                if (loopsCross(loops[i].poly, loops[j].poly)) {
                    report(ExactIssueKind::LoopsCross, &f, loops[j].wire->halfEdge->edge,
                           "two loops of the face cross");
                }
            }
        }
        if (loops.empty() || !loops[0].usable) return;
        for (size_t j = 1; j < loops.size(); ++j) {
            if (!loops[j].usable || loopsCross(loops[0].poly, loops[j].poly)) continue;
            const bool outside =
                std::any_of(loops[j].poly.begin(), loops[j].poly.end(), [&](const Vec2& p) {
                    return math::classifyPoint(loops[0].poly, p) == math::Containment::Outside;
                });
            if (outside) {
                report(ExactIssueKind::HoleOutsideOuterLoop, &f, loops[j].wire->halfEdge->edge,
                       "inner loop lies outside the outer loop");
            }
        }
    }
};

}  // namespace

std::vector<ExactIssue> validateExact(const Solid& solid, const math::ModelTolerance& tolerance) {
    Checker c{solid, tolerance, {}, {}};
    for (const Edge& e : solid.edges()) {
        if (e.alive && e.halfEdge && e.halfEdge->twin) c.checkEdge(e);
    }
    for (const Face& f : solid.faces()) {
        if (!f.alive) continue;
        c.loops.clear();
        c.checkFace(f);
    }
    return std::move(c.issues);
}

}  // namespace hz::topo
