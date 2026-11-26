#include "horizon/topology/ExactValidator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ExactLoopGeometry.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/topology/ExactTolerance.h"
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

// A pcurve's sample at index i of kSamples, at the unwrapping's own parameters.
Vec2 sample(const HalfEdge& h, int i) {
    return h.pcurve->evaluate(loopSampleParameter(h, i, kSamples));
}

/// @p c as a straight segment its parameter runs along at one speed (a NURBS of degree 1,
/// two points of one weight, as Horizon writes its edges), or null. Along it a point's
/// distance has one minimum, at its projection held to the segment, which is where
/// NurbsCurve::closestParameter's search ends too (to rounding): Newton's step is exact on
/// it, from any start, unless the curve's speed squared is below its 1e-15, where it keeps a
/// sample of 21 instead; so a segment that slow is not one here.
const geo::NurbsCurve* straightSegment(const geo::Curve3& c) {
    const auto* n = dynamic_cast<const geo::NurbsCurve*>(&c);
    if (n == nullptr || n->degree() != 1 || n->controlPointCount() != 2) return nullptr;
    if (!(n->weights()[0] == n->weights()[1])) return nullptr;
    const double span = n->tMax() - n->tMin();
    const Vec3 d = n->controlPoints()[1] - n->controlPoints()[0];
    const double speed2 = d.dot(d) / (span * span);
    if (!std::isfinite(speed2) || !(speed2 > 1e-12)) return nullptr;
    return n;
}

/// Whether checkPcurve's search, on an edge that is a straight segment, would find @p h's
/// pcurve on the curve and running with it: each sample's projection onto the segment is
/// within the tolerance by a margin far above rounding (the search's own point is the same
/// one, to rounding, so its distance is too), and the projections advance by the order test
/// with half its slack (rounding cannot move a step across the other half). It only ever
/// accepts: anything else is left to the search, so what is reported, and how far a pcurve
/// strays, are the search's, as ever.
bool passesOnASegment(const Edge& e, const HalfEdge& h, const math::ModelTolerance& tol) {
    const geo::NurbsCurve* seg = straightSegment(*e.exactCurve);
    if (seg == nullptr) return false;
    const geo::Surface& s = *h.face->exactSurface;
    const Vec3 p0 = seg->controlPoints()[0];
    const Vec3 d = seg->controlPoints()[1] - p0;
    const double len2 = d.dot(d);
    const double t0 = seg->tMin();
    const double span = seg->tMax() - t0;
    const double reach = p0.length() + seg->controlPoints()[1].length();
    std::vector<double> params;
    params.reserve(kSamples);
    for (int i = 0; i < kSamples; ++i) {
        const Vec2 uv = sample(h, i);
        const Vec3 p = s.evaluate(uv.x, uv.y);
        const double along = std::clamp((p - p0).dot(d) / len2, 0.0, 1.0);
        const double t = t0 + along * span;
        const double dist = (e.exactCurve->evaluate(t) - p).length();
        const double margin = 1e-12 * (reach + p.length());
        if (!std::isfinite(dist) || !tol.within(dist + margin, e.tolerance)) return false;
        params.push_back(t);
    }
    const bool forward = &h == e.halfEdge;
    params.front() = forward ? e.curveStart : e.curveEnd;
    params.back() = forward ? e.curveEnd : e.curveStart;
    const double dir = ((e.curveEnd >= e.curveStart) ? 1.0 : -1.0) * (forward ? 1.0 : -1.0);
    double sum = 0.0;
    double slackSum = 0.0;
    for (size_t i = 0; i + 1 < params.size(); ++i) {
        const double step = (params[i + 1] - params[i]) * dir;
        const double speed = e.exactCurve->derivatives(params[i]).d1.length();
        const double slack = speed > 0.0 ? tol.effective(e.tolerance) / speed : 0.0;
        if (!(step >= -0.5 * slack)) return false;
        sum += step;
        slackSum += slack;
    }
    return std::fabs(sum - std::fabs(e.curveEnd - e.curveStart)) <= 0.5 * slackSum;
}

/// What checkPcurve measures on @p h's pcurve (pcurveGaps), and, where it searched the curve for
/// each sample's nearest point (not on a segment the fast path accepts), the parameters found
/// in @p params, in sample order.
PcurveGaps measurePcurve(const Edge& e, const HalfEdge& h, const math::ModelTolerance& tol,
                         std::vector<double>& params) {
    const geo::Surface& s = *h.face->exactSurface;
    PcurveGaps gaps;
    const Vec2 a = sample(h, 0);
    const Vec2 b = sample(h, kSamples - 1);
    gaps.start = (s.evaluate(a.x, a.y) - h.origin->point).length();
    gaps.end = (s.evaluate(b.x, b.y) - h.next->origin->point).length();
    params.clear();
    if (!e.exactCurve || passesOnASegment(e, h, tol)) return gaps;
    for (int i = 0; i < kSamples; ++i) {
        const Vec2 uv = sample(h, i);
        const Vec3 p = s.evaluate(uv.x, uv.y);
        // The pcurve and the curve need not share parameters: measure to the nearest point.
        const double t = e.exactCurve->closestParameter(p);
        params.push_back(t);
        gaps.strays = std::max(gaps.strays, (e.exactCurve->evaluate(t) - p).length());
    }
    return gaps;
}

/// How many units in the last place of the magnitudes it is computed from two measures of one gap
/// may part by in rounding alone (pcurveRounding).
constexpr double kRoundingUlps = 16.0;

/// How far from the origin the terms @p s is evaluated from reach: its frame's origin and its
/// radii, or its farthest control point.
double reachOf(const geo::Surface& s) {
    switch (s.kind()) {
        case geo::SurfaceKind::Plane:
            return static_cast<const geo::Plane&>(s).frame().origin.length();
        case geo::SurfaceKind::Cylinder: {
            const auto& c = static_cast<const geo::Cylinder&>(s);
            return c.frame().origin.length() + std::fabs(c.radius());
        }
        case geo::SurfaceKind::Cone: {
            const auto& c = static_cast<const geo::Cone&>(s);
            return c.frame().origin.length() + std::fabs(c.radius());
        }
        case geo::SurfaceKind::Sphere: {
            const auto& c = static_cast<const geo::Sphere&>(s);
            return c.frame().origin.length() + std::fabs(c.radius());
        }
        case geo::SurfaceKind::Torus: {
            const auto& c = static_cast<const geo::Torus&>(s);
            return c.frame().origin.length() + std::fabs(c.majorRadius()) +
                   std::fabs(c.minorRadius());
        }
        case geo::SurfaceKind::Nurbs: {
            double far = 0.0;
            if (const auto* n = dynamic_cast<const geo::NurbsSurface*>(&s)) {
                for (const auto& row : n->controlPoints()) {
                    for (const Vec3& p : row) far = std::max(far, p.length());
                }
            }
            return far;
        }
    }
    return 0.0;
}

/// How far from the origin the terms @p c is evaluated from reach: its point, or its centre and
/// radius, or its farthest control point.
double reachOf(const geo::Curve3& c) {
    switch (c.kind()) {
        case geo::CurveKind::Line:
            if (const auto* l = dynamic_cast<const geo::LineCurve*>(&c)) {
                return l->origin().length();
            }
            return 0.0;
        case geo::CurveKind::Circle:
            if (const auto* k = dynamic_cast<const geo::CircleCurve*>(&c)) {
                return k->frame().origin.length() + std::fabs(k->radius());
            }
            return 0.0;
        case geo::CurveKind::Ellipse:
            if (const auto* k = dynamic_cast<const geo::EllipseCurve*>(&c)) {
                return k->frame().origin.length() + std::fabs(k->majorRadius()) +
                       std::fabs(k->minorRadius());
            }
            return 0.0;
        case geo::CurveKind::Nurbs: {
            double far = 0.0;
            if (const auto* n = dynamic_cast<const geo::NurbsCurve*>(&c)) {
                for (const Vec3& p : n->controlPoints()) far = std::max(far, p.length());
            }
            return far;
        }
    }
    return 0.0;
}

/// How far from the origin @p solid's points lie: its farthest live vertex's distance and its size.
double reachOf(const Solid& solid) {
    double farthest = 0.0;
    for (const Vertex& v : solid.vertices()) {
        if (v.alive) farthest = std::max(farthest, v.point.length());
    }
    return farthest + exactExtent(solid);
}

/// pcurveRounding, with its solid's reach (reachOf) @p solidReach.
double roundingAt(const HalfEdge& h, double solidReach) {
    if (!h.pcurve || h.face == nullptr || !h.face->exactSurface) return 0.0;
    const geo::Surface& s = *h.face->exactSurface;
    double magnitude = solidReach + reachOf(s);
    // A parameter of magnitude |u| is itself known only to a unit in its last place, which moves
    // its point by that times the surface's (or the curve's) speed: a cylinder placed far along
    // its own axis, a B-spline whose knots are large.
    double moved = 0.0;
    for (const int i : {0, kSamples / 2, kSamples - 1}) {
        const Vec2 uv = sample(h, i);
        const geo::SurfaceDerivatives d = s.derivatives(uv.x, uv.y);
        moved = std::max(moved, std::fabs(uv.x) * d.du.length() + std::fabs(uv.y) * d.dv.length());
    }
    if (h.edge != nullptr && h.edge->exactCurve) {
        const geo::Curve3& c = *h.edge->exactCurve;
        magnitude += reachOf(c);
        for (const double t : {h.edge->curveStart, h.edge->curveEnd}) {
            moved = std::max(moved, std::fabs(t) * c.derivatives(t).d1.length());
        }
    }
    const double rounding =
        kRoundingUlps * std::numeric_limits<double>::epsilon() * (magnitude + moved);
    return std::isfinite(rounding) ? rounding : 0.0;
}

struct Checker {
    const Solid& solid;
    const math::ModelTolerance& tol;
    std::vector<ExactIssue> issues;

    void report(ExactIssueKind kind, const Face* f, const Edge* e, std::string detail,
                const HalfEdge* h = nullptr) {
        issues.push_back({kind, f, e, std::move(detail), h});
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
        checkEdgeRange(e);
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

    /// A periodic curve's edge covers at most one period, plus its parametric tolerance (the
    /// effective tolerance over the curve's speed). Reported once per edge.
    void checkEdgeRange(const Edge& e) {
        if (!e.exactCurve || !e.exactCurve->periodic()) return;
        const geo::Interval range = e.exactCurve->range();
        const double period = range.max - range.min;
        if (!std::isfinite(period) || !(period > 0.0)) return;
        const double speed = e.exactCurve->derivatives(e.curveStart).d1.length();
        const double slack = speed > 0.0 ? tol.effective(e.tolerance) / speed : 0.0;
        const double span = std::fabs(e.curveEnd - e.curveStart);
        if (span > period + slack) {
            report(ExactIssueKind::RangeTooLong, nullptr, &e,
                   "curve range " + num(span) + " exceeds its period " + num(period));
        }
    }

    /// True when an unwrapped loop spans more than one period plus @p allowance (a distance) on
    /// a periodic axis; where the surface's derivative along it vanishes, nothing bounds it.
    static bool exceedsPeriod(const LoopPolygon& poly, const geo::Surface& s, double allowance,
                              double periodU, double periodV) {
        if (poly.empty()) return false;
        Vec2 lo = poly.front().a;
        Vec2 hi = lo;
        for (const LoopSegment& seg : poly) {
            for (const Vec2& p : {seg.a, seg.b}) {
                lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
                hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
            }
        }
        const auto d = s.derivatives(poly.front().a.x, poly.front().a.y);
        const auto over = [&](double extent, double period, double speed) {
            return period > 0.0 && speed > 0.0 && extent > period + allowance / speed;
        };
        return over(hi.x - lo.x, periodU, d.du.length()) ||
               over(hi.y - lo.y, periodV, d.dv.length());
    }

    void checkPcurve(const Edge& e, const HalfEdge& h) {
        std::vector<double> params;
        const PcurveGaps gaps = measurePcurve(e, h, tol, params);
        if (!tol.within(gaps.start, std::max(e.tolerance, h.origin->tolerance)) ||
            !tol.within(gaps.end, std::max(e.tolerance, h.next->origin->tolerance))) {
            report(ExactIssueKind::PcurveOffVertex, h.face, &e,
                   "pcurve ends are " + num(gaps.start) + " and " + num(gaps.end) +
                       " from their vertices",
                   &h);
        }
        if (params.empty()) return;  // no curve, or the segment's fast path accepts it
        // The ends are known: the vertices (checked above) sit at curveStart and curveEnd. Using
        // them, not projections, keeps a closed curve's end samples from projecting to the other
        // end of its range.
        const bool forward = &h == e.halfEdge;
        params.front() = forward ? e.curveStart : e.curveEnd;
        params.back() = forward ? e.curveEnd : e.curveStart;
        if (!tol.within(gaps.strays, e.tolerance)) {
            report(ExactIssueKind::PcurveOffCurve, h.face, &e,
                   "pcurve strays " + num(gaps.strays) + " from the edge's curve", &h);
        } else if (!advancesWithCurve(e, forward, params)) {
            report(ExactIssueKind::PcurveOffCurve, h.face, &e,
                   "pcurve runs against, or winds differently round, the edge's curve", &h);
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
    /// pole) every value of the parameter is the same point. Where the surface comes to a point
    /// between them (a pole or an apex, a pcurve ending on it or a hair off it, where the
    /// derivative at one end alone would part them), the run from one to the other is measured
    /// along the way, as unwrapLoop measures it (runsAlongPoint).
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
        if (close(a.x, b.x, d.du.length(), s.periodicU(), s.uRange()) &&
            close(a.y, b.y, d.dv.length(), s.periodicV(), s.vRange())) {
            return true;
        }
        return runsAlongPoint(s, a, b, allowance);
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
        if (!unwrapLoop(ring, kSamples, s, uPeriod(s), vPeriod(s), allowances, info.poly,
                        loopSense(f, outer))) {
            report(ExactIssueKind::LoopNotClosed, &f, w->halfEdge->edge,
                   "loop closes only after winding round the surface");
            return info;
        }
        info.usable = true;
        if (exceedsPeriod(info.poly, s, *std::max_element(allowances.begin(), allowances.end()),
                          uPeriod(s), vPeriod(s))) {
            report(ExactIssueKind::RangeTooLong, &f, w->halfEdge->edge,
                   "loop spans more than one period of the surface");
        }
        const double area = signedArea(info.poly);
        // Outer loops run counter-clockwise unless the face is reversed; inner loops the
        // other way.
        const bool wantPositive = outer != f.reversed;
        if ((area > 0.0) != wantPositive) {
            report(ExactIssueKind::LoopWrongOrientation, &f, w->halfEdge->edge,
                   "loop has signed area " + num(area) + " in (u, v)");
        }
        if (detail::LoopIndex(info.poly).selfIntersects(uvTolerance(s))) {
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
        // Each usable loop indexed once, for the pairs below (the answers are the plain
        // functions'; only what cannot cross or contain is skipped).
        std::vector<std::unique_ptr<detail::LoopIndex>> index(loops.size());
        for (size_t k = 0; k < loops.size(); ++k) {
            if (loops[k].usable) index[k] = std::make_unique<detail::LoopIndex>(loops[k].poly);
        }
        for (size_t i = 0; i < loops.size(); ++i) {
            if (!loops[i].usable) continue;
            for (size_t j = i + 1; j < loops.size(); ++j) {
                if (!loops[j].usable) continue;
                const bool crossing = index[i]->crosses(*index[j], uvTol);
                if (crossing) {
                    report(ExactIssueKind::LoopsCross, &f, loops[j].wire->halfEdge->edge,
                           "two loops of the face cross");
                    continue;
                }
                if (i == 0) {
                    const bool outside = std::any_of(
                        loops[j].poly.begin(), loops[j].poly.end(), [&](const LoopSegment& seg) {
                            return index[0]->clearly(math::Containment::Outside, seg.a);
                        });
                    if (outside) {
                        report(ExactIssueKind::HoleOutsideOuterLoop, &f,
                               loops[j].wire->halfEdge->edge,
                               "inner loop lies outside the outer loop");
                    }
                    continue;
                }
                // Two holes that do not cross are apart or nested; the mesher needs them apart.
                if (inside(loops[j].poly, *index[i]) || inside(loops[i].poly, *index[j])) {
                    report(ExactIssueKind::HoleInsideHole, &f, loops[j].wire->halfEdge->edge,
                           "inner loops " + std::to_string(i - 1) + " and " +
                               std::to_string(j - 1) + " lie one inside the other");
                }
            }
        }
    }

    /// True when a sample of @p loop lies clearly inside @p around (beyond each of its chords'
    /// sag): for loops that do not cross, the whole of @p loop is then inside.
    static bool inside(const LoopPolygon& loop, detail::LoopIndex& around) {
        return std::any_of(loop.begin(), loop.end(), [&](const LoopSegment& seg) {
            return around.clearly(math::Containment::Inside, seg.a);
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

PcurveGaps pcurveGaps(const HalfEdge& h, const math::ModelTolerance& tolerance) {
    if (!h.pcurve || h.edge == nullptr || h.face == nullptr || !h.face->exactSurface) return {};
    std::vector<double> params;
    return measurePcurve(*h.edge, h, tolerance, params);
}

double pcurveRounding(const Solid& solid, const HalfEdge& h) {
    return roundingAt(h, reachOf(solid));
}

RoundingTakeUp roundingShortfalls(const Solid& solid, const std::vector<ExactIssue>& issues,
                                  const math::ModelTolerance& tolerance) {
    RoundingTakeUp out;
    double solidReach = 0.0;
    bool reached = false;
    const auto refuse = [&out](const ExactIssue& issue) {
        out.shortfalls.clear();
        out.beyond = &issue;
        return out;
    };
    for (const ExactIssue& issue : issues) {
        const bool offCurve = issue.kind == ExactIssueKind::PcurveOffCurve;
        if ((!offCurve && issue.kind != ExactIssueKind::PcurveOffVertex) ||
            issue.halfEdge == nullptr || issue.halfEdge->edge == nullptr) {
            return refuse(issue);
        }
        const HalfEdge& h = *issue.halfEdge;
        Edge& e = *h.edge;
        const PcurveGaps gaps = pcurveGaps(h, tolerance);
        if (!reached) {  // once, at the first issue measured
            solidReach = reachOf(solid);
            reached = true;
        }
        const double rounding = roundingAt(h, solidReach);
        // What the check compared, with what, and the tolerance that bound it: an end's is its
        // vertex's where that is the larger, so the vertex's grows (by rounding), not the edge's.
        struct Compared {
            double measured;
            double* bound;
        };
        std::vector<Compared> compared;
        if (offCurve) {
            compared.push_back({gaps.strays, &e.tolerance});
        } else {
            for (const auto& end : {std::pair<double, Vertex*>{gaps.start, h.origin},
                                    std::pair<double, Vertex*>{gaps.end, h.next->origin}}) {
                compared.push_back({end.first, end.second->tolerance > e.tolerance
                                                   ? &end.second->tolerance
                                                   : &e.tolerance});
            }
        }
        bool explained = false;
        for (const Compared& c : compared) {
            const double allowed = tolerance.effective(*c.bound);
            if (c.measured <= allowed) continue;
            if (!(c.measured - allowed <= rounding)) return refuse(issue);  // NaN too
            out.shortfalls.push_back({c.bound, c.measured, &issue});
            explained = true;
        }
        // An issue of these kinds whose measures are within (a pcurve running against its curve).
        if (!explained) return refuse(issue);
    }
    return out;
}

}  // namespace hz::topo
