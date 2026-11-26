#include "horizon/modeling/Pattern.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ExactMass.h"
#include "horizon/geometry/Pcurve.h"
#include "horizon/geometry/curves/Arc2D.h"
#include "horizon/geometry/curves/Circle2D.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/curves/Nurbs2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Quaternion.h"
#include "horizon/math/RTree.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/ExactValidator.h"
#include "horizon/topology/LoopUnwrap.h"
#include "horizon/topology/Queries.h"

namespace hz::model {

using hz::math::BoundingBox;
using hz::math::Mat4;
using hz::math::Quaternion;
using hz::math::Vec3;
using namespace hz::topo;

namespace {

std::shared_ptr<geo::NurbsCurve> transformCurve(const geo::NurbsCurve& c, const Mat4& xform) {
    std::vector<Vec3> cp;
    cp.reserve(c.controlPoints().size());
    for (const auto& p : c.controlPoints()) cp.push_back(xform.transformPoint(p));
    return std::make_shared<geo::NurbsCurve>(std::move(cp), c.weights(), c.knots(), c.degree());
}

std::shared_ptr<geo::NurbsSurface> transformSurface(const geo::NurbsSurface& s, const Mat4& xform) {
    std::vector<std::vector<Vec3>> cp;
    cp.reserve(s.controlPoints().size());
    for (const auto& row : s.controlPoints()) {
        std::vector<Vec3> nrow;
        nrow.reserve(row.size());
        for (const auto& p : row) nrow.push_back(xform.transformPoint(p));
        cp.push_back(std::move(nrow));
    }
    return std::make_shared<geo::NurbsSurface>(std::move(cp), s.weights(), s.knotsU(), s.knotsV(),
                                               s.degreeU(), s.degreeV());
}

/// The uniform scale of @p xform when its upper-left 3x3 block M is a similarity (MtM = s^2 I,
/// reflection allowed), else 0. This is a property of a matrix, not a model distance, so it is
/// checked against a relative bound: every entry of MtM - s^2 I within 1e-9 of s^2.
double similarityScale(const Mat4& xform) {
    double mtm[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mtm[i][j] = 0.0;
            for (int k = 0; k < 3; ++k) mtm[i][j] += xform.m[k][i] * xform.m[k][j];
        }
    }
    const double s2 = (mtm[0][0] + mtm[1][1] + mtm[2][2]) / 3.0;
    if (!(s2 > 0.0) || !std::isfinite(s2)) return 0.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const double want = i == j ? s2 : 0.0;
            if (std::abs(mtm[i][j] - want) > 1e-9 * s2) return 0.0;
        }
    }
    return std::sqrt(s2);
}

TopologyID instanceId(const TopologyID& original, int instanceIndex) {
    if (instanceIndex == 0) return original;  // seed keeps identity
    if (!original.isValid()) return original;
    return original.child("pattern", instanceIndex);
}

/// Which way a solid's loops are wound, all of them: +1 when they face out of
/// its material, -1 when in (some builders wind every loop inward: a box
/// primitive does, a cylinder and the sewer outward), 0 for no faces. Six
/// times the volume they enclose, about a point of the solid's own (about the
/// origin, rounding swamped a small part far from it).
int windingOf(const Solid& solid) {
    const Vec3* origin = nullptr;
    double sixVolume = 0.0;
    const auto loop = [&](const Wire* wire) {
        if (!wire || !wire->halfEdge || !wire->halfEdge->origin) return;
        const HalfEdge* start = wire->halfEdge;
        if (!origin) origin = &start->origin->point;
        const Vec3 p0 = start->origin->point - *origin;
        for (const HalfEdge* he = start->next; he && he->next && he->next != start; he = he->next) {
            if (!he->origin || !he->next->origin) return;
            sixVolume +=
                p0.dot((he->origin->point - *origin).cross(he->next->origin->point - *origin));
        }
    };
    for (const auto& face : solid.faces()) {
        loop(face.outerLoop);
        for (const Wire* inner : face.innerLoops) loop(inner);
    }
    return sixVolume > 0.0 ? 1 : sixVolume < 0.0 ? -1 : 0;
}

/// The most @p xform's upper-left 3x3 block M stretches a length: its largest singular value,
/// the square root of MtM's largest eigenvalue (the closed form for a symmetric 3x3 matrix).
double largestStretch(const Mat4& xform) {
    double a[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            a[i][j] = 0.0;
            for (int k = 0; k < 3; ++k) a[i][j] += xform.m[k][i] * xform.m[k][j];
        }
    }
    const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
    double largest = std::max({a[0][0], a[1][1], a[2][2]});
    if (off > 0.0) {
        const double q = (a[0][0] + a[1][1] + a[2][2]) / 3.0;
        const double p = std::sqrt(((a[0][0] - q) * (a[0][0] - q) + (a[1][1] - q) * (a[1][1] - q) +
                                    (a[2][2] - q) * (a[2][2] - q) + 2.0 * off) /
                                   6.0);
        double b[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) b[i][j] = (a[i][j] - (i == j ? q : 0.0)) / p;
        }
        const double det = b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1]) -
                           b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0]) +
                           b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0]);
        largest = q + 2.0 * p * std::cos(std::acos(std::clamp(det / 2.0, -1.0, 1.0)) / 3.0);
    }
    return std::sqrt(std::max(largest, 0.0));
}

/// A length for a problem, to three figures.
std::string threeFigures(double x) {
    std::ostringstream out;
    out << std::setprecision(3) << x;
    return out.str();
}

/// @p c moved by @p shift in (u, v): a seam's other side, or a pcurve laid a period on. Null for
/// a kind of pcurve it does not know.
std::shared_ptr<const geo::Curve2D> shiftedPcurve(const geo::Curve2D& c, const math::Vec2& shift) {
    if (const auto* line = dynamic_cast<const geo::Line2D*>(&c)) {
        return std::make_shared<geo::Line2D>(line->start() + shift, line->end() + shift);
    }
    if (const auto* arc = dynamic_cast<const geo::Arc2D*>(&c)) {
        return std::make_shared<geo::Arc2D>(arc->center() + shift, arc->radius(), arc->startAngle(),
                                            arc->endAngle());
    }
    if (const auto* circle = dynamic_cast<const geo::Circle2D*>(&c)) {
        return std::make_shared<geo::Circle2D>(circle->center() + shift, circle->radius());
    }
    if (const auto* spline = dynamic_cast<const geo::Nurbs2D*>(&c)) {
        const geo::NurbsCurve& n = spline->curve();
        std::vector<math::Vec2> points;
        points.reserve(n.controlPoints().size());
        for (const Vec3& q : n.controlPoints()) points.push_back(math::Vec2(q.x, q.y) + shift);
        return std::make_shared<geo::Nurbs2D>(std::move(points), n.weights(), n.knots(),
                                              n.degree());
    }
    return nullptr;
}

/// The span a face's NURBS covers along one direction, from [@p lo, @p hi], the extent of the
/// face's pcurves there (unwrapped): a whole period where they span one, so the NURBS closes
/// where the typed surface does; otherwise a margin beside them each way (a pcurve may bulge
/// between the samples that measured it), at most to a whole period, held to the surface's own
/// @p range and, where @p stop is set (a cone's apex), not past it on the side the face lies.
std::pair<double, double> spanOf(double lo, double hi, double period, geo::Interval range,
                                 std::optional<double> stop) {
    const double width = hi - lo;
    if (period > 0.0) {
        if (width >= period * (1.0 - 1e-9)) return {lo, lo + period};
        const double margin = std::min(0.05 * width, 0.5 * (period - width));
        return {lo - margin, hi + margin};
    }
    const double margin = 0.1 * width;
    double from = std::max(lo - margin, range.min);
    double to = std::min(hi + margin, range.max);
    if (stop) {
        const double near = 1e-9 * width;
        if (*stop <= lo + near) from = std::max(from, *stop);
        if (*stop >= hi - near) to = std::min(to, *stop);
    }
    return {from, to};
}

/// Carries a solid's exact geometry through a matrix that is not a similarity, for
/// Pattern::transformed. A NURBS surface or curve is moved as it is: a NURBS follows any affine
/// matrix exactly and keeps its parameters, so its pcurves still hold. A typed one becomes the
/// NURBS it is over its face's box or its edge's range (Surface::toNurbs, Curve3::toNurbs), and
/// that is moved. Each pcurve on a face whose surface was so made is refitted with geo::pcurveOn
/// as the STEP reader fits one, and laid in the period of the one it replaces; each tolerance is
/// scaled by the most the matrix stretches a length, then grows to cover what is measured as the
/// reader's do (topo::ToleranceGrowth); and the copy is validated. The first thing that cannot
/// be done is the problem, and the copy then keeps no exact geometry. For a copy into a solid of
/// its own (finish() validates the whole of it).
class NonSimilarCarry {
public:
    NonSimilarCarry(const Solid& src, const Mat4& xform)
        : xform_(xform),
          srcTolerance_(topo::exactTolerance(src)),
          loopLimit_(2 * src.edges().size() + 1),
          stretch_(largestStretch(xform)) {
        const double det = xform.determinant3();
        if (!(std::abs(det) > 1e-12 * stretch_ * stretch_ * stretch_) || !std::isfinite(det)) {
            fail("the matrix flattens the solid (its determinant is " + threeFigures(det) + ")");
        }
    }

    /// Why the copy cannot keep its exact geometry; empty while it can.
    const std::string& problem() const { return problem_; }

    /// The most the matrix stretches a length: what each tolerance is scaled by.
    double stretch() const { return stretch_; }

    /// @p e's exact curve moved; null (and the problem said) where it cannot be.
    std::shared_ptr<const geo::Curve3> curve(const Edge& e) {
        if (!problem_.empty()) return nullptr;
        const geo::Curve3& c = *e.exactCurve;
        try {
            if (c.kind() == geo::CurveKind::Nurbs) return c.transformed(xform_);
            const auto nurbs =
                c.toNurbs(std::min(e.curveStart, e.curveEnd), std::max(e.curveStart, e.curveEnd));
            if (nurbs == nullptr) {
                fail("an edge's curve has no NURBS form");
                return nullptr;
            }
            return nurbs->transformed(xform_);
        } catch (const std::invalid_argument& ex) {
            fail(std::string("an edge's curve could not be made a NURBS: ") + ex.what());
            return nullptr;
        }
    }

    /// @p f's exact surface moved; null (and the problem said) where it cannot be.
    std::shared_ptr<const geo::Surface> surface(const Face& f) {
        if (!problem_.empty()) return nullptr;
        const geo::Surface& s = *f.exactSurface;
        try {
            if (s.kind() == geo::SurfaceKind::Nurbs) return s.transformed(xform_);
            const auto box = boxOf(f);
            if (!box) return nullptr;
            std::shared_ptr<const geo::Surface> moved =
                s.toNurbs(box->u0, box->u1, box->v0, box->v1)->transformed(xform_);
            converted_.insert(&f);
            return moved;
        } catch (const std::invalid_argument& ex) {
            fail(std::string("a face's surface could not be made a NURBS: ") + ex.what());
            return nullptr;
        }
    }

    /// Once @p dst holds the whole copy, its exact fields set by curve() and surface(): refits
    /// the pcurves on each face whose surface was made a NURBS (@p halfEdges pairs each source
    /// half-edge with its copy, @p faces each source face with its), grows the tolerances, and
    /// validates the copy.
    void finish(Solid& dst, const std::vector<std::pair<const HalfEdge*, HalfEdge*>>& halfEdges,
                const std::unordered_map<const void*, Face*>& faces) {
        if (!problem_.empty()) return;
        const topo::ToleranceGrowth growth(topo::exactExtent(dst));
        const std::string limitSays = ", beyond the limit of " + threeFigures(growth.limit);
        const auto grow = [&](double& tolerance, double gap, const char* what) {
            if (!growth.grow(tolerance, gap)) fail(what + threeFigures(gap) + limitSays);
        };
        // Each edge's curve ends at its vertices.
        for (Edge& e : dst.edges()) {
            if (!e.alive || !e.exactCurve || e.halfEdge == nullptr || e.halfEdge->twin == nullptr) {
                continue;
            }
            Vertex* ends[2] = {e.halfEdge->origin, e.halfEdge->twin->origin};
            const double at[2] = {e.curveStart, e.curveEnd};
            for (int k = 0; k < 2; ++k) {
                const double gap = (e.exactCurve->evaluate(at[k]) - ends[k]->point).length();
                grow(e.tolerance, gap, "an edge's curve ends ");
                grow(ends[k]->tolerance, gap, "an edge's curve ends ");
            }
        }
        std::unordered_set<const Face*> refit;
        for (const Face* f : converted_) {
            const auto it = faces.find(f);
            if (it != faces.end()) refit.insert(it->second);
        }
        std::unordered_set<const HalfEdge*> fitted;
        for (const auto& pair : halfEdges) {
            if (!problem_.empty()) return;
            HalfEdge& h = *pair.second;
            if (refit.count(h.face) == 0) continue;
            Edge& e = *h.edge;
            if (!e.exactCurve) {
                fail("an edge of a face has no exact curve");
                return;
            }
            if (h.twin != nullptr && h.twin->face == h.face && fitted.count(h.twin) > 0) {
                // A seam's other side: the same course, the other way.
                h.pcurve = h.twin->pcurve;
                h.pcurveStart = h.twin->pcurveEnd;
                h.pcurveEnd = h.twin->pcurveStart;
            } else {
                const bool forward = &h == e.halfEdge;
                const double t0 = forward ? e.curveStart : e.curveEnd;
                const double t1 = forward ? e.curveEnd : e.curveStart;
                geo::Pcurve pc = geo::pcurveOn(*h.face->exactSurface, *e.exactCurve, t0, t1,
                                               growth.resolution.linear, growth.limit);
                if (!pc.problem.empty() || pc.curve == nullptr) {
                    fail("a pcurve could not be refitted to the moved surface: " + pc.problem);
                    return;
                }
                h.pcurve = std::move(pc.curve);
                h.pcurveStart = pc.start;
                h.pcurveEnd = pc.end;
                grow(e.tolerance, pc.deviation, "a refitted pcurve strays ");
            }
            fitted.insert(&h);
            if (!layInPeriod(*pair.first, h)) return;
            const geo::Surface& s = *h.face->exactSurface;
            const math::Vec2 a = h.pcurve->evaluate(h.pcurveStart);
            const math::Vec2 b = h.pcurve->evaluate(h.pcurveEnd);
            grow(e.tolerance, (s.evaluate(a.x, a.y) - h.origin->point).length(),
                 "a refitted pcurve ends ");
            grow(e.tolerance, (s.evaluate(b.x, b.y) - h.next->origin->point).length(),
                 "a refitted pcurve ends ");
        }
        if (!problem_.empty()) return;
        // As the reader does (Rulings 64 and 66): where every issue is a shortfall by rounding
        // alone, each tolerance grows to the validator's own measure and the copy is validated
        // again (the checks those tolerances enter); otherwise the first issue is the problem.
        auto issues = topo::validateExact(dst, growth.resolution);
        if (!issues.empty()) {
            const topo::RoundingTakeUp takeUp =
                topo::roundingShortfalls(dst, issues, growth.resolution);
            if (takeUp.beyond != nullptr) {
                fail(takeUp.beyond->detail);
                return;
            }
            for (const topo::RoundingShortfall& s : takeUp.shortfalls) {
                grow(*s.tolerance, s.measured, "a refitted pcurve strays ");
            }
            if (!problem_.empty()) return;
            std::vector<topo::ExactIssue> again =
                topo::validateTakenUp(dst, growth.resolution, takeUp);
            issues = std::move(again);  // takeUp's issues are these, until now
        }
        if (!issues.empty()) fail(issues.front().detail);
    }

private:
    struct Box {
        double u0, u1, v0, v1;
    };

    void fail(std::string why) {
        if (problem_.empty()) problem_ = std::move(why);
    }

    /// The box in (u, v) that @p f's NURBS spans: its loops unwrapped as the validator unwraps
    /// them (each into the outer loop's window), their extent grown by spanOf. Records each
    /// half-edge's shift in that window, for layInPeriod.
    std::optional<Box> boxOf(const Face& f) {
        const geo::Surface& s = *f.exactSurface;
        const double pu = topo::uPeriod(s);
        const double pv = topo::vPeriod(s);
        std::vector<const Wire*> loops{f.outerLoop};
        loops.insert(loops.end(), f.innerLoops.begin(), f.innerLoops.end());
        constexpr double kInf = std::numeric_limits<double>::infinity();
        math::Vec2 lo(kInf, kInf);
        math::Vec2 hi(-kInf, -kInf);
        topo::LoopPolygon reference;
        std::vector<const HalfEdge*> ring;
        const std::size_t perEdge = topo::kLoopSamples - 1;
        for (const Wire* w : loops) {
            if (w == nullptr || !topo::loopHalfEdges(f, *w, loopLimit_, ring)) {
                fail("a loop of a face does not close");
                return std::nullopt;
            }
            std::vector<double> allowances;
            for (const HalfEdge* h : ring) {
                if (!h->pcurve || h->edge == nullptr || h->next == nullptr) {
                    fail("a half-edge of a face has no pcurve");
                    return std::nullopt;
                }
                allowances.push_back(topo::joinAllowance(*h, srcTolerance_));
            }
            topo::LoopPolygon poly;
            if (!topo::unwrapLoop(ring, topo::kLoopSamples, s, pu, pv, allowances, poly,
                                  topo::loopSense(f, w == f.outerLoop))) {
                fail("a loop of a face closes only after winding round its surface");
                return std::nullopt;
            }
            if (reference.empty()) {
                reference = poly;
            } else {
                topo::alignLoop(poly, reference, pu, pv);
            }
            for (std::size_t k = 0; k < ring.size(); ++k) {
                unwrapShift_[ring[k]] = poly[k * perEdge].shift;
            }
            for (const topo::LoopSegment& seg : poly) {
                for (int i = 0; i <= 4; ++i) {
                    const math::Vec2 q =
                        seg.curve->evaluate(seg.t0 + (seg.t1 - seg.t0) * i / 4) + seg.shift;
                    lo = {std::min(lo.x, q.x), std::min(lo.y, q.y)};
                    hi = {std::max(hi.x, q.x), std::max(hi.y, q.y)};
                }
            }
        }
        // A cone's NURBS stops at its apex, where the face reaches it: past it, the patch would
        // fold through onto the other nappe.
        std::optional<double> apex;
        if (const auto* cone = dynamic_cast<const geo::Cone*>(&s)) {
            const double slope = cone->axialScale() * std::tan(cone->halfAngle());
            if (slope != 0.0) apex = -cone->radius() / slope;
        }
        const auto u = spanOf(lo.x, hi.x, pu, s.uRange(), std::nullopt);
        const auto v = spanOf(lo.y, hi.y, pv, s.vRange(), apex);
        return Box{u.first, u.second, v.first, v.second};
    }

    /// Lays @p h's refitted pcurve in the period of @p was, the source half-edge it replaces: on
    /// a NURBS that closes, pcurveOn starts each in the domain, and a seam's two sides, or a
    /// loop through a pole, need the periods the original loop had. The NURBS spans the face's
    /// box with its knots where the typed surface's parameters are, so the two are a whole
    /// number of periods apart, to well within half of one.
    bool layInPeriod(const HalfEdge& was, HalfEdge& h) {
        const geo::Surface& s = *h.face->exactSurface;
        const double pu = topo::uPeriod(s);
        const double pv = topo::vPeriod(s);
        if (pu == 0.0 && pv == 0.0) return true;
        const auto mid = [](const HalfEdge& x) {
            return x.pcurve->evaluate(0.5 * (x.pcurveStart + x.pcurveEnd));
        };
        const auto shift = unwrapShift_.find(&was);
        const math::Vec2 gap =
            mid(was) + (shift == unwrapShift_.end() ? math::Vec2() : shift->second) - mid(h);
        const math::Vec2 move(pu > 0.0 ? std::round(gap.x / pu) * pu : 0.0,
                              pv > 0.0 ? std::round(gap.y / pv) * pv : 0.0);
        if (move.x == 0.0 && move.y == 0.0) return true;
        auto moved = shiftedPcurve(*h.pcurve, move);
        if (moved == nullptr) {
            fail("a refitted pcurve could not be laid a period on");
            return false;
        }
        h.pcurve = std::move(moved);
        return true;
    }

    const Mat4& xform_;
    math::ModelTolerance srcTolerance_;
    std::size_t loopLimit_;  ///< the most half-edges a loop of the source can have
    double stretch_;
    std::string problem_;
    std::unordered_set<const Face*> converted_;                    ///< source faces made NURBS
    std::unordered_map<const HalfEdge*, math::Vec2> unwrapShift_;  ///< source half-edges' shifts
};

// Deep-clone src — or, given `only`, just those of its shells — into dst with
// a transform, as a new set of shells. A transform that mirrors (its
// determinant negative, Phase 162) would turn every loop inside out: each is
// reversed as it is copied, and each surface with it, so the copy faces out
// as the original does.
//
// With @p outward, the copy is wound facing out, whichever way src is: for a
// solid gathered from several (collect, append, a pattern's bodies). Every
// computation that reads a solid's loops (mass properties, the Booleans,
// which shells are cavities) takes one sign for all of them, so a part of an
// inward-wound box and an outward-wound cylinder measured the box less the
// cylinder, and took the cylinder for a cavity in it. A solid with exact faces
// faces out as they say (Face::reversed), and is copied as it is: its corners,
// joined by straight lines, need not wind as its faces do (a C-shaped clip's
// enclose a negative volume), and turned round for them it was inside out.
//
// Exact geometry rides along through a similarity, moved as it is (Ruling 9). Through any other
// matrix it rides along only where @p nonSimilarProblem is given (transformed()): as the NURBS
// it is, by NonSimilarCarry, which says there why it cannot; otherwise the copy's exact fields
// are left empty.
void cloneInto(Solid& dst, const Solid& src, const Mat4& xform, int instanceIndex,
               const std::vector<const Shell*>* only = nullptr, bool outward = false,
               std::string* nonSimilarProblem = nullptr) {
    const bool mirrors = xform.determinant3() < 0.0;
    // A mirrored copy keeps src's winding (see above), so src says it.
    const bool flip = mirrors != (outward && !topo::hasExactFaces(src) && windingOf(src) < 0);
    const auto moveSurface = [&xform, flip](const geo::NurbsSurface& surface) {
        auto moved = transformSurface(surface, xform);
        return flip ? std::make_shared<geo::NurbsSurface>(moved->reversedU()) : moved;
    };
    // A shared exact surface or curve is moved once through a similarity.
    const double scale = similarityScale(xform);
    const bool similar = scale > 0.0;
    std::optional<NonSimilarCarry> carry;
    if (!similar && nonSimilarProblem != nullptr && topo::hasExactFaces(src)) {
        carry.emplace(src, xform);
    }
    const bool carryExact = similar || (carry && carry->problem().empty());
    const double toleranceScale = similar ? scale : carry ? carry->stretch() : 0.0;
    std::unordered_map<const geo::Surface*, std::shared_ptr<const geo::Surface>> exactSurfaces;
    std::unordered_map<const geo::Curve3*, std::shared_ptr<const geo::Curve3>> exactCurves;
    std::unordered_map<const void*, Vertex*> vmap;
    std::unordered_map<const void*, HalfEdge*> hmap;
    std::unordered_map<const void*, Edge*> emap;
    std::unordered_map<const void*, Wire*> wmap;
    std::unordered_map<const void*, Face*> fmap;
    std::unordered_map<const void*, Shell*> smap;
    const auto mapped = [](const auto& map, const void* key) {
        using Target = typename std::decay_t<decltype(map)>::mapped_type;
        const auto it = map.find(key);
        return it == map.end() ? Target{nullptr} : it->second;
    };
    // An ideal shared by many facets or chords (one cylinder, one circle) is
    // moved once, and stays shared: that sharing is what says they are one
    // face or one curve (drawings, STEP export as designed). Moved one by
    // one, each facet had a cylinder of its own.
    std::unordered_map<const void*, std::shared_ptr<geo::NurbsSurface>> idealSurfaces;
    std::unordered_map<const void*, std::shared_ptr<geo::NurbsCurve>> idealCurves;
    const auto idealSurface = [&](const std::shared_ptr<geo::NurbsSurface>& ideal) {
        auto& moved = idealSurfaces[ideal.get()];
        if (!moved) moved = moveSurface(*ideal);
        return moved;
    };
    const auto idealCurve = [&](const std::shared_ptr<geo::NurbsCurve>& ideal) {
        auto& moved = idealCurves[ideal.get()];
        if (!moved) moved = transformCurve(*ideal, xform);
        return moved;
    };

    // What to copy: the faces, then the loops, half-edges, vertices and edges
    // they use.
    std::vector<const Face*> faces;
    if (only) {
        for (const Shell* shell : *only)
            faces.insert(faces.end(), shell->faces.begin(), shell->faces.end());
    } else {
        for (const auto& f : src.faces()) faces.push_back(&f);
    }
    std::vector<const Wire*> wires;
    std::vector<const HalfEdge*> halfEdges;
    std::unordered_set<const void*> seenWire, seenHE;
    auto visitWire = [&](const Wire* w) {
        if (!w || !seenWire.insert(w).second) return;
        wires.push_back(w);
        const HalfEdge* start = w->halfEdge;
        const HalfEdge* cur = start;
        do {
            if (cur && seenHE.insert(cur).second) halfEdges.push_back(cur);
            cur = cur ? cur->next : nullptr;
        } while (cur && cur != start);
    };
    for (const Face* f : faces) {
        visitWire(f->outerLoop);
        for (const Wire* inner : f->innerLoops) visitWire(inner);
    }
    std::unordered_set<const void*> usedVertices, usedEdges;
    for (const HalfEdge* h : halfEdges) {
        usedVertices.insert(h->origin);
        usedEdges.insert(h->edge);
    }
    const auto copies = [only](const std::unordered_set<const void*>& used, const void* entity) {
        return only == nullptr || used.count(entity) > 0;
    };

    // Vertices.
    for (const auto& v : src.vertices()) {
        if (!copies(usedVertices, &v)) continue;
        Vertex* nv = dst.allocVertex();
        nv->point = xform.transformPoint(v.point);
        nv->topoId = instanceId(v.topoId, instanceIndex);
        if (carryExact) nv->tolerance = v.tolerance * toleranceScale;
        vmap[&v] = nv;
    }

    // Edges (with transformed curves).
    for (const auto& e : src.edges()) {
        if (!copies(usedEdges, &e)) continue;
        Edge* ne = dst.allocEdge();
        ne->topoId = instanceId(e.topoId, instanceIndex);
        if (e.curve) ne->curve = transformCurve(*e.curve, xform);
        if (e.analyticCurve) ne->analyticCurve = idealCurve(e.analyticCurve);
        if (carryExact && e.exactCurve) {
            if (similar) {
                auto& moved = exactCurves[e.exactCurve.get()];
                if (!moved) moved = e.exactCurve->transformed(xform);
                ne->exactCurve = moved;
            } else {
                ne->exactCurve = carry->curve(e);
            }
            ne->curveStart = e.curveStart;
            ne->curveEnd = e.curveEnd;
        }
        if (carryExact) ne->tolerance = e.tolerance * toleranceScale;
        emap[&e] = ne;
    }

    // Shells.
    for (const auto& s : src.shells()) {
        if (only && std::find(only->begin(), only->end(), &s) == only->end()) continue;
        Shell* ns = dst.allocShell();
        ns->solid = &dst;
        smap[&s] = ns;
    }

    // Faces (with transformed surfaces).
    for (const Face* f : faces) {
        Face* nf = dst.allocFace();
        nf->topoId = instanceId(f->topoId, instanceIndex);
        if (f->surface) nf->surface = moveSurface(*f->surface);
        // Each instance of a faceted boss must still resolve to its own
        // cylinder, so the ideal is carried and moved with the facets.
        if (f->analyticSurface) nf->analyticSurface = idealSurface(f->analyticSurface);
        if (carryExact && f->exactSurface) {
            if (similar) {
                auto& moved = exactSurfaces[f->exactSurface.get()];
                if (!moved) moved = f->exactSurface->transformed(xform);
                nf->exactSurface = moved;
            } else {
                nf->exactSurface = carry->surface(*f);
            }
            // The loops are reversed with `flip`, so their sense in (u, v) flips with it.
            nf->reversed = f->reversed != flip;
        }
        fmap[f] = nf;
    }

    for (const Wire* w : wires) wmap[w] = dst.allocWire();
    for (const HalfEdge* h : halfEdges) hmap[h] = dst.allocHalfEdge();
    std::vector<std::pair<const HalfEdge*, HalfEdge*>> carried;  // for NonSimilarCarry::finish

    // Remap all cross-references. (Within one shell of a closed solid every
    // reference stays inside the shell; one that does not is left null.)
    // Turned round (mirrored, or wound against dst), each half-edge runs the
    // other way along its edge: from its old end, its old prev after it. A
    // vertex then leaves by the half-edge that came into it, and an edge's
    // curve, which runs from its half-edge's origin, keeps its direction by
    // taking the old twin.
    for (const auto& v : src.vertices()) {
        Vertex* nv = mapped(vmap, &v);
        if (nv == nullptr) continue;
        const HalfEdge* out = v.halfEdge;
        nv->halfEdge = mapped(hmap, flip && out != nullptr ? out->prev : out);
    }
    for (const HalfEdge* h : halfEdges) {
        HalfEdge* nh = hmap[h];
        nh->origin = mapped(vmap, flip && h->next != nullptr ? h->next->origin : h->origin);
        nh->twin = mapped(hmap, h->twin);
        nh->next = mapped(hmap, flip ? h->prev : h->next);
        nh->prev = mapped(hmap, flip ? h->next : h->prev);
        nh->edge = mapped(emap, h->edge);
        nh->face = mapped(fmap, h->face);
        if (carryExact && h->pcurve) {
            // The pcurve keeps its (u, v) values; a half-edge turned round runs it backwards.
            nh->pcurve = h->pcurve;
            nh->pcurveStart = flip ? h->pcurveEnd : h->pcurveStart;
            nh->pcurveEnd = flip ? h->pcurveStart : h->pcurveEnd;
        }
        if (carry) carried.emplace_back(h, nh);
    }
    for (const auto& e : src.edges()) {
        Edge* ne = mapped(emap, &e);
        if (ne == nullptr) continue;
        const HalfEdge* along = e.halfEdge;
        if (flip && along != nullptr && along->twin != nullptr) along = along->twin;
        ne->halfEdge = mapped(hmap, along);
        // Taking the old twin keeps the edge's origin, so its curve keeps its direction; only
        // an edge with no twin to take runs the other way.
        if (flip && e.halfEdge != nullptr && e.halfEdge->twin == nullptr)
            std::swap(ne->curveStart, ne->curveEnd);
    }
    for (const Wire* w : wires) {
        wmap[w]->halfEdge = mapped(hmap, w->halfEdge);
    }
    for (const Face* f : faces) {
        Face* nf = fmap[f];
        nf->outerLoop = mapped(wmap, f->outerLoop);
        nf->innerLoops.clear();
        for (const Wire* inner : f->innerLoops) nf->innerLoops.push_back(mapped(wmap, inner));
        nf->shell = mapped(smap, f->shell);
    }
    for (const auto& shell : src.shells()) {
        Shell* ns = mapped(smap, &shell);
        if (!ns) continue;
        ns->faces.clear();
        for (const Face* fp : shell.faces) ns->faces.push_back(mapped(fmap, fp));
    }

    if (!carry) return;
    if (carry->problem().empty()) carry->finish(dst, carried, fmap);
    *nonSimilarProblem = carry->problem();
    if (nonSimilarProblem->empty()) return;
    // What could not be carried is not carried at all.
    for (const auto& v : src.vertices()) {
        if (Vertex* nv = mapped(vmap, &v)) nv->tolerance = 0.0;
    }
    for (const auto& e : src.edges()) {
        Edge* ne = mapped(emap, &e);
        if (ne == nullptr) continue;
        ne->exactCurve = nullptr;
        ne->curveStart = 0.0;
        ne->curveEnd = 0.0;
        ne->tolerance = 0.0;
    }
    for (const Face* f : faces) {
        fmap[f]->exactSurface = nullptr;
        fmap[f]->reversed = false;
    }
    for (const HalfEdge* h : halfEdges) {
        hmap[h]->pcurve = nullptr;
        hmap[h]->pcurveStart = 0.0;
        hmap[h]->pcurveEnd = 0.0;
    }
}

// Visit the vertices of every loop of a face.
template <typename Visit>
void forEachLoopVertex(const Face& face, Visit&& visit) {
    const auto walk = [&visit](const Wire* wire) {
        if (!wire || !wire->halfEdge) return;
        const HalfEdge* start = wire->halfEdge;
        const HalfEdge* he = start;
        do {
            if (!he->origin) return;
            visit(he->origin->point);
            he = he->next;
        } while (he && he != start);
    };
    walk(face.outerLoop);
    for (const Wire* inner : face.innerLoops) walk(inner);
}

BoundingBox boundsOf(const Solid& solid) {
    BoundingBox box;
    for (const auto& v : solid.vertices()) box.expand(v.point);
    return box;
}

int findRoot(std::vector<int>& parent, int i) {
    while (parent[static_cast<size_t>(i)] != i) {
        parent[static_cast<size_t>(i)] =
            parent[static_cast<size_t>(parent[static_cast<size_t>(i)])];
        i = parent[static_cast<size_t>(i)];
    }
    return i;
}

/// Whether @p a and @p b move a point to the same place, to the last few
/// digits: the same copy twice.
bool samePlace(const Mat4& a, const Mat4& b) {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            const double x = a.m[r][c];
            const double y = b.m[r][c];
            if (std::abs(x - y) > 1e-12 * std::max({1.0, std::abs(x), std::abs(y)})) return false;
        }
    }
    return true;
}

std::unique_ptr<topo::Solid> buildPattern(const Solid& source, const std::vector<Mat4>& transforms,
                                          const std::vector<int>& suppressed, NamingScheme naming,
                                          bool together, std::string* reason) {
    // Its joins share one budget: 10,000 copies that all overlap, each join
    // well within its own limits, took hours.
    const BooleanSeries series;
    std::unordered_set<int> skip(suppressed.begin(), suppressed.end());
    if (together) {
        // A copy where another already is adds nothing to their union: a
        // pattern of no spacing, of thousands of copies, joined thousands
        // of times one body to the same body.
        std::vector<size_t> kept;
        for (size_t k = 0; k < transforms.size(); ++k) {
            if (skip.count(static_cast<int>(k))) continue;
            const bool again = std::any_of(kept.begin(), kept.end(), [&](size_t j) {
                return samePlace(transforms[j], transforms[k]);
            });
            if (again) {
                skip.insert(static_cast<int>(k));
            } else if (kept.size() < 64) {
                kept.push_back(k);  // a repeat is found among the first few
            }
        }
    }

    // Stopped as the build is cancelled: copying 40 instances of a cylinder
    // of 1,024 facets, and the tool of each round, took seconds in a debug
    // build, between the Booleans that hear a cancel.
    const auto stopped = [reason]() -> std::unique_ptr<Solid> {
        if (reason) *reason = "cancelled";
        return nullptr;
    };

    // Each instance as its own solid, so overlapping ones can be merged.
    std::vector<std::unique_ptr<Solid>> instances;
    std::vector<BoundingBox> bounds;
    for (size_t k = 0; k < transforms.size(); ++k) {
        if (skip.count(static_cast<int>(k))) continue;
        if (BooleanCancellation::requested()) return stopped();
        auto inst = std::make_unique<Solid>();
        cloneInto(*inst, source, transforms[k], static_cast<int>(k));
        bounds.push_back(boundsOf(*inst));
        instances.push_back(std::move(inst));
    }

    // Group instances whose bounds touch or overlap.  Instances that meet
    // must become one body: left as separate shells they would count shared
    // material once per instance and present faces inside the part.
    const size_t n = instances.size();
    std::vector<int> parent(n);
    for (size_t i = 0; i < n; ++i) parent[i] = static_cast<int>(i);
    for (size_t i = 0; i < n; ++i) {
        const Vec3 span = bounds[i].max() - bounds[i].min();
        const double pad = 1e-9 * std::max(1.0, span.length());
        BoundingBox grown(bounds[i].min() - Vec3(pad, pad, pad),
                          bounds[i].max() + Vec3(pad, pad, pad));
        for (size_t j = i + 1; j < n; ++j) {
            if (grown.intersects(bounds[j])) {
                parent[static_cast<size_t>(findRoot(parent, static_cast<int>(i)))] =
                    findRoot(parent, static_cast<int>(j));
            }
        }
    }

    // Union each group in instance order.  A group the Boolean cannot merge
    // is refused rather than returned as overlapping shells.
    std::vector<std::unique_ptr<Solid>> bodies(n);
    if (together) {
        // In rounds: each takes every instance left whose box is clear of
        // the others it takes, as one tool of separate skins, and joins them
        // to the group at once. One instance at a time, each join remade all
        // the group had grown to: a grid of 40 overlapping cylinders of
        // 4,096 facets took 37 minutes. A grid whose copies overlap their
        // neighbours takes four rounds; copies that all overlap take one
        // each, as before.
        std::vector<std::vector<size_t>> members(n);
        for (size_t i = 0; i < n; ++i) {
            members[static_cast<size_t>(findRoot(parent, static_cast<int>(i)))].push_back(i);
        }
        for (size_t root = 0; root < n; ++root) {
            if (members[root].empty()) continue;
            std::unique_ptr<Solid> group = std::move(instances[members[root].front()]);
            std::vector<size_t> left(members[root].begin() + 1, members[root].end());
            while (!left.empty()) {
                if (BooleanCancellation::requested()) return stopped();
                math::RTree<size_t> taken;
                std::vector<size_t> round;
                std::vector<size_t> later;
                for (const size_t i : left) {
                    const Vec3 span = bounds[i].max() - bounds[i].min();
                    const double pad = 1e-9 * std::max(1.0, span.length());
                    const BoundingBox grown(bounds[i].min() - Vec3(pad, pad, pad),
                                            bounds[i].max() + Vec3(pad, pad, pad));
                    if (taken.query(grown).empty()) {
                        taken.insert(i, grown);
                        round.push_back(i);
                    } else {
                        later.push_back(i);
                    }
                }
                std::unique_ptr<Solid> tool;
                if (round.size() == 1) {
                    tool = std::move(instances[round.front()]);
                } else {
                    tool = std::make_unique<Solid>();
                    for (const size_t i : round) {
                        cloneInto(*tool, *instances[i], Mat4::identity(), 0, nullptr, true);
                    }
                }
                auto merged = BooleanOp::execute(*group, *tool, BooleanType::Union, reason, naming);
                if (!merged) return nullptr;
                group = std::move(merged);
                left = std::move(later);
            }
            bodies[root] = std::move(group);
        }
    } else {
        for (size_t i = 0; i < n; ++i) {
            const auto root = static_cast<size_t>(findRoot(parent, static_cast<int>(i)));
            if (!bodies[root]) {
                bodies[root] = std::move(instances[i]);
                continue;
            }
            auto merged = BooleanOp::execute(*bodies[root], *instances[i], BooleanType::Union,
                                             reason, naming);
            if (!merged) return nullptr;
            bodies[root] = std::move(merged);
        }
    }

    if (BooleanCancellation::requested()) return stopped();
    auto result = std::make_unique<Solid>();
    for (const auto& body : bodies) {
        if (body) cloneInto(*result, *body, Mat4::identity(), 0, nullptr, true);
    }
    return result;
}

}  // namespace

std::unique_ptr<topo::Solid> Pattern::linear(const topo::Solid& source, const Vec3& direction,
                                             double spacing, int count,
                                             const std::vector<int>& suppressed,
                                             NamingScheme naming, bool together,
                                             std::string* reason) {
    if (count < 1) return nullptr;
    Vec3 dir = direction.normalized();
    std::vector<Mat4> transforms;
    transforms.reserve(static_cast<size_t>(count));
    for (int k = 0; k < count; ++k) {
        transforms.push_back(Mat4::translation(dir * (spacing * k)));
    }
    return buildPattern(source, transforms, suppressed, naming, together, reason);
}

std::unique_ptr<topo::Solid> Pattern::grid(const topo::Solid& source, const Vec3& direction,
                                           double spacing, int count, const Vec3& direction2,
                                           double spacing2, int count2,
                                           const std::vector<int>& suppressed, NamingScheme naming,
                                           bool together, std::string* reason) {
    if (count < 1 || count2 < 1) return nullptr;
    const Vec3 dir = direction.normalized();
    const Vec3 dir2 = direction2.normalized();
    std::vector<Mat4> transforms;
    transforms.reserve(static_cast<size_t>(count) * static_cast<size_t>(count2));
    for (int row = 0; row < count2; ++row) {
        for (int column = 0; column < count; ++column) {
            transforms.push_back(
                Mat4::translation(dir * (spacing * column) + dir2 * (spacing2 * row)));
        }
    }
    return buildPattern(source, transforms, suppressed, naming, together, reason);
}

std::unique_ptr<topo::Solid> Pattern::collect(const topo::Solid& a, const topo::Solid& b) {
    auto out = std::make_unique<Solid>();
    cloneInto(*out, a, Mat4::identity(), 0, nullptr, true);
    cloneInto(*out, b, Mat4::identity(), 0, nullptr, true);
    return out;
}

namespace {

/// Whether @p shell is a part built exactly: its faces carry exact surfaces and none a NURBS
/// carrier. Its corners do not span its curved faces (a cylinder's two, one on each rim, enclose
/// nothing), nor is it described by curves for facetCurved to cut.
bool exactWithoutCarriers(const Shell& shell) {
    bool exact = false;
    for (const Face* f : shell.faces) {
        if (f == nullptr || !f->alive) continue;
        if (f->surface) return false;
        exact = exact || f->exactSurface != nullptr;
    }
    return exact;
}

/// The box round @p shell's exact geometry: its corners, points along its edges' exact curves
/// (topo::kLoopSamples on each), and over each curved face its surface at a grid of as many
/// points each way across the rectangle its outer loop spans in (u, v), unwrapped as the exact
/// validator unwraps it.
BoundingBox exactShellBox(const Shell& shell, const math::ModelTolerance& tolerance,
                          std::size_t limit) {
    constexpr int kSamples = topo::kLoopSamples;
    BoundingBox box;
    for (const Face* f : shell.faces) {
        if (f == nullptr || !f->alive) continue;
        std::vector<const Wire*> loops{f->outerLoop};
        loops.insert(loops.end(), f->innerLoops.begin(), f->innerLoops.end());
        for (const Wire* w : loops) {
            std::vector<const HalfEdge*> ring;
            if (w == nullptr || !topo::loopHalfEdges(*f, *w, limit, ring)) continue;
            for (const HalfEdge* h : ring) {
                box.expand(h->origin->point);
                const Edge* e = h->edge;
                if (e == nullptr || !e->exactCurve) continue;
                for (int i = 0; i < kSamples; ++i) {
                    const double t = e->curveStart + (e->curveEnd - e->curveStart) *
                                                         static_cast<double>(i) / (kSamples - 1);
                    box.expand(e->exactCurve->evaluate(t));
                }
            }
        }
        const auto& surface = f->exactSurface;
        if (!surface || surface->kind() == geo::SurfaceKind::Plane || f->outerLoop == nullptr) {
            continue;
        }
        std::vector<const HalfEdge*> ring;
        if (!topo::loopHalfEdges(*f, *f->outerLoop, limit, ring)) continue;
        std::vector<double> allowances;
        bool pcurves = true;
        for (const HalfEdge* h : ring) {
            pcurves = pcurves && h->pcurve != nullptr && h->edge != nullptr;
            if (pcurves) allowances.push_back(topo::joinAllowance(*h, tolerance));
        }
        topo::LoopPolygon poly;
        if (!pcurves ||
            !topo::unwrapLoop(ring, kSamples, *surface, topo::uPeriod(*surface),
                              topo::vPeriod(*surface), allowances, poly,
                              topo::loopSense(*f, true)) ||
            poly.empty()) {
            continue;
        }
        math::Vec2 lo = poly.front().a;
        math::Vec2 hi = lo;
        for (const auto& seg : poly) {
            lo = {std::min(lo.x, seg.a.x), std::min(lo.y, seg.a.y)};
            hi = {std::max(hi.x, seg.a.x), std::max(hi.y, seg.a.y)};
        }
        for (int i = 0; i < kSamples; ++i) {
            for (int j = 0; j < kSamples; ++j) {
                box.expand(surface->evaluate(lo.x + (hi.x - lo.x) * i / (kSamples - 1),
                                             lo.y + (hi.y - lo.y) * j / (kSamples - 1)));
            }
        }
    }
    return box;
}

}  // namespace

std::vector<std::vector<const topo::Shell*>> Pattern::bodyShells(const topo::Solid& solid) {
    // A shell is either the outside of a body or a cavity inside one: an
    // enclosed void comes out of a Boolean as a second shell of the same
    // body, facing into the void. Its signed volume tells them apart —
    // negative for a cavity — once the solid's overall orientation is taken
    // out (some construction paths build a whole solid facing inward).
    struct Part {
        const Shell* shell;
        double volume;
        BoundingBox box;
    };
    // One shell is one body, and needs no measuring (which facets a solid
    // described by curves).
    std::vector<const Shell*> filled;
    for (const auto& shell : solid.shells()) {
        if (!shell.faces.empty()) filled.push_back(&shell);
    }
    if (filled.size() <= 1) {
        if (filled.empty()) return {};
        return {filled};
    }

    // Measured on its facets when it is described by curves, whose corners
    // do not span its faces (measureShells).
    std::vector<Part> parts;
    auto measures = measureShells(solid);
    // A shell of a part built exactly by its exact faces: its signed volume integrated on them
    // (coarse: its sign is what tells a body from a cavity), its box round its exact geometry.
    // Where they cannot be integrated, it is measured by its corners, as well as can be.
    std::vector<bool> exact;
    exact.reserve(solid.shells().size());
    for (const auto& shell : solid.shells()) exact.push_back(exactWithoutCarriers(shell));
    if (std::find(exact.begin(), exact.end(), true) != exact.end()) {
        const auto measured = detail::measureShellVolumes(solid, true, nullptr, exact);
        if (measured.problem.empty() && measured.volumes.size() == measures.size()) {
            const math::ModelTolerance tolerance = topo::exactTolerance(solid);
            const std::size_t limit = 2 * solid.edges().size() + 1;
            std::size_t at = 0;
            for (const auto& shell : solid.shells()) {
                if (exact[at]) {
                    measures[at].volume = measured.volumes[at];
                    measures[at].box = exactShellBox(shell, tolerance, limit);
                }
                ++at;
            }
        }
    }
    std::size_t index = 0;
    for (const auto& shell : solid.shells()) {
        const ShellMeasure& measure = measures[index++];
        if (shell.faces.empty()) continue;
        parts.push_back({&shell, measure.volume, measure.box});
    }
    double total = 0.0;
    for (const Part& part : parts) total += part.volume;
    const double orientation = total < 0.0 ? -1.0 : 1.0;

    // Each body is an outer shell and the cavities it encloses: a cavity
    // belongs to the smallest outer shell whose bounds contain it.
    std::vector<std::vector<const Shell*>> groups;
    std::vector<const Part*> outers;
    for (const Part& part : parts) {
        if (part.volume * orientation >= 0.0) {
            groups.push_back({part.shell});
            outers.push_back(&part);
        }
    }
    const auto boxVolume = [](const BoundingBox& b) {
        const Vec3 d = b.max() - b.min();
        return d.x * d.y * d.z;
    };
    for (const Part& part : parts) {
        if (part.volume * orientation >= 0.0) continue;
        size_t best = groups.size();
        for (size_t i = 0; i < outers.size(); ++i) {
            const BoundingBox& outer = outers[i]->box;
            if (outer.contains(part.box) &&
                (best == groups.size() || boxVolume(outer) < boxVolume(outers[best]->box))) {
                best = i;
            }
        }
        if (best < groups.size()) {
            groups[best].push_back(part.shell);
        } else {
            groups.push_back({part.shell});  // enclosed by nothing: keep it as it is
        }
    }

    return groups;
}

std::vector<std::unique_ptr<topo::Solid>> Pattern::separate(const topo::Solid& solid) {
    const auto groups = bodyShells(solid);
    std::vector<std::unique_ptr<topo::Solid>> bodies;
    for (const auto& group : groups) {
        auto body = std::make_unique<Solid>();
        cloneInto(*body, solid, Mat4::identity(), 0, &group);
        bodies.push_back(std::move(body));
    }
    return bodies;
}

void Pattern::append(topo::Solid& into, const topo::Solid& source, const Mat4& xform) {
    cloneInto(into, source, xform, 0, nullptr, true);
}

std::unique_ptr<topo::Solid> Pattern::transformed(const topo::Solid& source, const Mat4& xform) {
    // Ruling 4: why a copy has no exact geometry is not asked here, as master never said.
    return transformed(source, xform, nullptr);
}

std::unique_ptr<topo::Solid> Pattern::transformed(const topo::Solid& source, const Mat4& xform,
                                                  std::string* problem) {
    auto out = std::make_unique<Solid>();
    std::string why;
    cloneInto(*out, source, xform, 0, nullptr, false, &why);
    if (problem != nullptr) *problem = std::move(why);
    return out;
}

std::unique_ptr<topo::Solid> Pattern::circular(const topo::Solid& source, const Vec3& axisPoint,
                                               const Vec3& axisDir, double angleStepRad, int count,
                                               const std::vector<int>& suppressed,
                                               NamingScheme naming, bool together,
                                               std::string* reason) {
    if (count < 1) return nullptr;
    Vec3 axis = axisDir.normalized();
    std::vector<Mat4> transforms;
    transforms.reserve(static_cast<size_t>(count));
    const Mat4 toOrigin = Mat4::translation(-axisPoint);
    const Mat4 fromOrigin = Mat4::translation(axisPoint);
    for (int k = 0; k < count; ++k) {
        Mat4 rot = Mat4::rotation(Quaternion::fromAxisAngle(axis, angleStepRad * k));
        transforms.push_back(fromOrigin * rot * toOrigin);
    }
    return buildPattern(source, transforms, suppressed, naming, together, reason);
}

}  // namespace hz::model
