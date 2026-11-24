#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <set>

#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Circle2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/curves/Nurbs2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/math/Constants.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/topology/ExactValidator.h"

using namespace hz;
using hz::math::Vec3;
using hz::topo::ExactIssueKind;

namespace {

const math::ModelTolerance kTol = math::ModelTolerance::forExtent(300.0);

std::set<ExactIssueKind> kindsOf(const topo::Solid& s) {
    std::set<ExactIssueKind> kinds;
    for (const auto& i : topo::validateExact(s, kTol)) kinds.insert(i.kind);
    return kinds;
}

/// The mutation must give @p expected, and nothing outside @p allowed (the kinds that the same
/// mutation unavoidably trips as well).
void expectKinds(const topo::Solid& s, ExactIssueKind expected,
                 const std::set<ExactIssueKind>& allowed = {}) {
    const auto kinds = kindsOf(s);
    EXPECT_TRUE(kinds.count(expected) == 1);
    for (const auto k : kinds) {
        EXPECT_TRUE(k == expected || allowed.count(k) == 1) << "unexpected kind " << int(k);
    }
}

}  // namespace

TEST(ExactValidator, TheExactPrimitivesAtAnyPlacementAreClean) {
    for (const Vec3& at : {Vec3{0, 0, 0}, Vec3{-7, 3, 11}, Vec3{100, -50, 20}}) {
        const auto block = model::exact::makeBlock(at, at + Vec3{2, 3, 4});
        EXPECT_TRUE(topo::validateExact(*block, kTol).empty());
        const auto plate =
            model::exact::makePlateWithHole(at, at + Vec3{40, 30, 10}, {at.x + 15, at.y + 12}, 5.0);
        EXPECT_TRUE(topo::validateExact(*plate, kTol).empty());
        for (const Vec3& dir : {Vec3{0, 0, 1}, Vec3{1, 1, 1}, Vec3{-2, 1, 0.5}}) {
            const auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis(at, dir), 5.0, 10.0);
            EXPECT_TRUE(topo::validateExact(*cyl, kTol).empty());
            geo::Frame left = geo::Frame::fromAxis(at, dir);
            left.y = left.y * -1.0;
            EXPECT_TRUE(
                topo::validateExact(*model::exact::makeCylinder(left, 5.0, 10.0), kTol).empty());
        }
    }
}

TEST(ExactValidator, ATinyAndAHugePartAreCleanAtTheirOwnScale) {
    for (const double s : {1e-4, 1e-3, 1e4, 1e5}) {
        const auto tol = math::ModelTolerance::forExtent(s * 40.0);
        const auto plate = model::exact::makePlateWithHole({0, 0, 0}, {s * 40, s * 40, s * 10},
                                                           {s * 20, s * 20}, s * 5);
        EXPECT_TRUE(topo::validateExact(*plate, tol).empty()) << s;
        const auto cyl = model::exact::makeCylinder(
            geo::Frame::fromAxis({s, 2 * s, 3 * s}, {1, 2, 3}), s * 5, s * 10);
        EXPECT_TRUE(topo::validateExact(*cyl, math::ModelTolerance::forExtent(s * 40.0)).empty())
            << s;
    }
}

TEST(ExactValidator, AMovedVertexIsOffItsCurves) {
    auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    block->vertices()[0].point.x += 0.5;
    // Its pcurves' ends no longer meet it either: the same disagreement seen from the face.
    expectKinds(*block, ExactIssueKind::VertexOffCurve, {ExactIssueKind::PcurveOffVertex});
}

TEST(ExactValidator, APcurveShiftedInVIsOffItsCurve) {
    auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    bool done = false;
    for (auto& e : block->edges()) {
        if (done) break;
        topo::HalfEdge* h = e.halfEdge;
        const auto* line = dynamic_cast<const geo::Line2D*>(h->pcurve.get());
        // An edge running along u: shifting it in v leaves the line it lies on.
        if (!line || std::abs(line->end().y - line->start().y) > 1e-9) continue;
        h->pcurve = std::make_shared<geo::Line2D>(line->start() + math::Vec2{0, 0.1},
                                                  line->end() + math::Vec2{0, 0.1});
        done = true;
    }
    ASSERT_TRUE(done);
    // The shifted pcurve also misses its vertices and no longer joins its neighbours.
    expectKinds(*block, ExactIssueKind::PcurveOffCurve,
                {ExactIssueKind::PcurveOffVertex, ExactIssueKind::LoopNotClosed});
}

TEST(ExactValidator, AFaceWithoutItsSurfaceIsMissingGeometry) {
    auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    block->faces()[2].exactSurface.reset();
    expectKinds(*block, ExactIssueKind::MissingGeometry);
    for (const auto& i : topo::validateExact(*block, kTol)) EXPECT_EQ(i.face, &block->faces()[2]);
}

TEST(ExactValidator, ASeamEdgeWithoutItsCurveIsMissingGeometry) {
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    bool done = false;
    for (auto& e : cyl->edges()) {
        if (e.halfEdge->face == e.halfEdge->twin->face) {
            e.exactCurve.reset();
            done = true;
        }
    }
    ASSERT_TRUE(done);
    expectKinds(*cyl, ExactIssueKind::MissingGeometry);
    for (const auto& i : topo::validateExact(*cyl, kTol)) {
        ASSERT_NE(i.edge, nullptr);
        EXPECT_EQ(i.edge->exactCurve, nullptr);
    }
}

TEST(ExactValidator, AFlippedFaceIsWronglyOriented) {
    auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    block->faces()[1].reversed = !block->faces()[1].reversed;
    expectKinds(*block, ExactIssueKind::LoopWrongOrientation);
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    for (auto& f : cyl->faces()) f.reversed = !f.reversed;
    expectKinds(*cyl, ExactIssueKind::LoopWrongOrientation);
}

TEST(ExactValidator, ABrokenJoinIsALoopNotClosed) {
    auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    topo::HalfEdge* h = block->faces()[0].outerLoop->halfEdge;
    h->next = h->next->next;  // skip a half-edge: the cycle no longer closes properly
    // Skipping a half-edge also leaves the skipped-over origin out of its neighbour's pcurve end.
    expectKinds(*block, ExactIssueKind::LoopNotClosed, {ExactIssueKind::PcurveOffVertex});
}

TEST(ExactValidator, AHoleMovedOutsideThePlateIsOutsideItsOuterLoop) {
    auto plate = model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 5.0);
    // Move the whole hole 100 along x, every piece of it, so nothing else disagrees.
    const auto move = math::Mat4::translation({100, 0, 0});
    std::set<topo::Vertex*> moved;
    for (auto& e : plate->edges()) {
        const bool onWall = e.halfEdge->face->topoId.tag() == "hole/wall" ||
                            e.halfEdge->twin->face->topoId.tag() == "hole/wall";
        if (!onWall) continue;
        e.exactCurve = e.exactCurve->transformed(move);
        for (topo::HalfEdge* h : {e.halfEdge, e.halfEdge->twin}) {
            moved.insert(h->origin);
            if (h->face->topoId.tag() == "hole/wall") continue;
            const auto* c = dynamic_cast<const geo::Circle2D*>(h->pcurve.get());
            ASSERT_NE(c, nullptr);
            h->pcurve =
                std::make_shared<geo::Circle2D>(c->center() + math::Vec2{100, 0}, c->radius());
        }
    }
    for (auto* v : moved) v->point = move.transformPoint(v->point);
    for (auto& f : plate->faces()) {
        if (f.topoId.tag() == "hole/wall") f.exactSurface = f.exactSurface->transformed(move);
    }
    expectKinds(*plate, ExactIssueKind::HoleOutsideOuterLoop);
}

TEST(ExactValidator, ARimCurveRunningTheWrongWayIsOffItsPcurves) {
    // The bug class a left-handed cylinder once had: same points, same start, opposite direction.
    // Only the order of the pcurve along the curve is wrong; ends and positions all agree.
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    bool done = false;
    for (auto& e : cyl->edges()) {
        if (!dynamic_cast<const geo::CircleCurve*>(e.exactCurve.get())) continue;
        std::swap(e.curveStart, e.curveEnd);
        done = true;
        break;
    }
    ASSERT_TRUE(done);
    expectKinds(*cyl, ExactIssueKind::PcurveOffCurve);
}

TEST(ExactValidator, ABulgingPcurveMakesALoopCrossItself) {
    auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    topo::Face& f = block->faces()[0];
    topo::HalfEdge* h = f.outerLoop->halfEdge;
    const math::Vec2 a = h->pcurve->evaluate(h->pcurveStart);
    const math::Vec2 b = h->pcurve->evaluate(h->pcurveEnd);
    math::Vec2 centroid{0, 0};
    int n = 0;
    for (const topo::HalfEdge* g = h; n == 0 || g != h; g = g->next, ++n) {
        centroid = centroid + g->pcurve->evaluate(g->pcurveStart);
    }
    centroid = centroid * (1.0 / n);
    const math::Vec2 mid = (a + b) * 0.5;
    const math::Vec2 far =
        mid + (centroid - mid) * 2.4;  // just beyond the opposite side (a larger bulge flips the
                                       // loop's area, adding LoopWrongOrientation)
    h->pcurve = std::make_shared<geo::Nurbs2D>(std::vector<math::Vec2>{a, far, b},
                                               std::vector<double>{1, 1, 1},
                                               std::vector<double>{0, 0, 0.5, 1, 1}, 1);
    h->pcurveStart = h->pcurve->tMin();
    h->pcurveEnd = h->pcurve->tMax();
    // Its ends and the joins are unchanged; it only leaves the edge's 3D line.
    expectKinds(*block, ExactIssueKind::LoopSelfIntersects, {ExactIssueKind::PcurveOffCurve});
}

TEST(ExactValidator, AHoleLoopCrossingTheOuterLoopIsLoopsCross) {
    auto plate = model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 5.0);
    bool done = false;
    for (auto& f : plate->faces()) {
        if (f.innerLoops.empty() || done) continue;
        topo::HalfEdge* h = f.innerLoops[0]->halfEdge;
        const auto* c = dynamic_cast<const geo::Circle2D*>(h->pcurve.get());
        ASSERT_NE(c, nullptr);
        // A larger circle through the same start point (c.x + r, c.y), same parameter range.
        const double big = 30.0;
        h->pcurve = std::make_shared<geo::Circle2D>(
            math::Vec2{c->center().x + c->radius() - big, c->center().y}, big);
        done = true;
    }
    ASSERT_TRUE(done);
    // The crossing is reported instead of HoleOutsideOuterLoop (only non-crossing loops are
    // classified); the larger circle also leaves the edge's 3D circle.
    expectKinds(*plate, ExactIssueKind::LoopsCross, {ExactIssueKind::PcurveOffCurve});
}

namespace {

/// Replaces each rim circle of a cylinder along z by the same circle as a NURBS curve (range
/// [0, 1], closed, not periodic), running the same way.
void makeRimsNurbs(topo::Solid& cyl) {
    for (auto& e : cyl.edges()) {
        const auto* c = dynamic_cast<const geo::CircleCurve*>(e.exactCurve.get());
        if (!c) continue;
        const Vec3 center = (c->evaluate(0.0) + c->evaluate(math::kPi)) * 0.5;
        e.exactCurve = std::make_shared<geo::NurbsCurve>(
            geo::NurbsCurve::makeCircle(center, c->radius(), Vec3{0, 0, 1}));
        e.curveStart = 0.0;
        e.curveEnd = 1.0;
    }
}

}  // namespace

TEST(ExactValidator, AClosedNurbsRimIsCleanAndItsReverseIsReported) {
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    makeRimsNurbs(*cyl);
    EXPECT_TRUE(topo::validateExact(*cyl, kTol).empty());
    for (auto& e : cyl->edges()) {
        if (dynamic_cast<const geo::NurbsCurve*>(e.exactCurve.get())) {
            std::swap(e.curveStart, e.curveEnd);
        }
    }
    expectKinds(*cyl, ExactIssueKind::PcurveOffCurve);
}

TEST(ExactValidator, APcurveWindingTwiceRoundAClosedEdgeIsReported) {
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    for (auto& f : cyl->faces()) {
        if (f.topoId.tag() == "cylinder/side") continue;
        topo::HalfEdge* h = f.outerLoop->halfEdge;  // a cap's rim: one full circle
        h->pcurveEnd = h->pcurveStart + 2.0 * (h->pcurveEnd - h->pcurveStart);
        break;
    }
    // The loop retraces itself, which is a self-overlap too.
    expectKinds(*cyl, ExactIssueKind::PcurveOffCurve, {ExactIssueKind::LoopSelfIntersects});
}

namespace {

/// One face on a half-cylinder (radius 2, height 3) spanning u in [3 pi/2, 5 pi/2], across the
/// seam of the surface's [0, 2 pi) range; its twins have no face. @p seamU is the u the right
/// side's pcurve is written at (5 pi/2 is continuous, pi/2 is what inversion would give).
struct HalfCylinderFace {
    topo::Solid solid;
    std::shared_ptr<geo::Cylinder> surface;
    explicit HalfCylinderFace(double seamU) {
        const geo::Frame frame = geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1});
        constexpr double r = 2.0;
        constexpr double h = 3.0;
        const double a = 1.5 * math::kPi;
        const double b = 2.5 * math::kPi;
        surface = std::make_shared<geo::Cylinder>(frame, r);
        topo::Face* f = solid.allocFace();
        f->exactSurface = surface;
        topo::Wire* w = solid.allocWire();
        f->outerLoop = w;
        const math::Vec2 uv[4] = {{a, 0}, {b, 0}, {b, h}, {a, h}};
        topo::Vertex* v[4];
        topo::HalfEdge* he[4];
        topo::HalfEdge* tw[4];
        topo::Edge* ed[4];
        for (int i = 0; i < 4; ++i) {
            v[i] = solid.allocVertex();
            v[i]->point = surface->evaluate(uv[i].x, uv[i].y);
            he[i] = solid.allocHalfEdge();
            tw[i] = solid.allocHalfEdge();
            ed[i] = solid.allocEdge();
        }
        for (int i = 0; i < 4; ++i) {
            const int j = (i + 1) % 4;
            he[i]->origin = v[i];
            he[i]->next = he[j];
            he[i]->prev = he[(i + 3) % 4];
            he[i]->face = f;
            he[i]->twin = tw[i];
            he[i]->edge = ed[i];
            tw[i]->origin = v[j];
            tw[i]->twin = he[i];
            tw[i]->edge = ed[i];
            ed[i]->halfEdge = he[i];
            v[i]->halfEdge = he[i];
            math::Vec2 s = uv[i];
            math::Vec2 t = uv[j];
            if (i == 1) s.x = t.x = seamU;
            he[i]->pcurve = std::make_shared<geo::Line2D>(s, t);
            he[i]->pcurveStart = 0.0;
            he[i]->pcurveEnd = 1.0;
        }
        w->halfEdge = he[0];
        // Bottom and top arcs; the top runs backwards, from b to a. Sides are lines along z.
        ed[0]->exactCurve = std::make_shared<geo::CircleCurve>(frame, r);
        ed[0]->curveStart = a;
        ed[0]->curveEnd = b;
        geo::Frame top = frame;
        top.origin = {0, 0, h};
        ed[2]->exactCurve = std::make_shared<geo::CircleCurve>(top, r);
        ed[2]->curveStart = b;
        ed[2]->curveEnd = a;
        ed[1]->exactCurve = std::make_shared<geo::LineCurve>(v[1]->point, Vec3{0, 0, 1});
        ed[1]->curveStart = 0.0;
        ed[1]->curveEnd = h;
        ed[3]->exactCurve = std::make_shared<geo::LineCurve>(v[3]->point, Vec3{0, 0, -1});
        ed[3]->curveStart = 0.0;
        ed[3]->curveEnd = h;
    }
};

}  // namespace

TEST(ExactValidator, AFaceAcrossTheSeamIsCleanWhetherItsPcurvesAreContinuousOrInverted) {
    for (const double seamU : {2.5 * math::kPi, 0.5 * math::kPi}) {
        HalfCylinderFace face(seamU);
        EXPECT_TRUE(topo::validateExact(face.solid, kTol).empty()) << seamU;
    }
}

namespace {

/// A planar annulus: an outer circle of radius R about the origin, counter-clockwise, and an
/// inner circle of radius r about @p inner, clockwise, starting @p phase round from its +x.
struct Annulus {
    topo::Solid solid;
    Annulus(double R, double r, const math::Vec2& inner, double phase) {
        const geo::Frame frame = geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1});
        topo::Face* f = solid.allocFace();
        f->exactSurface = std::make_shared<geo::Plane>(frame);
        for (int k = 0; k < 2; ++k) {
            const bool outer = k == 0;
            const math::Vec2 c = outer ? math::Vec2{0, 0} : inner;
            const double rad = outer ? R : r;
            const double ph = outer ? 0.0 : phase;
            topo::Wire* w = solid.allocWire();
            if (outer) {
                f->outerLoop = w;
            } else {
                f->innerLoops.push_back(w);
            }
            topo::Vertex* v = solid.allocVertex();
            v->point = {c.x + rad * std::cos(ph), c.y + rad * std::sin(ph), 0};
            topo::HalfEdge* he = solid.allocHalfEdge();
            topo::HalfEdge* tw = solid.allocHalfEdge();
            topo::Edge* e = solid.allocEdge();
            he->origin = tw->origin = v;
            he->next = he->prev = he;
            he->face = f;
            he->twin = tw;
            tw->twin = he;
            he->edge = tw->edge = e;
            e->halfEdge = he;
            v->halfEdge = he;
            w->halfEdge = he;
            geo::Frame cf = frame;
            cf.origin = {c.x, c.y, 0};
            if (outer) {
                he->pcurve = std::make_shared<geo::Circle2D>(c, rad);
                he->pcurveStart = ph;
                he->pcurveEnd = ph + math::kTwoPi;
                e->exactCurve = std::make_shared<geo::CircleCurve>(cf, rad);
                e->curveStart = ph;
                e->curveEnd = ph + math::kTwoPi;
            } else {
                // Clockwise: the 3D circle's frame is mirrored so it runs the same way.
                he->pcurve = std::make_shared<geo::Circle2D>(c, rad);
                he->pcurveStart = ph + math::kTwoPi;
                he->pcurveEnd = ph;
                cf.y = cf.y * -1.0;
                e->exactCurve = std::make_shared<geo::CircleCurve>(cf, rad);
                e->curveStart = -ph;
                e->curveEnd = math::kTwoPi - ph;
            }
        }
    }
};

}  // namespace

TEST(ExactValidator, AThinAnnulusIsCleanWhateverWhereItsLoopsStart) {
    const double phase =
        math::kPi / 16.0;  // a chord of the inner circle faces a vertex of the outer
    for (const double ph : {0.0, phase}) {
        Annulus thin(10.0, 9.85, {0, 0}, ph);
        EXPECT_TRUE(topo::validateExact(thin.solid, kTol).empty()) << ph;
    }
}

TEST(ExactValidator, AnAnnulusWhoseInnerCircleReallyCrossesTheOuterIsLoopsCross) {
    Annulus overlapping(10.0, 9.85, {1.0, 0}, math::kPi / 16.0);
    expectKinds(overlapping.solid, ExactIssueKind::LoopsCross);
}
