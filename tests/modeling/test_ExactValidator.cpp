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
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
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
    for (const double s : {1e-3, 1e5}) {
        const auto tol = math::ModelTolerance::forExtent(s * 40.0);
        const auto plate = model::exact::makePlateWithHole({0, 0, 0}, {s * 40, s * 40, s * 10},
                                                           {s * 20, s * 20}, s * 5);
        EXPECT_TRUE(topo::validateExact(*plate, tol).empty()) << s;
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
    EXPECT_TRUE(kindsOf(*block).count(ExactIssueKind::LoopNotClosed) == 1);
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
