#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/math/Constants.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/topology/Solid.h"

using namespace hz;
using hz::math::Vec3;

namespace {

const math::ModelTolerance kTol = math::ModelTolerance::forExtent(100.0);

/// Every edge's 3D curve and every half-edge's pcurve, through its face's surface, lands on the
/// vertices it should.
void expectGeometryMeetsTopology(const topo::Solid& solid) {
    for (const auto& e : solid.edges()) {
        if (!e.alive) continue;
        ASSERT_NE(e.exactCurve, nullptr);
        const topo::HalfEdge* he = e.halfEdge;
        EXPECT_TRUE(kTol.within((e.exactCurve->evaluate(e.curveStart) - he->origin->point).length(),
                                e.tolerance));
        EXPECT_TRUE(kTol.within(
            (e.exactCurve->evaluate(e.curveEnd) - he->twin->origin->point).length(), e.tolerance));
        for (const topo::HalfEdge* h : {static_cast<const topo::HalfEdge*>(he),
                                        static_cast<const topo::HalfEdge*>(he->twin)}) {
            ASSERT_NE(h->pcurve, nullptr);
            ASSERT_NE(h->face->exactSurface, nullptr);
            const auto& s = *h->face->exactSurface;
            const math::Vec2 a = h->pcurve->evaluate(h->pcurveStart);
            const math::Vec2 b = h->pcurve->evaluate(h->pcurveEnd);
            EXPECT_TRUE(kTol.within((s.evaluate(a.x, a.y) - h->origin->point).length()));
            EXPECT_TRUE(kTol.within((s.evaluate(b.x, b.y) - h->next->origin->point).length()));
        }
    }
}

}  // namespace

TEST(ExactPrimitives, TheirTopologyIsWholeFaces) {
    const auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    EXPECT_EQ(block->vertexCount(), 8u);
    EXPECT_EQ(block->edgeCount(), 12u);
    EXPECT_EQ(block->faceCount(), 6u);
    const auto cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    EXPECT_EQ(cyl->vertexCount(), 2u);
    EXPECT_EQ(cyl->edgeCount(), 3u);  // two rims, one seam
    EXPECT_EQ(cyl->faceCount(), 3u);
    const auto plate = model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 5.0);
    EXPECT_EQ(plate->vertexCount(), 10u);
    EXPECT_EQ(plate->edgeCount(), 15u);
    EXPECT_EQ(plate->faceCount(), 7u);
    for (const auto* s : {block.get(), cyl.get(), plate.get()}) {
        EXPECT_TRUE(s->isValid()) << s->validationReport();
        EXPECT_TRUE(s->checkEulerFormula());
    }
}

TEST(ExactPrimitives, EveryPieceHasItsExactGeometry) {
    const auto cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    for (const auto& f : cyl->faces()) ASSERT_NE(f.exactSurface, nullptr);
    for (const auto& e : cyl->edges()) {
        ASSERT_NE(e.exactCurve, nullptr);
        EXPECT_NE(e.halfEdge->pcurve, nullptr);
        EXPECT_NE(e.halfEdge->twin->pcurve, nullptr);
    }
    int sides = 0;
    for (const auto& f : cyl->faces()) {
        if (f.topoId.tag() == "cylinder/side") {
            ++sides;
            const auto* c = dynamic_cast<const geo::Cylinder*>(f.exactSurface.get());
            ASSERT_NE(c, nullptr);
            EXPECT_DOUBLE_EQ(c->radius(), 5.0);
            EXPECT_FALSE(f.reversed);
        }
    }
    EXPECT_EQ(sides, 1);
    int circles = 0;
    for (const auto& e : cyl->edges()) {
        if (const auto* c = dynamic_cast<const geo::CircleCurve*>(e.exactCurve.get())) {
            ++circles;
            EXPECT_DOUBLE_EQ(c->radius(), 5.0);
        }
    }
    EXPECT_EQ(circles, 2);
}

TEST(ExactPrimitives, TheSeamHasBothHalfEdgesInTheSideFace) {
    const auto cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    int seams = 0;
    for (const auto& e : cyl->edges()) {
        if (dynamic_cast<const geo::LineCurve*>(e.exactCurve.get()) == nullptr) continue;
        ++seams;
        EXPECT_EQ(e.halfEdge->face, e.halfEdge->twin->face);
        const double u0 = e.halfEdge->pcurve->evaluate(e.halfEdge->pcurveStart).x;
        const double u1 = e.halfEdge->twin->pcurve->evaluate(e.halfEdge->twin->pcurveStart).x;
        EXPECT_NEAR(std::min(u0, u1), 0.0, 1e-12);
        EXPECT_NEAR(std::max(u0, u1), math::kTwoPi, 1e-12);
    }
    EXPECT_EQ(seams, 1);
}

TEST(ExactPrimitives, ThePlateHasAReversedHoleWallAndInnerLoopsOnTopAndBottom) {
    const auto plate = model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 5.0);
    int walls = 0;
    int withInner = 0;
    for (const auto& f : plate->faces()) {
        if (f.topoId.tag() == "hole/wall") {
            ++walls;
            EXPECT_TRUE(f.reversed);
        } else {
            EXPECT_FALSE(f.reversed);
        }
        withInner += f.innerLoops.empty() ? 0 : 1;
    }
    EXPECT_EQ(walls, 1);
    EXPECT_EQ(withInner, 2);
}

TEST(ExactPrimitives, TheirGeometryMeetsTheirTopology) {
    const auto block = model::exact::makeBlock({-1, 2, 3}, {2, 3, 7});
    expectGeometryMeetsTopology(*block);
    const auto cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({1, 2, 3}, {1, 1, 1}), 5.0, 10.0);
    expectGeometryMeetsTopology(*cyl);
    const auto plate = model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 5.0);
    expectGeometryMeetsTopology(*plate);
}

namespace {

/// The signed area a loop's pcurves enclose in (u, v), positive when counter-clockwise.
double loopArea(const topo::Wire& w) {
    double twice = 0.0;
    const topo::HalfEdge* start = w.halfEdge;
    const topo::HalfEdge* he = start;
    do {
        constexpr int kSteps = 64;
        math::Vec2 prev = he->pcurve->evaluate(he->pcurveStart);
        for (int i = 1; i <= kSteps; ++i) {
            const double t = he->pcurveStart + (he->pcurveEnd - he->pcurveStart) * i / kSteps;
            const math::Vec2 p = he->pcurve->evaluate(t);
            twice += prev.x * p.y - p.x * prev.y;
            prev = p;
        }
        he = he->next;
    } while (he != start);
    return twice / 2.0;
}

}  // namespace

TEST(ExactPrimitives, OuterLoopsRunCounterClockwiseInUvAndInnerLoopsTheOtherWay) {
    const auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    const auto cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    const auto plate = model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 5.0);
    for (const auto* s : {block.get(), cyl.get(), plate.get()}) {
        for (const auto& f : s->faces()) {
            const double outer = loopArea(*f.outerLoop);
            EXPECT_EQ(outer > 0.0, !f.reversed) << f.topoId.tag();
            for (const auto* w : f.innerLoops) {
                EXPECT_EQ(loopArea(*w) > 0.0, f.reversed) << f.topoId.tag();
            }
        }
    }
}
