#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/topology/ExactValidator.h"
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

namespace {

geo::Frame leftHanded(const geo::Frame& f) {
    geo::Frame g = f;
    g.y = f.y * -1.0;
    return g;
}

/// At interior fractions of a half-edge's pcurve, its face's surface lies on the edge's curve
/// run in that half-edge's direction.
void expectPcurvesFollowCurves(const topo::Solid& solid, const math::ModelTolerance& tol) {
    for (const auto& e : solid.edges()) {
        for (const topo::HalfEdge* h : {static_cast<const topo::HalfEdge*>(e.halfEdge),
                                        static_cast<const topo::HalfEdge*>(e.halfEdge->twin)}) {
            // The curve run in h's direction: forward for e.halfEdge, backwards for its twin.
            const bool forward = h == e.halfEdge;
            for (const double f : {0.25, 0.5, 0.75}) {
                const math::Vec2 uv =
                    h->pcurve->evaluate(h->pcurveStart + f * (h->pcurveEnd - h->pcurveStart));
                const Vec3 p = h->face->exactSurface->evaluate(uv.x, uv.y);
                const double g = forward ? f : 1.0 - f;
                const Vec3 q =
                    e.exactCurve->evaluate(e.curveStart + g * (e.curveEnd - e.curveStart));
                EXPECT_TRUE(tol.within((p - q).length()))
                    << h->face->topoId.tag() << " f=" << f << " off by " << (p - q).length();
            }
        }
    }
}

/// Every face's outward normal points away from the solid's centroid; the hole's wall points
/// toward the hole's axis.
void expectNormalsOutward(const topo::Solid& solid, const Vec3& centroid,
                          const Vec3* holeAxisPoint) {
    for (const auto& f : solid.faces()) {
        const topo::HalfEdge* he = f.outerLoop->halfEdge;
        const math::Vec2 uv = he->pcurve->evaluate(he->pcurveStart);
        // A point strictly inside the pcurve's span avoids a degenerate corner.
        const math::Vec2 uv2 = he->pcurve->evaluate(0.5 * (he->pcurveStart + he->pcurveEnd));
        for (const math::Vec2& q : {uv, uv2}) {
            const auto d = f.exactSurface->derivatives(q.x, q.y);
            Vec3 n = d.du.cross(d.dv);
            if (f.reversed) n = n * -1.0;
            const Vec3 p = f.exactSurface->evaluate(q.x, q.y);
            if (f.topoId.tag() == "hole/wall") {
                ASSERT_NE(holeAxisPoint, nullptr);
                Vec3 radial = p - *holeAxisPoint;
                radial.z = 0.0;
                EXPECT_LT(n.dot(radial), 0.0);
            } else if (f.topoId.tag() == "cylinder/side") {
                Vec3 radial = p - centroid;
                const auto* c = dynamic_cast<const geo::Cylinder*>(f.exactSurface.get());
                radial = radial - c->frame().z * radial.dot(c->frame().z);
                EXPECT_GT(n.dot(radial), 0.0);
            } else {
                EXPECT_GT(n.dot(p - centroid), 0.0) << f.topoId.tag();
            }
        }
    }
}

}  // namespace

TEST(ExactPrimitives, TheirPcurvesFollowTheirCurvesAndTheirNormalsPointOut) {
    for (const Vec3& at : {Vec3{0, 0, 0}, Vec3{-7, 3, 11}, Vec3{100, -50, 20}}) {
        const auto tol = math::ModelTolerance::forExtent(200.0);
        const auto block = model::exact::makeBlock(at, at + Vec3{2, 3, 4});
        expectPcurvesFollowCurves(*block, tol);
        expectNormalsOutward(*block, at + Vec3{1, 1.5, 2}, nullptr);

        const auto plate =
            model::exact::makePlateWithHole(at, at + Vec3{40, 30, 10}, {at.x + 15, at.y + 12}, 5.0);
        expectPcurvesFollowCurves(*plate, tol);
        const Vec3 axis{at.x + 15, at.y + 12, at.z};
        expectNormalsOutward(*plate, at + Vec3{20, 15, 5}, &axis);

        for (const Vec3& dir : {Vec3{0, 0, 1}, Vec3{1, 1, 1}, Vec3{-2, 1, 0.5}}) {
            const geo::Frame right = geo::Frame::fromAxis(at, dir);
            for (const geo::Frame& base : {right, leftHanded(right)}) {
                const auto cyl = model::exact::makeCylinder(base, 5.0, 10.0);
                ASSERT_NE(cyl, nullptr);
                expectPcurvesFollowCurves(*cyl, tol);
                expectNormalsOutward(*cyl, at + right.z * 5.0, nullptr);
                expectGeometryMeetsTopology(*cyl);
            }
        }
    }
}

TEST(ExactPrimitives, ATinyBlockStillAgrees) {
    const auto tol = math::ModelTolerance::forExtent(1e-6);
    const auto block = model::exact::makeBlock({0, 0, 0}, {1e-6, 1e-6, 1e-6});
    ASSERT_NE(block, nullptr);
    expectPcurvesFollowCurves(*block, tol);
    for (const auto& e : block->edges()) {
        EXPECT_NEAR(e.curveEnd - e.curveStart, 1e-6, 1e-12);
    }
}

TEST(ExactPrimitives, DegenerateRequestsGiveNothing) {
    const geo::Frame f = geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1});
    EXPECT_EQ(model::exact::makeCylinder(f, 0.0, 10.0), nullptr);
    EXPECT_EQ(model::exact::makeCylinder(f, -1.0, 10.0), nullptr);
    EXPECT_EQ(model::exact::makeCylinder(f, 5.0, 0.0), nullptr);
    EXPECT_EQ(model::exact::makeCylinder(f, 5.0, -2.0), nullptr);
    EXPECT_EQ(model::exact::makeBlock({0, 0, 0}, {0, 1, 1}), nullptr);
    EXPECT_EQ(model::exact::makeBlock({0, 0, 0}, {1, -1, 1}), nullptr);
    EXPECT_EQ(model::exact::makeBlock({0, 0, 0}, {1, 1, 0}), nullptr);
    EXPECT_EQ(model::exact::makePlateWithHole({0, 0, 0}, {0, 40, 10}, {20, 20}, 5.0), nullptr);
    EXPECT_EQ(model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 0.0), nullptr);
    EXPECT_EQ(model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {3, 20}, 5.0), nullptr);
    EXPECT_EQ(model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 35}, 5.0), nullptr);
    EXPECT_EQ(model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {5, 20}, 5.0), nullptr);
}

namespace {

std::vector<math::Mat4> similarities() {
    return {math::Mat4::translation({3, 4, 5}) * math::Mat4::rotationZ(0.7),
            math::Mat4::scale({-1, 1, 1}), math::Mat4::scale({2, 2, 2}),
            math::Mat4::translation({-7, 2, 9}) * math::Mat4::rotationZ(1.1) *
                math::Mat4::scale({1, -1, 1})};
}

}  // namespace

TEST(ExactPrimitives, TransformedAndMirroredCopiesStayValid) {
    const auto plate = model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 5.0);
    const math::ModelTolerance tol = math::ModelTolerance::forExtent(60.0);
    for (const math::Mat4& m : {math::Mat4::translation({3, 4, 5}) * math::Mat4::rotationZ(0.7),
                                math::Mat4::scale({-1, 1, 1}), math::Mat4::scale({2, 2, 2})}) {
        const auto copy = model::Pattern::transformed(*plate, m);
        EXPECT_TRUE(topo::validateExact(*copy, tol).empty());
    }
}

TEST(ExactPrimitives, CylinderAndBlockCopiesStayValid) {
    const math::ModelTolerance tol = math::ModelTolerance::forExtent(100.0);
    const auto cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 12.0);
    const auto block = model::exact::makeBlock({0, 0, 0}, {10, 20, 30});
    for (const math::Mat4& m : similarities()) {
        EXPECT_TRUE(topo::validateExact(*model::Pattern::transformed(*cyl, m), tol).empty());
        EXPECT_TRUE(topo::validateExact(*model::Pattern::transformed(*block, m), tol).empty());
    }
}

TEST(ExactPrimitives, AppendedPlacedCopiesStayValid) {
    const math::ModelTolerance tol = math::ModelTolerance::forExtent(200.0);
    const auto cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 12.0);
    const auto block = model::exact::makeBlock({0, 0, 0}, {10, 20, 30});
    for (const math::Mat4& m : similarities()) {
        topo::Solid both;
        model::Pattern::append(both, *block, math::Mat4::translation({50, 0, 0}) * m);
        model::Pattern::append(both, *cyl, math::Mat4::translation({0, 60, 0}) * m);
        EXPECT_TRUE(topo::validateExact(both, tol).empty());
    }
}

TEST(ExactPrimitives, TolerancesScaleWithACopy) {
    auto block = model::exact::makeBlock({0, 0, 0}, {10, 20, 30});
    for (auto& e : block->edges()) e.tolerance = 0.25;
    for (auto& v : block->vertices()) v.tolerance = 0.5;
    const auto copy = model::Pattern::transformed(*block, math::Mat4::scale({2, 2, 2}));
    for (const auto& e : copy->edges()) EXPECT_DOUBLE_EQ(e.tolerance, 0.5);
    for (const auto& v : copy->vertices()) EXPECT_DOUBLE_EQ(v.tolerance, 1.0);
}

TEST(ExactPrimitives, ANonSimilarityLeavesTheCopyFaceted) {
    const auto block = model::exact::makeBlock({0, 0, 0}, {10, 20, 30});
    math::Mat4 shear = math::Mat4::identity();
    shear.m[0][1] = 0.5;
    for (const math::Mat4& m : {math::Mat4::scale({2, 1, 1}), shear}) {
        const auto copy = model::Pattern::transformed(*block, m);
        EXPECT_EQ(copy->faces().size(), block->faces().size());
        for (const auto& f : copy->faces()) {
            EXPECT_EQ(f.exactSurface, nullptr);
            EXPECT_FALSE(f.reversed);
        }
        for (const auto& e : copy->edges()) {
            EXPECT_EQ(e.exactCurve, nullptr);
            EXPECT_EQ(e.curveStart, 0.0);
            EXPECT_EQ(e.curveEnd, 0.0);
            EXPECT_EQ(e.tolerance, 0.0);
        }
        for (const auto& e : copy->edges()) {
            for (const topo::HalfEdge* h : {e.halfEdge, e.halfEdge->twin}) {
                EXPECT_EQ(h->pcurve, nullptr);
                EXPECT_EQ(h->pcurveStart, 0.0);
                EXPECT_EQ(h->pcurveEnd, 0.0);
            }
        }
    }
}
