#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "PoleSolids.h"
#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/TrimmedFaceMesher.h"
#include "horizon/topology/ExactTolerance.h"
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
    // 1e-8 across: below the 1e-7 at which normalized() gives zero, so an edge's direction must
    // be divided by its length.
    const auto tol = math::ModelTolerance::forExtent(1e-8);
    const auto block = model::exact::makeBlock({0, 0, 0}, {1e-8, 1e-8, 1e-8});
    ASSERT_NE(block, nullptr);
    expectPcurvesFollowCurves(*block, tol);
    for (const auto& e : block->edges()) {
        EXPECT_NEAR(e.curveEnd - e.curveStart, 1e-8, 1e-14);
    }
    EXPECT_TRUE(topo::validateExact(*block, tol).empty());
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

TEST(ExactPrimitives, CopiesScaledDownToATinyPartStayValid) {
    // Scaled by 1e-8, a copy's frames and lines are built from directions 1e-8 long; made unit
    // with normalized(), they were zero and the copy collapsed (at 0.001 mm the model's
    // resolution is 1e-10, so such a part is still 500 times it).
    const math::Mat4 m = math::Mat4::scale({1e-8, 1e-8, 1e-8});
    const auto cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    const auto plate = model::exact::makePlateWithHole({0, 0, 0}, {40, 40, 10}, {20, 20}, 5.0);
    const struct {
        const topo::Solid* solid;
        double size;
    } cases[] = {{cyl.get(), 15e-8}, {plate.get(), 58e-8}};
    for (const auto& c : cases) {
        const auto copy = model::Pattern::transformed(*c.solid, m);
        ASSERT_NE(copy, nullptr);
        const auto issues = topo::validateExact(*copy, math::ModelTolerance::forExtent(c.size));
        EXPECT_TRUE(issues.empty())
            << issues.size()
            << " issues, the first: " << (issues.empty() ? "" : issues.front().detail);
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

// Appended through a matrix that is not a similarity, a copy keeps no exact geometry: append is
// for placing solids rigidly. (Pattern::transformed carries it through any affine matrix as
// NURBS; test_ExactTransform.)
TEST(ExactPrimitives, ANonSimilarityLeavesAnAppendedCopyFaceted) {
    const auto block = model::exact::makeBlock({0, 0, 0}, {10, 20, 30});
    math::Mat4 shear = math::Mat4::identity();
    shear.m[0][1] = 0.5;
    for (const math::Mat4& m : {math::Mat4::scale({2, 1, 1}), shear}) {
        const auto copy = std::make_unique<topo::Solid>();
        model::Pattern::append(*copy, *block, m);
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

// ---- Stage 3: every primitive exact, and faces that meet at poles between meridians ----

namespace {

constexpr double kPi = math::kPi;

/// The frames each primitive is built in: the world's, and one turned and moved off it.
std::vector<geo::Frame> placements() {
    return {geo::Frame{}, geo::Frame::fromAxis({3, -2, 7}, Vec3{1, 2, 2} * (1.0 / 3.0), {0, 0, 1})};
}

/// The volume of the apple a spindle torus (R < r) makes, by Pappus: the part of its tube's disc
/// on the outer side of the axis, turned about it. The disc's part beyond the axis is a segment
/// of half-chord c = sqrt(r^2 - R^2), area r^2 acos(R / r) - R c, whose centroid lies 2 c^3 / (3
/// area) from the disc's centre; what is left has the moment pi r^2 R less that segment's about
/// the axis, R A_major + 2 c^3 / 3.
double appleVolume(double major, double minor) {
    const double c = std::sqrt(minor * minor - major * major);
    const double cut = minor * minor * std::acos(major / minor) - major * c;
    const double kept = kPi * minor * minor - cut;
    return math::kTwoPi * (kept * major + 2.0 * c * c * c / 3.0);
}

/// A primitive as built, what it should be: its face count and closed-form volume.
struct Expected {
    std::string what;
    std::function<std::unique_ptr<topo::Solid>(const geo::Frame&)> make;
    std::size_t faces;
    double volume;
};

std::vector<Expected> eachPrimitive() {
    return {
        {"box 2x3x4", [](const geo::Frame& f) { return model::exact::makeBox(f, 2, 3, 4); }, 6,
         24.0},
        {"sphere r = 3", [](const geo::Frame& f) { return model::exact::makeSphere(f, 3); }, 1,
         4.0 / 3.0 * kPi * 27.0},
        {"frustum (2, 1, 4)",
         [](const geo::Frame& f) { return model::exact::makeCone(f, 2, 1, 4); }, 3,
         kPi * 4.0 / 3.0 * (4.0 + 2.0 + 1.0)},
        {"apex cone (2, 0, 4)",
         [](const geo::Frame& f) { return model::exact::makeCone(f, 2, 0, 4); }, 2,
         kPi * 4.0 * 4.0 / 3.0},
        {"torus (5, 1)", [](const geo::Frame& f) { return model::exact::makeTorus(f, 5, 1); }, 1,
         2.0 * kPi * kPi * 5.0},
        {"spindle (1, 2)", [](const geo::Frame& f) { return model::exact::makeTorus(f, 1, 2); }, 1,
         appleVolume(1.0, 2.0)},
    };
}

/// @p solid validates, meshes at the read point with no problem, and measures @p volume to 1e-9.
void expectValidMeshedAndExact(const topo::Solid& solid, double volume) {
    const auto issues = topo::validateExact(solid, topo::exactTolerance(solid));
    EXPECT_TRUE(issues.empty()) << issues.size() << " issues, the first: "
                                << (issues.empty() ? "" : issues.front().detail);
    std::string problem;
    const auto polygons = model::BoundaryMesh::extractFacePolygons(
        solid, model::BoundaryMesh::defaultChordTolerance(solid), &problem);
    EXPECT_TRUE(problem.empty()) << problem;
    EXPECT_FALSE(polygons.empty());
    const model::MassProperties props = model::MassPropertiesCalculator::compute(solid);
    ASSERT_TRUE(props.valid) << props.problem;
    EXPECT_NEAR(props.volume, volume, 1e-9 * volume);
}

/// The set of @p solid's live face tags.
std::set<std::string> faceTags(const std::unique_ptr<topo::Solid>& solid) {
    std::set<std::string> tags;
    if (solid == nullptr) {
        ADD_FAILURE() << "the exact builder made nothing";
        return tags;
    }
    for (const auto& f : solid->faces()) {
        if (f.alive) tags.insert(f.topoId.tag());
    }
    return tags;
}

/// The logical faces of a faceted primitive of master's: its facets named as the Stable scheme
/// names them, each read back as the face it is of.
std::set<std::string> logicalTags(std::unique_ptr<topo::Solid> faceted) {
    std::set<std::string> tags;
    if (faceted == nullptr) {
        ADD_FAILURE() << "master's factory made nothing";
        return tags;
    }
    model::nameFacetsLogically(*faceted);
    for (const auto& f : faceted->faces()) {
        if (f.alive) tags.insert(model::logicalFace(f.topoId.tag()));
    }
    return tags;
}

}  // namespace

TEST(ExactPrimitives, EachIsValidAndExact) {
    for (const geo::Frame& frame : placements()) {
        for (const Expected& e : eachPrimitive()) {
            SCOPED_TRACE(e.what);
            const auto solid = e.make(frame);
            ASSERT_NE(solid, nullptr);
            EXPECT_TRUE(solid->isValid()) << solid->validationReport();
            EXPECT_EQ(solid->faceCount(), e.faces);
            for (const auto& f : solid->faces()) {
                EXPECT_NE(f.exactSurface, nullptr);
                EXPECT_EQ(f.surface, nullptr) << "a constructed face has no NURBS carrier";
            }
            expectGeometryMeetsTopology(*solid);
            expectValidMeshedAndExact(*solid, e.volume);
        }
    }
}

// A box spans its frame's own axes from its origin, a left-handed frame's y included: its faces
// still face out, their loops counter-clockwise about them.
TEST(ExactPrimitives, ABoxSpansItsFramesAxes) {
    for (const geo::Frame& right : placements()) {
        geo::Frame left = right;
        left.y = right.y * -1.0;
        for (const geo::Frame& frame : {right, left}) {
            const auto box = model::exact::makeBox(frame, 2, 3, 4);
            ASSERT_NE(box, nullptr);
            EXPECT_TRUE(box->isValid()) << box->validationReport();
            expectGeometryMeetsTopology(*box);
            expectValidMeshedAndExact(*box, 24.0);
            const Vec3 far = frame.origin + frame.x * 2.0 + frame.y * 3.0 + frame.z * 4.0;
            const auto& vs = box->vertices();
            EXPECT_TRUE(std::any_of(vs.begin(), vs.end(), [&far](const topo::Vertex& v) {
                return v.point.distanceTo(far) < 1e-12;
            }));
            for (const auto& f : box->faces()) {
                if (f.topoId.tag() != "box/back") continue;
                const auto* plane = dynamic_cast<const geo::Plane*>(f.exactSurface.get());
                ASSERT_NE(plane, nullptr);
                EXPECT_NEAR(plane->frame().z.dot(frame.y), 1.0, 1e-12) << "back faces +y";
            }
        }
    }
}

TEST(ExactPrimitives, NamesAreMastersLogicalNames) {
    const geo::Frame f;
    using model::PrimitiveFactory;
    EXPECT_EQ(faceTags(model::exact::makeBox(f, 2, 3, 4)),
              logicalTags(PrimitiveFactory::makeBox(2, 3, 4)));
    EXPECT_EQ(faceTags(model::exact::makeCylinder(f, 5, 10)),
              logicalTags(PrimitiveFactory::makeCylinder(5, 10)));
    EXPECT_EQ(faceTags(model::exact::makeSphere(f, 3)),
              logicalTags(PrimitiveFactory::makeSphere(3)));
    EXPECT_EQ(faceTags(model::exact::makeCone(f, 2, 1, 4)),
              logicalTags(PrimitiveFactory::makeCone(2, 1, 4)));
    EXPECT_EQ(faceTags(model::exact::makeCone(f, 2, 0, 4)),
              logicalTags(PrimitiveFactory::makeCone(2, 0, 4)));
    EXPECT_EQ(faceTags(model::exact::makeCone(f, 0, 2, 4)),
              logicalTags(PrimitiveFactory::makeCone(0, 2, 4)));
    EXPECT_EQ(faceTags(model::exact::makeCone(f, 2, 2, 4)),
              logicalTags(PrimitiveFactory::makeCone(2, 2, 4)));
    EXPECT_EQ(faceTags(model::exact::makeTorus(f, 5, 1)),
              logicalTags(PrimitiveFactory::makeTorus(5, 1)));
    // Master makes no spindle (it refuses R <= r): the apple is named as the torus it stands for.
    EXPECT_EQ(faceTags(model::exact::makeTorus(f, 1, 2)),
              logicalTags(PrimitiveFactory::makeTorus(5, 1)));
}

// A cone whose radii are alike is a cylinder, as master tags its side; a frustum narrowing upward
// opens downward (its half-angle positive, its normal out), a widening one upward.
TEST(ExactPrimitives, ConeSidesAreTheSurfacesMasterTagsThem) {
    const geo::Frame f;
    const auto kindOfSide = [](const topo::Solid& s) {
        for (const auto& face : s.faces()) {
            if (face.topoId.tag() == "cone/side") return face.exactSurface->kind();
        }
        return geo::SurfaceKind::Nurbs;
    };
    const auto alike = model::exact::makeCone(f, 2, 2, 4);
    ASSERT_NE(alike, nullptr);
    EXPECT_EQ(kindOfSide(*alike), geo::SurfaceKind::Cylinder);
    for (const auto& radii :
         {std::pair{2.0, 1.0}, std::pair{1.0, 2.0}, std::pair{2.0, 0.0}, std::pair{0.0, 2.0}}) {
        const auto cone = model::exact::makeCone(f, radii.first, radii.second, 4);
        ASSERT_NE(cone, nullptr);
        EXPECT_EQ(kindOfSide(*cone), geo::SurfaceKind::Cone);
        for (const auto& face : cone->faces()) {
            if (const auto* c = dynamic_cast<const geo::Cone*>(face.exactSurface.get())) {
                EXPECT_GT(c->halfAngle(), 0.0);
                EXPECT_GE(c->radius(), 0.0);
            }
        }
        expectValidMeshedAndExact(*cone,
                                  kPi * 4.0 / 3.0 *
                                      (radii.first * radii.first + radii.first * radii.second +
                                       radii.second * radii.second));
    }
}

// The surfaces of revolution seam at u = 0 along the frame's x, where master's rings start, and a
// left-handed frame builds the right-handed one's solid.
TEST(ExactPrimitives, TurnedPrimitivesSeamAlongTheFramesX) {
    for (const geo::Frame& right : placements()) {
        geo::Frame left = right;
        left.y = right.y * -1.0;
        for (const geo::Frame& frame : {right, left}) {
            for (const auto& solid :
                 {model::exact::makeSphere(frame, 3), model::exact::makeCone(frame, 2, 1, 4),
                  model::exact::makeCone(frame, 2, 0, 4), model::exact::makeTorus(frame, 5, 1),
                  model::exact::makeTorus(frame, 1, 2)}) {
                ASSERT_NE(solid, nullptr);
                for (const auto& f : solid->faces()) {
                    if (f.exactSurface->kind() == geo::SurfaceKind::Plane) continue;
                    // The seam's middle (u = 0) lies in the half-plane of the frame's x.
                    const topo::HalfEdge* he = f.outerLoop->halfEdge;
                    bool seen = false;
                    do {
                        const math::Vec2 a = he->pcurve->evaluate(he->pcurveStart);
                        const math::Vec2 b = he->pcurve->evaluate(he->pcurveEnd);
                        if (a.x == 0.0 && b.x == 0.0) {
                            seen = true;
                            const math::Vec2 mid =
                                he->pcurve->evaluate(0.5 * (he->pcurveStart + he->pcurveEnd));
                            const Vec3 q = f.exactSurface->evaluate(mid.x, mid.y) - frame.origin;
                            EXPECT_NEAR(q.dot(right.y), 0.0, 1e-12) << f.topoId.tag();
                            EXPECT_GT(q.dot(right.x), 0.0) << f.topoId.tag();
                        }
                        he = he->next;
                    } while (he != f.outerLoop->halfEdge);
                    EXPECT_TRUE(seen) << f.topoId.tag();
                }
            }
        }
    }
}

// A horn torus (R = r) is the apple of a tube that touches the axis: its two poles are one
// vertex, and its volume is the ring torus's, 2 pi^2 R r^2. Its pole is a cusp (its arc meets the
// axis along it), which the read point's mesher does not mesh: it refines toward the cusp until a
// triangle's sides are below the model's resolution, and says so (as for a spindle within some 5%
// of a horn). Tracked in stage 3's list; it is valid and measures exactly.
TEST(ExactPrimitives, AHornTorusIsAnAppleWithOnePole) {
    for (const geo::Frame& frame : placements()) {
        const auto horn = model::exact::makeTorus(frame, 2, 2);
        ASSERT_NE(horn, nullptr);
        EXPECT_EQ(horn->faceCount(), 1u);
        EXPECT_EQ(horn->edgeCount(), 1u);
        EXPECT_EQ(horn->vertexCount(), 1u);
        EXPECT_NEAR(horn->vertices().front().point.distanceTo(frame.origin), 0.0, 1e-12);
        expectGeometryMeetsTopology(*horn);
        const auto issues = topo::validateExact(*horn, topo::exactTolerance(*horn));
        EXPECT_TRUE(issues.empty()) << issues.front().detail;
        const model::MassProperties props = model::MassPropertiesCalculator::compute(*horn);
        ASSERT_TRUE(props.valid) << props.problem;
        const double volume = 2.0 * kPi * kPi * 2.0 * 4.0;
        EXPECT_NEAR(props.volume, volume, 1e-9 * volume);
    }
}

TEST(ExactPrimitives, ASphereSectorIsValidAndExact) {
    const double r = 3.0;
    for (const geo::Frame& frame : placements()) {
        for (const double angle : {kPi / 2.0, 0.01, math::kTwoPi - 0.01}) {
            SCOPED_TRACE(angle);
            const auto sector = model::exact::makeSphereSector(frame, r, angle);
            ASSERT_NE(sector, nullptr);
            EXPECT_TRUE(sector->isValid()) << sector->validationReport();
            EXPECT_TRUE(sector->checkEulerFormula());
            EXPECT_EQ(sector->faceCount(), 3u);
            expectGeometryMeetsTopology(*sector);
            // A lune of the angle's share of the ball: pi r^3 / 3 at a quarter turn.
            expectValidMeshedAndExact(*sector, 2.0 / 3.0 * angle * r * r * r);
        }
    }
}

TEST(ExactPrimitives, EveryEdgeIsNamed) {
    const geo::Frame f = placements().back();
    std::vector<std::unique_ptr<topo::Solid>> solids;
    for (const Expected& e : eachPrimitive()) solids.push_back(e.make(f));
    solids.push_back(model::exact::makeCylinder(f, 5, 10));
    solids.push_back(model::exact::makeCone(f, 0, 2, 4));
    solids.push_back(model::exact::makeSphereSector(f, 3, kPi / 2.0));
    for (auto& solid : solids) {
        ASSERT_NE(solid, nullptr);
        model::nameEdgesLogically(*solid);
        std::set<std::string> tags;
        for (const auto& e : solid->edges()) {
            const std::string& tag = e.topoId.tag();
            EXPECT_FALSE(tag.empty());
            EXPECT_TRUE(tags.insert(tag).second) << tag << " names two edges";
        }
    }
}

TEST(ExactPrimitives, DegeneratePrimitivesGiveNothing) {
    const geo::Frame f;
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_EQ(model::exact::makeBox(f, 0, 3, 4), nullptr);
    EXPECT_EQ(model::exact::makeBox(f, 2, -3, 4), nullptr);
    EXPECT_EQ(model::exact::makeBox(f, 2, 3, inf), nullptr);
    EXPECT_EQ(model::exact::makeSphere(f, 0), nullptr);
    EXPECT_EQ(model::exact::makeSphere(f, -1), nullptr);
    EXPECT_EQ(model::exact::makeCone(f, 0, 0, 4), nullptr);
    EXPECT_EQ(model::exact::makeCone(f, -1, 2, 4), nullptr);
    EXPECT_EQ(model::exact::makeCone(f, 2, 1, 0), nullptr);
    EXPECT_EQ(model::exact::makeCone(f, std::nan(""), 1, 4), nullptr);
    EXPECT_EQ(model::exact::makeTorus(f, 0, 1), nullptr);
    EXPECT_EQ(model::exact::makeTorus(f, 5, 0), nullptr);
    EXPECT_EQ(model::exact::makeSphereSector(f, 3, 0), nullptr);
    EXPECT_EQ(model::exact::makeSphereSector(f, 3, math::kTwoPi), nullptr);
    EXPECT_EQ(model::exact::makeSphereSector(f, 0, 1), nullptr);
}

namespace {

/// The area of each face of @p solid's mesh at the read point's chord tolerance, by its tag.
std::map<std::string, double> meshedAreas(const topo::Solid& solid) {
    model::MeshOptions options;
    options.chordTolerance = model::BoundaryMesh::defaultChordTolerance(solid);
    std::map<std::string, double> areas;
    for (const model::FaceMesh& m : model::TrimmedFaceMesher::meshSolid(solid, options)) {
        EXPECT_TRUE(m.problem.empty()) << m.problem;
        double& area = areas[m.face->topoId.tag()];
        for (const auto& t : m.triangles) {
            const Vec3& a = m.points[t[0]];
            area += 0.5 * (m.points[t[1]] - a).cross(m.points[t[2]] - a).length();
        }
    }
    return areas;
}

}  // namespace

// The spec's regression class "a ball split into two faces by two meridians": a lune whose
// sphere's seam lies inside it, its meridians' pcurves written within [0, 2 pi), and a ball cut
// in two along two meridians, each face's pcurves within [0, 2 pi). Laid straight across the
// window they are written in, their loops ran round the poles the wrong way: the validator and
// the mass said the loop ran against the face's sense, and the mesher meshed the other lune (a
// quarter lune's band came out three quarters of the sphere). Each runs round its poles the way
// its face's sense says, and is valid, meshed as itself and exact.
TEST(ExactPrimitives, LunesWhereverTheirSpheresSeamLiesAreValidAndExact) {
    const double r = 3.0;
    for (const geo::Frame& frame : placements()) {
        for (const double angle : {kPi / 2.0, 1.5 * kPi, 0.01, math::kTwoPi - 0.01}) {
            SCOPED_TRACE(angle);
            const auto lune = pole_solids::luneAcrossTheSeam(frame, r, angle);
            expectValidMeshedAndExact(*lune, 2.0 / 3.0 * angle * r * r * r);
            const double band = 2.0 * angle * r * r;
            EXPECT_NEAR(meshedAreas(*lune)["sphere/band"], band, 1e-3 * band);
        }
        for (const double split : {kPi / 2.0, 1.5 * kPi}) {
            SCOPED_TRACE(split);
            const auto ball = pole_solids::ballSplitByMeridians(frame, r, split);
            expectValidMeshedAndExact(*ball, 4.0 / 3.0 * kPi * r * r * r);
            auto areas = meshedAreas(*ball);
            EXPECT_NEAR(areas["sphere/a"], 2.0 * split * r * r, 1e-3 * 4.0 * kPi * r * r);
            EXPECT_NEAR(areas["sphere/b"], 2.0 * (math::kTwoPi - split) * r * r,
                        1e-3 * 4.0 * kPi * r * r);
        }
    }
}

// A cone sector to its apex: the side's loop runs along the apex from one generator to the
// other, a run of the sector's angle, not a whole turn.
TEST(ExactPrimitives, AConeSectorToItsApexIsValidAndExact) {
    const double r = 2.0;
    const double h = 4.0;
    for (const geo::Frame& frame : placements()) {
        for (const double angle : {kPi / 2.0, 0.01, math::kTwoPi - 0.01}) {
            SCOPED_TRACE(angle);
            const auto sector = pole_solids::coneSector(frame, r, h, angle);
            EXPECT_TRUE(sector->isValid()) << sector->validationReport();
            expectGeometryMeetsTopology(*sector);
            expectValidMeshedAndExact(*sector, angle * r * r * h / 6.0);
        }
    }
}
