#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "horizon/modeling/ReferenceGeometry.h"

using namespace hz::model;
using namespace hz::model::refgeo;
using hz::math::Vec3;

namespace {
constexpr double kTol = 1e-9;

void expectVecNear(const Vec3& a, const Vec3& b, double tol = kTol) {
    EXPECT_NEAR(a.x, b.x, tol);
    EXPECT_NEAR(a.y, b.y, tol);
    EXPECT_NEAR(a.z, b.z, tol);
}

DatumPlane xyPlane() {
    return DatumPlane{Vec3::Zero, Vec3::UnitZ, Vec3::UnitX};
}
}  // namespace

// ---------------------------------------------------------------------------
// Datum planes
// ---------------------------------------------------------------------------

TEST(ReferenceGeometryTest, PlaneOffsetShiftsAlongNormal) {
    DatumPlane p = planeOffset(xyPlane(), 5.0);
    expectVecNear(p.origin, Vec3(0, 0, 5));
    expectVecNear(p.normal, Vec3::UnitZ);
    expectVecNear(p.xAxis, Vec3::UnitX);
}

TEST(ReferenceGeometryTest, PlaneOffsetNormalizesNonUnitNormal) {
    DatumPlane base{Vec3::Zero, Vec3(0, 0, 2), Vec3::UnitX};  // non-unit normal
    DatumPlane p = planeOffset(base, 3.0);
    expectVecNear(p.origin, Vec3(0, 0, 3));  // 3 along the *unit* normal
    EXPECT_NEAR(p.normal.length(), 1.0, kTol);
}

TEST(ReferenceGeometryTest, PlaneThroughThreePoints) {
    auto p = planeThroughPoints(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0));
    ASSERT_TRUE(p.has_value());
    expectVecNear(p->origin, Vec3::Zero);
    expectVecNear(p->normal, Vec3::UnitZ);  // (1,0,0)×(0,1,0) = +Z
    expectVecNear(p->xAxis, Vec3::UnitX);
}

TEST(ReferenceGeometryTest, PlaneThroughCollinearPointsRejected) {
    auto p = planeThroughPoints(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(2, 0, 0));
    EXPECT_FALSE(p.has_value());
}

TEST(ReferenceGeometryTest, PlaneAtAngleRotatesAboutHinge) {
    // Rotate the XY plane 90° about the X axis → normal goes +Z → -Y (or +Y
    // depending on sense); it must become horizontal and contain the hinge.
    DatumPlane p = planeAtAngle(xyPlane(), Vec3(3, 0, 0), Vec3::UnitX, std::numbers::pi / 2.0);
    expectVecNear(p.origin, Vec3(3, 0, 0));                       // contains hinge origin
    EXPECT_NEAR(std::abs(p.normal.dot(Vec3::UnitZ)), 0.0, kTol);  // no longer +Z
    EXPECT_NEAR(p.normal.length(), 1.0, kTol);
    // Rotating +Z by +90° about +X gives -Y.
    expectVecNear(p.normal, Vec3(0, -1, 0));
}

TEST(ReferenceGeometryTest, PlaneMidplaneBetweenParallelPlanes) {
    DatumPlane a{Vec3(0, 0, 0), Vec3::UnitZ, Vec3::UnitX};
    DatumPlane b{Vec3(0, 0, 10), Vec3::UnitZ, Vec3::UnitX};
    DatumPlane m = planeMidplane(a, b);
    expectVecNear(m.origin, Vec3(0, 0, 5));
    expectVecNear(m.normal, Vec3::UnitZ);
}

TEST(ReferenceGeometryTest, PlaneMidplaneAlignsOpposedNormals) {
    // Two faces of a slab point in opposite directions; the midplane normal
    // should still be well-defined (not the near-zero sum of +Z and -Z).
    DatumPlane a{Vec3(0, 0, 0), Vec3::UnitZ, Vec3::UnitX};
    DatumPlane b{Vec3(0, 0, 4), Vec3(0, 0, -1), Vec3::UnitX};
    DatumPlane m = planeMidplane(a, b);
    expectVecNear(m.origin, Vec3(0, 0, 2));
    EXPECT_NEAR(m.normal.length(), 1.0, kTol);
    EXPECT_NEAR(std::abs(m.normal.dot(Vec3::UnitZ)), 1.0, kTol);
}

TEST(ReferenceGeometryTest, ToSketchPlaneIsOrthonormal) {
    auto p = planeThroughPoints(Vec3(1, 2, 3), Vec3(4, 2, 3), Vec3(1, 5, 3));
    ASSERT_TRUE(p.has_value());
    auto sp = p->toSketchPlane();
    EXPECT_NEAR(sp.normal().length(), 1.0, kTol);
    EXPECT_NEAR(sp.xAxis().length(), 1.0, kTol);
    EXPECT_NEAR(sp.yAxis().length(), 1.0, kTol);
    EXPECT_NEAR(sp.normal().dot(sp.xAxis()), 0.0, kTol);
    EXPECT_NEAR(sp.normal().dot(sp.yAxis()), 0.0, kTol);
    EXPECT_NEAR(sp.xAxis().dot(sp.yAxis()), 0.0, kTol);
}

// ---------------------------------------------------------------------------
// Datum axes
// ---------------------------------------------------------------------------

TEST(ReferenceGeometryTest, AxisThroughTwoPoints) {
    auto a = axisThroughPoints(Vec3(1, 1, 1), Vec3(1, 1, 6));
    ASSERT_TRUE(a.has_value());
    expectVecNear(a->origin, Vec3(1, 1, 1));
    expectVecNear(a->direction, Vec3::UnitZ);
}

TEST(ReferenceGeometryTest, AxisThroughCoincidentPointsRejected) {
    auto a = axisThroughPoints(Vec3(2, 2, 2), Vec3(2, 2, 2));
    EXPECT_FALSE(a.has_value());
}

TEST(ReferenceGeometryTest, AxisPlaneIntersection) {
    // XY plane (z=0) ∩ XZ plane (y=0) = the X axis.
    DatumPlane xy{Vec3::Zero, Vec3::UnitZ, Vec3::UnitX};
    DatumPlane xz{Vec3::Zero, Vec3::UnitY, Vec3::UnitX};
    auto a = axisPlaneIntersection(xy, xz);
    ASSERT_TRUE(a.has_value());
    // Direction is ±X; the axis passes through the origin.
    EXPECT_NEAR(std::abs(a->direction.dot(Vec3::UnitX)), 1.0, kTol);
    EXPECT_NEAR(a->origin.y, 0.0, kTol);
    EXPECT_NEAR(a->origin.z, 0.0, kTol);
}

TEST(ReferenceGeometryTest, AxisPlaneIntersectionParallelRejected) {
    DatumPlane a{Vec3(0, 0, 0), Vec3::UnitZ, Vec3::UnitX};
    DatumPlane b{Vec3(0, 0, 5), Vec3::UnitZ, Vec3::UnitX};
    EXPECT_FALSE(axisPlaneIntersection(a, b).has_value());
}

TEST(ReferenceGeometryTest, AxisFromDirectionNormalizes) {
    DatumAxis a = axisFromDirection(Vec3(2, 0, 0), Vec3(0, 0, 4));
    expectVecNear(a.origin, Vec3(2, 0, 0));
    expectVecNear(a.direction, Vec3::UnitZ);
}

// ---------------------------------------------------------------------------
// Datum points
// ---------------------------------------------------------------------------

TEST(ReferenceGeometryTest, PointAt) {
    DatumPoint p = pointAt(Vec3(1, 2, 3));
    expectVecNear(p.position, Vec3(1, 2, 3));
}

TEST(ReferenceGeometryTest, PointCentroidOfSquare) {
    std::vector<Vec3> corners{Vec3(0, 0, 0), Vec3(2, 0, 0), Vec3(2, 2, 0), Vec3(0, 2, 0)};
    auto c = pointCentroid(corners);
    ASSERT_TRUE(c.has_value());
    expectVecNear(c->position, Vec3(1, 1, 0));
}

TEST(ReferenceGeometryTest, PointCentroidEmptyRejected) {
    EXPECT_FALSE(pointCentroid({}).has_value());
}

TEST(ReferenceGeometryTest, PointLineIntersectionCrossingLines) {
    // X axis and Y axis cross at the origin.
    DatumAxis x{Vec3::Zero, Vec3::UnitX};
    DatumAxis y{Vec3::Zero, Vec3::UnitY};
    auto p = pointLineIntersection(x, y);
    ASSERT_TRUE(p.has_value());
    expectVecNear(p->position, Vec3::Zero);
}

TEST(ReferenceGeometryTest, PointLineIntersectionSkewLinesMidpoint) {
    // X axis at z=0 and a Y-parallel line at (3, ·, 4): closest approach is the
    // midpoint of the common perpendicular.
    DatumAxis a{Vec3(0, 0, 0), Vec3::UnitX};
    DatumAxis b{Vec3(3, 0, 4), Vec3::UnitY};
    auto p = pointLineIntersection(a, b);
    ASSERT_TRUE(p.has_value());
    expectVecNear(p->position, Vec3(3, 0, 2));  // midpoint between (3,0,0) and (3,0,4)
}

TEST(ReferenceGeometryTest, PointLineIntersectionParallelRejected) {
    DatumAxis a{Vec3(0, 0, 0), Vec3::UnitX};
    DatumAxis b{Vec3(0, 1, 0), Vec3::UnitX};
    EXPECT_FALSE(pointLineIntersection(a, b).has_value());
}
