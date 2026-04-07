#include <gtest/gtest.h>
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/math/Tolerance.h"

using namespace hz::draft;
using namespace hz::math;

TEST(SketchPlaneTest, DefaultIsXYPlane) {
    SketchPlane plane;
    EXPECT_TRUE(plane.origin().isApproxEqual(Vec3::Zero));
    EXPECT_TRUE(plane.normal().isApproxEqual(Vec3::UnitZ));
    EXPECT_TRUE(plane.xAxis().isApproxEqual(Vec3::UnitX));
    EXPECT_TRUE(plane.yAxis().isApproxEqual(Vec3::UnitY));
}

TEST(SketchPlaneTest, XYPlaneLocalEqualsWorld) {
    SketchPlane plane;  // Default XY
    Vec2 local(5.0, 3.0);
    Vec3 world = plane.localToWorld(local);
    EXPECT_NEAR(world.x, 5.0, 1e-10);
    EXPECT_NEAR(world.y, 3.0, 1e-10);
    EXPECT_NEAR(world.z, 0.0, 1e-10);
}

TEST(SketchPlaneTest, WorldToLocalOnXYPlane) {
    SketchPlane plane;
    Vec3 world(5.0, 3.0, 0.0);
    Vec2 local = plane.worldToLocal(world);
    EXPECT_NEAR(local.x, 5.0, 1e-10);
    EXPECT_NEAR(local.y, 3.0, 1e-10);
}

TEST(SketchPlaneTest, RoundTripOnXYPlane) {
    SketchPlane plane;
    Vec2 original(7.5, -2.3);
    Vec3 world = plane.localToWorld(original);
    Vec2 back = plane.worldToLocal(world);
    EXPECT_NEAR(back.x, original.x, 1e-10);
    EXPECT_NEAR(back.y, original.y, 1e-10);
}

TEST(SketchPlaneTest, OffsetXYPlane) {
    // XY plane elevated at Z=10
    SketchPlane plane(Vec3(0, 0, 10), Vec3::UnitZ, Vec3::UnitX);
    Vec2 local(5.0, 3.0);
    Vec3 world = plane.localToWorld(local);
    EXPECT_NEAR(world.x, 5.0, 1e-10);
    EXPECT_NEAR(world.y, 3.0, 1e-10);
    EXPECT_NEAR(world.z, 10.0, 1e-10);

    Vec2 back = plane.worldToLocal(world);
    EXPECT_NEAR(back.x, 5.0, 1e-10);
    EXPECT_NEAR(back.y, 3.0, 1e-10);
}

TEST(SketchPlaneTest, XZPlane) {
    // Sketch on XZ plane (normal = Y, X-axis = X)
    // Right-handed: yAxis = normal.cross(xAxis) = (0,1,0)x(1,0,0) = (0,0,-1)
    // So local Y maps to world -Z.
    SketchPlane plane(Vec3::Zero, Vec3::UnitY, Vec3::UnitX);
    Vec2 local(5.0, 3.0);
    Vec3 world = plane.localToWorld(local);
    EXPECT_NEAR(world.x, 5.0, 1e-10);
    EXPECT_NEAR(world.y, 0.0, 1e-10);
    EXPECT_NEAR(world.z, -3.0, 1e-10);

    // Round-trip
    Vec2 back = plane.worldToLocal(world);
    EXPECT_NEAR(back.x, 5.0, 1e-10);
    EXPECT_NEAR(back.y, 3.0, 1e-10);
}

TEST(SketchPlaneTest, AngledPlaneRoundTrip) {
    // 45-degree tilted plane
    Vec3 normal = Vec3(0, -1, 1).normalized();  // Tilted 45 from XY
    Vec3 xAxis = Vec3::UnitX;
    SketchPlane plane(Vec3(10, 20, 30), normal, xAxis);

    Vec2 local(4.0, 7.0);
    Vec3 world = plane.localToWorld(local);
    Vec2 back = plane.worldToLocal(world);
    EXPECT_NEAR(back.x, local.x, 1e-9);
    EXPECT_NEAR(back.y, local.y, 1e-9);
}

TEST(SketchPlaneTest, ArbitraryPlaneRoundTrip) {
    // Fully arbitrary plane
    Vec3 normal = Vec3(1, 2, 3).normalized();
    // X-axis must be perpendicular to normal -- use Gram-Schmidt
    Vec3 xAxis = Vec3(1, 0, 0);
    xAxis = (xAxis - normal * xAxis.dot(normal)).normalized();
    SketchPlane plane(Vec3(-5, 10, 15), normal, xAxis);

    Vec2 local(-3.5, 12.7);
    Vec3 world = plane.localToWorld(local);
    Vec2 back = plane.worldToLocal(world);
    EXPECT_NEAR(back.x, local.x, 1e-9);
    EXPECT_NEAR(back.y, local.y, 1e-9);
}

TEST(SketchPlaneTest, ProjectWorldPointOntoPlane) {
    SketchPlane plane;  // XY at Z=0
    // Point above the plane
    Vec3 point(5.0, 3.0, 10.0);
    Vec2 projected = plane.worldToLocal(point);
    EXPECT_NEAR(projected.x, 5.0, 1e-10);
    EXPECT_NEAR(projected.y, 3.0, 1e-10);
}

TEST(SketchPlaneTest, TransformMatricesAreInverses) {
    Vec3 normal = Vec3(1, 1, 1).normalized();
    Vec3 xAxis = Vec3(1, -1, 0).normalized();
    SketchPlane plane(Vec3(5, 10, 15), normal, xAxis);

    Mat4 toWorld = plane.localToWorldMatrix();
    Mat4 toLocal = plane.worldToLocalMatrix();

    // toWorld * toLocal should be identity
    Mat4 product = toWorld * toLocal;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            double expected = (r == c) ? 1.0 : 0.0;
            EXPECT_NEAR(product.at(r, c), expected, 1e-9)
                << "at (" << r << "," << c << ")";
        }
    }
}
