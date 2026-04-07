#include <gtest/gtest.h>

#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"

using namespace hz::draft;
using namespace hz::math;

TEST(SketchPlaneEdgeCaseTest, RayParallelToPlane) {
    SketchPlane plane;  // XY at Z=0
    Vec3 rayOrigin(0, 0, 5);
    Vec3 rayDir(1, 0, 0);  // Parallel to plane
    Vec2 result;
    EXPECT_FALSE(plane.rayIntersect(rayOrigin, rayDir, result));
}

TEST(SketchPlaneEdgeCaseTest, RayFromBehindPlane) {
    SketchPlane plane;  // XY at Z=0
    Vec3 rayOrigin(0, 0, -5);  // Below plane
    Vec3 rayDir(0, 0, 1);      // Shooting upward
    Vec2 result;
    EXPECT_TRUE(plane.rayIntersect(rayOrigin, rayDir, result));
    EXPECT_NEAR(result.x, 0.0, 1e-10);
    EXPECT_NEAR(result.y, 0.0, 1e-10);
}

TEST(SketchPlaneEdgeCaseTest, LargeCoordinates) {
    SketchPlane plane;
    Vec2 local(1e6, -1e6);
    Vec3 world = plane.localToWorld(local);
    Vec2 back = plane.worldToLocal(world);
    EXPECT_NEAR(back.x, local.x, 1e-3);
    EXPECT_NEAR(back.y, local.y, 1e-3);
}

TEST(SketchPlaneEdgeCaseTest, VerySmallCoordinates) {
    SketchPlane plane;
    Vec2 local(1e-8, 1e-8);
    Vec3 world = plane.localToWorld(local);
    Vec2 back = plane.worldToLocal(world);
    EXPECT_NEAR(back.x, local.x, 1e-15);
    EXPECT_NEAR(back.y, local.y, 1e-15);
}

TEST(SketchPlaneEdgeCaseTest, OriginAtLargeOffset) {
    SketchPlane plane(Vec3(1e6, 1e6, 1e6), Vec3::UnitZ, Vec3::UnitX);
    Vec2 local(10, 20);
    Vec3 world = plane.localToWorld(local);
    Vec2 back = plane.worldToLocal(world);
    EXPECT_NEAR(back.x, local.x, 1e-6);
    EXPECT_NEAR(back.y, local.y, 1e-6);
}

TEST(SketchPlaneEdgeCaseTest, RayAtGrazingAngle) {
    SketchPlane plane;
    Vec3 rayOrigin(0, 0, 1);
    Vec3 rayDir = Vec3(1, 0, -1e-6).normalized();  // Nearly parallel
    Vec2 result;
    // Should still intersect (not parallel enough to fail)
    EXPECT_TRUE(plane.rayIntersect(rayOrigin, rayDir, result));
}
