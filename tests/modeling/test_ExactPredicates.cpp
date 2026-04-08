#include <gtest/gtest.h>

#include "horizon/math/Vec3.h"
#include "horizon/modeling/ExactPredicates.h"
#include "horizon/modeling/PrimitiveFactory.h"

using hz::math::Vec3;
using hz::model::ExactPredicates;
using hz::model::PrimitiveFactory;

// --- orient3D tests ---

TEST(ExactPredicatesTest, PointAbovePlane) {
    Vec3 planePoint(0, 0, 0);
    Vec3 planeNormal(0, 0, 1);
    Vec3 testPoint(0, 0, 5);
    EXPECT_EQ(ExactPredicates::orient3D(planePoint, planeNormal, testPoint), 1);
}

TEST(ExactPredicatesTest, PointBelowPlane) {
    Vec3 planePoint(0, 0, 0);
    Vec3 planeNormal(0, 0, 1);
    Vec3 testPoint(0, 0, -3);
    EXPECT_EQ(ExactPredicates::orient3D(planePoint, planeNormal, testPoint), -1);
}

TEST(ExactPredicatesTest, PointOnPlane) {
    Vec3 planePoint(0, 0, 0);
    Vec3 planeNormal(0, 0, 1);
    Vec3 testPoint(5, 7, 0);
    EXPECT_EQ(ExactPredicates::orient3D(planePoint, planeNormal, testPoint), 0);
}

TEST(ExactPredicatesTest, PointNearPlaneWithinTolerance) {
    Vec3 planePoint(0, 0, 0);
    Vec3 planeNormal(0, 0, 1);
    Vec3 testPoint(1, 2, 1e-12);
    EXPECT_EQ(ExactPredicates::orient3D(planePoint, planeNormal, testPoint), 0);
}

TEST(ExactPredicatesTest, OrientWithTiltedPlane) {
    Vec3 planePoint(1, 1, 1);
    Vec3 planeNormal = Vec3(1, 1, 1).normalized();
    Vec3 above(3, 3, 3);
    Vec3 below(-1, -1, -1);
    EXPECT_EQ(ExactPredicates::orient3D(planePoint, planeNormal, above), 1);
    EXPECT_EQ(ExactPredicates::orient3D(planePoint, planeNormal, below), -1);
}

// --- classifyPoint tests ---

TEST(ExactPredicatesTest, PointInsideBox) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    // Center of box is at (1, 1, 1).
    Vec3 inside(1.0, 1.0, 1.0);
    EXPECT_EQ(ExactPredicates::classifyPoint(inside, *box), -1);
}

TEST(ExactPredicatesTest, PointOutsideBox) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    Vec3 outside(5.0, 5.0, 5.0);
    EXPECT_EQ(ExactPredicates::classifyPoint(outside, *box), 1);
}

TEST(ExactPredicatesTest, PointOnBoxSurface) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    // A vertex of the box (on boundary).
    Vec3 onSurface(0.0, 0.0, 0.0);
    EXPECT_EQ(ExactPredicates::classifyPoint(onSurface, *box), 0);
}
