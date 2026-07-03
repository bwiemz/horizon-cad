#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "horizon/modeling/Measure.h"

using namespace hz::model::measure;
using hz::math::Vec3;

TEST(MeasureTest, PointToPointDistance) {
    EXPECT_NEAR(distance(Vec3(0, 0, 0), Vec3(3, 4, 0)), 5.0, 1e-9);
    EXPECT_NEAR(distance(Vec3(1, 1, 1), Vec3(1, 1, 1)), 0.0, 1e-9);
}

TEST(MeasureTest, AngleBetweenDirections) {
    EXPECT_NEAR(angleBetween(Vec3(1, 0, 0), Vec3(0, 1, 0)), std::numbers::pi / 2.0, 1e-9);
    EXPECT_NEAR(angleBetween(Vec3(1, 0, 0), Vec3(1, 0, 0)), 0.0, 1e-9);
    EXPECT_NEAR(angleBetween(Vec3(1, 0, 0), Vec3(-1, 0, 0)), std::numbers::pi, 1e-9);
    // Magnitude-independent.
    EXPECT_NEAR(angleBetween(Vec3(2, 0, 0), Vec3(0, 5, 0)), std::numbers::pi / 2.0, 1e-9);
    // Degenerate input → 0.
    EXPECT_NEAR(angleBetween(Vec3(0, 0, 0), Vec3(1, 0, 0)), 0.0, 1e-12);
}

TEST(MeasureTest, PointToSegment) {
    // Perpendicular foot inside the segment.
    EXPECT_NEAR(pointToSegment(Vec3(1, 2, 0), Vec3(0, 0, 0), Vec3(4, 0, 0)), 2.0, 1e-9);
    // Beyond an endpoint → distance to that endpoint.
    EXPECT_NEAR(pointToSegment(Vec3(-3, 0, 0), Vec3(0, 0, 0), Vec3(4, 0, 0)), 3.0, 1e-9);
    // Degenerate segment → distance to the point.
    EXPECT_NEAR(pointToSegment(Vec3(0, 3, 0), Vec3(1, 3, 0), Vec3(1, 3, 0)), 1.0, 1e-9);
}

TEST(MeasureTest, SegmentToSegmentCrossing) {
    // Two segments crossing at the origin (coplanar) → distance 0.
    EXPECT_NEAR(segmentToSegment(Vec3(-1, 0, 0), Vec3(1, 0, 0), Vec3(0, -1, 0), Vec3(0, 1, 0)), 0.0,
                1e-9);
}

TEST(MeasureTest, SegmentToSegmentSkew) {
    // X-axis segment at z=0 and a Y-axis segment lifted to z=5 → gap 5.
    EXPECT_NEAR(segmentToSegment(Vec3(-5, 0, 0), Vec3(5, 0, 0), Vec3(0, -5, 5), Vec3(0, 5, 5)), 5.0,
                1e-9);
}

TEST(MeasureTest, SegmentToSegmentParallel) {
    // Parallel unit segments offset by 3 in Y.
    EXPECT_NEAR(segmentToSegment(Vec3(0, 0, 0), Vec3(4, 0, 0), Vec3(0, 3, 0), Vec3(4, 3, 0)), 3.0,
                1e-9);
}
