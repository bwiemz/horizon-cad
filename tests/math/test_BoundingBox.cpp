#include <gtest/gtest.h>

#include "horizon/math/BoundingBox.h"
#include "horizon/math/Constants.h"

using namespace hz::math;

// ---------------------------------------------------------------------------
// 1. Default BoundingBox is not valid
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, DefaultIsNotValid) {
    BoundingBox bb;
    EXPECT_FALSE(bb.isValid());
}

// ---------------------------------------------------------------------------
// 2. Expand with point makes it valid
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, ExpandWithPointMakesValid) {
    BoundingBox bb;
    bb.expand(Vec3(1.0, 2.0, 3.0));
    EXPECT_TRUE(bb.isValid());
}

// ---------------------------------------------------------------------------
// 3. Contains point inside
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, ContainsPointInside) {
    BoundingBox bb(Vec3(-1.0, -1.0, -1.0), Vec3(1.0, 1.0, 1.0));
    EXPECT_TRUE(bb.contains(Vec3(0.0, 0.0, 0.0)));
    EXPECT_TRUE(bb.contains(Vec3(0.5, 0.5, 0.5)));
    EXPECT_TRUE(bb.contains(Vec3(-0.5, -0.5, -0.5)));
}

// ---------------------------------------------------------------------------
// 4. Does not contain point outside
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, DoesNotContainPointOutside) {
    BoundingBox bb(Vec3(-1.0, -1.0, -1.0), Vec3(1.0, 1.0, 1.0));
    EXPECT_FALSE(bb.contains(Vec3(2.0, 0.0, 0.0)));
    EXPECT_FALSE(bb.contains(Vec3(0.0, -5.0, 0.0)));
    EXPECT_FALSE(bb.contains(Vec3(0.0, 0.0, 10.0)));
}

// ---------------------------------------------------------------------------
// 5. Intersects overlapping boxes
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, IntersectsOverlapping) {
    BoundingBox a(Vec3(-1.0, -1.0, -1.0), Vec3(1.0, 1.0, 1.0));
    BoundingBox b(Vec3(0.0, 0.0, 0.0), Vec3(2.0, 2.0, 2.0));
    EXPECT_TRUE(a.intersects(b));
    EXPECT_TRUE(b.intersects(a));
}

// ---------------------------------------------------------------------------
// 6. Does not intersect disjoint boxes
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, DoesNotIntersectDisjoint) {
    BoundingBox a(Vec3(-2.0, -2.0, -2.0), Vec3(-1.0, -1.0, -1.0));
    BoundingBox b(Vec3(1.0, 1.0, 1.0), Vec3(2.0, 2.0, 2.0));
    EXPECT_FALSE(a.intersects(b));
    EXPECT_FALSE(b.intersects(a));
}

// ---------------------------------------------------------------------------
// 7. Center and size correct
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, CenterAndSize) {
    BoundingBox bb(Vec3(0.0, 0.0, 0.0), Vec3(4.0, 6.0, 8.0));
    Vec3 center = bb.center();
    Vec3 size = bb.size();
    EXPECT_NEAR(center.x, 2.0, 1e-10);
    EXPECT_NEAR(center.y, 3.0, 1e-10);
    EXPECT_NEAR(center.z, 4.0, 1e-10);
    EXPECT_NEAR(size.x, 4.0, 1e-10);
    EXPECT_NEAR(size.y, 6.0, 1e-10);
    EXPECT_NEAR(size.z, 8.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 8. Expand with another BoundingBox
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, ExpandWithBoundingBox) {
    BoundingBox a(Vec3(-1.0, -1.0, -1.0), Vec3(1.0, 1.0, 1.0));
    BoundingBox b(Vec3(0.0, 0.0, 0.0), Vec3(3.0, 3.0, 3.0));
    a.expand(b);
    EXPECT_NEAR(a.min().x, -1.0, 1e-10);
    EXPECT_NEAR(a.min().y, -1.0, 1e-10);
    EXPECT_NEAR(a.min().z, -1.0, 1e-10);
    EXPECT_NEAR(a.max().x, 3.0, 1e-10);
    EXPECT_NEAR(a.max().y, 3.0, 1e-10);
    EXPECT_NEAR(a.max().z, 3.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 9. Constructed BoundingBox with min/max is valid
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, ConstructedWithMinMaxIsValid) {
    BoundingBox bb(Vec3(-1.0, -2.0, -3.0), Vec3(1.0, 2.0, 3.0));
    EXPECT_TRUE(bb.isValid());
}

// ---------------------------------------------------------------------------
// 10. Expand with multiple points
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, ExpandWithMultiplePoints) {
    BoundingBox bb;
    bb.expand(Vec3(1.0, 2.0, 3.0));
    bb.expand(Vec3(-1.0, -2.0, -3.0));
    bb.expand(Vec3(5.0, 0.0, 0.0));

    EXPECT_NEAR(bb.min().x, -1.0, 1e-10);
    EXPECT_NEAR(bb.min().y, -2.0, 1e-10);
    EXPECT_NEAR(bb.min().z, -3.0, 1e-10);
    EXPECT_NEAR(bb.max().x, 5.0, 1e-10);
    EXPECT_NEAR(bb.max().y, 2.0, 1e-10);
    EXPECT_NEAR(bb.max().z, 3.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 11. Reset makes BoundingBox invalid
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, ResetMakesInvalid) {
    BoundingBox bb(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 1.0, 1.0));
    EXPECT_TRUE(bb.isValid());
    bb.reset();
    EXPECT_FALSE(bb.isValid());
}

// ---------------------------------------------------------------------------
// 12. Diagonal
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, Diagonal) {
    BoundingBox bb(Vec3(0.0, 0.0, 0.0), Vec3(3.0, 4.0, 0.0));
    // diagonal = sqrt(3^2 + 4^2 + 0^2) = 5
    EXPECT_NEAR(bb.diagonal(), 5.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 13. Contains point on boundary
// ---------------------------------------------------------------------------
TEST(BoundingBoxTest, ContainsPointOnBoundary) {
    BoundingBox bb(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 1.0, 1.0));
    // Points exactly on the boundary should be contained
    EXPECT_TRUE(bb.contains(Vec3(0.0, 0.0, 0.0)));
    EXPECT_TRUE(bb.contains(Vec3(1.0, 1.0, 1.0)));
    EXPECT_TRUE(bb.contains(Vec3(1.0, 0.5, 0.5)));
}
