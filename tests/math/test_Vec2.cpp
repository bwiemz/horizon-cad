#include <gtest/gtest.h>

#include "horizon/math/Vec2.h"
#include "horizon/math/Constants.h"

using namespace hz::math;

// ---------------------------------------------------------------------------
// 1. Default constructor produces zero vector
// ---------------------------------------------------------------------------
TEST(Vec2Test, DefaultConstructorIsZero) {
    Vec2 v;
    EXPECT_DOUBLE_EQ(v.x, 0.0);
    EXPECT_DOUBLE_EQ(v.y, 0.0);
}

// ---------------------------------------------------------------------------
// 2. Parameterized constructor stores values
// ---------------------------------------------------------------------------
TEST(Vec2Test, ParameterizedConstructor) {
    Vec2 v(3.0, -7.5);
    EXPECT_DOUBLE_EQ(v.x, 3.0);
    EXPECT_DOUBLE_EQ(v.y, -7.5);
}

// ---------------------------------------------------------------------------
// 3. Addition
// ---------------------------------------------------------------------------
TEST(Vec2Test, Addition) {
    Vec2 a(1.0, 2.0);
    Vec2 b(3.0, 4.0);
    Vec2 c = a + b;
    EXPECT_NEAR(c.x, 4.0, 1e-10);
    EXPECT_NEAR(c.y, 6.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 4. Subtraction
// ---------------------------------------------------------------------------
TEST(Vec2Test, Subtraction) {
    Vec2 a(5.0, 8.0);
    Vec2 b(2.0, 3.0);
    Vec2 c = a - b;
    EXPECT_NEAR(c.x, 3.0, 1e-10);
    EXPECT_NEAR(c.y, 5.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 5. Scalar multiplication and division
// ---------------------------------------------------------------------------
TEST(Vec2Test, ScalarMultiply) {
    Vec2 v(2.0, -3.0);
    Vec2 r = v * 4.0;
    EXPECT_NEAR(r.x, 8.0, 1e-10);
    EXPECT_NEAR(r.y, -12.0, 1e-10);

    // left-hand scalar multiplication
    Vec2 l = 4.0 * v;
    EXPECT_NEAR(l.x, 8.0, 1e-10);
    EXPECT_NEAR(l.y, -12.0, 1e-10);
}

TEST(Vec2Test, ScalarDivide) {
    Vec2 v(8.0, -4.0);
    Vec2 r = v / 2.0;
    EXPECT_NEAR(r.x, 4.0, 1e-10);
    EXPECT_NEAR(r.y, -2.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 6. Dot product
// ---------------------------------------------------------------------------
TEST(Vec2Test, DotProduct) {
    Vec2 a(1.0, 0.0);
    Vec2 b(0.0, 1.0);
    EXPECT_NEAR(a.dot(b), 0.0, 1e-10);  // orthogonal

    Vec2 c(3.0, 4.0);
    Vec2 d(4.0, 3.0);
    EXPECT_NEAR(c.dot(d), 24.0, 1e-10);  // 3*4 + 4*3
}

// ---------------------------------------------------------------------------
// 7. Cross product (2D returns scalar)
// ---------------------------------------------------------------------------
TEST(Vec2Test, CrossProduct) {
    Vec2 a(1.0, 0.0);
    Vec2 b(0.0, 1.0);
    EXPECT_NEAR(a.cross(b), 1.0, 1e-10);   // i x j = +1
    EXPECT_NEAR(b.cross(a), -1.0, 1e-10);  // j x i = -1
}

// ---------------------------------------------------------------------------
// 8. Length and lengthSquared
// ---------------------------------------------------------------------------
TEST(Vec2Test, LengthAndLengthSquared) {
    Vec2 v(3.0, 4.0);
    EXPECT_NEAR(v.lengthSquared(), 25.0, 1e-10);
    EXPECT_NEAR(v.length(), 5.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 9. Normalize (including zero vector case)
// ---------------------------------------------------------------------------
TEST(Vec2Test, Normalize) {
    Vec2 v(3.0, 4.0);
    Vec2 n = v.normalized();
    EXPECT_NEAR(n.length(), 1.0, 1e-10);
    EXPECT_NEAR(n.x, 0.6, 1e-10);
    EXPECT_NEAR(n.y, 0.8, 1e-10);
}

TEST(Vec2Test, NormalizeZeroVector) {
    Vec2 v(0.0, 0.0);
    Vec2 n = v.normalized();
    EXPECT_DOUBLE_EQ(n.x, 0.0);
    EXPECT_DOUBLE_EQ(n.y, 0.0);
}

// ---------------------------------------------------------------------------
// 10. isApproxEqual
// ---------------------------------------------------------------------------
TEST(Vec2Test, IsApproxEqual) {
    Vec2 a(1.0, 2.0);
    Vec2 b(1.0, 2.0);
    EXPECT_TRUE(a.isApproxEqual(b));

    Vec2 c(1.0 + 1e-8, 2.0 - 1e-8);
    EXPECT_TRUE(a.isApproxEqual(c));

    Vec2 d(2.0, 3.0);
    EXPECT_FALSE(a.isApproxEqual(d));
}

// ---------------------------------------------------------------------------
// 11. Compound assignment operators
// ---------------------------------------------------------------------------
TEST(Vec2Test, CompoundAssignment) {
    Vec2 v(1.0, 2.0);
    v += Vec2(3.0, 4.0);
    EXPECT_NEAR(v.x, 4.0, 1e-10);
    EXPECT_NEAR(v.y, 6.0, 1e-10);

    v -= Vec2(1.0, 1.0);
    EXPECT_NEAR(v.x, 3.0, 1e-10);
    EXPECT_NEAR(v.y, 5.0, 1e-10);

    v *= 2.0;
    EXPECT_NEAR(v.x, 6.0, 1e-10);
    EXPECT_NEAR(v.y, 10.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 12. Unary negation
// ---------------------------------------------------------------------------
TEST(Vec2Test, UnaryNegation) {
    Vec2 v(3.0, -4.0);
    Vec2 n = -v;
    EXPECT_NEAR(n.x, -3.0, 1e-10);
    EXPECT_NEAR(n.y, 4.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 13. Perpendicular
// ---------------------------------------------------------------------------
TEST(Vec2Test, Perpendicular) {
    Vec2 v(1.0, 0.0);
    Vec2 p = v.perpendicular();
    EXPECT_NEAR(v.dot(p), 0.0, 1e-10);  // must be orthogonal
    EXPECT_NEAR(p.x, 0.0, 1e-10);
    EXPECT_NEAR(p.y, 1.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 14. distanceTo
// ---------------------------------------------------------------------------
TEST(Vec2Test, DistanceTo) {
    Vec2 a(0.0, 0.0);
    Vec2 b(3.0, 4.0);
    EXPECT_NEAR(a.distanceTo(b), 5.0, 1e-10);
    EXPECT_NEAR(b.distanceTo(a), 5.0, 1e-10);
}
