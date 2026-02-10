#include <gtest/gtest.h>

#include "horizon/math/Vec3.h"
#include "horizon/math/Constants.h"

using namespace hz::math;

// ---------------------------------------------------------------------------
// 1. Default constructor produces zero vector
// ---------------------------------------------------------------------------
TEST(Vec3Test, DefaultConstructorIsZero) {
    Vec3 v;
    EXPECT_DOUBLE_EQ(v.x, 0.0);
    EXPECT_DOUBLE_EQ(v.y, 0.0);
    EXPECT_DOUBLE_EQ(v.z, 0.0);
}

// ---------------------------------------------------------------------------
// 2. Parameterized constructor
// ---------------------------------------------------------------------------
TEST(Vec3Test, ParameterizedConstructor) {
    Vec3 v(1.0, -2.5, 3.7);
    EXPECT_DOUBLE_EQ(v.x, 1.0);
    EXPECT_DOUBLE_EQ(v.y, -2.5);
    EXPECT_DOUBLE_EQ(v.z, 3.7);
}

// ---------------------------------------------------------------------------
// 3. Addition
// ---------------------------------------------------------------------------
TEST(Vec3Test, Addition) {
    Vec3 a(1.0, 2.0, 3.0);
    Vec3 b(4.0, 5.0, 6.0);
    Vec3 c = a + b;
    EXPECT_NEAR(c.x, 5.0, 1e-10);
    EXPECT_NEAR(c.y, 7.0, 1e-10);
    EXPECT_NEAR(c.z, 9.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 4. Subtraction
// ---------------------------------------------------------------------------
TEST(Vec3Test, Subtraction) {
    Vec3 a(5.0, 7.0, 9.0);
    Vec3 b(1.0, 2.0, 3.0);
    Vec3 c = a - b;
    EXPECT_NEAR(c.x, 4.0, 1e-10);
    EXPECT_NEAR(c.y, 5.0, 1e-10);
    EXPECT_NEAR(c.z, 6.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 5. Scalar multiply and divide
// ---------------------------------------------------------------------------
TEST(Vec3Test, ScalarMultiply) {
    Vec3 v(1.0, -2.0, 3.0);
    Vec3 r = v * 3.0;
    EXPECT_NEAR(r.x, 3.0, 1e-10);
    EXPECT_NEAR(r.y, -6.0, 1e-10);
    EXPECT_NEAR(r.z, 9.0, 1e-10);

    // left-hand scalar multiplication
    Vec3 l = 3.0 * v;
    EXPECT_NEAR(l.x, 3.0, 1e-10);
    EXPECT_NEAR(l.y, -6.0, 1e-10);
    EXPECT_NEAR(l.z, 9.0, 1e-10);
}

TEST(Vec3Test, ScalarDivide) {
    Vec3 v(6.0, -9.0, 12.0);
    Vec3 r = v / 3.0;
    EXPECT_NEAR(r.x, 2.0, 1e-10);
    EXPECT_NEAR(r.y, -3.0, 1e-10);
    EXPECT_NEAR(r.z, 4.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 6. Dot product: orthogonal vectors = 0
// ---------------------------------------------------------------------------
TEST(Vec3Test, DotProductOrthogonal) {
    Vec3 a(1.0, 0.0, 0.0);
    Vec3 b(0.0, 1.0, 0.0);
    EXPECT_NEAR(a.dot(b), 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 7. Dot product: parallel vectors = product of lengths
// ---------------------------------------------------------------------------
TEST(Vec3Test, DotProductParallel) {
    Vec3 a(2.0, 0.0, 0.0);
    Vec3 b(5.0, 0.0, 0.0);
    EXPECT_NEAR(a.dot(b), 10.0, 1e-10);  // |a|*|b| = 2*5 = 10
}

// ---------------------------------------------------------------------------
// 8. Cross product: anticommutative
// ---------------------------------------------------------------------------
TEST(Vec3Test, CrossProductAnticommutative) {
    Vec3 a(1.0, 2.0, 3.0);
    Vec3 b(4.0, 5.0, 6.0);
    Vec3 axb = a.cross(b);
    Vec3 bxa = b.cross(a);
    EXPECT_NEAR(axb.x, -bxa.x, 1e-10);
    EXPECT_NEAR(axb.y, -bxa.y, 1e-10);
    EXPECT_NEAR(axb.z, -bxa.z, 1e-10);
}

// ---------------------------------------------------------------------------
// 9. Cross product: orthogonal to inputs
// ---------------------------------------------------------------------------
TEST(Vec3Test, CrossProductOrthogonalToInputs) {
    Vec3 a(1.0, 0.0, 0.0);
    Vec3 b(0.0, 1.0, 0.0);
    Vec3 c = a.cross(b);
    EXPECT_NEAR(c.dot(a), 0.0, 1e-10);
    EXPECT_NEAR(c.dot(b), 0.0, 1e-10);
    // i x j = k
    EXPECT_NEAR(c.x, 0.0, 1e-10);
    EXPECT_NEAR(c.y, 0.0, 1e-10);
    EXPECT_NEAR(c.z, 1.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 10. Length and lengthSquared
// ---------------------------------------------------------------------------
TEST(Vec3Test, LengthAndLengthSquared) {
    Vec3 v(1.0, 2.0, 2.0);
    EXPECT_NEAR(v.lengthSquared(), 9.0, 1e-10);
    EXPECT_NEAR(v.length(), 3.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 11. Normalize: unit length
// ---------------------------------------------------------------------------
TEST(Vec3Test, NormalizeUnitLength) {
    Vec3 v(3.0, 4.0, 0.0);
    Vec3 n = v.normalized();
    EXPECT_NEAR(n.length(), 1.0, 1e-10);
    EXPECT_NEAR(n.x, 0.6, 1e-10);
    EXPECT_NEAR(n.y, 0.8, 1e-10);
    EXPECT_NEAR(n.z, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 12. Normalize: zero vector returns zero
// ---------------------------------------------------------------------------
TEST(Vec3Test, NormalizeZeroVector) {
    Vec3 v(0.0, 0.0, 0.0);
    Vec3 n = v.normalized();
    EXPECT_DOUBLE_EQ(n.x, 0.0);
    EXPECT_DOUBLE_EQ(n.y, 0.0);
    EXPECT_DOUBLE_EQ(n.z, 0.0);
}

// ---------------------------------------------------------------------------
// 13. Static constants
// ---------------------------------------------------------------------------
TEST(Vec3Test, StaticConstants) {
    EXPECT_NEAR(Vec3::Zero.x, 0.0, 1e-10);
    EXPECT_NEAR(Vec3::Zero.y, 0.0, 1e-10);
    EXPECT_NEAR(Vec3::Zero.z, 0.0, 1e-10);

    EXPECT_NEAR(Vec3::UnitX.x, 1.0, 1e-10);
    EXPECT_NEAR(Vec3::UnitX.y, 0.0, 1e-10);
    EXPECT_NEAR(Vec3::UnitX.z, 0.0, 1e-10);

    EXPECT_NEAR(Vec3::UnitY.x, 0.0, 1e-10);
    EXPECT_NEAR(Vec3::UnitY.y, 1.0, 1e-10);
    EXPECT_NEAR(Vec3::UnitY.z, 0.0, 1e-10);

    EXPECT_NEAR(Vec3::UnitZ.x, 0.0, 1e-10);
    EXPECT_NEAR(Vec3::UnitZ.y, 0.0, 1e-10);
    EXPECT_NEAR(Vec3::UnitZ.z, 1.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 14. isApproxEqual
// ---------------------------------------------------------------------------
TEST(Vec3Test, IsApproxEqual) {
    Vec3 a(1.0, 2.0, 3.0);
    Vec3 b(1.0, 2.0, 3.0);
    EXPECT_TRUE(a.isApproxEqual(b));

    Vec3 c(1.0 + 1e-8, 2.0, 3.0 - 1e-8);
    EXPECT_TRUE(a.isApproxEqual(c));

    Vec3 d(2.0, 3.0, 4.0);
    EXPECT_FALSE(a.isApproxEqual(d));
}

// ---------------------------------------------------------------------------
// 15. distanceTo
// ---------------------------------------------------------------------------
TEST(Vec3Test, DistanceTo) {
    Vec3 a(0.0, 0.0, 0.0);
    Vec3 b(1.0, 2.0, 2.0);
    EXPECT_NEAR(a.distanceTo(b), 3.0, 1e-10);
    EXPECT_NEAR(b.distanceTo(a), 3.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 16. Compound assignment operators
// ---------------------------------------------------------------------------
TEST(Vec3Test, CompoundAssignment) {
    Vec3 v(1.0, 2.0, 3.0);
    v += Vec3(4.0, 5.0, 6.0);
    EXPECT_NEAR(v.x, 5.0, 1e-10);
    EXPECT_NEAR(v.y, 7.0, 1e-10);
    EXPECT_NEAR(v.z, 9.0, 1e-10);

    v -= Vec3(1.0, 1.0, 1.0);
    EXPECT_NEAR(v.x, 4.0, 1e-10);
    EXPECT_NEAR(v.y, 6.0, 1e-10);
    EXPECT_NEAR(v.z, 8.0, 1e-10);

    v *= 0.5;
    EXPECT_NEAR(v.x, 2.0, 1e-10);
    EXPECT_NEAR(v.y, 3.0, 1e-10);
    EXPECT_NEAR(v.z, 4.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 17. Unary negation
// ---------------------------------------------------------------------------
TEST(Vec3Test, UnaryNegation) {
    Vec3 v(1.0, -2.0, 3.0);
    Vec3 n = -v;
    EXPECT_NEAR(n.x, -1.0, 1e-10);
    EXPECT_NEAR(n.y, 2.0, 1e-10);
    EXPECT_NEAR(n.z, -3.0, 1e-10);
}
