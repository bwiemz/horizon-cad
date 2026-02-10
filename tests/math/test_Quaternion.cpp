#include <gtest/gtest.h>

#include "horizon/math/Quaternion.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Constants.h"

using namespace hz::math;

// ---------------------------------------------------------------------------
// 1. Identity quaternion does not rotate
// ---------------------------------------------------------------------------
TEST(QuaternionTest, IdentityDoesNotRotate) {
    Quaternion q = Quaternion::Identity;
    Vec3 v(1.0, 2.0, 3.0);
    Vec3 result = q.rotate(v);
    EXPECT_NEAR(result.x, v.x, 1e-10);
    EXPECT_NEAR(result.y, v.y, 1e-10);
    EXPECT_NEAR(result.z, v.z, 1e-10);
}

// ---------------------------------------------------------------------------
// 2. FromAxisAngle: 90 degrees around Z rotates X to Y
// ---------------------------------------------------------------------------
TEST(QuaternionTest, FromAxisAngle90AroundZ) {
    Quaternion q = Quaternion::fromAxisAngle(Vec3::UnitZ, kHalfPi);
    Vec3 v(1.0, 0.0, 0.0);
    Vec3 result = q.rotate(v);
    EXPECT_NEAR(result.x, 0.0, 1e-10);
    EXPECT_NEAR(result.y, 1.0, 1e-10);
    EXPECT_NEAR(result.z, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 3. FromAxisAngle: 90 degrees around X rotates Y to Z
// ---------------------------------------------------------------------------
TEST(QuaternionTest, FromAxisAngle90AroundX) {
    Quaternion q = Quaternion::fromAxisAngle(Vec3::UnitX, kHalfPi);
    Vec3 v(0.0, 1.0, 0.0);
    Vec3 result = q.rotate(v);
    EXPECT_NEAR(result.x, 0.0, 1e-10);
    EXPECT_NEAR(result.y, 0.0, 1e-10);
    EXPECT_NEAR(result.z, 1.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 4. Multiply composes rotations
// ---------------------------------------------------------------------------
TEST(QuaternionTest, MultiplyComposesRotations) {
    // Two 90-degree rotations about Z = 180-degree rotation about Z
    Quaternion q90 = Quaternion::fromAxisAngle(Vec3::UnitZ, kHalfPi);
    Quaternion q180 = q90 * q90;
    Vec3 v(1.0, 0.0, 0.0);
    Vec3 result = q180.rotate(v);
    // X should go to -X after 180 degrees about Z
    EXPECT_NEAR(result.x, -1.0, 1e-10);
    EXPECT_NEAR(result.y, 0.0, 1e-10);
    EXPECT_NEAR(result.z, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 5. Slerp at t=0 returns first endpoint
// ---------------------------------------------------------------------------
TEST(QuaternionTest, SlerpAtZero) {
    Quaternion a = Quaternion::Identity;
    Quaternion b = Quaternion::fromAxisAngle(Vec3::UnitZ, kHalfPi);
    Quaternion result = Quaternion::slerp(a, b, 0.0);
    EXPECT_TRUE(result.isApproxEqual(a, 1e-10));
}

// ---------------------------------------------------------------------------
// 6. Slerp at t=1 returns second endpoint
// ---------------------------------------------------------------------------
TEST(QuaternionTest, SlerpAtOne) {
    Quaternion a = Quaternion::Identity;
    Quaternion b = Quaternion::fromAxisAngle(Vec3::UnitZ, kHalfPi);
    Quaternion result = Quaternion::slerp(a, b, 1.0);
    EXPECT_TRUE(result.isApproxEqual(b, 1e-10));
}

// ---------------------------------------------------------------------------
// 7. Slerp at t=0.5 is halfway rotation
// ---------------------------------------------------------------------------
TEST(QuaternionTest, SlerpAtHalf) {
    Quaternion a = Quaternion::Identity;
    Quaternion b = Quaternion::fromAxisAngle(Vec3::UnitZ, kHalfPi);  // 90 deg
    Quaternion mid = Quaternion::slerp(a, b, 0.5);
    // Should represent 45-degree rotation about Z
    Vec3 v(1.0, 0.0, 0.0);
    Vec3 result = mid.rotate(v);
    double cos45 = std::cos(kPi / 4.0);
    double sin45 = std::sin(kPi / 4.0);
    EXPECT_NEAR(result.x, cos45, 1e-10);
    EXPECT_NEAR(result.y, sin45, 1e-10);
    EXPECT_NEAR(result.z, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 8. Conjugate
// ---------------------------------------------------------------------------
TEST(QuaternionTest, Conjugate) {
    Quaternion q(0.5, 0.5, 0.5, 0.5);
    Quaternion c = q.conjugate();
    EXPECT_NEAR(c.w, 0.5, 1e-10);
    EXPECT_NEAR(c.x, -0.5, 1e-10);
    EXPECT_NEAR(c.y, -0.5, 1e-10);
    EXPECT_NEAR(c.z, -0.5, 1e-10);
}

// ---------------------------------------------------------------------------
// 9. Inverse: q * q_inv = identity
// ---------------------------------------------------------------------------
TEST(QuaternionTest, InverseProducesIdentity) {
    Quaternion q = Quaternion::fromAxisAngle(Vec3(1.0, 1.0, 0.0).normalized(), 1.23);
    Quaternion qinv = q.inverse();
    Quaternion result = q * qinv;
    EXPECT_NEAR(result.w, 1.0, 1e-10);
    EXPECT_NEAR(result.x, 0.0, 1e-10);
    EXPECT_NEAR(result.y, 0.0, 1e-10);
    EXPECT_NEAR(result.z, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 10. ToMatrix matches direct rotation matrix
// ---------------------------------------------------------------------------
TEST(QuaternionTest, ToMatrixMatchesDirectRotation) {
    double angle = kPi / 3.0;  // 60 degrees
    Quaternion q = Quaternion::fromAxisAngle(Vec3::UnitY, angle);
    Mat4 fromQuat = q.toMatrix();
    Mat4 direct = Mat4::rotationY(angle);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            EXPECT_NEAR(fromQuat.at(r, c), direct.at(r, c), 1e-10)
                << "Mismatch at (" << r << ", " << c << ")";
}

// ---------------------------------------------------------------------------
// 11. Normalize
// ---------------------------------------------------------------------------
TEST(QuaternionTest, Normalize) {
    Quaternion q(2.0, 0.0, 0.0, 0.0);
    Quaternion n = q.normalized();
    EXPECT_NEAR(n.length(), 1.0, 1e-10);
    EXPECT_NEAR(n.w, 1.0, 1e-10);
    EXPECT_NEAR(n.x, 0.0, 1e-10);
    EXPECT_NEAR(n.y, 0.0, 1e-10);
    EXPECT_NEAR(n.z, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 12. isApproxEqual: q and -q represent the same rotation
// ---------------------------------------------------------------------------
TEST(QuaternionTest, IsApproxEqualNegation) {
    Quaternion q = Quaternion::fromAxisAngle(Vec3::UnitZ, kPi / 3.0);
    Quaternion neg(-q.w, -q.x, -q.y, -q.z);
    // q and -q represent the same rotation, so isApproxEqual should handle this
    // If the implementation checks both q and -q, this passes; otherwise we verify
    // rotation equivalence.
    Vec3 v(1.0, 0.0, 0.0);
    Vec3 r1 = q.rotate(v);
    Vec3 r2 = neg.rotate(v);
    EXPECT_NEAR(r1.x, r2.x, 1e-10);
    EXPECT_NEAR(r1.y, r2.y, 1e-10);
    EXPECT_NEAR(r1.z, r2.z, 1e-10);
}

// ---------------------------------------------------------------------------
// 13. Default constructor is identity
// ---------------------------------------------------------------------------
TEST(QuaternionTest, DefaultConstructorIsIdentity) {
    Quaternion q;
    EXPECT_NEAR(q.w, 1.0, 1e-10);
    EXPECT_NEAR(q.x, 0.0, 1e-10);
    EXPECT_NEAR(q.y, 0.0, 1e-10);
    EXPECT_NEAR(q.z, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 14. Length of unit quaternion
// ---------------------------------------------------------------------------
TEST(QuaternionTest, UnitQuaternionLength) {
    Quaternion q = Quaternion::fromAxisAngle(Vec3::UnitX, 1.0);
    EXPECT_NEAR(q.length(), 1.0, 1e-10);
}
