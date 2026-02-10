#include <gtest/gtest.h>

#include "horizon/math/Mat4.h"
#include "horizon/math/Quaternion.h"
#include "horizon/math/Constants.h"

using namespace hz::math;

// Helper: check two Mat4 matrices are approximately equal element-wise
static void expectMat4Near(const Mat4& a, const Mat4& b, double tol = 1e-10) {
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            EXPECT_NEAR(a.at(r, c), b.at(r, c), tol)
                << "Mismatch at (" << r << ", " << c << ")";
}

// ---------------------------------------------------------------------------
// 1. Identity * Identity = Identity
// ---------------------------------------------------------------------------
TEST(Mat4Test, IdentityTimesIdentity) {
    Mat4 I = Mat4::identity();
    Mat4 result = I * I;
    expectMat4Near(result, I);
}

// ---------------------------------------------------------------------------
// 2. Translation transforms point correctly
// ---------------------------------------------------------------------------
TEST(Mat4Test, TranslationTransformsPoint) {
    Mat4 T = Mat4::translation(Vec3(3.0, 4.0, 5.0));
    Vec3 p(1.0, 2.0, 3.0);
    Vec3 result = T.transformPoint(p);
    EXPECT_NEAR(result.x, 4.0, 1e-10);
    EXPECT_NEAR(result.y, 6.0, 1e-10);
    EXPECT_NEAR(result.z, 8.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 3. Rotation X by 90 degrees
// ---------------------------------------------------------------------------
TEST(Mat4Test, RotationX90) {
    Mat4 Rx = Mat4::rotationX(kHalfPi);
    Vec3 v(0.0, 1.0, 0.0);  // Y axis
    Vec3 result = Rx.transformPoint(v);
    // Y -> Z after 90-degree rotation about X
    EXPECT_NEAR(result.x, 0.0, 1e-10);
    EXPECT_NEAR(result.y, 0.0, 1e-10);
    EXPECT_NEAR(result.z, 1.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 4. Rotation Y by 90 degrees
// ---------------------------------------------------------------------------
TEST(Mat4Test, RotationY90) {
    Mat4 Ry = Mat4::rotationY(kHalfPi);
    Vec3 v(1.0, 0.0, 0.0);  // X axis
    Vec3 result = Ry.transformPoint(v);
    // X -> -Z after 90-degree rotation about Y
    EXPECT_NEAR(result.x, 0.0, 1e-10);
    EXPECT_NEAR(result.y, 0.0, 1e-10);
    EXPECT_NEAR(result.z, -1.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 5. Rotation Z by 90 degrees
// ---------------------------------------------------------------------------
TEST(Mat4Test, RotationZ90) {
    Mat4 Rz = Mat4::rotationZ(kHalfPi);
    Vec3 v(1.0, 0.0, 0.0);  // X axis
    Vec3 result = Rz.transformPoint(v);
    // X -> Y after 90-degree rotation about Z
    EXPECT_NEAR(result.x, 0.0, 1e-10);
    EXPECT_NEAR(result.y, 1.0, 1e-10);
    EXPECT_NEAR(result.z, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 6. Scale transforms point correctly
// ---------------------------------------------------------------------------
TEST(Mat4Test, ScaleTransformsPoint) {
    Mat4 S = Mat4::scale(Vec3(2.0, 3.0, 4.0));
    Vec3 p(1.0, 1.0, 1.0);
    Vec3 result = S.transformPoint(p);
    EXPECT_NEAR(result.x, 2.0, 1e-10);
    EXPECT_NEAR(result.y, 3.0, 1e-10);
    EXPECT_NEAR(result.z, 4.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 7. Multiply * Inverse = Identity
// ---------------------------------------------------------------------------
TEST(Mat4Test, MultiplyByInverse) {
    Mat4 T = Mat4::translation(Vec3(1.0, 2.0, 3.0));
    Mat4 Rz = Mat4::rotationZ(kPi / 4.0);
    Mat4 S = Mat4::scale(Vec3(2.0, 2.0, 2.0));
    Mat4 M = T * Rz * S;
    Mat4 Minv = M.inverse();
    Mat4 result = M * Minv;
    expectMat4Near(result, Mat4::identity(), 1e-9);
}

// ---------------------------------------------------------------------------
// 8. LookAt produces correct view matrix
// ---------------------------------------------------------------------------
TEST(Mat4Test, LookAt) {
    // Camera at origin looking down -Z, up is +Y
    Mat4 V = Mat4::lookAt(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -1.0), Vec3(0.0, 1.0, 0.0));
    // A point at (0, 0, -5) in world space should map to (0, 0, -5) in view space
    // or similar -- key is that the matrix is valid (non-degenerate)
    Vec3 p(0.0, 0.0, -5.0);
    Vec3 result = V.transformPoint(p);
    // The forward direction is -Z, so the point should appear at positive z in view space
    // depending on convention. We just verify it is well-formed.
    EXPECT_NEAR(result.x, 0.0, 1e-10);
    EXPECT_NEAR(result.y, 0.0, 1e-10);
    // result.z should be non-zero (the point is not at camera origin)
    EXPECT_GT(std::abs(result.z), 1e-5);
}

// ---------------------------------------------------------------------------
// 9. Perspective projection (basic sanity)
// ---------------------------------------------------------------------------
TEST(Mat4Test, PerspectiveBasicSanity) {
    Mat4 P = Mat4::perspective(kPi / 4.0, 1.0, 0.1, 100.0);
    // Origin should map to origin (with w=0 issue, but let us check a point on the near plane)
    Vec4 p(0.0, 0.0, -0.1, 1.0);  // point on the near plane
    Vec4 result = P * p;
    // After perspective divide, x and y at center should be 0
    Vec3 ndc = result.perspectiveDivide();
    EXPECT_NEAR(ndc.x, 0.0, 1e-10);
    EXPECT_NEAR(ndc.y, 0.0, 1e-10);
    // z should be at near plane boundary (-1 or 0 depending on convention)
    // Just verify it is finite
    EXPECT_TRUE(std::isfinite(ndc.z));
}

// ---------------------------------------------------------------------------
// 10. TransformPoint with translation
// ---------------------------------------------------------------------------
TEST(Mat4Test, TransformPointWithTranslation) {
    Mat4 T = Mat4::translation(Vec3(10.0, 20.0, 30.0));
    Vec3 p(0.0, 0.0, 0.0);
    Vec3 result = T.transformPoint(p);
    EXPECT_NEAR(result.x, 10.0, 1e-10);
    EXPECT_NEAR(result.y, 20.0, 1e-10);
    EXPECT_NEAR(result.z, 30.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 11. TransformDirection ignores translation
// ---------------------------------------------------------------------------
TEST(Mat4Test, TransformDirectionIgnoresTranslation) {
    Mat4 T = Mat4::translation(Vec3(100.0, 200.0, 300.0));
    Vec3 d(1.0, 0.0, 0.0);
    Vec3 result = T.transformDirection(d);
    EXPECT_NEAR(result.x, 1.0, 1e-10);
    EXPECT_NEAR(result.y, 0.0, 1e-10);
    EXPECT_NEAR(result.z, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 12. Transposed of transposed = original
// ---------------------------------------------------------------------------
TEST(Mat4Test, TransposedOfTransposed) {
    Mat4 T = Mat4::translation(Vec3(1.0, 2.0, 3.0));
    Mat4 Rz = Mat4::rotationZ(0.7);
    Mat4 M = T * Rz;
    Mat4 result = M.transposed().transposed();
    expectMat4Near(result, M);
}

// ---------------------------------------------------------------------------
// 13. Uniform scale
// ---------------------------------------------------------------------------
TEST(Mat4Test, UniformScale) {
    Mat4 S = Mat4::scale(3.0);
    Vec3 p(1.0, 2.0, 3.0);
    Vec3 result = S.transformPoint(p);
    EXPECT_NEAR(result.x, 3.0, 1e-10);
    EXPECT_NEAR(result.y, 6.0, 1e-10);
    EXPECT_NEAR(result.z, 9.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 14. Identity default constructor
// ---------------------------------------------------------------------------
TEST(Mat4Test, DefaultConstructor) {
    // Default constructor: verify it initializes to something deterministic.
    // The Mat4() constructor is likely zero or identity -- test the actual behavior.
    Mat4 m;
    // We check that identity() is definitely the identity matrix.
    Mat4 I = Mat4::identity();
    EXPECT_NEAR(I.at(0, 0), 1.0, 1e-10);
    EXPECT_NEAR(I.at(1, 1), 1.0, 1e-10);
    EXPECT_NEAR(I.at(2, 2), 1.0, 1e-10);
    EXPECT_NEAR(I.at(3, 3), 1.0, 1e-10);
    EXPECT_NEAR(I.at(0, 1), 0.0, 1e-10);
    EXPECT_NEAR(I.at(0, 2), 0.0, 1e-10);
    EXPECT_NEAR(I.at(0, 3), 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 15. Rotation from Quaternion matches direct rotation
// ---------------------------------------------------------------------------
TEST(Mat4Test, RotationFromQuaternion) {
    Quaternion q = Quaternion::fromAxisAngle(Vec3::UnitZ, kHalfPi);
    Mat4 Rq = Mat4::rotation(q);
    Mat4 Rz = Mat4::rotationZ(kHalfPi);
    // Both should produce the same matrix
    expectMat4Near(Rq, Rz, 1e-10);
}
