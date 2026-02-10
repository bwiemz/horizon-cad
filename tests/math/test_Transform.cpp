#include <gtest/gtest.h>

#include "horizon/math/Transform.h"
#include "horizon/math/Constants.h"

using namespace hz::math;

// ---------------------------------------------------------------------------
// 1. Identity transform does not change points
// ---------------------------------------------------------------------------
TEST(TransformTest, IdentityDoesNotChangePoints) {
    Transform t = Transform::Identity;
    Vec3 p(3.0, 4.0, 5.0);
    Vec3 result = t.transformPoint(p);
    EXPECT_NEAR(result.x, p.x, 1e-10);
    EXPECT_NEAR(result.y, p.y, 1e-10);
    EXPECT_NEAR(result.z, p.z, 1e-10);
}

// ---------------------------------------------------------------------------
// 2. Translation works
// ---------------------------------------------------------------------------
TEST(TransformTest, TranslationWorks) {
    Transform t;
    t.setTranslation(Vec3(10.0, 20.0, 30.0));
    Vec3 p(1.0, 2.0, 3.0);
    Vec3 result = t.transformPoint(p);
    EXPECT_NEAR(result.x, 11.0, 1e-10);
    EXPECT_NEAR(result.y, 22.0, 1e-10);
    EXPECT_NEAR(result.z, 33.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 3. Rotation works
// ---------------------------------------------------------------------------
TEST(TransformTest, RotationWorks) {
    Transform t;
    t.setRotation(Quaternion::fromAxisAngle(Vec3::UnitZ, kHalfPi));
    Vec3 p(1.0, 0.0, 0.0);
    Vec3 result = t.transformPoint(p);
    // 90 deg about Z: X -> Y
    EXPECT_NEAR(result.x, 0.0, 1e-10);
    EXPECT_NEAR(result.y, 1.0, 1e-10);
    EXPECT_NEAR(result.z, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 4. Scale works
// ---------------------------------------------------------------------------
TEST(TransformTest, ScaleWorks) {
    Transform t;
    t.setScale(Vec3(2.0, 3.0, 4.0));
    Vec3 p(1.0, 1.0, 1.0);
    Vec3 result = t.transformPoint(p);
    EXPECT_NEAR(result.x, 2.0, 1e-10);
    EXPECT_NEAR(result.y, 3.0, 1e-10);
    EXPECT_NEAR(result.z, 4.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 5. Compose two transforms
// ---------------------------------------------------------------------------
TEST(TransformTest, ComposeTwoTransforms) {
    Transform t1;
    t1.setTranslation(Vec3(1.0, 0.0, 0.0));

    Transform t2;
    t2.setTranslation(Vec3(0.0, 2.0, 0.0));

    Transform composed = t1 * t2;
    Vec3 p(0.0, 0.0, 0.0);
    Vec3 result = composed.transformPoint(p);
    // Both translations should be applied
    EXPECT_NEAR(result.x, 1.0, 1e-10);
    EXPECT_NEAR(result.y, 2.0, 1e-10);
    EXPECT_NEAR(result.z, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 6. Inverse undoes transform
// ---------------------------------------------------------------------------
TEST(TransformTest, InverseUndoesTransform) {
    Transform t;
    t.setTranslation(Vec3(5.0, -3.0, 7.0));
    t.setRotation(Quaternion::fromAxisAngle(Vec3::UnitY, kPi / 6.0));
    t.setScale(Vec3(2.0, 2.0, 2.0));

    Transform inv = t.inverse();
    Vec3 p(1.0, 2.0, 3.0);
    Vec3 forward = t.transformPoint(p);
    Vec3 back = inv.transformPoint(forward);

    EXPECT_NEAR(back.x, p.x, 1e-9);
    EXPECT_NEAR(back.y, p.y, 1e-9);
    EXPECT_NEAR(back.z, p.z, 1e-9);
}

// ---------------------------------------------------------------------------
// 7. TransformDirection ignores translation
// ---------------------------------------------------------------------------
TEST(TransformTest, TransformDirectionIgnoresTranslation) {
    Transform t;
    t.setTranslation(Vec3(100.0, 200.0, 300.0));
    Vec3 d(1.0, 0.0, 0.0);
    Vec3 result = t.transformDirection(d);
    EXPECT_NEAR(result.x, 1.0, 1e-10);
    EXPECT_NEAR(result.y, 0.0, 1e-10);
    EXPECT_NEAR(result.z, 0.0, 1e-10);
}

// ---------------------------------------------------------------------------
// 8. ToMatrix consistency: matrix and direct transform give same result
// ---------------------------------------------------------------------------
TEST(TransformTest, ToMatrixConsistency) {
    Transform t;
    t.setTranslation(Vec3(1.0, 2.0, 3.0));
    t.setRotation(Quaternion::fromAxisAngle(Vec3::UnitX, kPi / 4.0));
    t.setScale(Vec3(1.5, 1.5, 1.5));

    Mat4 mat = t.toMatrix();
    Vec3 p(4.0, 5.0, 6.0);
    Vec3 fromTransform = t.transformPoint(p);
    Vec3 fromMatrix = mat.transformPoint(p);

    EXPECT_NEAR(fromTransform.x, fromMatrix.x, 1e-9);
    EXPECT_NEAR(fromTransform.y, fromMatrix.y, 1e-9);
    EXPECT_NEAR(fromTransform.z, fromMatrix.z, 1e-9);
}

// ---------------------------------------------------------------------------
// 9. Default constructor creates identity transform
// ---------------------------------------------------------------------------
TEST(TransformTest, DefaultConstructorIsIdentity) {
    Transform t;
    EXPECT_NEAR(t.translation().x, 0.0, 1e-10);
    EXPECT_NEAR(t.translation().y, 0.0, 1e-10);
    EXPECT_NEAR(t.translation().z, 0.0, 1e-10);
    EXPECT_NEAR(t.rotation().w, 1.0, 1e-10);
    EXPECT_NEAR(t.rotation().x, 0.0, 1e-10);
    EXPECT_NEAR(t.rotation().y, 0.0, 1e-10);
    EXPECT_NEAR(t.rotation().z, 0.0, 1e-10);
    EXPECT_NEAR(t.scale().x, 1.0, 1e-10);
    EXPECT_NEAR(t.scale().y, 1.0, 1e-10);
    EXPECT_NEAR(t.scale().z, 1.0, 1e-10);
}
