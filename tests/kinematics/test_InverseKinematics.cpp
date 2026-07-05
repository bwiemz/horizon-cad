#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "horizon/kinematics/ForwardKinematics.h"
#include "horizon/kinematics/InverseKinematics.h"
#include "horizon/math/Mat4.h"

using hz::kin::endEffectorPosition;
using hz::kin::Joint;
using hz::kin::JointType;
using hz::kin::solveInverseKinematics;
using hz::math::Mat4;
using hz::math::Vec3;

namespace {
// A planar arm of `n` unit revolute links about +Z, all starting straight.
std::vector<Joint> planarArm(int n) {
    std::vector<Joint> joints;
    joints.push_back(Joint{JointType::Revolute, {0, 0, 0}, {0, 0, 1}, 0.0});
    for (int i = 1; i < n; ++i) {
        joints.push_back(Joint{JointType::Revolute, {1, 0, 0}, {0, 0, 1}, 0.0});
    }
    joints.push_back(Joint{JointType::Fixed, {1, 0, 0}, {0, 0, 1}, 0.0});  // 1-unit tip link
    return joints;
}
}  // namespace

// CCD drives a 3-link planar arm to a reachable target.
TEST(InverseKinematicsTest, ReachesReachableTarget) {
    auto joints = planarArm(3);
    const Vec3 target(1.5, 1.5, 0.0);  // within reach (max reach = 3)

    const bool ok = solveInverseKinematics(Mat4::identity(), joints, target);
    EXPECT_TRUE(ok);

    const Vec3 tip = endEffectorPosition(Mat4::identity(), joints);
    EXPECT_NEAR(tip.x, target.x, 1e-3);
    EXPECT_NEAR(tip.y, target.y, 1e-3);
    EXPECT_NEAR(tip.z, 0.0, 1e-6);  // stays in the z=0 plane
}

// An interior target on the +y axis is reached by folding the arm (not the
// exact full-reach point, which is a CCD singularity).
TEST(InverseKinematicsTest, ReachesAxisTarget) {
    auto joints = planarArm(2);
    const Vec3 target(0.0, 1.5, 0.0);  // on the y-axis, within reach (< 2)

    const bool ok = solveInverseKinematics(Mat4::identity(), joints, target);
    EXPECT_TRUE(ok);
    const Vec3 tip = endEffectorPosition(Mat4::identity(), joints);
    EXPECT_NEAR((tip - target).length(), 0.0, 1e-3);
}

// An unreachable target (beyond the arm's reach) does not converge, but the arm
// stretches toward it (the tip ends up near maximum extension in its direction).
TEST(InverseKinematicsTest, UnreachableTargetStretchesToward) {
    auto joints = planarArm(2);         // max reach 2
    const Vec3 target(10.0, 0.0, 0.0);  // far out of reach

    const bool ok = solveInverseKinematics(Mat4::identity(), joints, target);
    EXPECT_FALSE(ok);

    const Vec3 tip = endEffectorPosition(Mat4::identity(), joints);
    // Fully stretched toward +x: tip near (2, 0, 0).
    EXPECT_NEAR(tip.x, 2.0, 1e-2);
    EXPECT_NEAR(tip.y, 0.0, 1e-2);
}

// A chain already at the target converges immediately without moving.
TEST(InverseKinematicsTest, AlreadyAtTargetIsNoOp) {
    auto joints = planarArm(2);
    const Vec3 target = endEffectorPosition(Mat4::identity(), joints);  // current tip (2,0,0)

    const bool ok = solveInverseKinematics(Mat4::identity(), joints, target);
    EXPECT_TRUE(ok);
    for (const auto& j : joints) {
        if (j.type == JointType::Revolute) EXPECT_NEAR(j.value, 0.0, 1e-9);
    }
}
