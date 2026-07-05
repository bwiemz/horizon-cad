#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "horizon/kinematics/ForwardKinematics.h"
#include "horizon/math/Mat4.h"

using hz::kin::endEffectorPosition;
using hz::kin::forwardKinematics;
using hz::kin::Joint;
using hz::kin::JointType;
using hz::math::Mat4;
using hz::math::Vec3;

namespace {
constexpr double kPi = 3.14159265358979323846;

Joint revolute(const Vec3& origin, const Vec3& axis, double angle) {
    return Joint{JointType::Revolute, origin, axis, angle};
}
Joint prismatic(const Vec3& origin, const Vec3& axis, double d) {
    return Joint{JointType::Prismatic, origin, axis, d};
}
Joint fixed(const Vec3& origin) {
    return Joint{JointType::Fixed, origin, Vec3(0, 0, 1), 0.0};
}
}  // namespace

// The chain returns one frame per joint plus the base frame.
TEST(ForwardKinematicsTest, FrameCount) {
    std::vector<Joint> joints = {revolute({0, 0, 0}, {0, 0, 1}, 0.0), fixed({1, 0, 0})};
    const auto frames = forwardKinematics(Mat4::identity(), joints);
    EXPECT_EQ(frames.size(), 3u);
}

// A single revolute joint about +Z by 90 deg carries a unit link (0,0,0)->(1,0,0)
// to (0, 1, 0).
TEST(ForwardKinematicsTest, RevoluteZRotatesLink) {
    std::vector<Joint> joints = {revolute({0, 0, 0}, {0, 0, 1}, kPi / 2.0), fixed({1, 0, 0})};
    const Vec3 p = endEffectorPosition(Mat4::identity(), joints);
    EXPECT_NEAR(p.x, 0.0, 1e-9);
    EXPECT_NEAR(p.y, 1.0, 1e-9);
    EXPECT_NEAR(p.z, 0.0, 1e-9);
}

// A prismatic joint slides the end-effector along its axis.
TEST(ForwardKinematicsTest, PrismaticSlides) {
    std::vector<Joint> joints = {prismatic({0, 0, 0}, {1, 0, 0}, 3.0)};
    const Vec3 p = endEffectorPosition(Mat4::identity(), joints);
    EXPECT_NEAR(p.x, 3.0, 1e-9);
    EXPECT_NEAR(p.y, 0.0, 1e-9);
    EXPECT_NEAR(p.z, 0.0, 1e-9);

    // A non-unit axis is normalized, so displacement is still 3, not 3*|axis|.
    std::vector<Joint> joints2 = {prismatic({0, 0, 0}, {0, 2, 0}, 3.0)};
    const Vec3 q = endEffectorPosition(Mat4::identity(), joints2);
    EXPECT_NEAR(q.y, 3.0, 1e-9);
}

// A planar 2R arm (unit links) with the first joint at 90 deg reaches (0, 2, 0).
TEST(ForwardKinematicsTest, PlanarTwoLinkArm) {
    std::vector<Joint> joints = {
        revolute({0, 0, 0}, {0, 0, 1}, kPi / 2.0),  // shoulder
        revolute({1, 0, 0}, {0, 0, 1}, 0.0),        // elbow, 1 unit out
        fixed({1, 0, 0}),                           // 1-unit forearm to the tip
    };
    const Vec3 p = endEffectorPosition(Mat4::identity(), joints);
    EXPECT_NEAR(p.x, 0.0, 1e-9);
    EXPECT_NEAR(p.y, 2.0, 1e-9);

    // Fold the elbow back 90 deg: tip comes to (-1, 1, 0).
    joints[1].value = kPi / 2.0;
    const Vec3 folded = endEffectorPosition(Mat4::identity(), joints);
    EXPECT_NEAR(folded.x, -1.0, 1e-9);
    EXPECT_NEAR(folded.y, 1.0, 1e-9);
}

// A fixed joint just applies its origin offset; the base transform is respected.
TEST(ForwardKinematicsTest, FixedOffsetAndBase) {
    std::vector<Joint> joints = {fixed({2, 3, 4})};
    const Vec3 p = endEffectorPosition(Mat4::translation(Vec3(10, 0, 0)), joints);
    EXPECT_NEAR(p.x, 12.0, 1e-9);
    EXPECT_NEAR(p.y, 3.0, 1e-9);
    EXPECT_NEAR(p.z, 4.0, 1e-9);
}
