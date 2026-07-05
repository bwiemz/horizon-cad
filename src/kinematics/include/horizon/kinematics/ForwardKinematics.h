#pragma once

#include <vector>

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"

namespace hz::kin {

/// The degree of freedom a joint provides.
enum class JointType {
    Fixed,      ///< rigid offset, no motion
    Revolute,   ///< rotation about `axis` by `value` radians
    Prismatic,  ///< translation along `axis` by `value`
};

/// One joint in a serial kinematic chain, expressed relative to its parent link
/// frame: a fixed `origin` offset followed by the joint's motion about/along
/// `axis` by `value`. This is the constant-plus-variable form of a DH-style link.
struct Joint {
    JointType type = JointType::Fixed;
    math::Vec3 origin{0.0, 0.0, 0.0};  ///< offset from the parent frame to the joint
    math::Vec3 axis{0.0, 0.0, 1.0};    ///< rotation axis / slide direction (need not be unit)
    double value = 0.0;                ///< joint angle (radians) or displacement
};

/// The joint's local transform relative to its parent frame.
math::Mat4 jointTransform(const Joint& j);

/// The cumulative world frames of a serial chain: `result[0] == base` and
/// `result[i + 1] == result[i] * jointTransform(joints[i])`. The returned vector
/// has `joints.size() + 1` entries (base frame plus one per joint).
std::vector<math::Mat4> forwardKinematics(const math::Mat4& base, const std::vector<Joint>& joints);

/// World-space position of the end-effector (the origin of the last chain frame).
math::Vec3 endEffectorPosition(const math::Mat4& base, const std::vector<Joint>& joints);

}  // namespace hz::kin
