#include "horizon/kinematics/ForwardKinematics.h"

#include <cmath>

#include "horizon/math/Quaternion.h"

namespace hz::kin {

namespace {

math::Vec3 normalizedOr(const math::Vec3& v, const math::Vec3& fallback) {
    const double len = std::sqrt(v.dot(v));
    return len > 1e-12 ? v / len : fallback;
}

}  // namespace

math::Mat4 jointTransform(const Joint& j) {
    const math::Mat4 offset = math::Mat4::translation(j.origin);
    switch (j.type) {
        case JointType::Revolute: {
            const math::Vec3 axis = normalizedOr(j.axis, math::Vec3(0.0, 0.0, 1.0));
            return offset * math::Mat4::rotation(math::Quaternion::fromAxisAngle(axis, j.value));
        }
        case JointType::Prismatic: {
            const math::Vec3 axis = normalizedOr(j.axis, math::Vec3(0.0, 0.0, 1.0));
            return offset * math::Mat4::translation(axis * j.value);
        }
        case JointType::Fixed:
            break;
    }
    return offset;
}

std::vector<math::Mat4> forwardKinematics(const math::Mat4& base,
                                          const std::vector<Joint>& joints) {
    std::vector<math::Mat4> frames;
    frames.reserve(joints.size() + 1);
    frames.push_back(base);
    for (const Joint& j : joints) {
        frames.push_back(frames.back() * jointTransform(j));
    }
    return frames;
}

math::Vec3 endEffectorPosition(const math::Mat4& base, const std::vector<Joint>& joints) {
    const std::vector<math::Mat4> frames = forwardKinematics(base, joints);
    return frames.back().transformPoint(math::Vec3(0.0, 0.0, 0.0));
}

}  // namespace hz::kin
