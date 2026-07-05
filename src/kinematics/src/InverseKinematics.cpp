#include "horizon/kinematics/InverseKinematics.h"

#include <cmath>

namespace hz::kin {

namespace {

double length(const math::Vec3& v) {
    return std::sqrt(v.dot(v));
}

math::Vec3 normalizedOr(const math::Vec3& v, const math::Vec3& fallback) {
    const double len = length(v);
    return len > 1e-12 ? v / len : fallback;
}

// The rotation part of frame @p f applied to direction @p d (translation removed).
math::Vec3 rotateDir(const math::Mat4& f, const math::Vec3& d) {
    return f.transformPoint(d) - f.transformPoint(math::Vec3(0.0, 0.0, 0.0));
}

}  // namespace

bool solveInverseKinematics(const math::Mat4& base, std::vector<Joint>& joints,
                            const math::Vec3& target, int maxIterations, double tolerance) {
    auto endEffector = [&]() {
        const std::vector<math::Mat4> frames = forwardKinematics(base, joints);
        return frames.back().transformPoint(math::Vec3(0.0, 0.0, 0.0));
    };

    for (int iter = 0; iter < maxIterations; ++iter) {
        if (length(endEffector() - target) < tolerance) return true;

        // One tip-to-base sweep, re-evaluating the chain after each joint move.
        for (int j = static_cast<int>(joints.size()) - 1; j >= 0; --j) {
            if (joints[j].type != JointType::Revolute) continue;

            const std::vector<math::Mat4> frames = forwardKinematics(base, joints);
            const math::Vec3 endEff = frames.back().transformPoint(math::Vec3(0.0, 0.0, 0.0));
            const math::Mat4& parent = frames[j];

            const math::Vec3 pivot = parent.transformPoint(joints[j].origin);
            const math::Vec3 axis =
                normalizedOr(rotateDir(parent, normalizedOr(joints[j].axis, {0, 0, 1})), {0, 0, 1});

            // Vectors from the pivot to the current tip and to the target,
            // projected onto the plane perpendicular to the joint axis.
            math::Vec3 toEnd = endEff - pivot;
            math::Vec3 toTgt = target - pivot;
            toEnd = toEnd - axis * toEnd.dot(axis);
            toTgt = toTgt - axis * toTgt.dot(axis);
            if (length(toEnd) < 1e-9 || length(toTgt) < 1e-9) continue;

            // Signed angle from toEnd to toTgt about the axis.
            const double angle = std::atan2(axis.dot(toEnd.cross(toTgt)), toEnd.dot(toTgt));
            joints[j].value += angle;
        }
    }

    return length(endEffector() - target) < tolerance;
}

}  // namespace hz::kin
