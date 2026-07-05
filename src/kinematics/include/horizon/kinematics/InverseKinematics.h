#pragma once

#include <vector>

#include "horizon/kinematics/ForwardKinematics.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"

namespace hz::kin {

/// Solve the inverse kinematics of a serial chain by cyclic coordinate descent
/// (CCD): iteratively adjust each revolute joint's `value` (in place) so the
/// end-effector reaches @p target as closely as possible. Prismatic and fixed
/// joints are left unchanged.
///
/// Returns true if the final end-effector is within @p tolerance of the target
/// (i.e. the target was reachable and the solver converged); false otherwise, in
/// which case @p joints still holds the best pose found. @p maxIterations bounds
/// the number of full tip-to-base sweeps.
bool solveInverseKinematics(const math::Mat4& base, std::vector<Joint>& joints,
                            const math::Vec3& target, int maxIterations = 200,
                            double tolerance = 1e-4);

}  // namespace hz::kin
