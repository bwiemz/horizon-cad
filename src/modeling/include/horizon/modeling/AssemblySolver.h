#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "horizon/math/Mat4.h"
#include "horizon/modeling/MateGeometry.h"

namespace hz::model {

/// A component participating in an assembly solve.
struct SolverComponent {
    uint64_t id = 0;
    math::Mat4 transform = math::Mat4::identity();  ///< Current placement.
    bool grounded = false;                          ///< Fixed in space.
};

/// A mate with its geometry already resolved to frames in each part's
/// LOCAL coordinates (callers combine `MateGeometry` extraction with the
/// document's mate references).
struct SolverMate {
    MateType type = MateType::Coincident;
    uint64_t componentA = 0;
    uint64_t componentB = 0;  ///< Unused for Fixed.
    MateFrame frameA;
    MateFrame frameB;
    double value = 0.0;
};

enum class AssemblySolveStatus {
    Success,
    NotConverged,
    NoMates,
    InvalidReference,
};

struct AssemblySolveResult {
    AssemblySolveStatus status = AssemblySolveStatus::NoMates;
    int iterations = 0;
    double residualNorm = 0.0;

    /// Solved placements, keyed by component id (grounded components keep
    /// their input transform).
    std::map<uint64_t, math::Mat4> transforms;

    /// Number of redundant constraint equations (expected constrained DOF
    /// minus Jacobian rank). Zero for a well-posed mate scheme.
    int redundantCount = 0;

    /// Total unconstrained rigid-body DOF remaining across free components.
    int remainingDOF = 0;

    /// Per-component remaining DOF (6 = completely free).
    std::map<uint64_t, int> componentDOF;

    /// Components with no mate path to a grounded component (pre-analysis).
    std::vector<uint64_t> ungroundedComponents;

    std::string message;
};

/// 6-DOF-per-component Newton-Raphson mate solver with Levenberg-Marquardt
/// damping, mirroring the sketch solver's architecture.
///
/// Kinematic pre-analysis before the numerical solve:
///  - components grounded by Fixed mates are removed from the unknowns
///    (if nothing is grounded, the first component is grounded by
///    convention and reported in `message`)
///  - components with no mate path to ground are reported
///  - after convergence, Jacobian rank analysis reports redundant
///    constraints and remaining DOF
class AssemblySolver {
public:
    AssemblySolver() = default;

    AssemblySolveResult solve(const std::vector<SolverComponent>& components,
                              const std::vector<SolverMate>& mates) const;

    void setMaxIterations(int n) { m_maxIterations = n; }
    void setTolerance(double tol) { m_tolerance = tol; }

    /// Post-solve rank analysis (redundant-constraint count and per-component
    /// remaining DOF) uses a dense QR that costs O(unknowns^3). It is on by
    /// default; disable it for large assemblies where only the placements are
    /// needed, to keep the solve within the sparse-solver's near-linear cost.
    void setComputeDiagnostics(bool enable) { m_computeDiagnostics = enable; }

private:
    int m_maxIterations = 100;
    double m_tolerance = 1e-9;
    bool m_computeDiagnostics = true;
};

}  // namespace hz::model
