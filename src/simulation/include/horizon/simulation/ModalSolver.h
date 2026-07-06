#pragma once

#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/simulation/Material.h"
#include "horizon/simulation/TetMesh.h"

namespace hz::sim {

/// The result of a modal (free-vibration) analysis.
struct ModalResult {
    bool converged = false;  ///< false if the eigenproblem could not be formed/solved

    /// Natural frequencies in hertz, ascending. For an unconstrained (free-free)
    /// mesh the first six are the rigid-body modes and are ~0.
    std::vector<double> naturalFrequencies;

    /// Mode shape per natural frequency: one displacement vector per mesh node,
    /// mass-normalised (phi^T M phi = 1). Same ordering as `naturalFrequencies`.
    std::vector<std::vector<math::Vec3>> modeShapes;
};

/// Solves the undamped generalized eigenproblem K phi = omega^2 M phi for the
/// lowest natural frequencies and mode shapes of a tetrahedral mesh.
///
/// K is the assembled constant-strain stiffness and M the consistent mass matrix.
/// Fully-fixed nodes have their three DOFs removed before the solve; `fixedNodes`
/// may be empty, in which case the free-free spectrum (six near-zero rigid-body
/// modes followed by the elastic modes) is returned. The material must be valid
/// and have a positive density. Frequencies are f = omega / (2*pi) with
/// omega = sqrt(max(lambda, 0)); tiny negative eigenvalues from round-off are
/// clamped to zero.
class ModalSolver {
public:
    static ModalResult solve(const TetMesh& mesh, const ElasticMaterial& material,
                             const std::vector<int>& fixedNodes, int numModes);
};

}  // namespace hz::sim
