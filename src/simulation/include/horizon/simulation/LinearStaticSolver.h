#pragma once

#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/simulation/Material.h"
#include "horizon/simulation/TetMesh.h"

namespace hz::sim {

/// A concentrated force applied at a mesh node.
struct NodalLoad {
    int node = 0;
    math::Vec3 force;  ///< force vector at the node
};

/// The result of a linear-static analysis.
struct StaticResult {
    bool converged = false;                 ///< false if the system was singular / ill-posed
    std::vector<math::Vec3> displacements;  ///< per node (empty if not converged)
    std::vector<double> elementVonMises;    ///< per element (empty if not converged)
    double maxDisplacementMagnitude = 0.0;  ///< max nodal displacement magnitude
    double maxVonMises = 0.0;               ///< max element von Mises stress
};

/// Solves the linear-static equilibrium K u = f for a tetrahedral mesh.
///
/// The global stiffness matrix is assembled from the per-element constant-strain
/// stiffnesses, fully-fixed nodes have their three DOFs constrained to zero, and
/// the reduced free system is solved with a sparse Cholesky factorization. Enough
/// nodes must be fixed to remove all six rigid-body modes; otherwise the reduced
/// matrix is singular and `converged` is false.
class LinearStaticSolver {
public:
    static StaticResult solve(const TetMesh& mesh, const ElasticMaterial& material,
                              const std::vector<int>& fixedNodes,
                              const std::vector<NodalLoad>& loads);
};

}  // namespace hz::sim
