#pragma once

#include <array>
#include <vector>

#include "horizon/simulation/TetMesh.h"

namespace hz::sim {

/// A prescribed (Dirichlet) nodal temperature.
struct PrescribedTemperature {
    int node = 0;
    double temperature = 0.0;
};

/// A concentrated heat input (power, W) applied at a node.
struct HeatSource {
    int node = 0;
    double power = 0.0;
};

/// The result of a steady-state thermal analysis.
struct ThermalResult {
    bool converged = false;            ///< false if the system was singular / ill-posed
    std::vector<double> temperatures;  ///< per node (empty if not converged)
    std::vector<double> elementFlux;   ///< per element heat-flux magnitude |q| = k|grad T|
    double minTemperature = 0.0;       ///< min nodal temperature
    double maxTemperature = 0.0;       ///< max nodal temperature
    double maxFluxMagnitude = 0.0;     ///< max element flux magnitude
};

/// The 4x4 element conductivity ("Laplacian") matrix of a linear tetrahedron,
/// row-major: Ke_ij = k * V * (grad N_i . grad N_j). Symmetric, with a uniform
/// temperature in its null space. A degenerate element yields an all-zero matrix.
std::array<double, 16> elementConductivity(const TetMesh& mesh, const Tet4& element,
                                           double conductivity);

/// Solves steady-state heat conduction K T = Q on a tetrahedral mesh with
/// isotropic conductivity @p conductivity.
///
/// Prescribed (Dirichlet) temperatures — which may be non-zero — are moved to the
/// right-hand side, and the reduced free system is solved with a sparse Cholesky
/// factorization. At least one node must have a prescribed temperature; otherwise
/// the system is singular (temperature defined only up to a constant) and
/// `converged` is false. Recovers nodal temperatures and per-element heat flux.
class SteadyHeatSolver {
public:
    static ThermalResult solve(const TetMesh& mesh, double conductivity,
                               const std::vector<PrescribedTemperature>& fixedTemperatures,
                               const std::vector<HeatSource>& sources);
};

}  // namespace hz::sim
