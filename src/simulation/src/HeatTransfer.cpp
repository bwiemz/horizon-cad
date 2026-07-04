#include "horizon/simulation/HeatTransfer.h"

#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <algorithm>
#include <cmath>
#include <vector>

#include "horizon/simulation/LinearElement.h"

namespace hz::sim {

std::array<double, 16> elementConductivity(const TetMesh& mesh, const Tet4& element,
                                           double conductivity) {
    std::array<double, 16> out{};  // zero-initialised
    std::array<math::Vec3, 4> grad;
    double volume = 0.0;
    if (!tetShapeGradients(mesh, element, grad, volume)) return out;

    // Ke_ij = k * V * (grad_i . grad_j).
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            out[i * 4 + j] = conductivity * volume * grad[i].dot(grad[j]);
        }
    }
    return out;
}

ThermalResult SteadyHeatSolver::solve(const TetMesh& mesh, double conductivity,
                                      const std::vector<PrescribedTemperature>& fixedTemperatures,
                                      const std::vector<HeatSource>& sources) {
    ThermalResult result;
    const int nNodes = static_cast<int>(mesh.nodes.size());
    if (nNodes == 0 || conductivity <= 0.0 || fixedTemperatures.empty()) return result;

    // Prescribed temperatures (Dirichlet). A node listed twice keeps the last.
    std::vector<bool> isFixed(nNodes, false);
    std::vector<double> prescribed(nNodes, 0.0);
    for (const PrescribedTemperature& t : fixedTemperatures) {
        if (t.node < 0 || t.node >= nNodes) return result;
        isFixed[t.node] = true;
        prescribed[t.node] = t.temperature;
    }

    // Reduced index for each free node, or -1 if fixed.
    std::vector<int> reduced(nNodes, 0);
    int nFree = 0;
    for (int i = 0; i < nNodes; ++i) {
        reduced[i] = isFixed[i] ? -1 : nFree++;
    }
    if (nFree == 0) {  // fully prescribed: temperatures are simply the BCs
        result.temperatures = prescribed;
        result.elementFlux.assign(mesh.elements.size(), 0.0);
        result.minTemperature = *std::min_element(prescribed.begin(), prescribed.end());
        result.maxTemperature = *std::max_element(prescribed.begin(), prescribed.end());
        // Fluxes still follow from the (generally non-uniform) prescribed field.
    }

    // Assemble the global conductivity matrix as triplets, and accumulate the
    // Dirichlet contribution to the RHS (rhs -= K * prescribed).
    Eigen::VectorXd fullPrescribed = Eigen::VectorXd::Zero(nNodes);
    for (int i = 0; i < nNodes; ++i) fullPrescribed(i) = prescribed[i];

    std::vector<Eigen::Triplet<double>> tripletsFF;  // free-free
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(std::max(nFree, 1));
    // Nodal heat sources contribute to the free RHS directly.
    Eigen::VectorXd fullQ = Eigen::VectorXd::Zero(nNodes);
    for (const HeatSource& s : sources) {
        if (s.node >= 0 && s.node < nNodes) fullQ(s.node) += s.power;
    }

    for (const Tet4& e : mesh.elements) {
        const std::array<double, 16> ke = elementConductivity(mesh, e, conductivity);
        for (int a = 0; a < 4; ++a) {
            const int na = e.nodes[a];
            const int ra = reduced[na];
            if (ra == -1) continue;
            for (int b = 0; b < 4; ++b) {
                const int nb = e.nodes[b];
                const double v = ke[a * 4 + b];
                if (v == 0.0) continue;
                if (reduced[nb] == -1) {
                    // Fixed node: move to RHS.
                    rhs(ra) -= v * fullPrescribed(nb);
                } else {
                    tripletsFF.emplace_back(ra, reduced[nb], v);
                }
            }
        }
    }

    Eigen::VectorXd temperatures = fullPrescribed;
    if (nFree > 0) {
        for (int i = 0; i < nNodes; ++i) {
            if (reduced[i] != -1) rhs(reduced[i]) += fullQ(i);
        }

        Eigen::SparseMatrix<double> K(nFree, nFree);
        K.setFromTriplets(tripletsFF.begin(), tripletsFF.end());

        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(K);
        if (solver.info() != Eigen::Success) return result;
        const Eigen::VectorXd tFree = solver.solve(rhs);
        if (solver.info() != Eigen::Success || !tFree.allFinite()) return result;

        const double rhsNorm = rhs.norm();
        const double residual = (K * tFree - rhs).norm();
        if (rhsNorm > 0.0 && residual > 1e-6 * rhsNorm) return result;

        for (int i = 0; i < nNodes; ++i) {
            if (reduced[i] != -1) temperatures(i) = tFree(reduced[i]);
        }
    }

    result.temperatures.resize(nNodes);
    result.minTemperature = temperatures(0);
    result.maxTemperature = temperatures(0);
    for (int i = 0; i < nNodes; ++i) {
        result.temperatures[i] = temperatures(i);
        result.minTemperature = std::min(result.minTemperature, temperatures(i));
        result.maxTemperature = std::max(result.maxTemperature, temperatures(i));
    }

    // Per-element heat flux magnitude |q| = k |grad T|, grad T = sum_i grad_i T_i.
    result.elementFlux.reserve(mesh.elements.size());
    for (const Tet4& e : mesh.elements) {
        std::array<math::Vec3, 4> grad;
        double volume = 0.0;
        if (!tetShapeGradients(mesh, e, grad, volume)) {
            result.elementFlux.push_back(0.0);
            continue;
        }
        math::Vec3 gradT(0.0, 0.0, 0.0);
        for (int i = 0; i < 4; ++i) {
            gradT = gradT + grad[i] * temperatures(e.nodes[i]);
        }
        const double flux = conductivity * gradT.length();
        result.elementFlux.push_back(flux);
        result.maxFluxMagnitude = std::max(result.maxFluxMagnitude, flux);
    }

    result.converged = true;
    return result;
}

}  // namespace hz::sim
