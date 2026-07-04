#include "horizon/simulation/LinearStaticSolver.h"

#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <array>
#include <cmath>
#include <vector>

#include "horizon/simulation/LinearElement.h"

namespace hz::sim {

StaticResult LinearStaticSolver::solve(const TetMesh& mesh, const ElasticMaterial& material,
                                       const std::vector<int>& fixedNodes,
                                       const std::vector<NodalLoad>& loads) {
    StaticResult result;
    const int nNodes = static_cast<int>(mesh.nodes.size());
    const int nDof = 3 * nNodes;
    if (nNodes == 0 || !material.isValid() || fixedNodes.empty()) return result;

    // Map each global DOF to a reduced index, or -1 if it is fixed.
    std::vector<int> reduced(nDof, 0);
    for (int node : fixedNodes) {
        if (node < 0 || node >= nNodes) return result;  // out-of-range constraint
        for (int d = 0; d < 3; ++d) reduced[3 * node + d] = -1;
    }
    int nFree = 0;
    for (int i = 0; i < nDof; ++i) {
        if (reduced[i] != -1) reduced[i] = nFree++;
    }
    if (nFree == 0) return result;  // fully constrained: trivial

    // Assemble the reduced (free-free) global stiffness from element stiffnesses.
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<std::size_t>(mesh.elements.size()) * 144);
    for (const Tet4& e : mesh.elements) {
        const std::array<double, 144> ke = elementStiffness(mesh, e, material);
        std::array<int, 12> gdof;
        for (int i = 0; i < 4; ++i) {
            for (int d = 0; d < 3; ++d) gdof[3 * i + d] = 3 * e.nodes[i] + d;
        }
        for (int a = 0; a < 12; ++a) {
            const int ra = reduced[gdof[a]];
            if (ra == -1) continue;
            for (int b = 0; b < 12; ++b) {
                const int rb = reduced[gdof[b]];
                if (rb == -1) continue;
                const double v = ke[a * 12 + b];
                if (v != 0.0) triplets.emplace_back(ra, rb, v);
            }
        }
    }

    Eigen::SparseMatrix<double> K(nFree, nFree);
    K.setFromTriplets(triplets.begin(), triplets.end());

    // Reduced load vector (prescribed displacements are zero, so fixed DOFs add
    // no contribution to the free equations).
    Eigen::VectorXd f = Eigen::VectorXd::Zero(nFree);
    for (const NodalLoad& load : loads) {
        if (load.node < 0 || load.node >= nNodes) continue;
        const double comp[3] = {load.force.x, load.force.y, load.force.z};
        for (int d = 0; d < 3; ++d) {
            const int r = reduced[3 * load.node + d];
            if (r != -1) f(r) += comp[d];
        }
    }

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(K);
    if (solver.info() != Eigen::Success) return result;
    const Eigen::VectorXd uFree = solver.solve(f);
    if (solver.info() != Eigen::Success || !uFree.allFinite()) return result;

    // Reject an ill-posed system (residual large relative to the load) — this
    // catches insufficient constraints that leave a near-singular free matrix.
    const double fNorm = f.norm();
    const double residual = (K * uFree - f).norm();
    if (fNorm > 0.0 && residual > 1e-6 * fNorm) return result;

    // Scatter the solution back to full DOFs (fixed DOFs stay at zero).
    Eigen::VectorXd u = Eigen::VectorXd::Zero(nDof);
    for (int i = 0; i < nDof; ++i) {
        if (reduced[i] != -1) u(i) = uFree(reduced[i]);
    }

    // Nodal displacements.
    result.displacements.resize(nNodes);
    for (int i = 0; i < nNodes; ++i) {
        const math::Vec3 d(u(3 * i + 0), u(3 * i + 1), u(3 * i + 2));
        result.displacements[i] = d;
        result.maxDisplacementMagnitude =
            std::max(result.maxDisplacementMagnitude, std::sqrt(d.dot(d)));
    }

    // Per-element von Mises stress.
    result.elementVonMises.reserve(mesh.elements.size());
    for (const Tet4& e : mesh.elements) {
        std::array<double, 12> ue{};
        for (int i = 0; i < 4; ++i) {
            for (int d = 0; d < 3; ++d) ue[3 * i + d] = u(3 * e.nodes[i] + d);
        }
        const double vm = vonMises(elementStress(mesh, e, material, ue));
        result.elementVonMises.push_back(vm);
        result.maxVonMises = std::max(result.maxVonMises, vm);
    }

    result.converged = true;
    return result;
}

}  // namespace hz::sim
