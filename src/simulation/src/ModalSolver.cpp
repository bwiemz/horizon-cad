#include "horizon/simulation/ModalSolver.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "horizon/simulation/LinearElement.h"

namespace hz::sim {

namespace {
constexpr double kTwoPi = 6.28318530717958647692;
}

ModalResult ModalSolver::solve(const TetMesh& mesh, const ElasticMaterial& material,
                               const std::vector<int>& fixedNodes, int numModes) {
    ModalResult result;
    const int nNodes = static_cast<int>(mesh.nodes.size());
    const int nDof = 3 * nNodes;
    if (nNodes == 0 || mesh.elements.empty() || numModes <= 0) return result;
    if (!material.isValid() || material.density <= 0.0) return result;

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
    if (nFree == 0) return result;  // fully constrained: no modes

    // Assemble the reduced (free-free) stiffness and consistent mass, dense — the
    // generalized symmetric eigensolver needs dense operands and modal problems
    // run on modest meshes.
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(nFree, nFree);
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nFree, nFree);
    for (const Tet4& e : mesh.elements) {
        const std::array<double, 144> ke = elementStiffness(mesh, e, material);
        const std::array<double, 144> me = elementMass(mesh, e, material.density);
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
                K(ra, rb) += ke[a * 12 + b];
                M(ra, rb) += me[a * 12 + b];
            }
        }
    }

    // K phi = lambda M phi with M symmetric positive-definite. Eigenvalues come
    // back ascending; lambda = omega^2.
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> ges(K, M);
    if (ges.info() != Eigen::Success) return result;

    const Eigen::VectorXd& lambda = ges.eigenvalues();
    const Eigen::MatrixXd& vecs = ges.eigenvectors();
    if (!lambda.allFinite()) return result;

    const int count = std::min(numModes, static_cast<int>(lambda.size()));
    result.naturalFrequencies.reserve(count);
    result.modeShapes.reserve(count);
    for (int m = 0; m < count; ++m) {
        const double l = std::max(lambda(m), 0.0);  // clamp round-off negatives
        const double omega = std::sqrt(l);
        result.naturalFrequencies.push_back(omega / kTwoPi);

        // Scatter the mode shape back to full DOFs (fixed DOFs stay at zero). The
        // eigenvectors are already M-normalised (phi^T M phi = 1).
        std::vector<math::Vec3> shape(nNodes, math::Vec3(0.0, 0.0, 0.0));
        for (int i = 0; i < nDof; ++i) {
            if (reduced[i] == -1) continue;
            const double v = vecs(reduced[i], m);
            const int node = i / 3;
            const int d = i % 3;
            if (d == 0)
                shape[node].x = v;
            else if (d == 1)
                shape[node].y = v;
            else
                shape[node].z = v;
        }
        result.modeShapes.push_back(std::move(shape));
    }

    result.converged = true;
    return result;
}

}  // namespace hz::sim
