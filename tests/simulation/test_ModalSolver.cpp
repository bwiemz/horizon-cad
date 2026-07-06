#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "horizon/simulation/BoxMesher.h"
#include "horizon/simulation/LinearElement.h"
#include "horizon/simulation/Material.h"
#include "horizon/simulation/ModalSolver.h"
#include "horizon/simulation/TetMesh.h"

using hz::math::Vec3;
using hz::sim::ElasticMaterial;
using hz::sim::meshBox;
using hz::sim::ModalResult;
using hz::sim::ModalSolver;
using hz::sim::TetMesh;

namespace {

// Fix every node on the x = 0 face of a box mesh.
std::vector<int> fixX0Face(const TetMesh& mesh) {
    std::vector<int> fixed;
    for (int i = 0; i < static_cast<int>(mesh.nodes.size()); ++i) {
        if (std::abs(mesh.nodes[i].position.x) < 1e-9) fixed.push_back(i);
    }
    return fixed;
}

}  // namespace

// The consistent mass matrix conserves the element's mass: the sum of its
// translational-DOF block in any single direction equals rho * V.
TEST(ElementMassTest, ConservesMass) {
    TetMesh mesh;
    mesh.nodes = {{Vec3(0, 0, 0)}, {Vec3(1, 0, 0)}, {Vec3(0, 1, 0)}, {Vec3(0, 0, 1)}};
    mesh.elements = {hz::sim::Tet4{{0, 1, 2, 3}}};
    const double rho = 7850.0;
    const double V = std::abs(hz::sim::tetVolume(mesh, mesh.elements[0]));  // 1/6

    const std::array<double, 144> me = hz::sim::elementMass(mesh, mesh.elements[0], rho);

    // Sum the x-x block (rows/cols that are x DOFs): total should be rho * V.
    double sumX = 0.0;
    for (int r = 0; r < 12; ++r) {
        for (int c = 0; c < 12; ++c) {
            if (r % 3 == 0 && c % 3 == 0) sumX += me[r * 12 + c];
        }
    }
    EXPECT_NEAR(sumX, rho * V, 1e-9);

    // Cross-direction coupling is zero (x DOFs do not couple to y DOFs).
    EXPECT_EQ(me[0 * 12 + 1], 0.0);
    // Diagonal node block weighs twice the off-diagonal.
    EXPECT_NEAR(me[0 * 12 + 0], 2.0 * me[0 * 12 + 3], 1e-12);

    // Non-positive density yields an all-zero matrix.
    const std::array<double, 144> zero = hz::sim::elementMass(mesh, mesh.elements[0], 0.0);
    for (double v : zero) EXPECT_EQ(v, 0.0);
}

// A fixed-free bar has strictly positive, ascending natural frequencies.
TEST(ModalSolverTest, FixedBarFrequenciesPositiveAndOrdered) {
    TetMesh mesh = meshBox(10.0, 1.0, 1.0, 8, 2, 2);
    const ModalResult r = ModalSolver::solve(mesh, hz::sim::materials::steel(), fixX0Face(mesh), 5);
    ASSERT_TRUE(r.converged);
    ASSERT_EQ(r.naturalFrequencies.size(), 5u);
    ASSERT_EQ(r.modeShapes.size(), 5u);
    ASSERT_EQ(r.modeShapes.front().size(), mesh.nodes.size());

    EXPECT_GT(r.naturalFrequencies.front(), 0.0);
    for (std::size_t i = 1; i < r.naturalFrequencies.size(); ++i) {
        EXPECT_GE(r.naturalFrequencies[i], r.naturalFrequencies[i - 1] - 1e-6);
    }
    for (double f : r.naturalFrequencies) EXPECT_TRUE(std::isfinite(f));
}

// An unconstrained body has six rigid-body modes: the first six frequencies are
// ~0 (translation + rotation), the seventh is a genuine elastic mode.
TEST(ModalSolverTest, FreeBodyHasSixRigidModes) {
    TetMesh mesh = meshBox(1.0, 1.0, 1.0, 2, 2, 2);
    const ModalResult r =
        ModalSolver::solve(mesh, hz::sim::materials::steel(), /*fixedNodes=*/{}, 8);
    ASSERT_TRUE(r.converged);
    ASSERT_EQ(r.naturalFrequencies.size(), 8u);

    // The first elastic mode dwarfs the rigid-body residual frequencies.
    const double firstElastic = r.naturalFrequencies[6];
    EXPECT_GT(firstElastic, 1.0);
    EXPECT_LT(r.naturalFrequencies[5], 1e-3 * firstElastic);
}

// Exact invariance: the eigenproblem scales K by E, so every natural frequency
// scales by sqrt(E). Doubling Young's modulus multiplies all frequencies by
// sqrt(2), independent of the mesh.
TEST(ModalSolverTest, FrequencyScalesWithSqrtStiffness) {
    TetMesh mesh = meshBox(4.0, 1.0, 1.0, 4, 1, 1);
    const std::vector<int> fixed = fixX0Face(mesh);

    ElasticMaterial base = hz::sim::materials::steel();
    ElasticMaterial stiff = base;
    stiff.youngsModulus *= 2.0;

    const ModalResult a = ModalSolver::solve(mesh, base, fixed, 4);
    const ModalResult b = ModalSolver::solve(mesh, stiff, fixed, 4);
    ASSERT_TRUE(a.converged && b.converged);

    for (std::size_t i = 0; i < a.naturalFrequencies.size(); ++i) {
        EXPECT_NEAR(b.naturalFrequencies[i], std::sqrt(2.0) * a.naturalFrequencies[i],
                    1e-6 * b.naturalFrequencies[i] + 1e-9);
    }
}

// Exact invariance: the mass matrix scales with density, so every natural
// frequency scales by 1/sqrt(rho). Doubling density divides all frequencies by
// sqrt(2).
TEST(ModalSolverTest, FrequencyScalesWithInverseSqrtDensity) {
    TetMesh mesh = meshBox(4.0, 1.0, 1.0, 4, 1, 1);
    const std::vector<int> fixed = fixX0Face(mesh);

    ElasticMaterial base = hz::sim::materials::steel();
    ElasticMaterial heavy = base;
    heavy.density *= 2.0;

    const ModalResult a = ModalSolver::solve(mesh, base, fixed, 4);
    const ModalResult b = ModalSolver::solve(mesh, heavy, fixed, 4);
    ASSERT_TRUE(a.converged && b.converged);

    for (std::size_t i = 0; i < a.naturalFrequencies.size(); ++i) {
        EXPECT_NEAR(b.naturalFrequencies[i], a.naturalFrequencies[i] / std::sqrt(2.0),
                    1e-6 * a.naturalFrequencies[i] + 1e-9);
    }
}

// Exact invariance: uniformly scaling the geometry by 2 halves every natural
// frequency (f ~ 1/L). The node topology and constrained set are identical
// between the two meshes.
TEST(ModalSolverTest, FrequencyScalesWithInverseSize) {
    TetMesh small = meshBox(2.0, 2.0, 2.0, 2, 2, 2);
    TetMesh big = meshBox(4.0, 4.0, 4.0, 2, 2, 2);
    const ElasticMaterial mat = hz::sim::materials::aluminum();

    const ModalResult a = ModalSolver::solve(small, mat, fixX0Face(small), 4);
    const ModalResult b = ModalSolver::solve(big, mat, fixX0Face(big), 4);
    ASSERT_TRUE(a.converged && b.converged);

    for (std::size_t i = 0; i < a.naturalFrequencies.size(); ++i) {
        EXPECT_NEAR(b.naturalFrequencies[i], 0.5 * a.naturalFrequencies[i],
                    1e-6 * a.naturalFrequencies[i] + 1e-9);
    }
}

// Guards: bad requests report non-convergence rather than returning garbage.
TEST(ModalSolverTest, RejectsBadInput) {
    TetMesh mesh = meshBox(2.0, 1.0, 1.0, 2, 1, 1);
    const std::vector<int> fixed = fixX0Face(mesh);
    const ElasticMaterial steel = hz::sim::materials::steel();

    EXPECT_FALSE(ModalSolver::solve(mesh, steel, fixed, 0).converged);  // no modes asked
    EXPECT_FALSE(ModalSolver::solve({}, steel, {}, 4).converged);       // empty mesh
    ElasticMaterial massless = steel;
    massless.density = 0.0;
    EXPECT_FALSE(ModalSolver::solve(mesh, massless, fixed, 4).converged);  // no mass

    // Fixing every node leaves no free DOF -> no modes.
    std::vector<int> allNodes(mesh.nodes.size());
    for (int i = 0; i < static_cast<int>(mesh.nodes.size()); ++i) allNodes[i] = i;
    EXPECT_FALSE(ModalSolver::solve(mesh, steel, allNodes, 4).converged);
}
