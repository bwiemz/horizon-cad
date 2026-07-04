#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "horizon/simulation/BoxMesher.h"
#include "horizon/simulation/LinearElement.h"
#include "horizon/simulation/LinearStaticSolver.h"
#include "horizon/simulation/Material.h"
#include "horizon/simulation/TetMesh.h"

using hz::math::Vec3;
using hz::sim::ElasticMaterial;
using hz::sim::LinearStaticSolver;
using hz::sim::meshBox;
using hz::sim::NodalLoad;
using hz::sim::StaticResult;
using hz::sim::TetMesh;

// The box mesher produces the expected node/element counts and its tetrahedra
// exactly partition the box volume.
TEST(BoxMesherTest, CountsAndVolume) {
    const double lx = 2.0, ly = 1.0, lz = 1.0;
    const int nx = 4, ny = 2, nz = 2;
    TetMesh m = meshBox(lx, ly, lz, nx, ny, nz);

    EXPECT_EQ(m.nodes.size(), static_cast<std::size_t>((nx + 1) * (ny + 1) * (nz + 1)));
    EXPECT_EQ(m.elements.size(), static_cast<std::size_t>(nx * ny * nz * 6));

    double total = 0.0;
    for (const auto& e : m.elements) total += std::abs(hz::sim::tetVolume(m, e));
    EXPECT_NEAR(total, lx * ly * lz, 1e-9);
}

TEST(BoxMesherTest, RejectsBadInput) {
    EXPECT_TRUE(meshBox(1, 1, 1, 0, 1, 1).nodes.empty());
    EXPECT_TRUE(meshBox(1, 1, 1, 1, 1, 1).elements.size() == 6u);
    EXPECT_TRUE(meshBox(-1, 1, 1, 1, 1, 1).nodes.empty());
}

// A prismatic bar fixed at one end and pulled axially at the other stretches by
// the analytical amount delta = F L / (A E), with a uniform axial stress F / A.
TEST(LinearStaticSolverTest, BarInTensionMatchesAnalytical) {
    const double L = 10.0;   // length along x
    const double a = 1.0;    // square cross-section side
    const double A = a * a;  // cross-sectional area
    const int nx = 8, ny = 2, nz = 2;
    TetMesh mesh = meshBox(L, a, a, nx, ny, nz);

    const ElasticMaterial mat = hz::sim::materials::steel();
    const double E = mat.youngsModulus;
    const double F = 1.0e6;  // total axial force (N)

    // Fix every node on the x = 0 face; distribute F over the x = L face nodes.
    std::vector<int> fixed;
    std::vector<int> loadedNodes;
    for (int i = 0; i < static_cast<int>(mesh.nodes.size()); ++i) {
        const double x = mesh.nodes[i].position.x;
        if (std::abs(x) < 1e-9) fixed.push_back(i);
        if (std::abs(x - L) < 1e-9) loadedNodes.push_back(i);
    }
    ASSERT_FALSE(loadedNodes.empty());

    std::vector<NodalLoad> loads;
    const double per = F / static_cast<double>(loadedNodes.size());
    for (int n : loadedNodes) loads.push_back(NodalLoad{n, Vec3(per, 0.0, 0.0)});

    const StaticResult r = LinearStaticSolver::solve(mesh, mat, fixed, loads);
    ASSERT_TRUE(r.converged);

    // Analytical tip elongation and axial stress.
    const double expectedDelta = F * L / (A * E);
    const double expectedStress = F / A;

    // Max displacement is the axial tip displacement (bar pulled in +x); the
    // full assemble/solve/BC pipeline reproduces delta = FL/AE to a few percent.
    EXPECT_NEAR(r.maxDisplacementMagnitude, expectedDelta, 0.05 * expectedDelta);

    // Peak element von Mises is of order the nominal axial stress F/A. A coarse
    // constant-strain-tet mesh with a fully-fixed end and pointwise end loads
    // concentrates stress at the corners (St. Venant), so the peak overshoots the
    // nominal by tens of percent — bound it rather than pin it. (Exact stress
    // recovery is validated on a uniform strain field in the element tests.)
    EXPECT_GT(r.maxVonMises, 0.8 * expectedStress);
    EXPECT_LT(r.maxVonMises, 1.6 * expectedStress);
}

// With no constraints the free system is singular (rigid-body modes remain), so
// the solver reports non-convergence rather than returning garbage.
TEST(LinearStaticSolverTest, UnconstrainedIsNotConverged) {
    TetMesh mesh = meshBox(1, 1, 1, 1, 1, 1);
    std::vector<NodalLoad> loads{NodalLoad{7, Vec3(1.0, 0.0, 0.0)}};
    const StaticResult r =
        LinearStaticSolver::solve(mesh, hz::sim::materials::steel(), /*fixedNodes=*/{}, loads);
    EXPECT_FALSE(r.converged);
    EXPECT_TRUE(r.displacements.empty());
}

// A fixed bar under no load stays put: zero displacement and zero stress.
TEST(LinearStaticSolverTest, NoLoadYieldsZeroResponse) {
    TetMesh mesh = meshBox(4, 1, 1, 4, 1, 1);
    std::vector<int> fixed;
    for (int i = 0; i < static_cast<int>(mesh.nodes.size()); ++i) {
        if (std::abs(mesh.nodes[i].position.x) < 1e-9) fixed.push_back(i);
    }
    const StaticResult r =
        LinearStaticSolver::solve(mesh, hz::sim::materials::steel(), fixed, /*loads=*/{});
    ASSERT_TRUE(r.converged);
    EXPECT_NEAR(r.maxDisplacementMagnitude, 0.0, 1e-12);
    EXPECT_NEAR(r.maxVonMises, 0.0, 1e-6);
}
