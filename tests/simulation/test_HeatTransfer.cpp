#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "horizon/simulation/BoxMesher.h"
#include "horizon/simulation/HeatTransfer.h"
#include "horizon/simulation/TetMesh.h"

using hz::math::Vec3;
using hz::sim::HeatSource;
using hz::sim::meshBox;
using hz::sim::Node;
using hz::sim::PrescribedTemperature;
using hz::sim::SteadyHeatSolver;
using hz::sim::Tet4;
using hz::sim::TetMesh;
using hz::sim::ThermalResult;

namespace {
TetMesh unitTet() {
    TetMesh m;
    m.nodes = {Node{Vec3(0, 0, 0)}, Node{Vec3(1, 0, 0)}, Node{Vec3(0, 1, 0)}, Node{Vec3(0, 0, 1)}};
    m.elements = {Tet4{{0, 1, 2, 3}}};
    return m;
}
}  // namespace

// The element conductivity matrix is symmetric and a uniform temperature lies in
// its null space (no heat flows through an isothermal element).
TEST(HeatTransferTest, ElementConductivitySymmetricWithUniformNullSpace) {
    TetMesh m = unitTet();
    const auto k = hz::sim::elementConductivity(m, m.elements[0], 50.0);

    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            EXPECT_NEAR(k[r * 4 + c], k[c * 4 + r], 1e-9);
        }
        EXPECT_GT(k[r * 4 + r], 0.0);  // positive diagonal
    }
    // Row sums are zero (Ke * uniform temperature = 0).
    for (int r = 0; r < 4; ++r) {
        double sum = 0.0;
        for (int c = 0; c < 4; ++c) sum += k[r * 4 + c];
        EXPECT_NEAR(sum, 0.0, 1e-9);
    }
}

// 1D steady conduction: a bar held at T=0 on the x=0 face and T=100 on the x=L
// face develops a linear temperature profile T(x) = 100 x / L, with a uniform
// flux |q| = k * 100 / L.
TEST(HeatTransferTest, OneDimensionalConductionIsLinear) {
    const double L = 5.0, a = 1.0, k = 50.0, Thot = 100.0;
    const int nx = 5, ny = 1, nz = 1;
    TetMesh mesh = meshBox(L, a, a, nx, ny, nz);

    std::vector<PrescribedTemperature> fixed;
    for (int i = 0; i < static_cast<int>(mesh.nodes.size()); ++i) {
        const double x = mesh.nodes[i].position.x;
        if (std::abs(x) < 1e-9) fixed.push_back({i, 0.0});
        if (std::abs(x - L) < 1e-9) fixed.push_back({i, Thot});
    }

    const ThermalResult r = SteadyHeatSolver::solve(mesh, k, fixed, /*sources=*/{});
    ASSERT_TRUE(r.converged);

    // Every node's temperature matches the analytical linear profile.
    for (int i = 0; i < static_cast<int>(mesh.nodes.size()); ++i) {
        const double expected = Thot * mesh.nodes[i].position.x / L;
        EXPECT_NEAR(r.temperatures[i], expected, 1e-6);
    }
    EXPECT_NEAR(r.minTemperature, 0.0, 1e-6);
    EXPECT_NEAR(r.maxTemperature, Thot, 1e-6);
    // Uniform flux magnitude k * dT/dx = k * Thot / L.
    EXPECT_NEAR(r.maxFluxMagnitude, k * Thot / L, 1e-6);
}

// With no prescribed temperature the field is defined only up to a constant, so
// the solver reports non-convergence.
TEST(HeatTransferTest, NoPrescribedTemperatureIsNotConverged) {
    TetMesh mesh = meshBox(1, 1, 1, 1, 1, 1);
    std::vector<HeatSource> sources{HeatSource{0, 10.0}};
    const ThermalResult r = SteadyHeatSolver::solve(mesh, 50.0, /*fixedTemperatures=*/{}, sources);
    EXPECT_FALSE(r.converged);
    EXPECT_TRUE(r.temperatures.empty());
}

// A body held everywhere at one temperature is isothermal with no flux.
TEST(HeatTransferTest, UniformBoundaryGivesUniformField) {
    TetMesh mesh = meshBox(2, 1, 1, 2, 1, 1);
    std::vector<PrescribedTemperature> fixed;
    for (int i = 0; i < static_cast<int>(mesh.nodes.size()); ++i) {
        const double x = mesh.nodes[i].position.x;
        if (std::abs(x) < 1e-9 || std::abs(x - 2.0) < 1e-9) fixed.push_back({i, 42.0});
    }
    const ThermalResult r = SteadyHeatSolver::solve(mesh, 50.0, fixed, /*sources=*/{});
    ASSERT_TRUE(r.converged);
    for (double t : r.temperatures) EXPECT_NEAR(t, 42.0, 1e-6);
    EXPECT_NEAR(r.maxFluxMagnitude, 0.0, 1e-6);
}
