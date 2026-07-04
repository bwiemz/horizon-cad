#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "horizon/simulation/LinearElement.h"
#include "horizon/simulation/Material.h"
#include "horizon/simulation/TetMesh.h"

using hz::math::Vec3;
using hz::sim::ElasticMaterial;
using hz::sim::Node;
using hz::sim::Tet4;
using hz::sim::TetMesh;

namespace {

// A unit right tetrahedron: origin plus the three unit axis points.
TetMesh unitTet() {
    TetMesh m;
    m.nodes = {Node{Vec3(0, 0, 0)}, Node{Vec3(1, 0, 0)}, Node{Vec3(0, 1, 0)}, Node{Vec3(0, 0, 1)}};
    m.elements = {Tet4{{0, 1, 2, 3}}};
    return m;
}

double at(const std::array<double, 144>& k, int r, int c) {
    return k[r * 12 + c];
}

}  // namespace

// The signed volume of the unit right tetrahedron is 1/6.
TEST(LinearElementTest, UnitTetVolume) {
    TetMesh m = unitTet();
    EXPECT_NEAR(hz::sim::tetVolume(m, m.elements[0]), 1.0 / 6.0, 1e-12);
    EXPECT_EQ(m.dofCount(), 12);
}

// The element stiffness matrix is symmetric.
TEST(LinearElementTest, StiffnessIsSymmetric) {
    TetMesh m = unitTet();
    const auto k = hz::sim::elementStiffness(m, m.elements[0], hz::sim::materials::steel());
    double maxAsym = 0.0;
    for (int r = 0; r < 12; ++r) {
        for (int c = 0; c < 12; ++c) {
            maxAsym = std::max(maxAsym, std::abs(at(k, r, c) - at(k, c, r)));
        }
    }
    // Symmetric relative to the stiffness magnitude (~E scale).
    EXPECT_LT(maxAsym, 1.0);  // absolute residual tiny next to ~1e11 entries
    // Diagonal entries are positive (a real, loaded DOF resists displacement).
    for (int i = 0; i < 12; ++i) EXPECT_GT(at(k, i, i), 0.0);
}

// A rigid-body translation produces no elastic force: Ke * (uniform shift) = 0.
TEST(LinearElementTest, RigidTranslationIsInNullSpace) {
    TetMesh m = unitTet();
    const auto k = hz::sim::elementStiffness(m, m.elements[0], hz::sim::materials::aluminum());

    // Unit translation in x: every node's x DOF = 1, others 0.
    std::array<double, 12> u{};
    for (int i = 0; i < 4; ++i) u[3 * i + 0] = 1.0;

    // Scale a reference force by Young's modulus to judge "≈ 0".
    const double scale = hz::sim::materials::aluminum().youngsModulus;
    for (int r = 0; r < 12; ++r) {
        double f = 0.0;
        for (int c = 0; c < 12; ++c) f += at(k, r, c) * u[c];
        EXPECT_LT(std::abs(f), 1e-3 * scale);
    }
}

// A degenerate (zero-volume) element and an invalid material both yield a zero
// stiffness matrix rather than NaNs.
TEST(LinearElementTest, DegenerateAndInvalidYieldZero) {
    // Four coplanar nodes -> zero volume.
    TetMesh flat;
    flat.nodes = {Node{Vec3(0, 0, 0)}, Node{Vec3(1, 0, 0)}, Node{Vec3(0, 1, 0)},
                  Node{Vec3(1, 1, 0)}};
    flat.elements = {Tet4{{0, 1, 2, 3}}};
    const auto kFlat =
        hz::sim::elementStiffness(flat, flat.elements[0], hz::sim::materials::steel());
    for (double v : kFlat) EXPECT_EQ(v, 0.0);

    TetMesh m = unitTet();
    ElasticMaterial bad;  // E = 0 -> invalid
    EXPECT_FALSE(bad.isValid());
    const auto kBad = hz::sim::elementStiffness(m, m.elements[0], bad);
    for (double v : kBad) EXPECT_EQ(v, 0.0);
}

// Material presets are physically sensible and valid for analysis.
TEST(LinearElementTest, MaterialPresetsAreValid) {
    EXPECT_TRUE(hz::sim::materials::steel().isValid());
    EXPECT_TRUE(hz::sim::materials::aluminum().isValid());
    EXPECT_TRUE(hz::sim::materials::titanium().isValid());
    // Steel is stiffer than aluminium.
    EXPECT_GT(hz::sim::materials::steel().youngsModulus,
              hz::sim::materials::aluminum().youngsModulus);
}
