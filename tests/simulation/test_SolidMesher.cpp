#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/simulation/LinearStaticSolver.h"
#include "horizon/simulation/Material.h"
#include "horizon/simulation/SolidMesher.h"
#include "horizon/simulation/TetMesh.h"
#include "horizon/topology/Solid.h"

using hz::math::Vec3;
using hz::model::PrimitiveFactory;
using hz::sim::Aabb;
using hz::sim::LinearStaticSolver;
using hz::sim::meshSolidBoundingBox;
using hz::sim::NodalLoad;
using hz::sim::nodesOnPlane;
using hz::sim::solidAabb;
using hz::sim::StaticResult;
using hz::sim::TetMesh;

// The AABB of a box primitive spans [0,w] x [0,h] x [0,d].
TEST(SolidMesherTest, BoxAabb) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    const Aabb bb = solidAabb(*box);
    ASSERT_TRUE(bb.valid);
    EXPECT_NEAR(bb.min.x, 0.0, 1e-9);
    EXPECT_NEAR(bb.min.y, 0.0, 1e-9);
    EXPECT_NEAR(bb.min.z, 0.0, 1e-9);
    EXPECT_NEAR(bb.max.x, 4.0, 1e-9);
    EXPECT_NEAR(bb.max.y, 3.0, 1e-9);
    EXPECT_NEAR(bb.max.z, 2.0, 1e-9);
}

// Meshing a box solid yields a mesh covering its bounding box, and face-node
// selection picks the expected grid counts.
TEST(SolidMesherTest, MeshBoxAndSelectFaces) {
    auto box = PrimitiveFactory::makeBox(4.0, 2.0, 2.0);
    const int nx = 4, ny = 2, nz = 2;
    TetMesh mesh = meshSolidBoundingBox(*box, nx, ny, nz);

    EXPECT_EQ(mesh.nodes.size(), static_cast<std::size_t>((nx + 1) * (ny + 1) * (nz + 1)));
    EXPECT_EQ(mesh.elements.size(), static_cast<std::size_t>(nx * ny * nz * 6));

    // The x=0 and x=4 faces each hold (ny+1)*(nz+1) nodes.
    const auto face0 = nodesOnPlane(mesh, 0, 0.0);
    const auto faceL = nodesOnPlane(mesh, 0, 4.0);
    EXPECT_EQ(face0.size(), static_cast<std::size_t>((ny + 1) * (nz + 1)));
    EXPECT_EQ(faceL.size(), static_cast<std::size_t>((ny + 1) * (nz + 1)));
}

// End-to-end: mesh a box solid, fix one face and pull the opposite one, and
// recover the analytical bar-in-tension elongation delta = FL/(A E).
TEST(SolidMesherTest, ModelToFeaBarInTension) {
    const double L = 10.0, a = 1.0, A = a * a;
    auto box = PrimitiveFactory::makeBox(L, a, a);
    TetMesh mesh = meshSolidBoundingBox(*box, 8, 2, 2);
    ASSERT_FALSE(mesh.nodes.empty());

    const auto fixed = nodesOnPlane(mesh, 0, 0.0);  // x = 0 face
    const auto loaded = nodesOnPlane(mesh, 0, L);   // x = L face
    ASSERT_FALSE(fixed.empty());
    ASSERT_FALSE(loaded.empty());

    const auto mat = hz::sim::materials::steel();
    const double F = 1.0e6;
    std::vector<NodalLoad> loads;
    const double per = F / static_cast<double>(loaded.size());
    for (int n : loaded) loads.push_back(NodalLoad{n, Vec3(per, 0.0, 0.0)});

    const StaticResult r = LinearStaticSolver::solve(mesh, mat, fixed, loads);
    ASSERT_TRUE(r.converged);

    const double expectedDelta = F * L / (A * mat.youngsModulus);
    EXPECT_NEAR(r.maxDisplacementMagnitude, expectedDelta, 0.05 * expectedDelta);
}

// An empty solid produces an empty mesh.
TEST(SolidMesherTest, EmptySolidYieldsEmptyMesh) {
    hz::topo::Solid empty;
    EXPECT_FALSE(solidAabb(empty).valid);
    EXPECT_TRUE(meshSolidBoundingBox(empty, 2, 2, 2).nodes.empty());
}
