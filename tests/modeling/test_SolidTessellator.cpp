#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

using namespace hz::model;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// Basic tessellation tests
// ---------------------------------------------------------------------------

TEST(SolidTessellatorTest, BoxTessellation) {
    auto solid = PrimitiveFactory::makeBox(10.0, 5.0, 3.0);
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    EXPECT_FALSE(mesh.positions.empty());
    EXPECT_FALSE(mesh.normals.empty());
    EXPECT_FALSE(mesh.indices.empty());
    EXPECT_EQ(mesh.positions.size(), mesh.normals.size());
    EXPECT_EQ(mesh.indices.size() % 3, 0u);
}

TEST(SolidTessellatorTest, AllIndicesInRange) {
    auto solid = PrimitiveFactory::makeBox(10.0, 5.0, 3.0);
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    size_t vertexCount = mesh.positions.size() / 3;
    for (uint32_t idx : mesh.indices) {
        EXPECT_LT(idx, vertexCount);
    }
}

TEST(SolidTessellatorTest, CylinderTessellation) {
    auto solid = PrimitiveFactory::makeCylinder(5.0, 10.0);
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    EXPECT_FALSE(mesh.positions.empty());
    EXPECT_EQ(mesh.indices.size() % 3, 0u);
}

TEST(SolidTessellatorTest, SphereTessellation) {
    auto solid = PrimitiveFactory::makeSphere(5.0);
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    EXPECT_FALSE(mesh.positions.empty());
}

TEST(SolidTessellatorTest, NormalsAreUnitLength) {
    auto solid = PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    auto mesh = SolidTessellator::tessellate(*solid, 0.5);
    EXPECT_FALSE(mesh.normals.empty());
    // Every normal should be approximately unit length.
    for (size_t i = 0; i < mesh.normals.size(); i += 3) {
        Vec3 n(mesh.normals[i], mesh.normals[i + 1], mesh.normals[i + 2]);
        double len = n.length();
        EXPECT_NEAR(len, 1.0, 0.01) << "Normal at vertex " << (i / 3) << " has length " << len;
    }
}

TEST(SolidTessellatorTest, ConeTessellation) {
    auto solid = PrimitiveFactory::makeCone(3.0, 1.0, 5.0);
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    EXPECT_FALSE(mesh.positions.empty());
    EXPECT_EQ(mesh.indices.size() % 3, 0u);
    size_t vertexCount = mesh.positions.size() / 3;
    for (uint32_t idx : mesh.indices) {
        EXPECT_LT(idx, vertexCount);
    }
}

TEST(SolidTessellatorTest, TorusTessellation) {
    auto solid = PrimitiveFactory::makeTorus(5.0, 1.5);
    auto mesh = SolidTessellator::tessellate(*solid, 0.1);
    EXPECT_FALSE(mesh.positions.empty());
    EXPECT_EQ(mesh.indices.size() % 3, 0u);
    size_t vertexCount = mesh.positions.size() / 3;
    for (uint32_t idx : mesh.indices) {
        EXPECT_LT(idx, vertexCount);
    }
}

TEST(SolidTessellatorTest, EmptySolidProducesEmptyMesh) {
    hz::topo::Solid empty;
    auto mesh = SolidTessellator::tessellate(empty, 0.1);
    EXPECT_TRUE(mesh.positions.empty());
    EXPECT_TRUE(mesh.normals.empty());
    EXPECT_TRUE(mesh.indices.empty());
}

// ---------------------------------------------------------------------------
// Display geometry matches the solid, once per face.
//
// SolidTessellator emits a face's whole *untrimmed* carrier when that carrier
// is curved.  With the old box-topology primitives that meant every face on a
// shared surface re-emitted the entire surface: a sphere bound one sphere
// patch to six faces and drew it six times — 480,000 triangles of overlapping
// geometry for a shape that needs a couple of thousand, and a mesh whose
// enclosed volume was three times the solid's.
//
// Faceted primitives carry planar patches that match their loops, so every
// face takes the loop path and is emitted exactly once.  These bounds pin
// that: a regression to shared curved carriers would blow straight past them.
// ---------------------------------------------------------------------------

TEST(SolidTessellatorTest, CurvedPrimitivesEmitEachFaceExactlyOnce) {
    struct Case {
        const char* name;
        std::unique_ptr<hz::topo::Solid> solid;
    };
    std::vector<Case> cases;
    cases.push_back({"cylinder", PrimitiveFactory::makeCylinder(5.0, 10.0)});
    cases.push_back({"sphere", PrimitiveFactory::makeSphere(5.0)});
    cases.push_back({"cone", PrimitiveFactory::makeCone(5.0, 2.0, 10.0)});
    cases.push_back({"torus", PrimitiveFactory::makeTorus(10.0, 3.0)});

    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        ASSERT_NE(c.solid, nullptr);
        const auto mesh = SolidTessellator::tessellate(*c.solid, 0.01);
        const size_t tris = mesh.indices.size() / 3;

        // A loop of k vertices triangulates to exactly k-2 triangles, so one
        // pass over the faces gives the count the mesh must have.
        size_t expected = 0;
        for (const auto& f : c.solid->faces()) {
            expected += static_cast<size_t>(hz::topo::loopSize(f.outerLoop)) - 2;
        }
        EXPECT_EQ(tris, expected) << "each face must be emitted once — " << tris
                                  << " triangles for " << c.solid->faceCount() << " faces";
    }
}

TEST(SolidTessellatorTest, TessellatedMeshEnclosesTheSolidsVolume) {
    // The display mesh and the mass-properties integrator must agree; when
    // curved carriers were re-emitted per face they did not (a cylinder's mesh
    // enclosed 2388 against the solid's 785).
    auto cyl = PrimitiveFactory::makeCylinder(5.0, 10.0);
    ASSERT_NE(cyl, nullptr);
    const auto mesh = SolidTessellator::tessellate(*cyl, 0.01);

    double vol6 = 0.0;
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        Vec3 p[3];
        for (int k = 0; k < 3; ++k) {
            const size_t base = 3 * static_cast<size_t>(mesh.indices[i + static_cast<size_t>(k)]);
            p[k] = Vec3(mesh.positions[base], mesh.positions[base + 1], mesh.positions[base + 2]);
        }
        vol6 += p[0].dot(p[1].cross(p[2]));
    }
    const double meshVolume = std::abs(vol6) / 6.0;
    const double solidVolume = MassPropertiesCalculator::compute(*cyl).volume;
    EXPECT_NEAR(meshVolume, solidVolume, 1e-6 * solidVolume);
}

// -- What the mesh says of the solid (Phase 132) ------------------------------

// Every triangle names its face, and the edges are the box's twelve.
TEST(SolidTessellatorTest, TrianglesNameTheirFacesAndTheEdgesAreThePartsOwn) {
    const auto box = hz::model::PrimitiveFactory::makeBox(2.0, 3.0, 4.0);
    const auto mesh = hz::model::SolidTessellator::tessellate(*box, 0.1);
    ASSERT_TRUE(mesh.hasFaces());
    EXPECT_EQ(mesh.faceTags.size(), 6u);
    EXPECT_NE(std::find(mesh.faceTags.begin(), mesh.faceTags.end(), "box/top"),
              mesh.faceTags.end());
    EXPECT_EQ(mesh.edges.size(), 12u);
    for (const auto& edge : mesh.edges) {
        EXPECT_EQ(edge.points.size(), 6u) << "a straight edge is a line";
        EXPECT_FALSE(edge.tag.empty());
    }
}

// A faceted cylinder shows its two rims, not a seam between every facet.
TEST(SolidTessellatorTest, ACylindersFacetsDoNotShowAsEdges) {
    const auto cylinder = hz::model::PrimitiveFactory::makeCylinder(1.0, 2.0, 16);
    const auto mesh = hz::model::SolidTessellator::tessellate(*cylinder, 0.1);
    ASSERT_FALSE(mesh.edges.empty());
    for (const auto& edge : mesh.edges) {
        const float z0 = edge.points[2];
        const float z1 = edge.points[edge.points.size() - 1];
        EXPECT_FLOAT_EQ(z0, z1) << edge.tag << " runs up the side: a seam between facets";
    }
    EXPECT_EQ(mesh.edges.size(), 32u) << "sixteen chords round each rim";
}
