#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/SolidTessellator.h"

#include "horizon/math/Vec3.h"

#include <gtest/gtest.h>

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
