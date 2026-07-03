#include <gtest/gtest.h>

#include <cmath>

#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Shell.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz::model;
using hz::math::Vec3;
using hz::topo::TopologyID;

namespace {

struct BBox {
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9, minZ = 1e9, maxZ = -1e9;
};

BBox boundsOf(const hz::topo::Solid& solid) {
    BBox b;
    for (const auto& v : solid.vertices()) {
        b.minX = std::min(b.minX, v.point.x);
        b.maxX = std::max(b.maxX, v.point.x);
        b.minY = std::min(b.minY, v.point.y);
        b.maxY = std::max(b.maxY, v.point.y);
        b.minZ = std::min(b.minZ, v.point.z);
        b.maxZ = std::max(b.maxZ, v.point.z);
    }
    return b;
}

}  // namespace

// ---------------------------------------------------------------------------
// BoxTopRemovedProducesCup
// ---------------------------------------------------------------------------

TEST(ShellTest, BoxTopRemovedProducesCup) {
    // 10x10x5 box; remove the top ("box/top"), wall thickness 1 → cup.
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
    ASSERT_NE(box, nullptr);

    ShellResult r = Shell::execute(std::move(box), 1.0, {TopologyID::make("box", "top")});
    ASSERT_TRUE(r.ok) << r.message;
    ASSERT_NE(r.solid, nullptr);
    EXPECT_TRUE(r.solid->checkEulerFormula());
    EXPECT_TRUE(r.solid->checkManifold());

    // The outer envelope is unchanged: 10x10x5.
    BBox b = boundsOf(*r.solid);
    EXPECT_NEAR(b.maxX - b.minX, 10.0, 1e-6);
    EXPECT_NEAR(b.maxY - b.minY, 10.0, 1e-6);
    EXPECT_NEAR(b.maxZ - b.minZ, 5.0, 1e-6);

    // Cup topology: 4-ring stack over an N=4 profile → 3N+2 = 14 faces,
    // 4N = 16 vertices, 7N = 28 edges.
    EXPECT_EQ(r.solid->faceCount(), 14u);
    EXPECT_EQ(r.solid->vertexCount(), 16u);
    EXPECT_EQ(r.solid->edgeCount(), 28u);

    // The cavity floor sits one wall-thickness above the base (z = 1); the
    // inner walls span x,y in [1, 9] (offset inward by 1 from [0, 10]).
    int cavityFloorVerts = 0, innerCornerVerts = 0;
    for (const auto& v : r.solid->vertices()) {
        if (std::abs(v.point.z - 1.0) < 1e-6) ++cavityFloorVerts;
        if (std::abs(v.point.x - 1.0) < 1e-6 || std::abs(v.point.x - 9.0) < 1e-6) {
            if (v.point.x > 0.5 && v.point.x < 9.5) ++innerCornerVerts;
        }
    }
    EXPECT_EQ(cavityFloorVerts, 4);  // inner base ring
    EXPECT_GT(innerCornerVerts, 0);
}

// ---------------------------------------------------------------------------
// TooThickRejected
// ---------------------------------------------------------------------------

TEST(ShellTest, TooThickRejected) {
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
    // Inradius of the 10x10 top is 5; thickness 5 must be rejected.
    ShellResult r = Shell::execute(std::move(box), 5.0, {TopologyID::make("box", "top")});
    EXPECT_FALSE(r.ok);
    EXPECT_FALSE(r.message.empty());
    EXPECT_EQ(r.solid, nullptr);
}

// ---------------------------------------------------------------------------
// InvalidInputsRejected
// ---------------------------------------------------------------------------

TEST(ShellTest, InvalidInputsRejected) {
    // No removed face.
    {
        auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
        ShellResult r = Shell::execute(std::move(box), 1.0, {});
        EXPECT_FALSE(r.ok);
    }
    // Non-positive thickness.
    {
        auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
        ShellResult r = Shell::execute(std::move(box), 0.0, {TopologyID::make("box", "top")});
        EXPECT_FALSE(r.ok);
    }
    // Unknown removed face id.
    {
        auto box = PrimitiveFactory::makeBox(10.0, 10.0, 5.0);
        ShellResult r = Shell::execute(std::move(box), 1.0, {TopologyID::make("box", "nope")});
        EXPECT_FALSE(r.ok);
    }
    // Null solid.
    {
        ShellResult r = Shell::execute(nullptr, 1.0, {TopologyID::make("box", "top")});
        EXPECT_FALSE(r.ok);
    }
}

// ---------------------------------------------------------------------------
// RemoveBottomAlsoWorks
// ---------------------------------------------------------------------------

TEST(ShellTest, RemoveBottomAlsoWorks) {
    auto box = PrimitiveFactory::makeBox(8.0, 12.0, 6.0);
    ShellResult r = Shell::execute(std::move(box), 1.0, {TopologyID::make("box", "bottom")});
    ASSERT_TRUE(r.ok) << r.message;
    EXPECT_TRUE(r.solid->checkManifold());
    EXPECT_GT(r.solid->faceCount(), 6u);
}
