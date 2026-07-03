#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <set>

#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz::model;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// LinearPatternReplicatesBox
// ---------------------------------------------------------------------------

TEST(PatternTest, LinearPatternReplicatesBox) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    ASSERT_NE(box, nullptr);

    // 4 boxes spaced 5 apart along +X.
    auto pattern = Pattern::linear(*box, Vec3(1, 0, 0), 5.0, 4);
    ASSERT_NE(pattern, nullptr);

    // Each box: 8V, 12E, 6F, 1 shell → 4 copies.
    EXPECT_EQ(pattern->vertexCount(), 32u);
    EXPECT_EQ(pattern->edgeCount(), 48u);
    EXPECT_EQ(pattern->faceCount(), 24u);
    EXPECT_EQ(pattern->shellCount(), 4u);
    EXPECT_TRUE(pattern->checkEulerFormula());
    EXPECT_TRUE(pattern->checkManifold());

    // Instances span x in [0, 2] .. [15, 17].
    double minX = 1e9, maxX = -1e9;
    for (const auto& v : pattern->vertices()) {
        minX = std::min(minX, v.point.x);
        maxX = std::max(maxX, v.point.x);
    }
    EXPECT_NEAR(minX, 0.0, 1e-9);
    EXPECT_NEAR(maxX, 17.0, 1e-9);  // 3*5 + 2
}

// ---------------------------------------------------------------------------
// LinearPatternTopologyIds
// ---------------------------------------------------------------------------

TEST(PatternTest, LinearPatternTopologyIds) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    auto pattern = Pattern::linear(*box, Vec3(1, 0, 0), 5.0, 3);
    ASSERT_NE(pattern, nullptr);

    // Instance 0 keeps "box/top"; instances 1,2 get pattern children.
    std::set<std::string> tags;
    for (const auto& f : pattern->faces()) tags.insert(f.topoId.tag());

    EXPECT_TRUE(tags.count("box/top"));            // seed
    EXPECT_TRUE(tags.count("box/top/pattern:1"));  // copy 1
    EXPECT_TRUE(tags.count("box/top/pattern:2"));  // copy 2

    // The genealogy relationship holds.
    auto seed = hz::topo::TopologyID::make("box", "top");
    auto copy1 = seed.child("pattern", 1);
    EXPECT_TRUE(copy1.isDescendantOf(seed));
}

// ---------------------------------------------------------------------------
// LinearPatternSuppression
// ---------------------------------------------------------------------------

TEST(PatternTest, LinearPatternSuppression) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    // 5 instances, suppress indices 1 and 3 → 3 bodies.
    auto pattern = Pattern::linear(*box, Vec3(1, 0, 0), 5.0, 5, {1, 3});
    ASSERT_NE(pattern, nullptr);
    EXPECT_EQ(pattern->shellCount(), 3u);
    EXPECT_EQ(pattern->faceCount(), 18u);
}

// ---------------------------------------------------------------------------
// CircularPatternPlacesInstances
// ---------------------------------------------------------------------------

TEST(PatternTest, CircularPatternPlacesInstances) {
    // A small box offset from the origin, patterned 4x at 90 degrees about Z.
    auto box = PrimitiveFactory::makeBox(1.0, 1.0, 1.0);
    // Shift it out along +X so rotation moves it around.
    for (auto& v : const_cast<std::deque<hz::topo::Vertex>&>(box->vertices())) {
        v.point.x += 10.0;
    }

    auto pattern = Pattern::circular(*box, Vec3(0, 0, 0), Vec3(0, 0, 1), std::numbers::pi / 2.0, 4);
    ASSERT_NE(pattern, nullptr);
    EXPECT_EQ(pattern->shellCount(), 4u);
    EXPECT_TRUE(pattern->checkEulerFormula());
    EXPECT_TRUE(pattern->checkManifold());

    // The four instances sit near +X, +Y, -X, -Y (radius ~10-11).
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    for (const auto& v : pattern->vertices()) {
        minX = std::min(minX, v.point.x);
        maxX = std::max(maxX, v.point.x);
        minY = std::min(minY, v.point.y);
        maxY = std::max(maxY, v.point.y);
    }
    // Instance at +X reaches x~11; instance rotated to -X reaches x~-10.
    EXPECT_GT(maxX, 10.0);
    EXPECT_LT(minX, -9.0);
    EXPECT_GT(maxY, 10.0);
    EXPECT_LT(minY, -9.0);
}

// ---------------------------------------------------------------------------
// CountOneReturnsSingleBody
// ---------------------------------------------------------------------------

TEST(PatternTest, CountOneReturnsSingleBody) {
    auto box = PrimitiveFactory::makeBox(3.0, 3.0, 3.0);
    auto pattern = Pattern::linear(*box, Vec3(1, 0, 0), 5.0, 1);
    ASSERT_NE(pattern, nullptr);
    EXPECT_EQ(pattern->shellCount(), 1u);
    EXPECT_EQ(pattern->faceCount(), 6u);
    EXPECT_TRUE(pattern->isValid());

    // Invalid count.
    EXPECT_EQ(Pattern::linear(*box, Vec3(1, 0, 0), 5.0, 0), nullptr);
}
