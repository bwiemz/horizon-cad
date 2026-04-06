#include <gtest/gtest.h>

#include "horizon/math/BoundingBox.h"
#include "horizon/math/RTree.h"

using namespace hz::math;

// ---------------------------------------------------------------------------
// 1. Empty tree range query returns nothing
// ---------------------------------------------------------------------------
TEST(RTreeTest, EmptyTreeRangeQueryReturnsNothing) {
    RTree<uint64_t> tree;
    BoundingBox query(Vec3(0, 0, -1e9), Vec3(10, 10, 1e9));
    auto results = tree.query(query);
    EXPECT_TRUE(results.empty());
}

// ---------------------------------------------------------------------------
// 2. Insert one and query hit
// ---------------------------------------------------------------------------
TEST(RTreeTest, InsertOneAndQueryHit) {
    RTree<uint64_t> tree;
    BoundingBox box(Vec3(1, 1, 0), Vec3(3, 3, 0));
    tree.insert(1, box);
    BoundingBox query(Vec3(0, 0, -1e9), Vec3(5, 5, 1e9));
    auto results = tree.query(query);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], 1u);
}

// ---------------------------------------------------------------------------
// 3. Insert one and query miss
// ---------------------------------------------------------------------------
TEST(RTreeTest, InsertOneAndQueryMiss) {
    RTree<uint64_t> tree;
    BoundingBox box(Vec3(1, 1, 0), Vec3(3, 3, 0));
    tree.insert(1, box);
    BoundingBox query(Vec3(10, 10, -1e9), Vec3(20, 20, 1e9));
    auto results = tree.query(query);
    EXPECT_TRUE(results.empty());
}

// ---------------------------------------------------------------------------
// 4. Insert multiple and query subset
// ---------------------------------------------------------------------------
TEST(RTreeTest, InsertMultipleAndQuerySubset) {
    RTree<uint64_t> tree;
    tree.insert(1, BoundingBox(Vec3(0, 0, 0), Vec3(2, 2, 0)));
    tree.insert(2, BoundingBox(Vec3(5, 5, 0), Vec3(7, 7, 0)));
    tree.insert(3, BoundingBox(Vec3(1, 1, 0), Vec3(3, 3, 0)));
    BoundingBox query(Vec3(0, 0, -1e9), Vec3(4, 4, 1e9));
    auto results = tree.query(query);
    ASSERT_EQ(results.size(), 2u);
    std::sort(results.begin(), results.end());
    EXPECT_EQ(results[0], 1u);
    EXPECT_EQ(results[1], 3u);
}

// ---------------------------------------------------------------------------
// 5. Size reflects inserts
// ---------------------------------------------------------------------------
TEST(RTreeTest, SizeReflectsInserts) {
    RTree<uint64_t> tree;
    EXPECT_EQ(tree.size(), 0u);
    tree.insert(1, BoundingBox(Vec3(0, 0, 0), Vec3(1, 1, 0)));
    EXPECT_EQ(tree.size(), 1u);
    tree.insert(2, BoundingBox(Vec3(2, 2, 0), Vec3(3, 3, 0)));
    EXPECT_EQ(tree.size(), 2u);
}

// ---------------------------------------------------------------------------
// 6. Empty and clear
// ---------------------------------------------------------------------------
TEST(RTreeTest, EmptyAndClear) {
    RTree<uint64_t> tree;
    EXPECT_TRUE(tree.empty());
    tree.insert(1, BoundingBox(Vec3(0, 0, 0), Vec3(1, 1, 0)));
    EXPECT_FALSE(tree.empty());
    tree.clear();
    EXPECT_TRUE(tree.empty());
    EXPECT_EQ(tree.size(), 0u);
    auto results = tree.query(BoundingBox(Vec3(-1e9, -1e9, -1e9), Vec3(1e9, 1e9, 1e9)));
    EXPECT_TRUE(results.empty());
}

// ---------------------------------------------------------------------------
// 7. Stress test triggers node splits
// ---------------------------------------------------------------------------
TEST(RTreeTest, ManyInsertsForceSplits) {
    RTree<uint64_t> tree;
    constexpr int count = 200;
    for (int i = 0; i < count; ++i) {
        double x = static_cast<double>(i * 3);
        tree.insert(static_cast<uint64_t>(i),
                    BoundingBox(Vec3(x, 0, 0), Vec3(x + 1, 1, 0)));
    }
    EXPECT_EQ(tree.size(), static_cast<size_t>(count));

    // Query should find only items in range [0, 10] x [0, 1]
    auto results = tree.query(BoundingBox(Vec3(0, 0, -1e9), Vec3(10, 1, 1e9)));
    // Items at x=0,3,6,9 -> i=0,1,2,3  (x+1 = 1,4,7,10 all within [0,10])
    ASSERT_EQ(results.size(), 4u);
    std::sort(results.begin(), results.end());
    EXPECT_EQ(results[0], 0u);
    EXPECT_EQ(results[1], 1u);
    EXPECT_EQ(results[2], 2u);
    EXPECT_EQ(results[3], 3u);
}
