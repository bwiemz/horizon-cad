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

// ---------------------------------------------------------------------------
// 8. Deep tree with multi-level internal-node splits (small MaxChildren)
// ---------------------------------------------------------------------------
TEST(RTreeTest, DeepTreeMultiLevelSplits) {
    // Use small MaxChildren to force deep tree with multi-level splits.
    RTree<uint64_t, 4, 2> tree;
    for (uint64_t i = 0; i < 500; ++i) {
        double x = static_cast<double>(i) * 2.0;
        tree.insert(i, BoundingBox(Vec3(x, 0, 0), Vec3(x + 1, 1, 0)));
    }
    EXPECT_EQ(tree.size(), 500u);

    // Query everything — must return all 500.
    BoundingBox everything(Vec3(-1, -1, -1e9), Vec3(1001, 2, 1e9));
    auto all = tree.query(everything);
    EXPECT_EQ(all.size(), 500u);

    // Query a small range — should return exactly 1.
    BoundingBox small(Vec3(100, -1, -1e9), Vec3(101.5, 2, 1e9));
    auto few = tree.query(small);
    EXPECT_EQ(few.size(), 1u);
    if (!few.empty()) EXPECT_EQ(few[0], 50u);
}

// ---------------------------------------------------------------------------
// 9. Remove an existing entry decrements size and excludes it from queries
// ---------------------------------------------------------------------------
TEST(RTreeTest, RemoveExistingEntry) {
    RTree<uint64_t> tree;
    tree.insert(1, BoundingBox(Vec3(0, 0, 0), Vec3(2, 2, 0)));
    tree.insert(2, BoundingBox(Vec3(5, 5, 0), Vec3(7, 7, 0)));
    tree.remove(1);
    EXPECT_EQ(tree.size(), 1u);
    BoundingBox queryAll(Vec3(-100, -100, -1e9), Vec3(100, 100, 1e9));
    auto results = tree.query(queryAll);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], 2u);
}

// ---------------------------------------------------------------------------
// 10. Removing a non-existent value is a no-op
// ---------------------------------------------------------------------------
TEST(RTreeTest, RemoveNonExistentIsNoOp) {
    RTree<uint64_t> tree;
    tree.insert(1, BoundingBox(Vec3(0, 0, 0), Vec3(2, 2, 0)));
    tree.remove(999);
    EXPECT_EQ(tree.size(), 1u);
}

// ---------------------------------------------------------------------------
// 11. Stress: insert 1000 items in a grid, query a small window
// ---------------------------------------------------------------------------
TEST(RTreeTest, InsertManyAndQueryCorrectly) {
    RTree<uint64_t> tree;
    for (uint64_t i = 0; i < 1000; ++i) {
        double x = static_cast<double>(i % 100) * 3.0;
        double y = static_cast<double>(i / 100) * 3.0;
        tree.insert(i, BoundingBox(Vec3(x, y, 0), Vec3(x + 1, y + 1, 0)));
    }
    EXPECT_EQ(tree.size(), 1000u);
    BoundingBox smallQuery(Vec3(14.5, 14.5, -1e9), Vec3(16.5, 16.5, 1e9));
    auto results = tree.query(smallQuery);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], 505u);
}

// ---------------------------------------------------------------------------
// 12. Query spanning the entire space returns every inserted entry
// ---------------------------------------------------------------------------
TEST(RTreeTest, QueryAllReturnsEverything) {
    RTree<uint64_t> tree;
    for (uint64_t i = 0; i < 100; ++i) {
        double x = static_cast<double>(i);
        tree.insert(i, BoundingBox(Vec3(x, 0, 0), Vec3(x + 0.5, 0.5, 0)));
    }
    BoundingBox everything(Vec3(-1, -1, -1e9), Vec3(200, 200, 1e9));
    auto results = tree.query(everything);
    EXPECT_EQ(results.size(), 100u);
}
