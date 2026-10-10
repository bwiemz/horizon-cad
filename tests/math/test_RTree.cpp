#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <iterator>
#include <map>
#include <random>
#include <vector>

#include "../PortableRandom.h"
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
        tree.insert(static_cast<uint64_t>(i), BoundingBox(Vec3(x, 0, 0), Vec3(x + 1, 1, 0)));
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
    if (!few.empty()) {
        EXPECT_EQ(few[0], 50u);
    }
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

// ---------------------------------------------------------------------------
// Removal without rebuilding (Phase 122)
// ---------------------------------------------------------------------------

namespace {

BoundingBox boxAt(double x, double y, double w = 1.0, double h = 1.0) {
    return BoundingBox(Vec3(x, y, 0), Vec3(x + w, y + h, 0));
}

std::vector<uint64_t> sorted(std::vector<uint64_t> v) {
    std::sort(v.begin(), v.end());
    return v;
}

}  // namespace

// Random inserts, removals (with right, wrong and no box hints) and queries,
// checked against a brute-force list after every step.
TEST(RTreeTest, RemovalMatchesBruteForceUnderRandomChurn) {
    RTree<uint64_t> tree;
    std::map<uint64_t, BoundingBox> truth;
    std::mt19937 rng(12345);
    hz::test::Uniform coord(0.0, 500.0);
    hz::test::Uniform size(0.1, 20.0);
    uint64_t nextId = 1;

    for (int step = 0; step < 6000; ++step) {
        const int op = static_cast<int>(rng() % 10);
        if (op < 5 || truth.empty()) {
            const BoundingBox b = boxAt(coord(rng), coord(rng), size(rng), size(rng));
            tree.insert(nextId, b);
            truth[nextId++] = b;
        } else if (op < 8) {
            auto it = truth.begin();
            std::advance(it, static_cast<long>(rng() % truth.size()));
            const int hint = static_cast<int>(rng() % 3);
            if (hint == 0) {
                EXPECT_TRUE(tree.remove(it->first, it->second));  // the right box
            } else if (hint == 1) {
                EXPECT_TRUE(tree.remove(it->first, boxAt(-900, -900)));  // a wrong box
            } else {
                tree.remove(it->first);  // no box
            }
            truth.erase(it);
        } else {
            const BoundingBox q = boxAt(coord(rng), coord(rng), 60.0, 60.0);
            std::vector<uint64_t> expected;
            for (const auto& [id, b] : truth) {
                if (b.intersects(q)) expected.push_back(id);
            }
            ASSERT_EQ(sorted(tree.query(q)), sorted(expected)) << "step " << step;
        }
        ASSERT_EQ(tree.size(), truth.size()) << "step " << step;
    }

    // Everything left is still found, and removing it all empties the tree.
    const BoundingBox all(Vec3(-1e9, -1e9, -1e9), Vec3(1e9, 1e9, 1e9));
    EXPECT_EQ(tree.query(all).size(), truth.size());
    for (const auto& [id, b] : truth) EXPECT_TRUE(tree.remove(id, b));
    EXPECT_TRUE(tree.empty());
    EXPECT_TRUE(tree.query(all).empty());
    EXPECT_EQ(tree.nodeCount(), 1u);
}

// Removing and inserting over and over reuses nodes instead of growing.
TEST(RTreeTest, ChurnDoesNotGrowTheTree) {
    RTree<uint64_t> tree;
    for (uint64_t i = 0; i < 2000; ++i) {
        tree.insert(i, boxAt(static_cast<double>(i % 50) * 2.0, static_cast<double>(i / 50) * 2.0));
    }
    const size_t nodesAfterFill = tree.nodeCount();
    for (int round = 0; round < 20; ++round) {
        for (uint64_t i = 0; i < 2000; i += 2) {
            ASSERT_TRUE(tree.remove(
                i, boxAt(static_cast<double>(i % 50) * 2.0, static_cast<double>(i / 50) * 2.0)));
        }
        for (uint64_t i = 0; i < 2000; i += 2) {
            tree.insert(
                i, boxAt(static_cast<double>(i % 50) * 2.0, static_cast<double>(i / 50) * 2.0));
        }
    }
    EXPECT_EQ(tree.size(), 2000u);
    EXPECT_LE(tree.nodeCount(), nodesAfterFill * 2) << "freed nodes are not being reused";
}

// A value inserted twice is removed twice by remove(value, box), once each,
// and entirely by remove(value).
TEST(RTreeTest, DuplicateValuesAreRemovedOneAtATimeOrAllAtOnce) {
    RTree<uint64_t> tree;
    tree.insert(7, boxAt(0, 0));
    tree.insert(7, boxAt(50, 50));
    tree.insert(8, boxAt(10, 10));
    EXPECT_TRUE(tree.remove(7, boxAt(50, 50)));
    EXPECT_EQ(tree.size(), 2u);
    EXPECT_EQ(tree.query(boxAt(50, 50)).size(), 0u);
    EXPECT_EQ(tree.query(boxAt(0, 0)).size(), 1u);

    tree.insert(7, boxAt(50, 50));
    tree.remove(7);
    EXPECT_EQ(tree.size(), 1u);
    EXPECT_FALSE(tree.remove(7, boxAt(0, 0)));
}

// ---------------------------------------------------------------------------
// 12. A tree flat in Z is built as well as one spread in X
// ---------------------------------------------------------------------------
// A tree whose entries are spread in Z, or in a single plane, is built by the
// same code as one spread in X, and has to be built as well. The area a node
// is measured by when it chooses where to put an entry was its size in x
// times its size in y and nothing else, so for anything flat in Z that area
// was zero: every candidate measured the same, the choice between them fell
// to whatever came first, and the tree packed worse than it does otherwise.
// A part modelled in one plane, and anything a section view or a drawing
// passes through, is exactly that.
TEST(RTreeTest, ATreeOfEntriesFlatInZIsBuiltAsWellAsOneSpreadInX) {
    constexpr size_t kEntries = 20000;

    // The same 20,000 boxes laid out three ways, at the same spread.
    const auto build = [](int plane) {
        RTree<uint64_t> tree;
        for (uint64_t i = 0; i < kEntries; ++i) {
            const double a = static_cast<double>((i * 2654435761u) % 1000) / 1000.0;
            const double b = static_cast<double>((i * 40503u) % 1000) / 1000.0;
            const double c = static_cast<double>((i * 2246822519u) % 1000) / 1000.0;
            const Vec3 corner = plane == 0  // spread in x
                                    ? Vec3(a * 100.0, b, c)
                                    : plane == 1  // the yz plane: x is 0 throughout
                                          ? Vec3(0.0, b * 100.0, c)
                                          : Vec3(0.0, b * 100.0, 0.0);  // a line
            tree.insert(i, BoundingBox(corner, corner + Vec3(0.001, 0.001, 0.001)));
        }
        return tree;
    };

    const size_t spread = build(0).nodeCount();
    const size_t inAPlane = build(1).nodeCount();
    const size_t onALine = build(2).nodeCount();

    // How the points are arranged does not change how many nodes they need:
    // only their spread across all three axes does. Measured: 2092 nodes
    // spread in x against 2245 in the yz plane, so a tree measuring all three
    // axes closes that gap and one measuring two does not. The bound sits
    // between the two, with room either side of it.
    EXPECT_LE(inAPlane, spread * 105 / 100)
        << spread << " nodes spread in x, " << inAPlane << " in the yz plane";
    EXPECT_LE(onALine, spread * 105 / 100)
        << spread << " nodes spread in x, " << onALine << " along a line";
}

// The measure a node is chosen by has to be one a real node is never zero by.
// Every 2D entity is flat in z by construction — its bounding box has zero
// extent there — so measuring the room a node takes by multiplying its three
// sizes gives zero for a whole drawing's spatial index at once: no candidate
// is better than any other, so every entry goes where the last split left it,
// and the tree ends up nearly three times the nodes it needs. Measured on
// 4000 lines: 991 nodes and 20,000 queries in 27 ms, against 369 and 11 ms
// for the sum. Summed, no node in a real tree is ever zero.
//
// The bound below is on the node count rather than on time, which is what
// actually differs and does not vary with the machine. Time is checked only
// as loose a bound, since CI runs this with four at a time.
TEST(RTreeTest, ADrawingSizedIndexIsBuiltWithTheNodesItNeeds) {
    constexpr uint64_t kLines = 4000;
    RTree<uint64_t> tree;
    std::vector<BoundingBox> boxes;
    for (uint64_t i = 0; i < kLines; ++i) {
        const double x = static_cast<double>(i % 200) * 2.0;
        const double y = static_cast<double>(i / 200) * 2.0;
        const BoundingBox box = boxAt(x, y);
        boxes.push_back(box);
        tree.insert(i, box);
    }
    EXPECT_EQ(tree.size(), kLines);

    // 4000 leaves at 16 to a node needs about 270 leaves and 300 nodes. The
    // measure that reads zero everywhere nearly triples that; the bound sits
    // between the two.
    EXPECT_LT(tree.nodeCount(), 500u) << "4000 flat entries made " << tree.nodeCount() << " nodes";

    // Whatever shape it came out, every query is answered correctly: held
    // against a plain sweep of the same boxes, not against the tree's idea.
    constexpr int kQueries = 300;
    size_t hits = 0;
    const auto start = std::chrono::steady_clock::now();
    for (int q = 0; q < kQueries; ++q) {
        const BoundingBox query =
            boxAt(static_cast<double>(q % 200) * 2.0, static_cast<double>(q / 200) * 2.0, 1.5, 1.5);
        const auto found = tree.query(query);
        hits += found.size();

        std::vector<uint64_t> expected;
        for (size_t i = 0; i < boxes.size(); ++i) {
            if (boxes[i].intersects(query)) {
                expected.push_back(i);
            }
        }
        EXPECT_EQ(sorted(found), sorted(expected)) << "query " << q;
    }
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    EXPECT_GT(hits, 0u);
    EXPECT_LT(ms, 100.0 * kQueries) << kQueries << " drawing-sized queries took " << ms << " ms";
}

// A query answers the same whatever the shape of the entries it holds: held
// against a plain sweep of the same boxes, not against the tree's own idea of
// them.
TEST(RTreeTest, EveryQueryAgreesWithASweepWhateverTheEntriesShape) {
    for (const int plane : {0, 1, 2}) {
        RTree<uint64_t> tree;
        std::vector<BoundingBox> boxes;
        for (uint64_t i = 0; i < 2000; ++i) {
            const double a = static_cast<double>((i * 2654435761u) % 1000) / 1000.0;
            const double b = static_cast<double>((i * 40503u) % 1000) / 1000.0;
            const double c = static_cast<double>((i * 2246822519u) % 1000) / 1000.0;
            const Vec3 corner = plane == 0   ? Vec3(a * 100.0, b, c)
                                : plane == 1 ? Vec3(0.0, b * 100.0, c)
                                             : Vec3(0.0, b * 100.0, 0.0);
            const BoundingBox box(corner, corner + Vec3(0.05, 0.05, 0.05));
            boxes.push_back(box);
            tree.insert(i, box);
        }

        for (int q = 0; q < 200; ++q) {
            const double x = static_cast<double>((q * 40503u) % 1000) / 10.0 - 10.0;
            const double y = static_cast<double>((q * 2246822519u) % 1000) / 10.0 - 10.0;
            const Vec3 corner(x, y, plane == 2 ? 0.0 : x * 0.01);
            const BoundingBox query(corner, corner + Vec3(2.0, 2.0, 2.0));

            std::vector<uint64_t> expected;
            for (size_t i = 0; i < boxes.size(); ++i) {
                if (boxes[i].intersects(query)) {
                    expected.push_back(i);
                }
            }
            EXPECT_EQ(sorted(tree.query(query)), sorted(expected))
                << "plane " << plane << ", q " << q;
        }
    }
}
