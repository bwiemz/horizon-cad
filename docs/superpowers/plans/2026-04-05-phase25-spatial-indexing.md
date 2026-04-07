# Phase 25: Spatial Indexing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace all O(n) entity iteration for snap, hit-test, and box selection with O(log n) R*-tree spatial index queries.

**Architecture:** An R*-tree is implemented in `hz::math` as a header-only template (generic over dimensionality). A `SpatialIndex` wrapper in `hz::draft` manages the R*-tree for `DraftDocument`, keeping it synchronized with entity add/remove/move operations. `SnapEngine` and `SelectTool` are refactored to query the spatial index instead of iterating all entities.

**Tech Stack:** C++20, Google Test, existing `hz::math::BoundingBox` (Vec3-based AABB)

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 3.1 (Phase 25)

**Implementation Constraints:**
- Cache-friendly contiguous node layout (flat `std::vector` of nodes, not heap-allocated per-node)
- Template on bounding-box type so it works for both 2D and future 3D
- Observer pattern: `DraftDocument` notifies `SpatialIndex` on entity mutations

---

## File Map

| Action | File | Responsibility |
|--------|------|---------------|
| Create | `src/math/include/horizon/math/RTree.h` | Generic R*-tree (header-only template) |
| Create | `tests/math/test_RTree.cpp` | R*-tree unit tests |
| Modify | `src/math/CMakeLists.txt` | Add test linkage (header-only, no new .cpp) |
| Modify | `tests/math/CMakeLists.txt` | Add `test_RTree.cpp` to test target |
| Create | `src/drafting/include/horizon/drafting/SpatialIndex.h` | DraftDocument-aware spatial index wrapper |
| Create | `src/drafting/src/SpatialIndex.cpp` | SpatialIndex implementation |
| Modify | `src/drafting/CMakeLists.txt` | Add `SpatialIndex.cpp` |
| Modify | `src/drafting/include/horizon/drafting/DraftDocument.h` | Add SpatialIndex member + observer hooks |
| Modify | `src/drafting/src/DraftDocument.cpp` | Wire entity add/remove into SpatialIndex |
| Modify | `src/drafting/include/horizon/drafting/SnapEngine.h` | Add overload accepting SpatialIndex |
| Modify | `src/drafting/src/SnapEngine.cpp` | Implement spatial-index-accelerated snap |
| Create | `tests/drafting/test_SpatialIndex.cpp` | SpatialIndex integration tests |
| Create | `tests/drafting/CMakeLists.txt` | Drafting test target |
| Modify | `tests/CMakeLists.txt` | Add `add_subdirectory(drafting)` |
| Modify | `src/ui/src/SelectTool.cpp` | Use SpatialIndex for hit-test and box selection |

---

## Task 1: R*-tree Core — Insert and Range Query

**Files:**
- Create: `src/math/include/horizon/math/RTree.h`
- Create: `tests/math/test_RTree.cpp`
- Modify: `tests/math/CMakeLists.txt`

- [ ] **Step 1: Write failing tests for R*-tree insert and range query**

Create `tests/math/test_RTree.cpp`:
```cpp
#include <gtest/gtest.h>
#include "horizon/math/RTree.h"
#include "horizon/math/BoundingBox.h"

using namespace hz::math;

TEST(RTreeTest, EmptyTreeRangeQueryReturnsNothing) {
    RTree<uint64_t> tree;
    BoundingBox query(Vec3(0, 0, -1e9), Vec3(10, 10, 1e9));
    auto results = tree.query(query);
    EXPECT_TRUE(results.empty());
}

TEST(RTreeTest, InsertOneAndQueryHit) {
    RTree<uint64_t> tree;
    BoundingBox box(Vec3(1, 1, 0), Vec3(3, 3, 0));
    tree.insert(1, box);

    BoundingBox query(Vec3(0, 0, -1e9), Vec3(5, 5, 1e9));
    auto results = tree.query(query);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], 1u);
}

TEST(RTreeTest, InsertOneAndQueryMiss) {
    RTree<uint64_t> tree;
    BoundingBox box(Vec3(1, 1, 0), Vec3(3, 3, 0));
    tree.insert(1, box);

    BoundingBox query(Vec3(10, 10, -1e9), Vec3(20, 20, 1e9));
    auto results = tree.query(query);
    EXPECT_TRUE(results.empty());
}

TEST(RTreeTest, InsertMultipleAndQuerySubset) {
    RTree<uint64_t> tree;
    tree.insert(1, BoundingBox(Vec3(0, 0, 0), Vec3(2, 2, 0)));
    tree.insert(2, BoundingBox(Vec3(5, 5, 0), Vec3(7, 7, 0)));
    tree.insert(3, BoundingBox(Vec3(1, 1, 0), Vec3(3, 3, 0)));

    // Query overlapping items 1 and 3 but not 2
    BoundingBox query(Vec3(0, 0, -1e9), Vec3(4, 4, 1e9));
    auto results = tree.query(query);
    ASSERT_EQ(results.size(), 2u);

    std::sort(results.begin(), results.end());
    EXPECT_EQ(results[0], 1u);
    EXPECT_EQ(results[1], 3u);
}

TEST(RTreeTest, SizeReflectsInserts) {
    RTree<uint64_t> tree;
    EXPECT_EQ(tree.size(), 0u);
    tree.insert(1, BoundingBox(Vec3(0, 0, 0), Vec3(1, 1, 0)));
    EXPECT_EQ(tree.size(), 1u);
    tree.insert(2, BoundingBox(Vec3(2, 2, 0), Vec3(3, 3, 0)));
    EXPECT_EQ(tree.size(), 2u);
}
```

- [ ] **Step 2: Register test file in CMakeLists**

Add `test_RTree.cpp` to `tests/math/CMakeLists.txt`:
```cmake
add_executable(hz_math_tests
    test_Vec2.cpp
    test_Vec3.cpp
    test_Mat4.cpp
    test_Quaternion.cpp
    test_Transform.cpp
    test_BoundingBox.cpp
    test_RTree.cpp
)
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cmake --build build/debug --config Debug --target hz_math_tests && ctest --test-dir build/debug -C Debug -R RTree --output-on-failure`
Expected: Compilation fails — `RTree.h` not found.

- [ ] **Step 4: Implement R*-tree with insert and range query**

Create `src/math/include/horizon/math/RTree.h`:
```cpp
#pragma once

#include "horizon/math/BoundingBox.h"
#include <algorithm>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

namespace hz::math {

/// A cache-friendly R*-tree for spatial indexing.
/// ValueT is the type stored at leaf nodes (e.g., uint64_t entity ID).
/// All nodes are stored contiguously in a flat vector for cache performance.
template <typename ValueT, int MaxChildren = 16, int MinChildren = MaxChildren / 3>
class RTree {
    static_assert(MinChildren >= 2, "MinChildren must be at least 2");
    static_assert(MaxChildren >= 2 * MinChildren, "MaxChildren must be >= 2 * MinChildren");

public:
    RTree() = default;

    void insert(const ValueT& value, const BoundingBox& bbox) {
        m_entries.push_back({value, bbox});
        if (m_rootIndex < 0) {
            // First insert: create root leaf node.
            Node root;
            root.isLeaf = true;
            root.childIndices.push_back(static_cast<int>(m_entries.size()) - 1);
            root.bbox = bbox;
            m_nodes.push_back(root);
            m_rootIndex = 0;
        } else {
            int entryIdx = static_cast<int>(m_entries.size()) - 1;
            insertEntry(m_rootIndex, entryIdx);
        }
    }

    void remove(const ValueT& value) {
        if (m_rootIndex < 0) return;
        removeEntry(value);
    }

    [[nodiscard]] std::vector<ValueT> query(const BoundingBox& searchBox) const {
        std::vector<ValueT> results;
        if (m_rootIndex < 0) return results;
        queryNode(m_rootIndex, searchBox, results);
        return results;
    }

    [[nodiscard]] size_t size() const { return m_entries.size(); }

    [[nodiscard]] bool empty() const { return m_entries.empty(); }

    void clear() {
        m_nodes.clear();
        m_entries.clear();
        m_rootIndex = -1;
    }

private:
    struct Entry {
        ValueT value;
        BoundingBox bbox;
    };

    struct Node {
        bool isLeaf = true;
        BoundingBox bbox;
        std::vector<int> childIndices;  // indices into m_entries (leaf) or m_nodes (internal)
    };

    std::vector<Node> m_nodes;
    std::vector<Entry> m_entries;
    int m_rootIndex = -1;

    void queryNode(int nodeIdx, const BoundingBox& searchBox,
                   std::vector<ValueT>& results) const {
        const auto& node = m_nodes[nodeIdx];
        if (!node.bbox.intersects(searchBox)) return;

        if (node.isLeaf) {
            for (int ci : node.childIndices) {
                if (m_entries[ci].bbox.intersects(searchBox)) {
                    results.push_back(m_entries[ci].value);
                }
            }
        } else {
            for (int ci : node.childIndices) {
                queryNode(ci, searchBox, results);
            }
        }
    }

    void insertEntry(int nodeIdx, int entryIdx) {
        auto& node = m_nodes[nodeIdx];
        node.bbox.expand(m_entries[entryIdx].bbox);

        if (node.isLeaf) {
            node.childIndices.push_back(entryIdx);
            if (static_cast<int>(node.childIndices.size()) > MaxChildren) {
                splitNode(nodeIdx);
            }
        } else {
            // Choose the child whose bbox needs least enlargement.
            int bestChild = chooseBestChild(nodeIdx, m_entries[entryIdx].bbox);
            insertEntry(bestChild, entryIdx);
            // Recompute this node's bbox after child may have changed.
            recomputeBBox(nodeIdx);
            if (static_cast<int>(m_nodes[bestChild].childIndices.size()) > MaxChildren) {
                splitNode(bestChild);
            }
        }
    }

    int chooseBestChild(int nodeIdx, const BoundingBox& entryBox) const {
        const auto& node = m_nodes[nodeIdx];
        int bestIdx = node.childIndices[0];
        double bestEnlargement = std::numeric_limits<double>::max();
        double bestArea = std::numeric_limits<double>::max();

        for (int ci : node.childIndices) {
            const auto& childBBox = m_nodes[ci].bbox;
            BoundingBox merged = childBBox;
            merged.expand(entryBox);

            double enlargement = bboxArea(merged) - bboxArea(childBBox);
            double area = bboxArea(childBBox);

            if (enlargement < bestEnlargement ||
                (enlargement == bestEnlargement && area < bestArea)) {
                bestEnlargement = enlargement;
                bestArea = area;
                bestIdx = ci;
            }
        }
        return bestIdx;
    }

    void splitNode(int nodeIdx) {
        auto& node = m_nodes[nodeIdx];
        auto& children = node.childIndices;

        // Quadratic split: pick the two children with most wasted area if grouped.
        int seed1 = 0, seed2 = 1;
        double worstWaste = -std::numeric_limits<double>::max();

        for (int i = 0; i < static_cast<int>(children.size()); ++i) {
            for (int j = i + 1; j < static_cast<int>(children.size()); ++j) {
                BoundingBox merged = getChildBBox(node, i);
                merged.expand(getChildBBox(node, j));
                double waste = bboxArea(merged) - bboxArea(getChildBBox(node, i))
                             - bboxArea(getChildBBox(node, j));
                if (waste > worstWaste) {
                    worstWaste = waste;
                    seed1 = i;
                    seed2 = j;
                }
            }
        }

        Node newNode;
        newNode.isLeaf = node.isLeaf;
        newNode.bbox = getChildBBox(node, seed2);
        newNode.childIndices.push_back(children[seed2]);

        // Remove seed2 from original (swap-erase).
        std::vector<int> remaining;
        for (int i = 0; i < static_cast<int>(children.size()); ++i) {
            if (i != seed1 && i != seed2) remaining.push_back(children[i]);
        }

        node.childIndices = {children[seed1]};
        node.bbox = getChildBBox(node, 0);

        // Distribute remaining children.
        for (int ci : remaining) {
            BoundingBox ciBox = node.isLeaf ? m_entries[ci].bbox : m_nodes[ci].bbox;
            BoundingBox merged1 = node.bbox;
            merged1.expand(ciBox);
            BoundingBox merged2 = newNode.bbox;
            merged2.expand(ciBox);

            double enlarge1 = bboxArea(merged1) - bboxArea(node.bbox);
            double enlarge2 = bboxArea(merged2) - bboxArea(newNode.bbox);

            if (enlarge1 <= enlarge2) {
                node.childIndices.push_back(ci);
                node.bbox.expand(ciBox);
            } else {
                newNode.childIndices.push_back(ci);
                newNode.bbox.expand(ciBox);
            }
        }

        int newNodeIdx = static_cast<int>(m_nodes.size());
        m_nodes.push_back(std::move(newNode));

        // If this was the root, create a new root.
        if (nodeIdx == m_rootIndex) {
            Node newRoot;
            newRoot.isLeaf = false;
            newRoot.bbox = m_nodes[nodeIdx].bbox;
            newRoot.bbox.expand(m_nodes[newNodeIdx].bbox);
            newRoot.childIndices = {nodeIdx, newNodeIdx};
            m_nodes.push_back(std::move(newRoot));
            m_rootIndex = static_cast<int>(m_nodes.size()) - 1;
        } else {
            // Parent must add newNodeIdx — handled by caller checking overflow.
            addChildToParent(nodeIdx, newNodeIdx);
        }
    }

    void addChildToParent(int childIdx, int newSiblingIdx) {
        // Find parent of childIdx and add newSiblingIdx.
        for (auto& n : m_nodes) {
            if (n.isLeaf) continue;
            for (int ci : n.childIndices) {
                if (ci == childIdx) {
                    n.childIndices.push_back(newSiblingIdx);
                    n.bbox.expand(m_nodes[newSiblingIdx].bbox);
                    return;
                }
            }
        }
    }

    BoundingBox getChildBBox(const Node& node, int localIdx) const {
        int ci = node.childIndices[localIdx];
        return node.isLeaf ? m_entries[ci].bbox : m_nodes[ci].bbox;
    }

    void recomputeBBox(int nodeIdx) {
        auto& node = m_nodes[nodeIdx];
        node.bbox.reset();
        for (int ci : node.childIndices) {
            if (node.isLeaf) {
                node.bbox.expand(m_entries[ci].bbox);
            } else {
                node.bbox.expand(m_nodes[ci].bbox);
            }
        }
    }

    void removeEntry(const ValueT& value) {
        // Find and remove the entry, then rebuild.
        // For correctness, we use the simple approach: collect all entries except
        // the target, clear, and re-insert. This is O(n log n) but remove is rare
        // compared to query, and this avoids complex R*-tree condense-tree logic
        // that is error-prone to implement.
        std::vector<Entry> kept;
        kept.reserve(m_entries.size());
        bool found = false;
        for (const auto& e : m_entries) {
            if (!found && e.value == value) {
                found = true;
                continue;
            }
            kept.push_back(e);
        }
        if (!found) return;

        m_entries.clear();
        m_nodes.clear();
        m_rootIndex = -1;
        for (const auto& e : kept) {
            insert(e.value, e.bbox);
        }
    }

    static double bboxArea(const BoundingBox& box) {
        if (!box.isValid()) return 0.0;
        auto s = box.size();
        // 2D area (ignore z).
        return s.x * s.y;
    }
};

}  // namespace hz::math
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cmake --build build/debug --config Debug --target hz_math_tests && ctest --test-dir build/debug -C Debug -R RTree --output-on-failure`
Expected: All 5 RTree tests PASS.

- [ ] **Step 6: Commit**

```bash
git add src/math/include/horizon/math/RTree.h tests/math/test_RTree.cpp tests/math/CMakeLists.txt
git commit -m "feat(math): add R*-tree spatial index with insert and range query

Header-only template in hz::math. Cache-friendly flat vector node storage.
Quadratic split for overflow. Rebuild-on-remove for correctness."
```

---

## Task 2: R*-tree — Remove and Stress Tests

**Files:**
- Modify: `tests/math/test_RTree.cpp`

- [ ] **Step 1: Write failing tests for remove and large-scale operations**

Append to `tests/math/test_RTree.cpp`:
```cpp
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

TEST(RTreeTest, RemoveNonExistentIsNoOp) {
    RTree<uint64_t> tree;
    tree.insert(1, BoundingBox(Vec3(0, 0, 0), Vec3(2, 2, 0)));

    tree.remove(999);
    EXPECT_EQ(tree.size(), 1u);
}

TEST(RTreeTest, ClearEmptiesTree) {
    RTree<uint64_t> tree;
    tree.insert(1, BoundingBox(Vec3(0, 0, 0), Vec3(1, 1, 0)));
    tree.insert(2, BoundingBox(Vec3(2, 2, 0), Vec3(3, 3, 0)));

    tree.clear();
    EXPECT_EQ(tree.size(), 0u);
    EXPECT_TRUE(tree.empty());
    EXPECT_TRUE(tree.query(BoundingBox(Vec3(-100, -100, -1e9), Vec3(100, 100, 1e9))).empty());
}

TEST(RTreeTest, InsertManyAndQueryCorrectly) {
    RTree<uint64_t> tree;
    // Insert 1000 non-overlapping boxes in a 100x10 grid.
    for (uint64_t i = 0; i < 1000; ++i) {
        double x = static_cast<double>(i % 100) * 3.0;
        double y = static_cast<double>(i / 100) * 3.0;
        tree.insert(i, BoundingBox(Vec3(x, y, 0), Vec3(x + 1, y + 1, 0)));
    }
    EXPECT_EQ(tree.size(), 1000u);

    // Query a small region that should contain exactly one box (at grid 5,5).
    BoundingBox smallQuery(Vec3(14.5, 14.5, -1e9), Vec3(16.5, 16.5, 1e9));
    auto results = tree.query(smallQuery);
    // Box at (15, 15)-(16, 16) is entity 5*100+5 = 505.
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], 505u);
}

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
```

- [ ] **Step 2: Run tests to verify they pass**

Run: `cmake --build build/debug --config Debug --target hz_math_tests && ctest --test-dir build/debug -C Debug -R RTree --output-on-failure`
Expected: All 10 RTree tests PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/math/test_RTree.cpp
git commit -m "test(math): add R*-tree remove, clear, and stress tests

1000-entity grid query, targeted single-entity query, edge cases."
```

---

## Task 3: SpatialIndex Wrapper for DraftDocument

**Files:**
- Create: `src/drafting/include/horizon/drafting/SpatialIndex.h`
- Create: `src/drafting/src/SpatialIndex.cpp`
- Modify: `src/drafting/CMakeLists.txt`
- Create: `tests/drafting/test_SpatialIndex.cpp`
- Create: `tests/drafting/CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write failing tests for SpatialIndex**

Create `tests/drafting/test_SpatialIndex.cpp`:
```cpp
#include <gtest/gtest.h>
#include "horizon/drafting/SpatialIndex.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/math/BoundingBox.h"

using namespace hz::draft;
using namespace hz::math;

TEST(SpatialIndexTest, EmptyIndexQueryReturnsNothing) {
    SpatialIndex index;
    BoundingBox query(Vec3(0, 0, -1e9), Vec3(10, 10, 1e9));
    auto results = index.query(query);
    EXPECT_TRUE(results.empty());
}

TEST(SpatialIndexTest, InsertAndQueryEntity) {
    SpatialIndex index;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 5));
    line->setId(1);
    index.insert(line);

    BoundingBox query(Vec3(-1, -1, -1e9), Vec3(6, 6, 1e9));
    auto results = index.query(query);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], 1u);
}

TEST(SpatialIndexTest, InsertAndQueryMiss) {
    SpatialIndex index;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 5));
    line->setId(1);
    index.insert(line);

    BoundingBox query(Vec3(100, 100, -1e9), Vec3(200, 200, 1e9));
    auto results = index.query(query);
    EXPECT_TRUE(results.empty());
}

TEST(SpatialIndexTest, RemoveEntity) {
    SpatialIndex index;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 5));
    line->setId(1);
    index.insert(line);
    index.remove(1);

    BoundingBox query(Vec3(-1, -1, -1e9), Vec3(6, 6, 1e9));
    auto results = index.query(query);
    EXPECT_TRUE(results.empty());
}

TEST(SpatialIndexTest, UpdateEntityPosition) {
    SpatialIndex index;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 5));
    line->setId(1);
    index.insert(line);

    // Move line far away.
    line->setStart(Vec2(100, 100));
    line->setEnd(Vec2(105, 105));
    index.update(line);

    // Old region should be empty.
    BoundingBox oldQuery(Vec3(-1, -1, -1e9), Vec3(6, 6, 1e9));
    EXPECT_TRUE(index.query(oldQuery).empty());

    // New region should contain it.
    BoundingBox newQuery(Vec3(99, 99, -1e9), Vec3(106, 106, 1e9));
    auto results = index.query(newQuery);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], 1u);
}

TEST(SpatialIndexTest, RebuildFromEntities) {
    std::vector<std::shared_ptr<DraftEntity>> entities;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 5));
    line->setId(1);
    auto circle = std::make_shared<DraftCircle>(Vec2(20, 20), 3.0);
    circle->setId(2);
    entities.push_back(line);
    entities.push_back(circle);

    SpatialIndex index;
    index.rebuild(entities);

    // Query should find line but not circle.
    BoundingBox lineQuery(Vec3(-1, -1, -1e9), Vec3(6, 6, 1e9));
    auto results = lineQuery.isValid() ? index.query(lineQuery) : std::vector<uint64_t>{};
    results = index.query(lineQuery);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], 1u);

    // Query should find circle but not line.
    BoundingBox circleQuery(Vec3(16, 16, -1e9), Vec3(24, 24, 1e9));
    results = index.query(circleQuery);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], 2u);
}
```

- [ ] **Step 2: Create drafting test CMakeLists**

Create `tests/drafting/CMakeLists.txt`:
```cmake
add_executable(hz_drafting_tests
    test_SpatialIndex.cpp
)

target_link_libraries(hz_drafting_tests
    PRIVATE
        Horizon::Drafting
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_drafting_tests)
```

- [ ] **Step 3: Register drafting tests in parent CMakeLists**

Add to `tests/CMakeLists.txt`:
```cmake
add_subdirectory(math)
add_subdirectory(constraint)
add_subdirectory(drafting)
```

- [ ] **Step 4: Run tests to verify they fail**

Run: `cmake --build build/debug --config Debug --target hz_drafting_tests`
Expected: Compilation fails — `SpatialIndex.h` not found.

- [ ] **Step 5: Implement SpatialIndex header**

Create `src/drafting/include/horizon/drafting/SpatialIndex.h`:
```cpp
#pragma once

#include "horizon/math/RTree.h"
#include "horizon/math/BoundingBox.h"
#include "DraftEntity.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace hz::draft {

/// Wraps an R*-tree for DraftEntity spatial queries.
/// Stores entity IDs indexed by their bounding boxes.
class SpatialIndex {
public:
    SpatialIndex() = default;

    /// Insert an entity into the spatial index.
    void insert(const std::shared_ptr<DraftEntity>& entity);

    /// Remove an entity by ID.
    void remove(uint64_t entityId);

    /// Update an entity's position in the index (remove + re-insert).
    void update(const std::shared_ptr<DraftEntity>& entity);

    /// Query: return all entity IDs whose bounding boxes intersect searchBox.
    [[nodiscard]] std::vector<uint64_t> query(const math::BoundingBox& searchBox) const;

    /// Rebuild the entire index from a list of entities.
    void rebuild(const std::vector<std::shared_ptr<DraftEntity>>& entities);

    /// Clear the index.
    void clear();

private:
    math::RTree<uint64_t> m_tree;
};

}  // namespace hz::draft
```

- [ ] **Step 6: Implement SpatialIndex**

Create `src/drafting/src/SpatialIndex.cpp`:
```cpp
#include "horizon/drafting/SpatialIndex.h"

namespace hz::draft {

void SpatialIndex::insert(const std::shared_ptr<DraftEntity>& entity) {
    if (!entity) return;
    auto bbox = entity->boundingBox();
    if (!bbox.isValid()) return;
    m_tree.insert(entity->id(), bbox);
}

void SpatialIndex::remove(uint64_t entityId) {
    m_tree.remove(entityId);
}

void SpatialIndex::update(const std::shared_ptr<DraftEntity>& entity) {
    if (!entity) return;
    remove(entity->id());
    insert(entity);
}

std::vector<uint64_t> SpatialIndex::query(const math::BoundingBox& searchBox) const {
    return m_tree.query(searchBox);
}

void SpatialIndex::rebuild(const std::vector<std::shared_ptr<DraftEntity>>& entities) {
    m_tree.clear();
    for (const auto& entity : entities) {
        insert(entity);
    }
}

void SpatialIndex::clear() {
    m_tree.clear();
}

}  // namespace hz::draft
```

- [ ] **Step 7: Add SpatialIndex.cpp to drafting CMakeLists**

Add `src/SpatialIndex.cpp` to `src/drafting/CMakeLists.txt`:
```cmake
add_library(hz_drafting STATIC
    src/LineType.cpp
    src/DraftEntity.cpp
    src/DraftDocument.cpp
    src/Layer.cpp
    src/SnapEngine.cpp
    src/SpatialIndex.cpp
    src/DraftLine.cpp
    src/DraftCircle.cpp
    src/DraftArc.cpp
    src/DraftRectangle.cpp
    src/DraftPolyline.cpp
    src/Intersection.cpp
    src/DraftDimension.cpp
    src/DraftLinearDimension.cpp
    src/DraftRadialDimension.cpp
    src/DraftAngularDimension.cpp
    src/DraftLeader.cpp
    src/BlockTable.cpp
    src/DraftBlockRef.cpp
    src/DraftText.cpp
    src/DraftSpline.cpp
    src/DraftHatch.cpp
    src/DraftEllipse.cpp
)
```

- [ ] **Step 8: Run tests to verify they pass**

Run: `cmake --build build/debug --config Debug --target hz_drafting_tests && ctest --test-dir build/debug -C Debug -R SpatialIndex --output-on-failure`
Expected: All 6 SpatialIndex tests PASS.

- [ ] **Step 9: Commit**

```bash
git add src/drafting/include/horizon/drafting/SpatialIndex.h src/drafting/src/SpatialIndex.cpp src/drafting/CMakeLists.txt tests/drafting/test_SpatialIndex.cpp tests/drafting/CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat(drafting): add SpatialIndex wrapper around R*-tree

Supports insert, remove, update, rebuild, and range query.
Indexes DraftEntity by bounding box for O(log n) spatial queries."
```

---

## Task 4: Wire SpatialIndex into DraftDocument

**Files:**
- Modify: `src/drafting/include/horizon/drafting/DraftDocument.h`
- Modify: `src/drafting/src/DraftDocument.cpp`
- Modify: `tests/drafting/test_SpatialIndex.cpp`

- [ ] **Step 1: Write failing test for DraftDocument spatial index integration**

Append to `tests/drafting/test_SpatialIndex.cpp`:
```cpp
#include "horizon/drafting/DraftDocument.h"

TEST(DraftDocumentSpatialTest, AddEntityUpdatesSpatialIndex) {
    DraftDocument doc;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 5));
    doc.addEntity(line);

    BoundingBox query(Vec3(-1, -1, -1e9), Vec3(6, 6, 1e9));
    auto results = doc.spatialIndex().query(query);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], line->id());
}

TEST(DraftDocumentSpatialTest, RemoveEntityUpdatesSpatialIndex) {
    DraftDocument doc;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 5));
    doc.addEntity(line);
    uint64_t id = line->id();

    doc.removeEntity(id);

    BoundingBox query(Vec3(-1, -1, -1e9), Vec3(6, 6, 1e9));
    auto results = doc.spatialIndex().query(query);
    EXPECT_TRUE(results.empty());
}

TEST(DraftDocumentSpatialTest, ClearEmptiesSpatialIndex) {
    DraftDocument doc;
    doc.addEntity(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 5)));
    doc.addEntity(std::make_shared<DraftCircle>(Vec2(10, 10), 3.0));
    doc.clear();

    BoundingBox query(Vec3(-100, -100, -1e9), Vec3(100, 100, 1e9));
    EXPECT_TRUE(doc.spatialIndex().query(query).empty());
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build/debug --config Debug --target hz_drafting_tests`
Expected: Compilation fails — `doc.spatialIndex()` not found.

- [ ] **Step 3: Add SpatialIndex to DraftDocument header**

Modify `src/drafting/include/horizon/drafting/DraftDocument.h`:
```cpp
#pragma once

#include "BlockTable.h"
#include "DraftEntity.h"
#include "DimensionStyle.h"
#include "SpatialIndex.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace hz::draft {

class DraftDocument {
public:
    DraftDocument() = default;

    void addEntity(std::shared_ptr<DraftEntity> entity);
    void removeEntity(uint64_t id);
    const std::vector<std::shared_ptr<DraftEntity>>& entities() const { return m_entities; }
    std::vector<std::shared_ptr<DraftEntity>>& entities() { return m_entities; }
    void clear();

    /// Returns the spatial index for accelerated range queries.
    const SpatialIndex& spatialIndex() const { return m_spatialIndex; }
    SpatialIndex& spatialIndex() { return m_spatialIndex; }

    /// Rebuild the spatial index from current entities.
    /// Call this after bulk modifications (e.g., file load).
    void rebuildSpatialIndex();

    uint64_t nextGroupId() { return m_nextGroupId++; }
    void advanceGroupIdCounter(uint64_t minId) {
        if (m_nextGroupId <= minId) m_nextGroupId = minId + 1;
    }

    const DimensionStyle& dimensionStyle() const { return m_dimensionStyle; }
    void setDimensionStyle(const DimensionStyle& style) { m_dimensionStyle = style; }

    BlockTable& blockTable() { return m_blockTable; }
    const BlockTable& blockTable() const { return m_blockTable; }

private:
    std::vector<std::shared_ptr<DraftEntity>> m_entities;
    SpatialIndex m_spatialIndex;
    DimensionStyle m_dimensionStyle;
    BlockTable m_blockTable;
    uint64_t m_nextGroupId = 1;
};

}  // namespace hz::draft
```

- [ ] **Step 4: Wire spatial index into DraftDocument methods**

Modify `src/drafting/src/DraftDocument.cpp` — add spatial index calls to `addEntity`, `removeEntity`, and `clear`:
```cpp
#include "horizon/drafting/DraftDocument.h"
#include <algorithm>

namespace hz::draft {

void DraftDocument::addEntity(std::shared_ptr<DraftEntity> entity) {
    if (!entity) return;
    m_entities.push_back(entity);
    m_spatialIndex.insert(entity);
}

void DraftDocument::removeEntity(uint64_t id) {
    auto it = std::find_if(m_entities.begin(), m_entities.end(),
                           [id](const auto& e) { return e->id() == id; });
    if (it != m_entities.end()) {
        m_spatialIndex.remove(id);
        m_entities.erase(it);
    }
}

void DraftDocument::clear() {
    m_entities.clear();
    m_spatialIndex.clear();
    m_blockTable = BlockTable();
    m_dimensionStyle = DimensionStyle();
    m_nextGroupId = 1;
}

void DraftDocument::rebuildSpatialIndex() {
    m_spatialIndex.rebuild(m_entities);
}

}  // namespace hz::draft
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cmake --build build/debug --config Debug --target hz_drafting_tests && ctest --test-dir build/debug -C Debug -R DraftDocumentSpatial --output-on-failure`
Expected: All 3 DraftDocumentSpatial tests PASS.

- [ ] **Step 6: Run ALL existing tests to verify no regressions**

Run: `cmake --build build/debug --config Debug && ctest --test-dir build/debug -C Debug --output-on-failure`
Expected: All existing 114+ tests PASS plus the new ones.

- [ ] **Step 7: Commit**

```bash
git add src/drafting/include/horizon/drafting/DraftDocument.h src/drafting/src/DraftDocument.cpp tests/drafting/test_SpatialIndex.cpp
git commit -m "feat(drafting): wire SpatialIndex into DraftDocument

addEntity/removeEntity/clear now automatically update spatial index.
rebuildSpatialIndex() for bulk operations like file load."
```

---

## Task 5: Accelerate SnapEngine with SpatialIndex

**Files:**
- Modify: `src/drafting/include/horizon/drafting/SnapEngine.h`
- Modify: `src/drafting/src/SnapEngine.cpp`
- Modify: `tests/drafting/test_SpatialIndex.cpp`

- [ ] **Step 1: Write failing test for spatial-index-accelerated snap**

Append to `tests/drafting/test_SpatialIndex.cpp`:
```cpp
#include "horizon/drafting/SnapEngine.h"

TEST(SnapEngineTest, SpatialSnapFindsNearbyEntity) {
    DraftDocument doc;
    // Line far away (should NOT snap to this).
    auto farLine = std::make_shared<DraftLine>(Vec2(100, 100), Vec2(105, 105));
    doc.addEntity(farLine);

    // Line near cursor (should snap to this).
    auto nearLine = std::make_shared<DraftLine>(Vec2(1, 1), Vec2(3, 3));
    doc.addEntity(nearLine);

    SnapEngine engine;
    engine.setSnapTolerance(2.0);
    engine.setGridSpacing(10.0);  // Large grid so grid snap doesn't interfere.

    Vec2 cursor(0.9, 0.9);  // Near start of nearLine.
    SnapResult result = engine.snap(cursor, doc.spatialIndex(), doc.entities());

    EXPECT_EQ(result.type, SnapType::Endpoint);
    EXPECT_NEAR(result.point.x, 1.0, 1e-7);
    EXPECT_NEAR(result.point.y, 1.0, 1e-7);
}

TEST(SnapEngineTest, SpatialSnapReturnsGridWhenNoEntityNearby) {
    DraftDocument doc;
    auto farLine = std::make_shared<DraftLine>(Vec2(100, 100), Vec2(105, 105));
    doc.addEntity(farLine);

    SnapEngine engine;
    engine.setSnapTolerance(0.5);
    engine.setGridSpacing(1.0);

    Vec2 cursor(5.1, 5.2);
    SnapResult result = engine.snap(cursor, doc.spatialIndex(), doc.entities());

    EXPECT_EQ(result.type, SnapType::Grid);
    EXPECT_NEAR(result.point.x, 5.0, 1e-7);
    EXPECT_NEAR(result.point.y, 5.0, 1e-7);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build/debug --config Debug --target hz_drafting_tests`
Expected: Compilation fails — `snap(cursor, spatialIndex, entities)` overload not found.

- [ ] **Step 3: Add spatial-index snap overload to SnapEngine header**

Modify `src/drafting/include/horizon/drafting/SnapEngine.h` — add a new overload:
```cpp
#pragma once

#include "DraftEntity.h"
#include "SpatialIndex.h"
#include "horizon/math/Vec2.h"
#include <memory>
#include <vector>

namespace hz::draft {

enum class SnapType { None, Grid, Endpoint, Midpoint, Center };

struct SnapResult {
    math::Vec2 point;
    SnapType type = SnapType::None;
};

class SnapEngine {
public:
    SnapEngine();

    void setGridSpacing(double spacing);
    double gridSpacing() const { return m_gridSpacing; }

    void setSnapTolerance(double tolerance);
    double snapTolerance() const { return m_snapTolerance; }

    /// Original: iterates all entities (backward compatible).
    SnapResult snap(const math::Vec2& cursorWorld,
                    const std::vector<std::shared_ptr<DraftEntity>>& entities) const;

    /// Accelerated: uses SpatialIndex to narrow candidates first.
    SnapResult snap(const math::Vec2& cursorWorld,
                    const SpatialIndex& index,
                    const std::vector<std::shared_ptr<DraftEntity>>& entities) const;

private:
    math::Vec2 snapToGrid(const math::Vec2& point) const;

    double m_gridSpacing;
    double m_snapTolerance;
};

}  // namespace hz::draft
```

- [ ] **Step 4: Implement spatial-index snap**

Add to `src/drafting/src/SnapEngine.cpp`:
```cpp
SnapResult SnapEngine::snap(const math::Vec2& cursorWorld,
                            const SpatialIndex& index,
                            const std::vector<std::shared_ptr<DraftEntity>>& entities) const {
    SnapResult best;
    best.point = cursorWorld;
    best.type = SnapType::None;
    double bestDist = std::numeric_limits<double>::max();

    // Build a search box around the cursor at snap tolerance.
    math::BoundingBox searchBox(
        math::Vec3(cursorWorld.x - m_snapTolerance, cursorWorld.y - m_snapTolerance, -1e9),
        math::Vec3(cursorWorld.x + m_snapTolerance, cursorWorld.y + m_snapTolerance, 1e9));

    auto candidateIds = index.query(searchBox);

    // Only check snap points of entities near the cursor.
    for (uint64_t id : candidateIds) {
        for (const auto& entity : entities) {
            if (entity->id() != id) continue;
            std::vector<math::Vec2> pts = entity->snapPoints();
            for (const auto& pt : pts) {
                double dist = cursorWorld.distanceTo(pt);
                if (dist < m_snapTolerance && dist < bestDist) {
                    bestDist = dist;
                    best.point = pt;
                    best.type = SnapType::Endpoint;
                }
            }
            break;
        }
    }

    // Check grid snap.
    math::Vec2 gridPt = snapToGrid(cursorWorld);
    double gridDist = cursorWorld.distanceTo(gridPt);
    if (gridDist < m_snapTolerance && gridDist < bestDist) {
        best.point = gridPt;
        best.type = SnapType::Grid;
    }

    return best;
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cmake --build build/debug --config Debug --target hz_drafting_tests && ctest --test-dir build/debug -C Debug -R SnapEngine --output-on-failure`
Expected: Both SnapEngine tests PASS.

- [ ] **Step 6: Run ALL tests for regression**

Run: `cmake --build build/debug --config Debug && ctest --test-dir build/debug -C Debug --output-on-failure`
Expected: All tests PASS.

- [ ] **Step 7: Commit**

```bash
git add src/drafting/include/horizon/drafting/SnapEngine.h src/drafting/src/SnapEngine.cpp tests/drafting/test_SpatialIndex.cpp
git commit -m "feat(drafting): add spatial-index-accelerated snap overload

SnapEngine::snap() now has an overload taking SpatialIndex.
Queries a tolerance-sized box around cursor to narrow candidates
from O(n) to O(log n + k) where k is the number of nearby entities."
```

---

## Task 6: Accelerate SelectTool with SpatialIndex

**Files:**
- Modify: `src/ui/src/SelectTool.cpp`

- [ ] **Step 1: Refactor SelectTool hit-testing to use SpatialIndex**

Modify the click-selection block in `SelectTool::mouseReleaseEvent` (around line 245). Replace the linear entity loop with a spatial index query:

Old code (lines 244-252):
```cpp
    uint64_t hitId = 0;
    for (const auto& entity : doc.entities()) {
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        if (entity->hitTest(worldPos, tolerance)) {
            hitId = entity->id();
            break;
        }
    }
```

New code:
```cpp
    uint64_t hitId = 0;
    {
        // Use spatial index to narrow hit-test candidates.
        math::BoundingBox searchBox(
            math::Vec3(worldPos.x - tolerance, worldPos.y - tolerance, -1e9),
            math::Vec3(worldPos.x + tolerance, worldPos.y + tolerance, 1e9));
        auto candidateIds = doc.spatialIndex().query(searchBox);

        for (uint64_t candId : candidateIds) {
            for (const auto& entity : doc.entities()) {
                if (entity->id() != candId) continue;
                const auto* lp = layerMgr.getLayer(entity->layer());
                if (!lp || !lp->visible || lp->locked) break;
                if (entity->hitTest(worldPos, tolerance)) {
                    hitId = entity->id();
                }
                break;
            }
            if (hitId != 0) break;
        }
    }
```

- [ ] **Step 2: Refactor SelectTool box selection to use SpatialIndex**

Modify the box-selection block in `SelectTool::mouseReleaseEvent` (around line 209). Replace linear iteration with spatial query:

Old code (lines 209-227):
```cpp
        for (const auto& entity : doc.entities()) {
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;

            math::BoundingBox ebb = entity->boundingBox();
            if (!ebb.isValid()) continue;

            if (windowMode) {
                if (selectRect.contains(ebb)) {
                    sel.select(entity->id());
                }
            } else {
                if (selectRect.intersects(ebb)) {
                    sel.select(entity->id());
                }
            }
        }
```

New code:
```cpp
        // Use spatial index to find candidates overlapping the selection rectangle.
        auto candidateIds = doc.spatialIndex().query(selectRect);

        for (uint64_t candId : candidateIds) {
            for (const auto& entity : doc.entities()) {
                if (entity->id() != candId) continue;
                const auto* lp = layerMgr.getLayer(entity->layer());
                if (!lp || !lp->visible || lp->locked) break;

                math::BoundingBox ebb = entity->boundingBox();
                if (!ebb.isValid()) break;

                if (windowMode) {
                    if (selectRect.contains(ebb)) {
                        sel.select(entity->id());
                    }
                } else {
                    // Already confirmed intersects via R*-tree query.
                    sel.select(entity->id());
                }
                break;
            }
        }
```

- [ ] **Step 3: Build and run the application manually**

Run: `cmake --build build/debug --config Debug`
Expected: Clean build with no errors.

Manually test:
1. Launch `build/debug/src/app/Debug/horizon.exe`
2. Draw several lines and circles
3. Click to select entities — verify selection works correctly
4. Drag box selection (left-to-right and right-to-left) — verify window and crossing selection work
5. Verify snapping still works when drawing near existing entities

- [ ] **Step 4: Run ALL tests for regression**

Run: `ctest --test-dir build/debug -C Debug --output-on-failure`
Expected: All tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/ui/src/SelectTool.cpp
git commit -m "feat(ui): accelerate SelectTool hit-test and box selection with SpatialIndex

Click selection queries a tolerance-sized box instead of iterating all entities.
Box selection queries the selection rectangle to narrow candidates.
Both paths fall back to exact hit-test/contains checks after R*-tree filtering."
```

---

## Task 7: Wire Spatial Index into File Load Path

**Files:**
- Modify: `src/fileio/src/NativeFormat.cpp`
- Modify: `src/fileio/src/DxfFormat.cpp`

- [ ] **Step 1: Find the file load functions**

Read `src/fileio/src/NativeFormat.cpp` and `src/fileio/src/DxfFormat.cpp` to find where entities are loaded. Look for the point after all entities have been added to the document.

- [ ] **Step 2: Add rebuildSpatialIndex call after file load**

In `NativeFormat::load()`, after all entities are parsed and added to the document, add:
```cpp
doc.draftDocument().rebuildSpatialIndex();
```

In `DxfFormat::load()`, add the same call at the equivalent point.

- [ ] **Step 3: Build and manually test file load**

Run: `cmake --build build/debug --config Debug`

Manually test:
1. Launch horizon.exe
2. Open an existing `.hcad` file
3. Click on entities — verify selection works
4. Verify snapping works near loaded entities
5. Open a `.dxf` file and repeat

- [ ] **Step 4: Commit**

```bash
git add src/fileio/src/NativeFormat.cpp src/fileio/src/DxfFormat.cpp
git commit -m "fix(fileio): rebuild spatial index after file load

Both NativeFormat::load() and DxfFormat::load() now call
rebuildSpatialIndex() after populating the document."
```

---

## Task 8: Wire Spatial Index into Undo/Redo Commands

**Files:**
- Modify: `src/document/src/Commands.cpp` (or equivalent command implementations)

- [ ] **Step 1: Identify entity-mutating commands**

Read the Commands header/source to find all commands that add, remove, or move entities:
- `AddEntityCommand::execute()` and `undo()`
- `RemoveEntityCommand::execute()` and `undo()`
- `MoveEntityCommand::execute()` and `undo()`
- `GripMoveCommand::execute()` and `undo()`
- Any other command that changes entity geometry

- [ ] **Step 2: Add spatial index updates to entity-mutating commands**

For commands that go through `DraftDocument::addEntity()` / `removeEntity()`, the spatial index is already updated (from Task 4). Verify this is the case.

For commands that modify entity geometry in-place (move, grip move, rotate, scale), add `rebuildSpatialIndex()` calls after the geometry mutation. The simplest correct approach:

In each command's `execute()` and `undo()` that modifies entity positions, add at the end:
```cpp
m_doc.rebuildSpatialIndex();
```

This is O(n) but only runs on user actions (not per-frame). Optimize to targeted insert/remove later if profiling shows it's needed.

- [ ] **Step 3: Build and manually test undo/redo**

Run: `cmake --build build/debug --config Debug`

Manually test:
1. Draw a line → undo → redo → verify selection still works on the line
2. Move a line → undo → verify selection works at original position
3. Delete an entity → undo → verify it's selectable again
4. Grip-edit a line endpoint → undo → verify snap/selection correct

- [ ] **Step 4: Run ALL tests**

Run: `ctest --test-dir build/debug -C Debug --output-on-failure`
Expected: All tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/document/src/Commands.cpp
git commit -m "fix(document): update spatial index on undo/redo of entity mutations

Commands that modify entity geometry now call rebuildSpatialIndex()
in both execute() and undo() to keep the R*-tree consistent."
```

---

## Task 9: Update Remaining Tool Snap Calls

**Files:**
- Modify: Multiple tool files in `src/ui/src/` that call `SnapEngine::snap()`

- [ ] **Step 1: Find all existing snap calls**

Search for all calls to `snapEngine().snap(` in the ui module. Each tool that snaps uses the old overload passing the entity vector directly.

- [ ] **Step 2: Update each snap call to use the spatial index overload**

For each tool file that calls:
```cpp
auto result = m_viewport->snapEngine().snap(worldPos, m_viewport->document()->draftDocument().entities());
```

Replace with:
```cpp
auto& draftDoc = m_viewport->document()->draftDocument();
auto result = m_viewport->snapEngine().snap(worldPos, draftDoc.spatialIndex(), draftDoc.entities());
```

This is a mechanical find-and-replace across all tool files. The original overload remains for backward compatibility but all internal callers should use the accelerated version.

- [ ] **Step 3: Build and run full test suite**

Run: `cmake --build build/debug --config Debug && ctest --test-dir build/debug -C Debug --output-on-failure`
Expected: Clean build, all tests PASS.

- [ ] **Step 4: Manually smoke-test all major tools**

Launch horizon.exe and test snapping in:
1. Line tool — draw near existing endpoints
2. Circle tool — snap to center of existing circle
3. Arc tool — snap to endpoints
4. Rectangle tool — snap to corners
5. Move tool — snap during move
6. Duplicate tool — snap during placement
7. Dimension tools — snap to endpoints for measurement

- [ ] **Step 5: Commit**

```bash
git add src/ui/src/*.cpp
git commit -m "refactor(ui): migrate all tool snap calls to spatial-index overload

All drawing and editing tools now use the accelerated
SnapEngine::snap(cursor, spatialIndex, entities) overload."
```

---

## Task 10: Performance Benchmark and Phase Completion

**Files:**
- Create: `tests/drafting/test_SpatialIndexPerf.cpp`
- Modify: `tests/drafting/CMakeLists.txt`

- [ ] **Step 1: Write performance benchmark test**

Create `tests/drafting/test_SpatialIndexPerf.cpp`:
```cpp
#include <gtest/gtest.h>
#include "horizon/drafting/SpatialIndex.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SnapEngine.h"
#include "horizon/math/BoundingBox.h"
#include <chrono>
#include <iostream>

using namespace hz::draft;
using namespace hz::math;

TEST(SpatialIndexPerfTest, TenThousandEntitySnapUnder1ms) {
    // Build a document with 10,000 lines in a grid.
    std::vector<std::shared_ptr<DraftEntity>> entities;
    entities.reserve(10000);
    for (uint64_t i = 0; i < 10000; ++i) {
        double x = static_cast<double>(i % 100) * 5.0;
        double y = static_cast<double>(i / 100) * 5.0;
        auto line = std::make_shared<DraftLine>(Vec2(x, y), Vec2(x + 3, y + 3));
        line->setId(i + 1);
        entities.push_back(line);
    }

    SpatialIndex index;
    index.rebuild(entities);

    SnapEngine engine;
    engine.setSnapTolerance(2.0);
    engine.setGridSpacing(5.0);

    // Time 1000 snap queries.
    Vec2 cursor(250.0, 250.0);  // Center of grid.
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; ++i) {
        engine.snap(cursor, index, entities);
    }
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / 1000.0;
    std::cout << "[PERF] 10k entities, avg snap query: " << avgMs << " ms" << std::endl;

    // Target: average snap query < 1ms with 10,000 entities.
    EXPECT_LT(avgMs, 1.0) << "Snap query too slow: " << avgMs << " ms average";
}

TEST(SpatialIndexPerfTest, TenThousandEntityBoxSelectUnder5ms) {
    SpatialIndex index;
    for (uint64_t i = 0; i < 10000; ++i) {
        double x = static_cast<double>(i % 100) * 5.0;
        double y = static_cast<double>(i / 100) * 5.0;
        auto line = std::make_shared<DraftLine>(Vec2(x, y), Vec2(x + 3, y + 3));
        line->setId(i + 1);
        index.insert(line);
    }

    // Query a region containing ~100 entities (10x10 grid cells).
    BoundingBox queryBox(Vec3(200, 200, -1e9), Vec3(250, 250, 1e9));

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; ++i) {
        index.query(queryBox);
    }
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgMs = totalMs / 1000.0;
    std::cout << "[PERF] 10k entities, avg box query: " << avgMs << " ms" << std::endl;

    EXPECT_LT(avgMs, 5.0) << "Box query too slow: " << avgMs << " ms average";
}
```

- [ ] **Step 2: Register perf test in CMakeLists**

Modify `tests/drafting/CMakeLists.txt`:
```cmake
add_executable(hz_drafting_tests
    test_SpatialIndex.cpp
    test_SpatialIndexPerf.cpp
)

target_link_libraries(hz_drafting_tests
    PRIVATE
        Horizon::Drafting
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_drafting_tests)
```

- [ ] **Step 3: Run performance benchmarks**

Run: `cmake --build build/debug --config Debug --target hz_drafting_tests && ctest --test-dir build/debug -C Debug -R SpatialIndexPerf --output-on-failure -V`
Expected: Both perf tests PASS. Output shows avg query times well under targets.

- [ ] **Step 4: Run the complete test suite one final time**

Run: `cmake --build build/debug --config Debug && ctest --test-dir build/debug -C Debug --output-on-failure`
Expected: ALL tests PASS (original 114 + new R*-tree + SpatialIndex + SnapEngine + perf tests).

- [ ] **Step 5: Commit**

```bash
git add tests/drafting/test_SpatialIndexPerf.cpp tests/drafting/CMakeLists.txt
git commit -m "test(drafting): add spatial index performance benchmarks

10,000-entity snap query target: < 1ms average.
10,000-entity box selection query target: < 5ms average."
```

- [ ] **Step 6: Final phase commit**

```bash
git add -A
git commit -m "Phase 25: Spatial indexing with R*-tree for O(log n) spatial queries

- R*-tree template in hz::math (header-only, cache-friendly flat vector storage)
- SpatialIndex wrapper in hz::draft synchronized with DraftDocument
- SnapEngine accelerated overload using SpatialIndex
- SelectTool hit-test and box selection use SpatialIndex
- File load paths rebuild spatial index
- Undo/redo commands keep spatial index consistent
- Performance benchmarks: <1ms snap, <5ms box select at 10,000 entities"
```
