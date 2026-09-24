#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "horizon/math/BoundingBox.h"

namespace hz::math {

/// Header-only R*-tree spatial index for 2D CAD entities.
///
/// Stores nodes in a flat std::vector for cache-friendly traversal.
/// Uses quadratic split when a node overflows MaxChildren.
///
/// Template parameters:
///   ValueT       - stored value type (e.g. uint64_t for entity IDs)
///   MaxChildren  - maximum entries per node before split
///   MinChildren  - minimum entries per node after split
template <typename ValueT, int MaxChildren = 16, int MinChildren = MaxChildren / 3>
class RTree {
public:
    RTree() { clear(); }

    /// Insert a value with its bounding box.
    void insert(const ValueT& value, const BoundingBox& bbox) {
        Entry entry{value, bbox};

        if (m_size == 0) {
            // First insert: root is a leaf, add directly
            m_nodes[m_root].entries.push_back(entry);
            m_nodes[m_root].bbox = bbox;
        } else {
            // Choose the best leaf and insert
            int leafIdx = chooseLeaf(m_root, bbox);
            m_nodes[leafIdx].entries.push_back(entry);
            m_nodes[leafIdx].bbox.expand(bbox);

            // Handle overflow by splitting up the tree.
            // Step 1: if the leaf overflows, split it to produce a newSibling.
            int current = leafIdx;
            int newSibling = -1;
            if (static_cast<int>(m_nodes[current].entries.size()) > MaxChildren) {
                newSibling = splitNode(current);
            }

            // Step 2: walk upward while we have a split to propagate.
            while (newSibling >= 0) {
                int parent = m_nodes[current].parent;
                if (parent < 0) {
                    // current is root -- create new root containing current and newSibling
                    int newRoot = allocateNode();
                    m_nodes[newRoot].isLeaf = false;
                    m_nodes[newRoot].bbox = m_nodes[current].bbox;
                    m_nodes[newRoot].bbox.expand(m_nodes[newSibling].bbox);
                    m_nodes[newRoot].children.push_back(current);
                    m_nodes[newRoot].children.push_back(newSibling);
                    m_nodes[current].parent = newRoot;
                    m_nodes[newSibling].parent = newRoot;
                    m_root = newRoot;
                    newSibling = -1;  // done
                } else {
                    // Insert newSibling into parent
                    m_nodes[parent].children.push_back(newSibling);
                    m_nodes[newSibling].parent = parent;
                    recomputeBBox(parent);
                    current = parent;
                    newSibling = -1;
                    // If parent overflows, split it and continue upward
                    if (static_cast<int>(m_nodes[current].children.size()) > MaxChildren) {
                        newSibling = splitInternalNode(current);
                    }
                }
            }

            // Propagate bbox enlargement up to root
            propagateBBox(leafIdx);
        }
        ++m_size;
    }

    /// Query all values whose bounding boxes intersect the search box.
    [[nodiscard]] std::vector<ValueT> query(const BoundingBox& searchBox) const {
        std::vector<ValueT> results;
        if (m_size == 0) return results;
        queryNode(m_root, searchBox, results);
        return results;
    }

    /// Number of entries in the tree.
    [[nodiscard]] size_t size() const { return m_size; }

    /// Whether the tree is empty.
    [[nodiscard]] bool empty() const { return m_size == 0; }

    /// Remove all entries and reset the tree.
    void clear() {
        m_nodes.clear();
        m_free.clear();
        m_size = 0;
        m_root = allocateNode();
        m_nodes[m_root].isLeaf = true;
        m_nodes[m_root].parent = -1;
    }

    /// Remove one entry equal to @p value, looking for it where @p where says
    /// it is: the box it was inserted with. Returns whether an entry was
    /// removed. O(log n) when the box is right; when it is not, the whole tree
    /// is searched, so a stale box costs time, not correctness.
    ///
    /// The entry leaves its leaf; a node left with fewer than MinChildren is
    /// dissolved and its entries inserted again, and the root shrinks when it
    /// has a single child (Guttman's CondenseTree). Nothing is rebuilt.
    bool remove(const ValueT& value, const BoundingBox& where) {
        Slot slot;
        if (!findEntry(m_root, value, &where, slot) && !findEntry(m_root, value, nullptr, slot)) {
            return false;
        }
        eraseEntry(slot);
        return true;
    }

    /// Remove every entry equal to @p value. Searches the whole tree; prefer
    /// remove(value, where) when the entry's box is known. A no-op if the
    /// value is not present.
    void remove(const ValueT& value) {
        Slot slot;
        while (m_size > 0 && findEntry(m_root, value, nullptr, slot)) eraseEntry(slot);
    }

    /// Nodes in use: at most about 2n / MinChildren for n entries, however
    /// many inserts and removals led there (freed nodes are reused).
    [[nodiscard]] size_t nodeCount() const { return m_nodes.size() - m_free.size(); }

private:
    struct Entry {
        ValueT value;
        BoundingBox bbox;
    };

    struct Node {
        bool isLeaf = true;
        int parent = -1;
        BoundingBox bbox;
        std::vector<Entry> entries;  // only used if isLeaf
        std::vector<int> children;   // only used if !isLeaf
    };

    /// Where an entry lives: its leaf and its index in the leaf's entries.
    struct Slot {
        int leaf = -1;
        size_t index = 0;
    };

    std::vector<Node> m_nodes;
    std::vector<int> m_free;  ///< indices of nodes freed by removals, reused first
    int m_root = 0;
    size_t m_size = 0;

    /// Allocate a node, reusing a freed one if there is one; return its index.
    /// Reuse does not reallocate m_nodes, so references to other nodes stay
    /// valid; a new node may.
    int allocateNode() {
        if (!m_free.empty()) {
            const int index = m_free.back();
            m_free.pop_back();
            m_nodes[static_cast<size_t>(index)] = Node{};
            return index;
        }
        m_nodes.emplace_back();
        return static_cast<int>(m_nodes.size()) - 1;
    }

    void freeNode(int index) {
        Node& node = m_nodes[static_cast<size_t>(index)];
        node.entries.clear();
        node.children.clear();
        node.parent = -1;
        m_free.push_back(index);
    }

    /// Find an entry equal to @p value, descending only into nodes whose box
    /// intersects @p where when it is given.
    bool findEntry(int nodeIdx, const ValueT& value, const BoundingBox* where, Slot& out) const {
        const Node& node = m_nodes[static_cast<size_t>(nodeIdx)];
        if (where != nullptr && (!node.bbox.isValid() || !node.bbox.intersects(*where))) {
            return false;
        }
        if (node.isLeaf) {
            for (size_t i = 0; i < node.entries.size(); ++i) {
                const Entry& entry = node.entries[i];
                // With a box, the entry must be there too: a value inserted
                // twice is removed where the caller says, not wherever.
                if (entry.value == value && (where == nullptr || entry.bbox.intersects(*where))) {
                    out = Slot{nodeIdx, i};
                    return true;
                }
            }
            return false;
        }
        for (int child : node.children) {
            if (findEntry(child, value, where, out)) return true;
        }
        return false;
    }

    /// Take the entry at @p slot out of the tree and condense the path above it.
    void eraseEntry(const Slot& slot) {
        auto& entries = m_nodes[static_cast<size_t>(slot.leaf)].entries;
        entries.erase(entries.begin() + static_cast<std::ptrdiff_t>(slot.index));
        --m_size;
        if (m_size == 0) {
            clear();
            return;
        }

        // Walk up from the leaf: dissolve underfull nodes, keeping their
        // entries to insert again; tighten the boxes of the rest.
        std::vector<Entry> orphans;
        int current = slot.leaf;
        while (current != m_root) {
            const int parent = m_nodes[static_cast<size_t>(current)].parent;
            const Node& node = m_nodes[static_cast<size_t>(current)];
            const size_t count = node.isLeaf ? node.entries.size() : node.children.size();
            if (count < static_cast<size_t>(MinChildren)) {
                auto& siblings = m_nodes[static_cast<size_t>(parent)].children;
                siblings.erase(std::find(siblings.begin(), siblings.end(), current));
                collectAndFree(current, orphans);
            } else {
                recomputeBBox(current);
            }
            current = parent;
        }
        recomputeBBox(m_root);

        // A root with a single child is replaced by that child.
        while (!m_nodes[static_cast<size_t>(m_root)].isLeaf &&
               m_nodes[static_cast<size_t>(m_root)].children.size() == 1) {
            const int child = m_nodes[static_cast<size_t>(m_root)].children.front();
            freeNode(m_root);
            m_root = child;
            m_nodes[static_cast<size_t>(m_root)].parent = -1;
        }
        if (!m_nodes[static_cast<size_t>(m_root)].isLeaf &&
            m_nodes[static_cast<size_t>(m_root)].children.empty()) {
            m_nodes[static_cast<size_t>(m_root)].isLeaf = true;
            m_nodes[static_cast<size_t>(m_root)].bbox.reset();
        }

        m_size -= orphans.size();
        // insert() puts the first entry of an empty tree into a leaf root.
        if (m_size == 0) clear();
        for (const Entry& orphan : orphans) insert(orphan.value, orphan.bbox);
    }

    /// Move every entry under @p nodeIdx into @p out and free its nodes.
    void collectAndFree(int nodeIdx, std::vector<Entry>& out) {
        Node& node = m_nodes[static_cast<size_t>(nodeIdx)];
        if (node.isLeaf) {
            out.insert(out.end(), node.entries.begin(), node.entries.end());
        } else {
            const std::vector<int> children = node.children;
            for (int child : children) collectAndFree(child, out);
        }
        freeNode(nodeIdx);
    }

    /// Compute 2D area of a bounding box (x * y, ignoring z).
    static double bboxArea(const BoundingBox& bb) {
        if (!bb.isValid()) return 0.0;
        Vec3 s = bb.size();
        return s.x * s.y;
    }

    /// Compute the 2D area enlargement needed to include `addition` in `base`.
    static double areaEnlargement(const BoundingBox& base, const BoundingBox& addition) {
        BoundingBox merged = base;
        merged.expand(addition);
        return bboxArea(merged) - bboxArea(base);
    }

    /// Choose the best leaf node to insert a new entry with the given bbox.
    int chooseLeaf(int nodeIdx, const BoundingBox& bbox) const {
        int current = nodeIdx;
        while (!m_nodes[current].isLeaf) {
            const auto& children = m_nodes[current].children;
            int bestChild = children[0];
            double bestEnlargement = areaEnlargement(m_nodes[children[0]].bbox, bbox);
            double bestArea = bboxArea(m_nodes[children[0]].bbox);

            for (size_t i = 1; i < children.size(); ++i) {
                double enlargement = areaEnlargement(m_nodes[children[i]].bbox, bbox);
                double area = bboxArea(m_nodes[children[i]].bbox);
                if (enlargement < bestEnlargement ||
                    (enlargement == bestEnlargement && area < bestArea)) {
                    bestChild = children[i];
                    bestEnlargement = enlargement;
                    bestArea = area;
                }
            }
            current = bestChild;
        }
        return current;
    }

    /// Quadratic split for a leaf node that has overflowed.
    /// Distributes entries between existing node and a new sibling.
    /// Returns the index of the new sibling node.
    int splitNode(int nodeIdx) {
        auto& node = m_nodes[nodeIdx];
        auto entries = std::move(node.entries);
        node.entries.clear();

        int newIdx = allocateNode();
        // IMPORTANT: after allocateNode, references to m_nodes elements are
        // potentially invalidated. Re-acquire references.
        auto& left = m_nodes[nodeIdx];
        auto& right = m_nodes[newIdx];
        right.isLeaf = true;
        right.parent = left.parent;

        // Pick seeds: the pair with the largest waste (area of combined bbox
        // minus individual areas).
        int seed1 = 0, seed2 = 1;
        double worstWaste = -std::numeric_limits<double>::max();
        for (size_t i = 0; i < entries.size(); ++i) {
            for (size_t j = i + 1; j < entries.size(); ++j) {
                BoundingBox merged = entries[i].bbox;
                merged.expand(entries[j].bbox);
                double waste =
                    bboxArea(merged) - bboxArea(entries[i].bbox) - bboxArea(entries[j].bbox);
                if (waste > worstWaste) {
                    worstWaste = waste;
                    seed1 = static_cast<int>(i);
                    seed2 = static_cast<int>(j);
                }
            }
        }

        left.entries.push_back(entries[seed1]);
        left.bbox = entries[seed1].bbox;
        right.entries.push_back(entries[seed2]);
        right.bbox = entries[seed2].bbox;

        // Mark seeds as assigned
        std::vector<bool> assigned(entries.size(), false);
        assigned[seed1] = true;
        assigned[seed2] = true;

        // Distribute remaining entries
        for (size_t round = 0; round < entries.size(); ++round) {
            // Check if all assigned
            bool allDone = true;
            for (size_t i = 0; i < entries.size(); ++i) {
                if (!assigned[i]) {
                    allDone = false;
                    break;
                }
            }
            if (allDone) break;

            // If one group needs all remaining to reach MinChildren, assign them
            int remaining = 0;
            for (size_t i = 0; i < entries.size(); ++i) {
                if (!assigned[i]) ++remaining;
            }
            if (static_cast<int>(m_nodes[nodeIdx].entries.size()) + remaining <= MinChildren) {
                for (size_t i = 0; i < entries.size(); ++i) {
                    if (!assigned[i]) {
                        m_nodes[nodeIdx].entries.push_back(entries[i]);
                        m_nodes[nodeIdx].bbox.expand(entries[i].bbox);
                        assigned[i] = true;
                    }
                }
                break;
            }
            if (static_cast<int>(m_nodes[newIdx].entries.size()) + remaining <= MinChildren) {
                for (size_t i = 0; i < entries.size(); ++i) {
                    if (!assigned[i]) {
                        m_nodes[newIdx].entries.push_back(entries[i]);
                        m_nodes[newIdx].bbox.expand(entries[i].bbox);
                        assigned[i] = true;
                    }
                }
                break;
            }

            // Pick the entry with maximum preference difference
            int bestEntry = -1;
            double bestDiff = -std::numeric_limits<double>::max();
            for (size_t i = 0; i < entries.size(); ++i) {
                if (assigned[i]) continue;
                double d1 = areaEnlargement(m_nodes[nodeIdx].bbox, entries[i].bbox);
                double d2 = areaEnlargement(m_nodes[newIdx].bbox, entries[i].bbox);
                double diff = std::abs(d1 - d2);
                if (diff > bestDiff) {
                    bestDiff = diff;
                    bestEntry = static_cast<int>(i);
                }
            }

            if (bestEntry < 0) break;

            double enlLeft = areaEnlargement(m_nodes[nodeIdx].bbox, entries[bestEntry].bbox);
            double enlRight = areaEnlargement(m_nodes[newIdx].bbox, entries[bestEntry].bbox);
            if (enlLeft < enlRight || (enlLeft == enlRight && bboxArea(m_nodes[nodeIdx].bbox) <=
                                                                  bboxArea(m_nodes[newIdx].bbox))) {
                m_nodes[nodeIdx].entries.push_back(entries[bestEntry]);
                m_nodes[nodeIdx].bbox.expand(entries[bestEntry].bbox);
            } else {
                m_nodes[newIdx].entries.push_back(entries[bestEntry]);
                m_nodes[newIdx].bbox.expand(entries[bestEntry].bbox);
            }
            assigned[bestEntry] = true;
        }

        return newIdx;
    }

    /// Quadratic split for an internal node that has overflowed.
    /// Distributes children between existing node and a new sibling.
    /// Returns the index of the new sibling node.
    int splitInternalNode(int nodeIdx) {
        auto children = std::move(m_nodes[nodeIdx].children);
        m_nodes[nodeIdx].children.clear();

        int newIdx = allocateNode();
        // Re-acquire references after potential reallocation
        auto& left = m_nodes[nodeIdx];
        auto& right = m_nodes[newIdx];
        right.isLeaf = false;
        right.parent = left.parent;

        // Pick seeds
        int seed1 = 0, seed2 = 1;
        double worstWaste = -std::numeric_limits<double>::max();
        for (size_t i = 0; i < children.size(); ++i) {
            for (size_t j = i + 1; j < children.size(); ++j) {
                BoundingBox merged = m_nodes[children[i]].bbox;
                merged.expand(m_nodes[children[j]].bbox);
                double waste = bboxArea(merged) - bboxArea(m_nodes[children[i]].bbox) -
                               bboxArea(m_nodes[children[j]].bbox);
                if (waste > worstWaste) {
                    worstWaste = waste;
                    seed1 = static_cast<int>(i);
                    seed2 = static_cast<int>(j);
                }
            }
        }

        left.children.push_back(children[seed1]);
        left.bbox = m_nodes[children[seed1]].bbox;
        right.children.push_back(children[seed2]);
        right.bbox = m_nodes[children[seed2]].bbox;

        std::vector<bool> assigned(children.size(), false);
        assigned[seed1] = true;
        assigned[seed2] = true;

        for (size_t round = 0; round < children.size(); ++round) {
            bool allDone = true;
            for (size_t i = 0; i < children.size(); ++i) {
                if (!assigned[i]) {
                    allDone = false;
                    break;
                }
            }
            if (allDone) break;

            int remaining = 0;
            for (size_t i = 0; i < children.size(); ++i) {
                if (!assigned[i]) ++remaining;
            }
            if (static_cast<int>(m_nodes[nodeIdx].children.size()) + remaining <= MinChildren) {
                for (size_t i = 0; i < children.size(); ++i) {
                    if (!assigned[i]) {
                        m_nodes[nodeIdx].children.push_back(children[i]);
                        m_nodes[children[i]].parent = nodeIdx;
                        m_nodes[nodeIdx].bbox.expand(m_nodes[children[i]].bbox);
                        assigned[i] = true;
                    }
                }
                break;
            }
            if (static_cast<int>(m_nodes[newIdx].children.size()) + remaining <= MinChildren) {
                for (size_t i = 0; i < children.size(); ++i) {
                    if (!assigned[i]) {
                        m_nodes[newIdx].children.push_back(children[i]);
                        m_nodes[children[i]].parent = newIdx;
                        m_nodes[newIdx].bbox.expand(m_nodes[children[i]].bbox);
                        assigned[i] = true;
                    }
                }
                break;
            }

            int bestEntry = -1;
            double bestDiff = -std::numeric_limits<double>::max();
            for (size_t i = 0; i < children.size(); ++i) {
                if (assigned[i]) continue;
                double d1 = areaEnlargement(m_nodes[nodeIdx].bbox, m_nodes[children[i]].bbox);
                double d2 = areaEnlargement(m_nodes[newIdx].bbox, m_nodes[children[i]].bbox);
                double diff = std::abs(d1 - d2);
                if (diff > bestDiff) {
                    bestDiff = diff;
                    bestEntry = static_cast<int>(i);
                }
            }

            if (bestEntry < 0) break;

            double enlLeft =
                areaEnlargement(m_nodes[nodeIdx].bbox, m_nodes[children[bestEntry]].bbox);
            double enlRight =
                areaEnlargement(m_nodes[newIdx].bbox, m_nodes[children[bestEntry]].bbox);
            if (enlLeft < enlRight || (enlLeft == enlRight && bboxArea(m_nodes[nodeIdx].bbox) <=
                                                                  bboxArea(m_nodes[newIdx].bbox))) {
                m_nodes[nodeIdx].children.push_back(children[bestEntry]);
                m_nodes[children[bestEntry]].parent = nodeIdx;
                m_nodes[nodeIdx].bbox.expand(m_nodes[children[bestEntry]].bbox);
            } else {
                m_nodes[newIdx].children.push_back(children[bestEntry]);
                m_nodes[children[bestEntry]].parent = newIdx;
                m_nodes[newIdx].bbox.expand(m_nodes[children[bestEntry]].bbox);
            }
            assigned[bestEntry] = true;
        }

        // Update parent pointers for children that ended up in right
        for (int childIdx : m_nodes[newIdx].children) {
            m_nodes[childIdx].parent = newIdx;
        }
        // Update parent pointers for children that stayed in left
        for (int childIdx : m_nodes[nodeIdx].children) {
            m_nodes[childIdx].parent = nodeIdx;
        }

        return newIdx;
    }

    /// Recompute a node's bounding box from its children (internal) or entries (leaf).
    void recomputeBBox(int nodeIdx) {
        auto& node = m_nodes[nodeIdx];
        node.bbox.reset();
        if (node.isLeaf) {
            for (const auto& entry : node.entries) {
                node.bbox.expand(entry.bbox);
            }
        } else {
            for (int childIdx : node.children) {
                node.bbox.expand(m_nodes[childIdx].bbox);
            }
        }
    }

    /// Propagate bounding box enlargement from a node up to the root.
    void propagateBBox(int nodeIdx) {
        int current = m_nodes[nodeIdx].parent;
        while (current >= 0) {
            recomputeBBox(current);
            current = m_nodes[current].parent;
        }
    }

    /// Recursively query a node for entries intersecting the search box.
    void queryNode(int nodeIdx, const BoundingBox& searchBox, std::vector<ValueT>& results) const {
        const auto& node = m_nodes[nodeIdx];
        if (!node.bbox.isValid() || !node.bbox.intersects(searchBox)) return;

        if (node.isLeaf) {
            for (const auto& entry : node.entries) {
                if (entry.bbox.intersects(searchBox)) {
                    results.push_back(entry.value);
                }
            }
        } else {
            for (int childIdx : node.children) {
                queryNode(childIdx, searchBox, results);
            }
        }
    }
};

}  // namespace hz::math
