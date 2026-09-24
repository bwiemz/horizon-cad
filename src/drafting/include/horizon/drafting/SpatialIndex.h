#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "DraftEntity.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/RTree.h"

namespace hz::draft {

/// The entities of a drawing by where they are: which ones might lie in a box.
///
/// Each entity is indexed under the bounding box it had when it was inserted
/// or last updated; an entity whose geometry changes must be update()d.
/// Insert, remove and update are O(log n): the index remembers each entity's
/// box, so it finds the entry without scanning, even if the entity has since
/// moved (Phase 122; removal used to rebuild the whole tree).
class SpatialIndex {
public:
    SpatialIndex() = default;

    /// Index @p entity at its current bounding box. An entity already in the
    /// index is moved there. An entity without a valid box is not indexed.
    void insert(const std::shared_ptr<DraftEntity>& entity);
    void remove(uint64_t entityId);
    /// Re-index @p entity at its current bounding box.
    void update(const std::shared_ptr<DraftEntity>& entity);

    [[nodiscard]] std::vector<uint64_t> query(const math::BoundingBox& searchBox) const;

    [[nodiscard]] size_t size() const { return m_boxes.size(); }
    [[nodiscard]] bool contains(uint64_t entityId) const { return m_boxes.count(entityId) != 0; }

    void rebuild(const std::vector<std::shared_ptr<DraftEntity>>& entities);
    void clear();

private:
    math::RTree<uint64_t> m_tree;
    std::unordered_map<uint64_t, math::BoundingBox> m_boxes;  ///< the box each id is indexed under
};

}  // namespace hz::draft
