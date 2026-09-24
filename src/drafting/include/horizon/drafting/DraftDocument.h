#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "BlockTable.h"
#include "DimensionStyle.h"
#include "DraftEntity.h"
#include "SpatialIndex.h"

namespace hz::draft {

/// An entity and where it stood in the drawing order.
struct PlacedEntity {
    size_t position = 0;
    std::shared_ptr<DraftEntity> entity;
};

class DraftDocument {
public:
    static constexpr size_t npos = static_cast<size_t>(-1);

    DraftDocument() = default;

    /// Add @p entity at the end of the drawing order (drawn last).
    void addEntity(std::shared_ptr<DraftEntity> entity);
    /// Insert @p entity at @p position in the drawing order; a position past
    /// the end appends it. Undo uses this to put an entity back where it was.
    void insertEntity(size_t position, std::shared_ptr<DraftEntity> entity);
    /// Remove the entity with @p id. Returns the position it had in the
    /// drawing order, or npos if there is no such entity. Fast for the most
    /// recently added entities, which undo removes first.
    size_t removeEntity(uint64_t id);
    /// Remove every entity whose id is in @p ids, in one pass over the
    /// drawing. Returns what was removed, with positions, in drawing order,
    /// for restoreEntities().
    std::vector<PlacedEntity> removeEntities(const std::vector<uint64_t>& ids);
    /// Put back what removeEntities() returned, each at its old position.
    void restoreEntities(const std::vector<PlacedEntity>& placed);

    /// The entity with @p id, or nullptr. O(1).
    [[nodiscard]] DraftEntity* findEntity(uint64_t id) const;
    /// The entity with @p id as a shared pointer, or null. O(1).
    [[nodiscard]] std::shared_ptr<DraftEntity> sharedEntity(uint64_t id) const;

    /// Put @p replacement in the place of the entity with @p id, keeping its
    /// position in the drawing order. Returns false if there is no such
    /// entity. The replacement should carry the same id.
    bool replaceEntity(uint64_t id, std::shared_ptr<DraftEntity> replacement);

    /// Re-index the entity with @p id after its geometry changed, so picking,
    /// snapping and box selection see where it is now. O(log n).
    void updateEntityBounds(uint64_t id);

    /// The drawing's entities, in drawing order. The mutable overload is for
    /// changing entities in place: add and remove them only through the
    /// functions above, which keep the id lookup and the spatial index in
    /// step (rebuildSpatialIndex() resynchronizes both).
    const std::vector<std::shared_ptr<DraftEntity>>& entities() const { return m_entities; }
    std::vector<std::shared_ptr<DraftEntity>>& entities() { return m_entities; }
    void clear();

    /// Returns a unique group ID and increments the internal counter.
    uint64_t nextGroupId() { return m_nextGroupId++; }

    /// Ensure the next group ID is greater than the given value (used on file load).
    void advanceGroupIdCounter(uint64_t minId) {
        if (m_nextGroupId <= minId) m_nextGroupId = minId + 1;
    }

    const DimensionStyle& dimensionStyle() const { return m_dimensionStyle; }
    void setDimensionStyle(const DimensionStyle& style) { m_dimensionStyle = style; }

    BlockTable& blockTable() { return m_blockTable; }
    const BlockTable& blockTable() const { return m_blockTable; }

    const SpatialIndex& spatialIndex() const { return m_spatialIndex; }
    SpatialIndex& spatialIndex() { return m_spatialIndex; }
    /// Rebuild the spatial index and the id lookup from entities(): after a
    /// change to many entities' geometry at once.
    void rebuildSpatialIndex();

private:
    std::vector<std::shared_ptr<DraftEntity>> m_entities;
    std::unordered_map<uint64_t, std::shared_ptr<DraftEntity>> m_byId;
    DimensionStyle m_dimensionStyle;
    BlockTable m_blockTable;
    uint64_t m_nextGroupId = 1;
    SpatialIndex m_spatialIndex;
};

}  // namespace hz::draft
