#pragma once

#include "BlockTable.h"
#include "DraftEntity.h"
#include "DimensionStyle.h"
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

private:
    std::vector<std::shared_ptr<DraftEntity>> m_entities;
    DimensionStyle m_dimensionStyle;
    BlockTable m_blockTable;
    uint64_t m_nextGroupId = 1;
};

}  // namespace hz::draft
