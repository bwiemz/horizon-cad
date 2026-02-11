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

    const DimensionStyle& dimensionStyle() const { return m_dimensionStyle; }
    void setDimensionStyle(const DimensionStyle& style) { m_dimensionStyle = style; }

    BlockTable& blockTable() { return m_blockTable; }
    const BlockTable& blockTable() const { return m_blockTable; }

private:
    std::vector<std::shared_ptr<DraftEntity>> m_entities;
    DimensionStyle m_dimensionStyle;
    BlockTable m_blockTable;
};

}  // namespace hz::draft
