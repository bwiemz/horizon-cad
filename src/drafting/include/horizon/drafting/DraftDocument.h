#pragma once

#include "DraftEntity.h"
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
    void clear();

private:
    std::vector<std::shared_ptr<DraftEntity>> m_entities;
};

}  // namespace hz::draft
