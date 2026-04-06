#pragma once

#include "horizon/math/RTree.h"
#include "horizon/math/BoundingBox.h"
#include "DraftEntity.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace hz::draft {

class SpatialIndex {
public:
    SpatialIndex() = default;

    void insert(const std::shared_ptr<DraftEntity>& entity);
    void remove(uint64_t entityId);
    void update(const std::shared_ptr<DraftEntity>& entity);

    [[nodiscard]] std::vector<uint64_t> query(const math::BoundingBox& searchBox) const;

    void rebuild(const std::vector<std::shared_ptr<DraftEntity>>& entities);
    void clear();

private:
    math::RTree<uint64_t> m_tree;
};

}  // namespace hz::draft
