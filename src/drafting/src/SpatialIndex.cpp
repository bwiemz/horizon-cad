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
