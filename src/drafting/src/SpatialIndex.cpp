#include "horizon/drafting/SpatialIndex.h"

namespace hz::draft {

void SpatialIndex::insert(const std::shared_ptr<DraftEntity>& entity) {
    if (!entity) return;
    remove(entity->id());  // one entry per id
    const math::BoundingBox bbox = entity->boundingBox();
    if (!bbox.isValid()) return;
    m_tree.insert(entity->id(), bbox);
    m_boxes.emplace(entity->id(), bbox);
}

void SpatialIndex::remove(uint64_t entityId) {
    const auto it = m_boxes.find(entityId);
    if (it == m_boxes.end()) return;
    m_tree.remove(entityId, it->second);
    m_boxes.erase(it);
}

void SpatialIndex::update(const std::shared_ptr<DraftEntity>& entity) {
    insert(entity);
}

std::vector<uint64_t> SpatialIndex::query(const math::BoundingBox& searchBox) const {
    return m_tree.query(searchBox);
}

void SpatialIndex::rebuild(const std::vector<std::shared_ptr<DraftEntity>>& entities) {
    clear();
    for (const auto& entity : entities) {
        insert(entity);
    }
}

void SpatialIndex::clear() {
    m_tree.clear();
    m_boxes.clear();
}

}  // namespace hz::draft
