#include "horizon/drafting/SpatialIndex.h"

namespace hz::draft {

void SpatialIndex::insert(const std::shared_ptr<DraftEntity>& entity) {
    if (!entity) return;
    remove(entity->id());  // one entry per id
    const math::BoundingBox bbox = entity->boundingBox();
    if (!bbox.isValid()) return;
    m_tree.insert(entity->id(), bbox);
    m_boxes.emplace(entity->id(), bbox);

    math::BoundingBox beyond;
    for (const SnapPoint& sp : entity->typedSnapPoints()) {
        const math::Vec3 p(sp.point.x, sp.point.y, 0.0);
        if (!bbox.contains(p)) beyond.expand(p);
    }
    if (beyond.isValid()) {
        m_snapTree.insert(entity->id(), beyond);
        m_snapBoxes.emplace(entity->id(), beyond);
    }
}

void SpatialIndex::remove(uint64_t entityId) {
    const auto it = m_boxes.find(entityId);
    if (it == m_boxes.end()) return;
    m_tree.remove(entityId, it->second);
    m_boxes.erase(it);

    const auto snap = m_snapBoxes.find(entityId);
    if (snap == m_snapBoxes.end()) return;
    m_snapTree.remove(entityId, snap->second);
    m_snapBoxes.erase(snap);
}

void SpatialIndex::update(const std::shared_ptr<DraftEntity>& entity) {
    insert(entity);
}

std::vector<uint64_t> SpatialIndex::query(const math::BoundingBox& searchBox) const {
    return m_tree.query(searchBox);
}

std::vector<uint64_t> SpatialIndex::querySnaps(const math::BoundingBox& searchBox) const {
    std::vector<uint64_t> ids = m_tree.query(searchBox);
    if (!m_snapBoxes.empty()) {
        const std::vector<uint64_t> beyond = m_snapTree.query(searchBox);
        ids.insert(ids.end(), beyond.begin(), beyond.end());
    }
    return ids;
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
    m_snapTree.clear();
    m_snapBoxes.clear();
}

}  // namespace hz::draft
