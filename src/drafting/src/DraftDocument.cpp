#include "horizon/drafting/DraftDocument.h"

#include <algorithm>
#include <atomic>
#include <unordered_set>
#include <utility>

namespace hz::draft {

void DraftDocument::addEntity(std::shared_ptr<DraftEntity> entity) {
    insertEntity(npos, std::move(entity));
}

std::uint64_t DraftDocument::Revision::next() {
    static std::atomic<std::uint64_t> counter{0};
    return ++counter;
}

void DraftDocument::insertEntity(size_t position, std::shared_ptr<DraftEntity> entity) {
    if (!entity) return;
    m_revision.bump();
    m_spatialIndex.insert(entity);
    m_byId[entity->id()] = entity;
    const size_t at = std::min(position, m_entities.size());
    m_entities.insert(m_entities.begin() + static_cast<std::ptrdiff_t>(at), std::move(entity));
}

size_t DraftDocument::removeEntity(uint64_t id) {
    const auto found = m_byId.find(id);
    if (found == m_byId.end()) return npos;
    m_revision.bump();
    const DraftEntity* target = found->second.get();
    m_byId.erase(found);
    m_spatialIndex.remove(id);

    // Search from the end: undo removes the most recently added first.
    const auto it = std::find_if(m_entities.rbegin(), m_entities.rend(),
                                 [target](const auto& e) { return e.get() == target; });
    if (it == m_entities.rend()) return npos;
    const auto forward = std::next(it).base();
    const auto position = static_cast<size_t>(forward - m_entities.begin());
    m_entities.erase(forward);
    return position;
}

std::vector<PlacedEntity> DraftDocument::removeEntities(const std::vector<uint64_t>& ids) {
    std::unordered_set<const DraftEntity*> doomed;
    doomed.reserve(ids.size());
    for (uint64_t id : ids) {
        const auto found = m_byId.find(id);
        if (found == m_byId.end()) continue;
        doomed.insert(found->second.get());
        m_spatialIndex.remove(id);
        m_byId.erase(found);
    }
    if (!doomed.empty()) m_revision.bump();

    std::vector<PlacedEntity> removed;
    removed.reserve(doomed.size());
    std::vector<std::shared_ptr<DraftEntity>> kept;
    kept.reserve(m_entities.size() - std::min(m_entities.size(), doomed.size()));
    for (size_t i = 0; i < m_entities.size(); ++i) {
        if (doomed.count(m_entities[i].get()) != 0) {
            removed.push_back(PlacedEntity{i, std::move(m_entities[i])});
        } else {
            kept.push_back(std::move(m_entities[i]));
        }
    }
    m_entities = std::move(kept);
    return removed;
}

void DraftDocument::restoreEntities(const std::vector<PlacedEntity>& placed) {
    if (!placed.empty()) m_revision.bump();
    std::vector<std::shared_ptr<DraftEntity>> merged;
    merged.reserve(m_entities.size() + placed.size());
    size_t next = 0;
    // Positions are where each entity stood in the full drawing, so filling
    // up to each one from the remaining entities puts it back exactly there.
    for (const PlacedEntity& p : placed) {
        while (merged.size() < p.position && next < m_entities.size()) {
            merged.push_back(std::move(m_entities[next++]));
        }
        if (!p.entity) continue;
        m_spatialIndex.insert(p.entity);
        m_byId[p.entity->id()] = p.entity;
        merged.push_back(p.entity);
    }
    while (next < m_entities.size()) merged.push_back(std::move(m_entities[next++]));
    m_entities = std::move(merged);
}

bool DraftDocument::replaceEntity(uint64_t id, std::shared_ptr<DraftEntity> replacement) {
    const auto found = m_byId.find(id);
    if (found == m_byId.end() || !replacement) return false;
    const DraftEntity* target = found->second.get();
    const auto it = std::find_if(m_entities.rbegin(), m_entities.rend(),
                                 [target](const auto& e) { return e.get() == target; });
    if (it == m_entities.rend()) return false;
    m_revision.bump();
    m_byId.erase(found);
    m_spatialIndex.remove(id);
    *it = replacement;
    m_byId[replacement->id()] = replacement;
    m_spatialIndex.insert(replacement);
    return true;
}

DraftEntity* DraftDocument::findEntity(uint64_t id) const {
    const auto it = m_byId.find(id);
    return it == m_byId.end() ? nullptr : it->second.get();
}

std::shared_ptr<DraftEntity> DraftDocument::sharedEntity(uint64_t id) const {
    const auto it = m_byId.find(id);
    return it == m_byId.end() ? nullptr : it->second;
}

void DraftDocument::updateEntityBounds(uint64_t id) {
    const auto it = m_byId.find(id);
    if (it == m_byId.end()) return;
    m_revision.bump();
    m_spatialIndex.update(it->second);
}

void DraftDocument::clear() {
    m_revision.bump();
    m_entities.clear();
    m_byId.clear();
    m_spatialIndex.clear();
    m_blockTable.clear();
    m_nextGroupId = 1;
}

void DraftDocument::rebuildSpatialIndex() {
    m_revision.bump();
    m_spatialIndex.rebuild(m_entities);
    m_byId.clear();
    m_byId.reserve(m_entities.size());
    for (const auto& entity : m_entities) {
        if (entity) m_byId[entity->id()] = entity;
    }
}

}  // namespace hz::draft
