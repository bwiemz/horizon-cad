#include "horizon/drafting/DraftDocument.h"
#include <algorithm>

namespace hz::draft {

void DraftDocument::addEntity(std::shared_ptr<DraftEntity> entity) {
    if (entity) {
        m_entities.push_back(std::move(entity));
    }
}

void DraftDocument::removeEntity(uint64_t id) {
    m_entities.erase(
        std::remove_if(m_entities.begin(), m_entities.end(),
            [id](const std::shared_ptr<DraftEntity>& e) {
                return e->id() == id;
            }),
        m_entities.end());
}

void DraftDocument::clear() {
    m_entities.clear();
    m_blockTable.clear();
    m_nextGroupId = 1;
}

}  // namespace hz::draft
