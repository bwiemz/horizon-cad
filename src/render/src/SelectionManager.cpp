#include "horizon/render/SelectionManager.h"

namespace hz::render {

SelectionManager::SelectionManager() = default;
SelectionManager::~SelectionManager() = default;

void SelectionManager::select(uint64_t id) {
    if (m_selected.insert(id).second) ++m_revision;
}

void SelectionManager::deselect(uint64_t id) {
    if (m_selected.erase(id) > 0) ++m_revision;
}

void SelectionManager::toggle(uint64_t id) {
    auto it = m_selected.find(id);
    if (it != m_selected.end()) {
        m_selected.erase(it);
    } else {
        m_selected.insert(id);
    }
    ++m_revision;
}

void SelectionManager::clearSelection() {
    if (m_selected.empty()) return;
    m_selected.clear();
    ++m_revision;
}

bool SelectionManager::isSelected(uint64_t id) const {
    return m_selected.find(id) != m_selected.end();
}

std::vector<uint64_t> SelectionManager::selectedIds() const {
    return {m_selected.begin(), m_selected.end()};
}

}  // namespace hz::render
