#include "horizon/render/SelectionManager.h"

namespace hz::render {

SelectionManager::SelectionManager() = default;
SelectionManager::~SelectionManager() = default;

void SelectionManager::select(uint64_t id) {
    m_selected.insert(id);
}

void SelectionManager::deselect(uint64_t id) {
    m_selected.erase(id);
}

void SelectionManager::toggle(uint64_t id) {
    auto it = m_selected.find(id);
    if (it != m_selected.end()) {
        m_selected.erase(it);
    } else {
        m_selected.insert(id);
    }
}

void SelectionManager::clearSelection() {
    m_selected.clear();
}

bool SelectionManager::isSelected(uint64_t id) const {
    return m_selected.find(id) != m_selected.end();
}

std::vector<uint64_t> SelectionManager::selectedIds() const {
    return {m_selected.begin(), m_selected.end()};
}

}  // namespace hz::render
