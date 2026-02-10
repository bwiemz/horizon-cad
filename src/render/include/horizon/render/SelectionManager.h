#pragma once

#include <cstdint>
#include <set>
#include <vector>

namespace hz::render {

/// Tracks the set of currently selected object IDs.
class SelectionManager {
public:
    SelectionManager();
    ~SelectionManager();

    void select(uint32_t id);
    void deselect(uint32_t id);
    void toggle(uint32_t id);
    void clearSelection();

    bool isSelected(uint32_t id) const;

    std::vector<uint32_t> selectedIds() const;
    size_t count() const { return m_selected.size(); }
    bool empty() const { return m_selected.empty(); }

private:
    std::set<uint32_t> m_selected;
};

}  // namespace hz::render
