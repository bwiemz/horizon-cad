#include "horizon/drafting/BlockTable.h"
#include <algorithm>

namespace hz::draft {

bool BlockTable::addBlock(std::shared_ptr<BlockDefinition> block) {
    if (!block || block->name.empty()) return false;
    auto [it, inserted] = m_blocks.emplace(block->name, std::move(block));
    return inserted;
}

bool BlockTable::removeBlock(const std::string& name) {
    return m_blocks.erase(name) > 0;
}

std::shared_ptr<BlockDefinition> BlockTable::findBlock(const std::string& name) const {
    auto it = m_blocks.find(name);
    return (it != m_blocks.end()) ? it->second : nullptr;
}

std::vector<std::string> BlockTable::blockNames() const {
    std::vector<std::string> names;
    names.reserve(m_blocks.size());
    for (const auto& [name, _] : m_blocks) {
        names.push_back(name);
    }
    std::sort(names.begin(), names.end());
    return names;
}

void BlockTable::clear() {
    m_blocks.clear();
}

}  // namespace hz::draft
