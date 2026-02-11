#pragma once

#include "BlockDefinition.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace hz::draft {

/// Stores named block definitions.  Owned by DraftDocument.
class BlockTable {
public:
    BlockTable() = default;

    /// Add a block definition.  Returns false if name already exists.
    bool addBlock(std::shared_ptr<BlockDefinition> block);

    /// Remove a block definition by name.  Returns false if not found.
    bool removeBlock(const std::string& name);

    /// Look up a block by name.  Returns nullptr if not found.
    std::shared_ptr<BlockDefinition> findBlock(const std::string& name) const;

    /// All block names (sorted alphabetically).
    std::vector<std::string> blockNames() const;

    /// Total number of definitions.
    size_t size() const { return m_blocks.size(); }

    void clear();

private:
    std::unordered_map<std::string, std::shared_ptr<BlockDefinition>> m_blocks;
};

}  // namespace hz::draft
