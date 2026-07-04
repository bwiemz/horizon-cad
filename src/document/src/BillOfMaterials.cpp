#include "horizon/document/BillOfMaterials.h"

#include <filesystem>
#include <unordered_map>

#include "horizon/document/AssemblyDocument.h"

namespace hz::doc {

int BillOfMaterials::totalQuantity() const {
    int total = 0;
    for (const BomLine& line : lines) {
        total += line.quantity;
    }
    return total;
}

namespace {

/// A display name for a part: the file stem of its path (directory and extension
/// stripped), falling back to @p fallback when the path is empty.
std::string partDisplayName(const std::string& partPath, const std::string& fallback) {
    if (partPath.empty()) return fallback;
    const std::string stem = std::filesystem::path(partPath).stem().string();
    return stem.empty() ? fallback : stem;
}

}  // namespace

BillOfMaterials BomGenerator::generate(const AssemblyDocument& assembly) {
    BillOfMaterials bom;

    // Group by the part reference. An empty partPath keys on the instance name so
    // distinct unsaved parts don't all collapse into one line.
    std::unordered_map<std::string, std::size_t> lineByKey;  // key -> index in bom.lines

    for (const ComponentInstance& c : assembly.components()) {
        if (c.suppressed) continue;

        const std::string key = c.partPath.empty() ? ("@" + c.name) : c.partPath;
        auto it = lineByKey.find(key);
        if (it == lineByKey.end()) {
            BomLine line;
            line.item = static_cast<int>(bom.lines.size()) + 1;
            line.partName = partDisplayName(c.partPath, c.name);
            line.partPath = c.partPath;
            line.quantity = 1;
            lineByKey.emplace(key, bom.lines.size());
            bom.lines.push_back(std::move(line));
        } else {
            ++bom.lines[it->second].quantity;
        }
    }

    return bom;
}

}  // namespace hz::doc
