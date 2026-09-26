#include "horizon/document/BillOfMaterials.h"

#include <filesystem>
#include <unordered_map>

#include "horizon/document/AssemblyDocument.h"

namespace hz::doc {

int BillOfMaterials::totalQuantity() const {
    int total = 0;
    for (const BomLine& line : lines) {
        if (line.level == 0) total += line.quantity;  // an indented BOM's own, once
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

/// @p c's part's name, said mirrored for a mirrored one (Phase 162).
std::string partDisplayName(const ComponentInstance& c) {
    const std::string name = partDisplayName(c.partPath, c.name);
    return c.mirrored ? name + " (mirrored)" : name;
}

}  // namespace

namespace {

/// The key a component's file is one line by: canonical, a relative path
/// taken from @p assembly's folder; an unsaved part's by its name.
std::string lineKey(const AssemblyDocument& assembly, const ComponentInstance& c) {
    // A mirrored part (Phase 162) is another part: a line of its own.
    if (c.mirrored) {
        ComponentInstance own = c;
        own.mirrored = false;
        return lineKey(assembly, own) + "#mirrored";
    }
    if (c.partPath.empty()) return "@" + c.name;
    std::filesystem::path file(c.partPath);
    if (file.is_relative() && !assembly.filePath().empty()) {
        file = std::filesystem::path(assembly.filePath()).parent_path() / file;
    }
    std::error_code ec;
    const auto canonical = std::filesystem::weakly_canonical(file, ec);
    return ec ? file.lexically_normal().string() : canonical.string();
}

/// @p assembly's own lines, grouped by file, in order of first appearance,
/// each with the first component of it (for a subassembly's own lines).
struct Grouped {
    std::vector<BomLine> lines;
    std::vector<const ComponentInstance*> first;
};

Grouped group(const AssemblyDocument& assembly) {
    Grouped out;
    std::unordered_map<std::string, std::size_t> lineByKey;
    for (const ComponentInstance& c : assembly.components()) {
        if (c.suppressed) continue;
        const std::string key = lineKey(assembly, c);
        const auto it = lineByKey.find(key);
        if (it != lineByKey.end()) {
            ++out.lines[it->second].quantity;
            continue;
        }
        BomLine line;
        line.item = static_cast<int>(out.lines.size()) + 1;
        line.partName = partDisplayName(c);
        line.partPath = c.partPath;
        line.quantity = 1;
        line.assembly = c.isAssembly();
        lineByKey.emplace(key, out.lines.size());
        out.lines.push_back(std::move(line));
        out.first.push_back(&c);
    }
    return out;
}

/// The indented lines of @p assembly, at @p level, numbered after @p above.
void indent(const AssemblyDocument& assembly, int level, const std::string& above,
            std::vector<BomLine>& out) {
    Grouped grouped = group(assembly);
    for (std::size_t i = 0; i < grouped.lines.size(); ++i) {
        BomLine line = grouped.lines[i];
        line.level = level;
        line.index =
            above.empty() ? std::to_string(line.item) : above + "." + std::to_string(line.item);
        const std::string index = line.index;
        out.push_back(std::move(line));
        if (const auto& sub = grouped.first[i]->resolvedAssembly) {
            indent(*sub, level + 1, index, out);
        }
    }
}

/// Every part at any depth of @p assembly gathered into @p out by file: each
/// placement of a subassembly walked, so its parts are counted as often.
void flatten(const AssemblyDocument& assembly,
             std::unordered_map<std::string, std::size_t>& lineByKey, std::vector<BomLine>& out) {
    for (const ComponentInstance& c : assembly.components()) {
        if (c.suppressed) continue;
        if (c.resolvedAssembly) {
            flatten(*c.resolvedAssembly, lineByKey, out);
            continue;
        }
        const std::string key = lineKey(assembly, c);
        const auto it = lineByKey.find(key);
        if (it != lineByKey.end()) {
            ++out[it->second].quantity;
            continue;
        }
        BomLine line;
        line.item = static_cast<int>(out.size()) + 1;
        line.index = std::to_string(line.item);
        line.partName = partDisplayName(c);
        line.partPath = c.partPath;
        line.quantity = 1;
        line.assembly = c.isAssembly();  // one not resolved: a line of its own
        lineByKey.emplace(key, out.size());
        out.push_back(std::move(line));
    }
}

}  // namespace

BillOfMaterials BomGenerator::generate(const AssemblyDocument& assembly, BomKind kind) {
    BillOfMaterials bom;
    switch (kind) {
        case BomKind::TopLevel:
            bom.lines = group(assembly).lines;
            for (auto& line : bom.lines) line.index = std::to_string(line.item);
            break;
        case BomKind::Indented:
            indent(assembly, 0, {}, bom.lines);
            break;
        case BomKind::PartsOnly: {
            std::unordered_map<std::string, std::size_t> lineByKey;
            flatten(assembly, lineByKey, bom.lines);
            break;
        }
    }
    return bom;
}

}  // namespace hz::doc
