#include "horizon/fileio/AssemblyStepExport.h"

#include <filesystem>
#include <functional>
#include <map>
#include <utility>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/math/Mat4.h"

namespace hz::io {

namespace {

namespace fs = std::filesystem;

/// The file @p comp places, a relative path taken from @p dir.
std::string partFile(const doc::ComponentInstance& comp, const std::string& dir) {
    fs::path path(comp.partPath);
    if (path.is_relative() && !dir.empty()) path = fs::path(dir) / path;
    return path.lexically_normal().string();
}

}  // namespace

AssemblyStepExport gatherAssemblyStep(doc::AssemblyDocument& assembly,
                                      doc::DocumentManager& documents) {
    AssemblyStepExport out;
    // Each part read once, by its file, however many components place it.
    const std::string dir = assembly.filePath().empty()
                                ? std::string()
                                : fs::path(assembly.filePath()).parent_path().string();
    std::map<std::string, std::size_t> partOf;
    // A subassembly (Phase 159) is written as its own parts, each placed by
    // its transform within it composed with the subassembly's: one level.
    std::function<void(const doc::ComponentInstance&, const std::string&, const math::Mat4&,
                       const std::string&)>
        place = [&](const doc::ComponentInstance& comp, const std::string& in,
                    const math::Mat4& outer, const std::string& name) {
            const std::string file = partFile(comp, in);
            if (comp.resolvedAssembly) {
                const std::string subDir = fs::path(file).parent_path().string();
                for (const auto& child : comp.resolvedAssembly->components()) {
                    if (!child.suppressed) {
                        place(child, subDir, outer * comp.transform, name + "/" + child.name);
                    }
                }
                return;
            }
            const topo::Solid* solid = comp.solid();
            if (solid == nullptr) {
                out.unread.push_back(name);
                return;
            }
            // A mirrored one (Phase 162) is a part of its own: its mirror.
            const auto [known, added] =
                partOf.emplace(comp.mirrored ? file + "#mirrored" : file, out.parts.size());
            if (added) {
                std::string partName = fs::path(file).stem().string();
                if (comp.mirrored) partName += " (mirrored)";
                out.parts.push_back({std::move(partName), {solid}});
            }
            out.occurrences.push_back({known->second, name, outer * comp.transform});
        };
    for (auto& comp : assembly.components()) {
        if (comp.suppressed) continue;
        if (comp.solid() == nullptr) {
            documents.resolveComponent(comp, doc::ComponentState::Resolved, dir);
        }
        place(comp, dir, math::Mat4::identity(), comp.name);
    }
    return out;
}

}  // namespace hz::io
