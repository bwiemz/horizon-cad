#include "horizon/fileio/AssemblyStepExport.h"

#include <filesystem>
#include <functional>
#include <map>
#include <utility>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/topology/Solid.h"

namespace hz::io {

namespace {

namespace fs = std::filesystem;

/// The file @p comp places, a relative path taken from @p dir.
std::string partFile(const doc::ComponentInstance& comp, const std::string& dir) {
    fs::path path(comp.partPath);
    if (path.is_relative() && !dir.empty()) path = fs::path(dir) / path;
    return path.lexically_normal().string();
}

/// What @p comp's part is written as: as the part's own export writes it, its exact counterpart
/// where it has one (Document::sharedExactSolid), mirrored for a mirrored component, kept in
/// @p held; else the solid it places (ComponentInstance::solid). A mirror that could not keep the
/// exact geometry (Pattern::transformed's problem; a reflection keeps it) is the component's
/// faceted mirror, as before stage 3, its curved faces said in the write's report.
const topo::Solid* exportedSolid(const doc::ComponentInstance& comp,
                                 std::vector<std::shared_ptr<const topo::Solid>>& held) {
    std::shared_ptr<const topo::Solid> exact =
        comp.resolvedPart ? comp.resolvedPart->sharedExactSolid() : nullptr;
    if (exact == nullptr) return comp.solid();
    if (comp.mirrored) {
        std::string problem;
        std::shared_ptr<const topo::Solid> mirror =
            model::Pattern::transformed(*exact, doc::ComponentInstance::ownMirror(), &problem);
        if (mirror == nullptr || !problem.empty()) return comp.solid();
        exact = std::move(mirror);
    }
    held.push_back(std::move(exact));
    return held.back().get();
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
                out.parts.push_back({std::move(partName), {exportedSolid(comp, out.held)}});
            }
            out.occurrences.push_back({known->second, name, outer * comp.transform});
        };
    for (auto& comp : assembly.components()) {
        if (comp.suppressed) continue;
        // Its part built with its exact counterpart, for the export alone (DocumentManager).
        documents.resolveForExport(comp, dir);
        place(comp, dir, math::Mat4::identity(), comp.name);
    }
    return out;
}

}  // namespace hz::io
