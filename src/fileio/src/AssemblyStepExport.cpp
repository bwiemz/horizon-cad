#include "horizon/fileio/AssemblyStepExport.h"

#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <utility>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/fileio/NativeFormat.h"
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

/// What @p comp's part, in @p file, is written from (AssemblyStepJob::Source), added to
/// @p sources; none where it cannot be read. The part open (the component's, or a tab's), built
/// as it is written (not needsBuild: a tab's, with its counterpart, or one with none by design),
/// is written as it is: its counterpart shared, or a copy of its faceted solid, which the window
/// may build again while the export is finished. One built without its counterpart (for its
/// components alone) or not since its features changed is a snapshot of it, to be built; one not
/// open, its file read again into a copy. The part is never built here.
std::optional<std::size_t> readSource(const doc::ComponentInstance& comp, const std::string& file,
                                      doc::DocumentManager& documents,
                                      std::vector<AssemblyStepJob::Source>& sources) {
    AssemblyStepJob::Source source;
    const std::shared_ptr<doc::Document> open =
        comp.resolvedPart ? comp.resolvedPart : documents.findByPath(file);
    if (open && !open->needsBuild()) {
        source.ready = open->sharedExactSolid();
        if (!source.ready && open->solid() != nullptr) {
            source.ready = model::Pattern::transformed(*open->solid(), math::Mat4::identity());
        }
    } else if (open) {
        SharedImports imports;
        const std::string snapshot = NativeFormat::documentToJson(*open, false, &imports);
        auto copy = std::make_shared<doc::Document>();
        if (NativeFormat::documentFromJson(snapshot, *copy, nullptr, nullptr, &imports)) {
            source.copy = std::move(copy);
            source.previous = open->exactCounterpart();
        }
    } else {
        auto copy = std::make_shared<doc::Document>();
        if (documents.readPart(file, *copy)) source.copy = std::move(copy);
    }
    if (!source.ready && !source.copy) return std::nullopt;
    sources.push_back(std::move(source));
    return sources.size() - 1;
}

}  // namespace

std::size_t AssemblyStepJob::builds() const {
    std::size_t copies = 0;
    for (const Source& source : sources) copies += source.copy ? 1 : 0;
    return copies;
}

AssemblyStepJob prepareAssemblyStep(doc::AssemblyDocument& assembly,
                                    doc::DocumentManager& documents) {
    AssemblyStepJob job;
    const std::string dir = assembly.filePath().empty()
                                ? std::string()
                                : fs::path(assembly.filePath()).parent_path().string();
    // Each part read once, by its file, however many components place it; written once, and its
    // mirror once (Phase 162: a part of its own).
    std::map<std::string, std::optional<std::size_t>> sourceOf;
    std::map<std::string, std::size_t> partOf;
    // A subassembly (Phase 159) is written as its own parts, each placed by its transform within
    // it composed with the subassembly's: one level.
    std::function<void(doc::ComponentInstance&, const std::string&, const math::Mat4&,
                       const std::string&)>
        place = [&](doc::ComponentInstance& comp, const std::string& in, const math::Mat4& outer,
                    const std::string& name) {
            const std::string file = partFile(comp, in);
            if (comp.isAssembly()) {
                if (!comp.resolvedAssembly) {
                    documents.resolveComponent(comp, doc::ComponentState::Lightweight, in);
                }
                if (!comp.resolvedAssembly) {
                    job.unread.push_back(name);
                    return;
                }
                const std::string subDir = fs::path(file).parent_path().string();
                for (auto& child : comp.resolvedAssembly->components()) {
                    if (!child.suppressed) {
                        place(child, subDir, outer * comp.transform, name + "/" + child.name);
                    }
                }
                return;
            }
            auto known = sourceOf.find(file);
            if (known == sourceOf.end()) {
                known =
                    sourceOf.emplace(file, readSource(comp, file, documents, job.sources)).first;
            }
            const std::optional<std::size_t>& source = known->second;
            if (!source) {
                job.unread.push_back(name);
                return;
            }
            const auto placed =
                partOf.emplace(comp.mirrored ? file + "#mirrored" : file, job.parts.size());
            if (placed.second) {
                std::string partName = fs::path(file).stem().string();
                if (comp.mirrored) partName += " (mirrored)";
                job.parts.push_back({*source, comp.mirrored, std::move(partName)});
            }
            job.occurrences.push_back({placed.first->second, name, outer * comp.transform});
        };
    for (auto& comp : assembly.components()) {
        if (!comp.suppressed) place(comp, dir, math::Mat4::identity(), comp.name);
    }
    return job;
}

AssemblyStepExport finishAssemblyStep(AssemblyStepJob job, const std::atomic<bool>* cancelled) {
    // Each source's solid: as it is, or its copy built with its exact counterpart, and the
    // counterpart's solid, else the copy's faceted solid (kept by the copy, the export's own).
    std::vector<std::shared_ptr<const topo::Solid>> solids(job.sources.size());
    for (std::size_t k = 0; k < job.sources.size(); ++k) {
        if (cancelled != nullptr && cancelled->load()) return {};
        AssemblyStepJob::Source& source = job.sources[k];
        if (source.ready) {
            solids[k] = std::move(source.ready);
            continue;
        }
        doc::Document& copy = *source.copy;
        doc::BuildOptions options;
        options.previous = source.previous;
        copy.applyBuild(copy.buildWithDiagnostics(nullptr, options));
        if (auto exact = copy.sharedExactSolid()) {
            solids[k] = std::move(exact);
        } else if (copy.solid() != nullptr) {
            solids[k] = std::shared_ptr<const topo::Solid>(source.copy, copy.solid());
        }
    }
    AssemblyStepExport out;
    out.unread = std::move(job.unread);
    // Each part, mirrored as its components are; one that cannot be had is left out.
    std::vector<std::optional<std::size_t>> written(job.parts.size());
    for (std::size_t p = 0; p < job.parts.size(); ++p) {
        const AssemblyStepJob::Part& part = job.parts[p];
        std::shared_ptr<const topo::Solid> solid = solids[part.source];
        if (!solid) continue;
        if (part.mirrored) {
            // A reflection keeps the exact geometry (Pattern::transformed); a copy that lost it is
            // left out, not written in its place.
            std::string problem;
            std::shared_ptr<const topo::Solid> mirror =
                model::Pattern::transformed(*solid, doc::ComponentInstance::ownMirror(), &problem);
            if (!mirror || !problem.empty()) continue;
            solid = std::move(mirror);
        }
        written[p] = out.parts.size();
        out.parts.push_back({part.name, {solid.get()}});
        out.held.push_back(std::move(solid));
    }
    for (const StepOccurrence& occurrence : job.occurrences) {
        const std::optional<std::size_t>& at = written[occurrence.part];
        if (!at) {
            out.unread.push_back(occurrence.name);
            continue;
        }
        out.occurrences.push_back({*at, occurrence.name, occurrence.transform});
    }
    return out;
}

AssemblyStepExport gatherAssemblyStep(doc::AssemblyDocument& assembly,
                                      doc::DocumentManager& documents) {
    return finishAssemblyStep(prepareAssemblyStep(assembly, documents));
}

}  // namespace hz::io
