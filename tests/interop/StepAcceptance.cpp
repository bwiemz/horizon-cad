#include "StepAcceptance.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepAssemblyFiles.h"
#include "horizon/math/Units.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/topology/Solid.h"

namespace hz::interop {

namespace fs = std::filesystem;
using io::StepFormat;
using model::MassPropertiesCalculator;

using json = nlohmann::json;

namespace {

/// A relative tolerance for the ideal volume: enough to compare with
/// another reader's, quick enough for a corpus.
constexpr double kIdealTolerance = 1e-6;

void include(Bounds& bounds, const topo::Solid& solid) {
    for (const auto& vertex : solid.vertices()) {
        const math::Vec3& p = vertex.point;
        if (bounds.empty) {
            bounds.min = bounds.max = p;
            bounds.empty = false;
            continue;
        }
        bounds.min = math::Vec3(std::min(bounds.min.x, p.x), std::min(bounds.min.y, p.y),
                                std::min(bounds.min.z, p.z));
        bounds.max = math::Vec3(std::max(bounds.max.x, p.x), std::max(bounds.max.y, p.y),
                                std::max(bounds.max.z, p.z));
    }
}

io::StepReadOptions optionsFor(std::optional<math::LengthUnit> unit) {
    io::StepReadOptions options;
    options.unknownLengthUnit = unit;
    return options;
}

json vec(const math::Vec3& v) {
    return json::array({v.x, v.y, v.z});
}

}  // namespace

std::vector<CorpusEntry> readManifest(const fs::path& manifest) {
    std::ifstream in(manifest);
    if (!in) throw std::runtime_error("cannot read " + manifest.string());
    const json root = json::parse(in);
    std::vector<CorpusEntry> entries;
    for (const json& file : root.at("files")) {
        CorpusEntry entry;
        entry.file = file.at("file").get<std::string>();
        if (file.contains("unit_if_unnamed")) {
            entry.unitIfUnnamed =
                math::lengthUnitFrom(file.at("unit_if_unnamed").get<std::string>());
            if (!entry.unitIfUnnamed) {
                throw std::runtime_error(entry.file + ": unit_if_unnamed is not a unit");
            }
        }
        entry.manifest = file;
        entries.push_back(std::move(entry));
    }
    return entries;
}

StepSummary summarize(const fs::path& step, const fs::path& work,
                      std::optional<math::LengthUnit> unitIfUnnamed) {
    StepSummary summary;
    summary.file = step.filename().string();
    const std::string name = step.stem().string();
    std::error_code ec;
    fs::create_directories(work / "export", ec);

    // Read as File ▸ Import ▸ STEP as a New Part reads it.
    io::ImportReport report;
    const auto started = std::chrono::steady_clock::now();
    auto solids = StepFormat::load(step.string(), &report);
    if (solids.empty() && StepFormat::lastLengthUnitUnknown()) {
        summary.unitUnknown = true;
        if (unitIfUnnamed) {
            report = {};
            solids = StepFormat::load(step.string(), &report, nullptr, optionsFor(unitIfUnnamed));
        }
    }
    summary.readMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count();
    summary.skipped = report.skipped;
    summary.approximated = report.approximated;
    summary.converted = report.converted;
    if (solids.empty()) {
        summary.error = StepFormat::lastError();
        return summary;
    }
    // Bodies as OpenCASCADE counts solids: one solid read here holds a
    // compound's several, and a body's cavities are shells of it.
    for (const auto& solid : solids) summary.bodies += model::Pattern::bodyShells(*solid).size();
    summary.onIdealSurfaces = true;
    for (const auto& solid : solids) {
        summary.allValid = summary.allValid && solid->isValid();
        summary.facetedVolume += MassPropertiesCalculator::compute(*solid).volume;
        const auto ideal = MassPropertiesCalculator::computeIdeal(*solid, nullptr, kIdealTolerance);
        summary.idealVolume += ideal.properties.volume;
        summary.onIdealSurfaces = summary.onIdealSurfaces && ideal.onIdealSurfaces;
        // Its box from its facets' corners, which lie on its surfaces: a
        // solid as read has few vertices (a cylinder, two), which bound it
        // short.
        const auto faceted =
            model::describedByCurves(*solid) ? model::facetCurved(*solid) : model::FacetedSolid{};
        include(summary.bounds, faceted.solid ? *faceted.solid : *solid);
    }

    // Sent out again, and read back.
    std::vector<const topo::Solid*> placed;
    placed.reserve(solids.size());
    for (const auto& solid : solids) placed.push_back(solid.get());
    const fs::path exported = work / "export" / (name + ".step");
    summary.exportPath = exported.string();
    if (StepFormat::save(exported.string(), placed)) {
        const auto again = StepFormat::load(exported.string());
        summary.reimportedBodies = 0;
        for (const auto& solid : again) {
            *summary.reimportedBodies += model::Pattern::bodyShells(*solid).size();
            summary.reimportedVolume += MassPropertiesCalculator::compute(*solid).volume;
        }
        if (again.empty()) summary.exportError = StepFormat::lastError();
    } else {
        summary.exportError = StepFormat::lastError();
    }

    // Kept as a part, a body each, saved and read back.
    {
        doc::Document part;
        part.setType(doc::DocumentType::Part);
        for (auto& solid : solids) {
            part.featureTree().addFeature(std::make_unique<doc::ImportedBodyFeature>(
                std::shared_ptr<const topo::Solid>(std::move(solid)), summary.file));
        }
        if (!part.rebuildModel() || part.failedFeatureIndex() != -1) {
            summary.importBuildError = part.lastBuildMessage();
        }
        const fs::path kept = work / (name + ".hzpart");
        doc::Document read;
        std::string error;
        if (!io::NativeFormat::save(kept.string(), part, &error)) {
            summary.reopenError = "not saved: " + error;
        } else if (!io::NativeFormat::load(kept.string(), read, &error)) {
            summary.reopenError = "not read back: " + error;
        } else if (!read.rebuildModel() || read.solid() == nullptr) {
            summary.reopenError = "read back, it does not build: " + read.lastBuildMessage();
        } else {
            summary.reopenedVolume = MassPropertiesCalculator::compute(*read.solid()).volume;
        }
    }

    // Read as an assembly, and kept as one (File ▸ Import ▸ STEP as an
    // Assembly), then read back.
    auto assembly =
        StepFormat::loadAssembly(step.string(), nullptr, nullptr, optionsFor(unitIfUnnamed));
    summary.parts = assembly.parts.size();
    summary.occurrences = assembly.occurrences.size();
    summary.structured = assembly.structured;
    if (!assembly.parts.empty()) {
        const fs::path dir = work / (name + " assembly");
        io::StepAssemblyFiles files;
        std::string error;
        if (!io::saveStepAssembly(assembly, (dir / (name + ".hzasm")).string(),
                                  (dir / "parts").string(), summary.file, &files, &error)) {
            summary.keepError = "not kept: " + error;
        } else {
            doc::AssemblyDocument read;
            if (io::NativeFormat::loadAssembly(files.assembly, read, &error)) {
                summary.keptComponents = read.components().size();
            } else {
                summary.keepError = "not read back: " + error;
            }
        }
    }
    return summary;
}

nlohmann::json toJson(const StepSummary& s) {
    json out = {
        {"file", s.file},
        {"error", s.error},
        {"unitUnknown", s.unitUnknown},
        {"bodies", s.bodies},
        {"allValid", s.allValid},
        {"facetedVolume", s.facetedVolume},
        {"idealVolume", s.idealVolume},
        {"onIdealSurfaces", s.onIdealSurfaces},
        {"skipped", s.skipped},
        {"approximated", s.approximated},
        {"converted", s.converted},
        {"readMs", s.readMs},
        {"parts", s.parts},
        {"occurrences", s.occurrences},
        {"structured", s.structured},
        {"importBuildError", s.importBuildError},
        {"reopenError", s.reopenError},
        {"keepError", s.keepError},
        {"exportPath", s.exportPath},
        {"exportError", s.exportError},
        {"reimportedVolume", s.reimportedVolume},
    };
    out["bounds"] =
        s.bounds.empty ? json(nullptr) : json::array({vec(s.bounds.min), vec(s.bounds.max)});
    out["reopenedVolume"] = s.reopenedVolume ? json(*s.reopenedVolume) : json(nullptr);
    out["keptComponents"] = s.keptComponents ? json(*s.keptComponents) : json(nullptr);
    out["reimportedBodies"] = s.reimportedBodies ? json(*s.reimportedBodies) : json(nullptr);
    return out;
}

}  // namespace hz::interop
