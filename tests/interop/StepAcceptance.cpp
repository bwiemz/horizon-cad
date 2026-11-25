#include "StepAcceptance.h"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
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
#include "horizon/modeling/BoundaryMesh.h"
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

/// Whether every live face of @p solid is on its exact surface (and it has one).
bool whollyExact(const topo::Solid& solid) {
    bool any = false;
    for (const auto& face : solid.faces()) {
        if (!face.alive) continue;
        if (!face.exactSurface) return false;
        any = true;
    }
    return any;
}

/// "solid <n>", as the read names @p solid: its faces are named step/solid:<n - 1>/…. Empty
/// when they are not.
std::string readName(const topo::Solid& solid) {
    static const std::string prefix = "step/solid:";
    for (const auto& face : solid.faces()) {
        if (!face.alive) continue;
        const std::string& tag = face.topoId.tag();
        if (tag.rfind(prefix, 0) != 0) continue;
        const char* first = tag.data() + prefix.size();
        const char* last = tag.data() + tag.size();
        std::size_t index = 0;
        const auto [end, error] = std::from_chars(first, last, index);
        if (error == std::errc() && end != first) return "solid " + std::to_string(index + 1);
    }
    return {};
}

/// The solids a read left out for want of their exact form, each named as the read names it,
/// with why: from its @p error when it read none ("solid #<id> could not be made exact:
/// <why>"), else from its report's @p skipped items ("solid <n>: <why>"; one that could not be
/// rebuilt at all is "solid <n> (#<id>): …", and not among them).
std::vector<SolidReading> leftOutForExactness(const std::string& error,
                                              const std::vector<std::string>& skipped) {
    std::vector<SolidReading> out;
    const auto add = [&out](std::string solid, std::string why) {
        SolidReading reading;
        reading.solid = std::move(solid);
        reading.exactProblem = std::move(why);
        out.push_back(std::move(reading));
    };
    static const std::string notExact = " could not be made exact: ";
    if (const auto at = error.find(notExact); error.rfind("solid #", 0) == 0 && at != error.npos) {
        add(error.substr(0, at), error.substr(at + notExact.size()));
    }
    static const std::string solidN = "solid ";
    for (const std::string& item : skipped) {
        if (item.rfind(solidN, 0) != 0) continue;
        const auto digits = item.find_first_not_of("0123456789", solidN.size());
        if (digits == solidN.size() || digits == item.npos || item.compare(digits, 2, ": ") != 0) {
            continue;
        }
        add(item.substr(0, digits), item.substr(digits + 2));
    }
    return out;
}

/// @p solid as read: whether exact, its exact volume (from @p mass, compute()'s of it), and its
/// read-point mesh against it.
SolidReading reading(const topo::Solid& solid, const model::MassProperties& mass) {
    SolidReading out;
    out.solid = readName(solid);
    out.exact = whollyExact(solid);
    if (!out.exact) {
        out.exactProblem = "read without its exact form";
        return out;
    }
    if (!mass.valid) {
        out.exactProblem = "its exact volume could not be integrated: " + mass.problem;
        return out;
    }
    out.exactVolume = mass.volume;
    // The same integration's own estimate of its error (computeIdeal at its default tolerance
    // integrates an exact solid as compute() does).
    out.exactError = MassPropertiesCalculator::computeIdeal(solid).estimatedError;
    // At the read point's own tolerance, where each triangle lies within it of the surface: the
    // mesh's volume within its area times that of the exact one.
    const double tolerance = model::BoundaryMesh::defaultChordTolerance(solid);
    const auto mesh =
        model::BoundaryMesh::extractFacePolygons(solid, tolerance, &out.meshProblem, nullptr);
    out.meshVolume = model::BoundaryMesh::signedVolume(mesh);
    out.meshAllowance = mass.surfaceArea * tolerance;
    return out;
}

json toJson(const SolidReading& s) {
    return {
        {"solid", s.solid},
        {"exact", s.exact},
        {"exactVolume", s.exactVolume},
        {"exactProblem", s.exactProblem},
        {"exactError", s.exactError},
        {"meshVolume", s.meshVolume},
        {"meshAllowance", s.meshAllowance},
        {"meshProblem", s.meshProblem},
    };
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
        summary.solids = leftOutForExactness(summary.error, {});
        return summary;
    }
    // Bodies as OpenCASCADE counts solids: one solid read here holds a
    // compound's several, and a body's cavities are shells of it.
    for (const auto& solid : solids) summary.bodies += model::Pattern::bodyShells(*solid).size();
    summary.onIdealSurfaces = true;
    for (const auto& solid : solids) {
        summary.allValid = summary.allValid && solid->isValid();
        const auto mass = MassPropertiesCalculator::compute(*solid);
        summary.facetedVolume += mass.volume;
        summary.solids.push_back(reading(*solid, mass));
        if (summary.solids.back().solid.empty()) {
            summary.solids.back().solid = "solid read " + std::to_string(summary.solids.size());
        }
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
    for (auto& out : leftOutForExactness({}, report.skipped)) summary.solids.push_back(out);

    // Kept as a part, a body each, as File ▸ Import ▸ STEP as a New Part
    // keeps it.
    doc::Document part;
    part.setType(doc::DocumentType::Part);
    for (auto& solid : solids) {
        part.featureTree().addFeature(std::make_unique<doc::ImportedBodyFeature>(
            std::shared_ptr<const topo::Solid>(std::move(solid)), summary.file));
    }
    if (!part.rebuildModel() || part.failedFeatureIndex() != -1) {
        summary.importBuildError = part.lastBuildMessage();
    }

    // Sent out as File ▸ Export ▸ STEP sends it, the part's model: its exact
    // counterpart, the bodies as read, where it has one (stage 2), else its
    // faceted copy (its curved faces on their surfaces, as designed); and
    // read back. The solids as read went out before, a path no user takes, a
    // band round a hole written without the seam other readers need (#184).
    const fs::path exported = work / "export" / (name + ".step");
    summary.exportPath = exported.string();
    if (!summary.importBuildError.empty() || part.solid() == nullptr) {
        summary.exportError = "not exported: the part does not build";
    } else if (StepFormat::WriteReport report;
               StepFormat::save(exported.string(), {part.exactOrFacetedSolid()}, {}, &report)) {
        summary.exportFaceted = report.faceted;
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

    // Saved as a part file and read back: measured as the part is shown, measured and exported
    // (its exact counterpart), and its faceted copy apart.
    {
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
            summary.reopenedVolume =
                MassPropertiesCalculator::compute(*read.exactOrFacetedSolid()).volume;
            summary.reopenedFacetedVolume = MassPropertiesCalculator::compute(*read.solid()).volume;
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
    out["reopenedFacetedVolume"] =
        s.reopenedFacetedVolume ? json(*s.reopenedFacetedVolume) : json(nullptr);
    out["keptComponents"] = s.keptComponents ? json(*s.keptComponents) : json(nullptr);
    out["reimportedBodies"] = s.reimportedBodies ? json(*s.reimportedBodies) : json(nullptr);
    out["exportFaceted"] = s.exportFaceted;
    out["solids"] = json::array();
    for (const SolidReading& solid : s.solids) out["solids"].push_back(toJson(solid));
    return out;
}

double exactVolume(const StepSummary& summary) {
    double volume = 0.0;
    for (const SolidReading& solid : summary.solids) {
        if (solid.exact) volume += solid.exactVolume;
    }
    return volume;
}

double exactError(const StepSummary& summary) {
    double volume = 0.0;
    double error = 0.0;
    for (const SolidReading& solid : summary.solids) {
        if (!solid.exact) continue;
        volume += std::abs(solid.exactVolume);
        error += solid.exactError * std::abs(solid.exactVolume);
    }
    return volume > 0.0 ? error / volume : 0.0;
}

std::vector<std::string> checkExactFailures(const StepSummary& summary, const json& manifest) {
    // The list as the manifest gives it: each entry once, matched or not.
    struct Listed {
        std::string solid;
        std::string why;
        bool matched = false;
    };
    std::vector<Listed> listed;
    if (manifest.contains("exact_failures")) {
        for (const json& entry : manifest["exact_failures"]) {
            listed.push_back(
                {entry.at("solid").get<std::string>(), entry.at("why").get<std::string>()});
        }
    }
    std::vector<std::string> problems;
    for (const SolidReading& solid : summary.solids) {
        if (solid.exactProblem.empty()) continue;
        const auto same = std::find_if(listed.begin(), listed.end(), [&solid](const Listed& l) {
            return l.solid == solid.solid;
        });
        if (same == listed.end()) {
            problems.push_back(
                solid.solid +
                " is not exact, and exact_failures does not list it: " + solid.exactProblem);
            continue;
        }
        same->matched = true;
        if (solid.exactProblem.find(same->why) == std::string::npos) {
            problems.push_back(solid.solid + " fails for another reason now: " +
                               solid.exactProblem + " (exact_failures says: " + same->why + ")");
        }
    }
    for (const Listed& l : listed) {
        if (!l.matched) {
            problems.push_back(l.solid + " reads exact now: take it out of exact_failures (" +
                               l.why + ")");
        }
    }
    return problems;
}

std::vector<std::string> checkMeshAgainstMass(const StepSummary& summary) {
    std::vector<std::string> problems;
    for (const SolidReading& solid : summary.solids) {
        if (!solid.exact || !solid.exactProblem.empty()) continue;
        if (!solid.meshProblem.empty()) {
            problems.push_back(solid.solid +
                               ": the read point could not mesh it: " + solid.meshProblem);
            continue;
        }
        const double apart = solid.meshVolume - solid.exactVolume;
        if (!(std::abs(apart) <= solid.meshAllowance)) {
            std::ostringstream line;
            line.precision(12);
            line << solid.solid << ": its read-point mesh measures " << solid.meshVolume
                 << " mm³, its exact volume " << solid.exactVolume << ": " << apart
                 << " apart, beyond the mesh's allowance of " << solid.meshAllowance;
            problems.push_back(line.str());
        }
    }
    return problems;
}

}  // namespace hz::interop
