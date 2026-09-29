#pragma once

#include <cstddef>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "horizon/fileio/StepFormat.h"
#include "horizon/math/Units.h"
#include "horizon/math/Vec3.h"

namespace hz::interop {

/// A box in millimetres; empty when nothing was read.
struct Bounds {
    math::Vec3 min;
    math::Vec3 max;
    bool empty = true;
};

/// What reading one STEP file from another CAD system gave, and what became
/// of it through Horizon CAD: kept as a part and read back, kept as an
/// assembly of part files and read back, and sent out as STEP again and read
/// back. The external-file acceptance suite (tests/interop/step) compares it
/// with its manifest, and tools/interop/occt_check.py with what OpenCASCADE,
/// an independent reader, finds in the same file and in the export.
struct StepSummary {
    std::string file;  ///< its name in the corpus

    // --- Read as a part (StepFormat::load): each placement a body ---
    /// Why it was not read at all; empty when it was.
    std::string error;
    /// It names no length unit this version reads (it was read in the unit
    /// the manifest gives, or not at all).
    bool unitUnknown = false;
    std::size_t bodies = 0;
    bool allValid = true;          ///< every body passes Solid::isValid()
    double facetedVolume = 0.0;    ///< mm³, as built in facets
    double idealVolume = 0.0;      ///< mm³, on the surfaces the facets record
    bool onIdealSurfaces = false;  ///< every body measured on its ideals
    Bounds bounds;                 ///< of the facets' corners
    std::vector<std::string> skipped;
    std::vector<std::string> approximated;
    std::vector<std::string> converted;
    double readMs = 0.0;

    // --- Read as an assembly (StepFormat::loadAssembly) ---
    std::size_t parts = 0;
    std::size_t occurrences = 0;
    bool structured = false;  ///< placed by an assembly, not parts alone

    // --- Through Horizon CAD ---
    /// Kept as a part (a body each, as File ▸ Import ▸ STEP does) and
    /// built: why that failed, empty when it did not.
    std::string importBuildError;
    /// That part saved, read back and built: its volume then. Empty when it
    /// could not be.
    std::optional<double> reopenedVolume;
    std::string reopenError;
    /// Kept as an assembly of part files (saveStepAssembly), read back:
    /// its components. Empty when it could not be.
    std::optional<std::size_t> keptComponents;
    std::string keepError;
    /// Sent out as STEP (the bodies, placed) to this path, and read back.
    std::string exportPath;
    std::optional<std::size_t> reimportedBodies;
    double reimportedVolume = 0.0;  ///< faceted, mm³
    std::string exportError;
    /// The curved faces the export kept in facets, each with why (File ▸
    /// Export's report): an export so kept measures as its facets there.
    std::vector<std::string> exportFaceted;
};

/// One file of the corpus, as its manifest (manifest.json) describes it:
/// where it came from, its licence, and what reading it must give.
struct CorpusEntry {
    std::string file;
    /// The unit to read it in when it names none this version reads.
    std::optional<math::LengthUnit> unitIfUnnamed;
    nlohmann::json manifest;  ///< its whole entry
};

/// The entries of @p manifest, in order. Throws when it cannot be read.
std::vector<CorpusEntry> readManifest(const std::filesystem::path& manifest);

/// Read @p step as all of the above, writing what it keeps under @p work
/// (made if need be; the export goes to <work>/export/<name>.step). A file
/// naming no unit this version reads is read in @p unitIfUnnamed when given.
StepSummary summarize(const std::filesystem::path& step, const std::filesystem::path& work,
                      std::optional<math::LengthUnit> unitIfUnnamed = std::nullopt);

nlohmann::json toJson(const StepSummary& summary);

}  // namespace hz::interop
