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

/// One solid of a file: one Horizon CAD read (each placement of a part its own), or one the read
/// left out because its exact form could not be made (nothing faceted comes in for it).
struct SolidReading {
    /// What the read calls it: "solid <n>", the file's n-th solid (a part placed again is
    /// numbered after the file's own); or, for a file none of whose solids is read, "solid #<id>",
    /// the entity its error names (the first left out: the others are named once it reads it).
    std::string solid;
    /// It came in with its exact form: every face on its exact surface.
    bool exact = false;
    /// mm³, integrated on its exact faces (MassPropertiesCalculator::compute); 0 when not exact.
    double exactVolume = 0.0;
    /// Why it is not exact, as the read says; or why its exact volume could not be integrated.
    /// Empty when neither. Where it is not, the solid is one for the corpus's regression list
    /// (the manifest's exact_failures).
    std::string exactProblem;
    /// The exact volume's own estimate of its relative error (computeIdeal's estimatedError at
    /// its default tolerance, the integration compute() makes): the quadrature's, and what its
    /// edges standing off their surfaces leave uncertain. 0 when not exact.
    double exactError = 0.0;
    /// The read point's mesh of an exact solid (BoundaryMesh::extractFacePolygons at a chord
    /// tolerance of 1e-4 of its size): its volume, mm³; the most that may differ from
    /// exactVolume, its area times that tolerance; and the face it could not mesh, with why
    /// (empty when it meshed every face).
    double meshVolume = 0.0;
    double meshAllowance = 0.0;
    std::string meshProblem;
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
    bool allValid = true;  ///< every body passes Solid::isValid()
    /// mm³, as MassPropertiesCalculator::compute measures the bodies as read: integrated on
    /// their exact faces where they have them, else in facets.
    double facetedVolume = 0.0;
    /// mm³, as computeIdeal measures them: on their exact faces where they have them, else on
    /// the surfaces the facets record.
    double idealVolume = 0.0;
    bool onIdealSurfaces = false;  ///< every body measured on its ideals
    Bounds bounds;                 ///< of the facets' corners
    /// Each solid read, in the order read, then each left out for want of its exact form.
    std::vector<SolidReading> solids;
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
    /// That part saved, read back and built: its volume then, as the part is
    /// measured (Document::exactOrFacetedSolid: its exact counterpart where it
    /// has one). Empty when it could not be.
    std::optional<double> reopenedVolume;
    /// The same part's faceted copy (Document::solid()), which features and
    /// Booleans work on: its volume.
    std::optional<double> reopenedFacetedVolume;
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

/// mm³: the exact volume of what @p summary read, its exact solids' exactVolume summed.
double exactVolume(const StepSummary& summary);

/// The relative error @p summary's exact volume is known to: its exact solids' own estimates
/// (SolidReading::exactError), each weighed by its volume.
double exactError(const StepSummary& summary);

/// Where @p summary disagrees with the corpus's regression list, @p manifest's exact_failures
/// (each {"solid", "why"}, as the read names the solid and says why): a solid not exact, or
/// whose exact volume could not be integrated, that the list does not name with that why; and a
/// listed one that is exact now, or fails for another reason. One line each; none when they
/// agree.
std::vector<std::string> checkExactFailures(const StepSummary& summary,
                                            const nlohmann::json& manifest);

/// Where an exact solid's read-point mesh (SolidReading::meshVolume) differs from its exact
/// volume by more than its allowance, or could not be made. One line each; none when every one
/// agrees.
std::vector<std::string> checkMeshAgainstMass(const StepSummary& summary);

}  // namespace hz::interop
