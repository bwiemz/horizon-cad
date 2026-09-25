#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "horizon/fileio/ImportReport.h"
#include "horizon/math/Mat4.h"
#include "horizon/topology/Solid.h"

namespace hz::io {

/// How StepFormat writes solids.
struct StepWriteOptions {
    /// Each curved face written once, on the surface its facets stand in
    /// for, bounded by its edges' circles (Phase 151): the part as
    /// designed, as other systems read it. False: each facet as it is,
    /// the part as modelled, which reads back exactly.
    bool asDesigned = true;
};
/// What a write as designed could not do: the curved faces kept in
/// facets, each with why; and, for an assembly, the occurrences left out.
struct StepWriteReport {
    std::vector<std::string> faceted;
    std::vector<std::string> leftOut;
};

/// Where a part is placed in an assembly, read from STEP or written to it
/// (Phase 153).
struct StepOccurrence {
    /// The part placed: an index into the assembly's parts.
    std::size_t part = 0;
    /// What the file calls this occurrence; the part's name when it says
    /// nothing more.
    std::string name;
    /// The part's own coordinates into the assembly's, in millimetres:
    /// rigid, a rotation and a move.
    math::Mat4 transform = math::Mat4::identity();
};

/// A STEP file's parts, each once, and where each is placed (Phase 153).
struct StepAssembly {
    struct Part {
        /// Its product's name, or its shape's when it has no product.
        std::string name;
        /// Its solids, in its own coordinates, in millimetres.
        std::vector<std::unique_ptr<topo::Solid>> bodies;
    };
    std::vector<Part> parts;
    /// Every placement of every part, the assemblies within the assembly
    /// flattened: each carries the placements of all the assemblies above.
    std::vector<StepOccurrence> occurrences;
    /// Whether the file places its parts in an assembly. False for a file
    /// of parts alone: each placed once, where it was drawn.
    bool structured = false;
};

/// A part to write in an assembly (Phase 153): its name and solids.
struct StepWritePart {
    std::string name;
    std::vector<const topo::Solid*> bodies;
};

/// STEP AP242 (ISO 10303-21) import/export of B-Rep solids.
///
/// The writer emits every Horizon solid as one MANIFOLD_SOLID_BREP per shell
/// (grouped in a shared ADVANCED_BREP_SHAPE_REPRESENTATION) whose faces and
/// edges carry exact (rational) B-spline geometry. A curved face, modelled as
/// facets that record their surface, is written as designed (Phase 151): one
/// face on that surface, bounded by circles, where it can be; otherwise as
/// its facets, and reported.  The reader parses a
/// practical AP242 subset: B-spline curves/surfaces (polynomial and rational),
/// LINE and CIRCLE edge geometry (including OCC-style SURFACE_CURVE /
/// SEAM_CURVE wrappers), PLANE / CYLINDRICAL_SURFACE face geometry, and the
/// ADVANCED_FACE same_sense flag (baked into the surface orientation), which
/// covers Horizon round-trips plus typical advanced-B-Rep exports from other
/// CAD systems.
///
/// Assemblies (Phase 153): a file's product structure places its parts.
/// Each PRODUCT_DEFINITION's shape is found through its
/// SHAPE_DEFINITION_REPRESENTATION (and the SHAPE_REPRESENTATION_RELATIONSHIPs
/// from it without a transformation), and each NEXT_ASSEMBLY_USAGE_OCCURRENCE
/// places its part in its assembly by the ITEM_DEFINED_TRANSFORMATION of its
/// CONTEXT_DEPENDENT_SHAPE_REPRESENTATION, assemblies within assemblies
/// compounded. The writer writes an assembly the same way.
///
/// Faces keep their holes (FACE_BOUND). Solids come in in millimetres: the
/// LENGTH_UNIT of each shape representation's context — an SI unit with any
/// prefix, or a conversion-based unit such as the inch — scales them. A
/// solid that cannot be rebuilt is reported and the rest still come in.
///
/// Known limitations (documented, by design of this first slice):
/// - Faces on analytic surfaces import with an untrimmed carrier patch sized
///   to the loop extent (PLANE, spanning the boundary-curve control hulls) or
///   the full closed surface (CYLINDRICAL_SURFACE); non-rectangular planar
///   faces and partial cylindrical faces therefore over-cover visually until
///   surface trimming lands.
/// - BREP_WITH_VOIDS (inner cavities) is not yet mapped.
/// - Curved faces are measured by their vertex polygons (see
///   MassProperties), so a round hole's wall, bounded by a few arcs, adds
///   little to a volume, although the faces around it are drawn with it.
class StepFormat {
public:
    using WriteOptions = StepWriteOptions;
    using WriteReport = StepWriteReport;

    /// Write solids to an AP242 Part-21 file. Returns false on I/O failure or
    /// when @p solids is empty (see lastError()).
    static bool save(const std::string& filePath, const std::vector<const topo::Solid*>& solids,
                     const WriteOptions& options = {}, WriteReport* report = nullptr);

    /// Serialize solids to Part-21 text (the exact bytes save() would write).
    static std::string toString(const std::vector<const topo::Solid*>& solids,
                                const WriteOptions& options = {}, WriteReport* report = nullptr);

    /// Read all MANIFOLD_SOLID_BREP solids from a Part-21 file, in
    /// millimetres whatever the file's length unit. A solid that cannot be
    /// rebuilt is left out and named in @p report; the others still come in.
    /// Returns an empty vector when none can be read (see lastError()).
    ///
    /// @p cancelled, when given, is looked at all the way through: once it
    /// is set, reading stops and nothing is returned (lastError() is
    /// "cancelled"). An import on a worker thread passes its Cancel flag.
    static std::vector<std::unique_ptr<topo::Solid>> load(
        const std::string& filePath, ImportReport* report = nullptr,
        const std::atomic<bool>* cancelled = nullptr);

    /// Parse Part-21 text and reconstruct the solids it holds, as load().
    /// An assembly's parts come in placed, each as many times as the
    /// assembly places it (Phase 153).
    static std::vector<std::unique_ptr<topo::Solid>> fromString(
        const std::string& text, ImportReport* report = nullptr,
        const std::atomic<bool>* cancelled = nullptr);

    /// Read a file's parts, each once, and where its assembly places them
    /// (Phase 153). A file of parts alone places each once, where drawn.
    /// A part none of whose solids can be rebuilt is left out, with its
    /// placements, and named in @p report. No parts when none can be read
    /// (see lastError()); @p cancelled as for load().
    static StepAssembly loadAssembly(const std::string& filePath, ImportReport* report = nullptr,
                                     const std::atomic<bool>* cancelled = nullptr);
    static StepAssembly assemblyFromString(const std::string& text, ImportReport* report = nullptr,
                                           const std::atomic<bool>* cancelled = nullptr);

    /// Write an assembly named @p name (Phase 153): each of @p parts once,
    /// as its own product, and each of @p occurrences as a use of it in the
    /// assembly, placed by its transform. An occurrence whose transform is
    /// not rigid, or whose part has no solid, is left out and said in
    /// @p report. Returns false on I/O failure or when nothing is placed
    /// (see lastError()).
    static bool saveAssembly(const std::string& filePath, const std::string& name,
                             const std::vector<StepWritePart>& parts,
                             const std::vector<StepOccurrence>& occurrences,
                             const WriteOptions& options = {}, WriteReport* report = nullptr);
    /// The Part-21 text saveAssembly() would write.
    static std::string assemblyToString(const std::string& name,
                                        const std::vector<StepWritePart>& parts,
                                        const std::vector<StepOccurrence>& occurrences,
                                        const WriteOptions& options = {},
                                        WriteReport* report = nullptr);

    /// Human-readable description of the most recent load/save failure on this
    /// thread's last call (empty when the call succeeded).
    static const std::string& lastError();
};

}  // namespace hz::io
