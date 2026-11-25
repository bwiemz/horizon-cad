#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "horizon/fileio/ImportReport.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Units.h"
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
/// How StepFormat reads a file.
struct StepReadOptions {
    /// The unit a file's lengths are read in where it names none, or one
    /// this reader cannot read. None: such a file is not read (lastError()
    /// says why, and lastLengthUnitUnknown() is true), so that its user can
    /// say which unit it is in; taken as millimetres, a file in inches came
    /// in 25.4 times too small, with only a warning to say so.
    std::optional<math::LengthUnit> unknownLengthUnit;
    /// For STEP embedded in native part files only (an imported body, saved before the exact
    /// kernel read its faces): a solid whose exact form cannot be made is kept as it was read
    /// before, its carriers alone and no exact fields, rather than left out, and the read's
    /// ImportReport::converted says "solid <i>: not exact (<why>); kept as it was". A STEP
    /// import never sets it: what cannot be exact is left out with why (decision 1b). Facet
    /// approximations are not reported again: the import that made the body reported them.
    bool keepWhenNotExact = false;
};

/// What a write as designed could not do: the curved faces kept in
/// facets, each with why, and the exact faces and edges not written as their
/// kind (a surface or curve whose numbers no STEP entity holds), each with
/// why and what took its place; and, for an assembly, the occurrences left
/// out.
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
/// (grouped in a shared ADVANCED_BREP_SHAPE_REPRESENTATION). A face with an
/// exact surface goes out as its kind: a PLANE, CYLINDRICAL_SURFACE,
/// CONICAL_SURFACE, SPHERICAL_SURFACE or TOROIDAL_SURFACE placed by its frame
/// (written as the geometry it makes: STEP's placements are right-handed and
/// carry no parameter scale), or its B-spline; an edge with an exact curve as
/// a LINE (its VECTOR's magnitude the line's scale), CIRCLE, ELLIPSE or
/// B-spline, a closed edge running with its curve and an open one on a
/// B-spline over its own span of it. No PCURVE is written. A flat face with
/// none goes out on its own PLANE; another face with none on its carrier
/// B-spline. A curved face, modelled as facets that record their surface, is
/// written as designed (Phase 151): one face on that surface, bounded by
/// circles, where it can be; otherwise as its facets, and reported. Each
/// face's same_sense says truly which way it faces, and a body wound facing
/// in (as the kernel's box is) goes out turned round, facing out.  The reader parses a
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
/// Exact faces (the exact kernel, stage 2): each solid comes in with its exact form beside its
/// carriers (Face::surface and Edge::curve, unchanged). A face keeps the surface its file gives
/// it: a PLANE, CYLINDRICAL_SURFACE, CONICAL_SURFACE, SPHERICAL_SURFACE or TOROIDAL_SURFACE as
/// that surface (STEP's parametrisations are the kernel's), and a B_SPLINE_SURFACE as that very
/// B-spline, never recognised as a quadric. A SURFACE_OF_LINEAR_EXTRUSION is the one exception:
/// with no typed extrusion yet, it comes in as its B-spline carrier. Its surface is never turned
/// round: Face::reversed says which way the face faces. The reader's rule for it: on a surface
/// where a loop determines the region it bounds (a plane, a cylinder, a cone, a swept surface's
/// B-spline, a B-spline neither closed both ways nor coming to a pole), the face faces the way
/// its outer loop runs, counter-clockwise in (u, v) about its outward normal: the loops are the
/// topology and same_sense only repeats it, and Horizon's own writer wrote same_sense .T.
/// whichever way a face's surface faced. On a sphere, a torus or a B-spline closed both ways or
/// with a pole, a loop bounds two regions, so the face takes !same_sense with its shell's and
/// bounds' flags; where its loop runs the other way, the face is the surface outside its loop
/// (a ball's skin round a flat: it holds a pole or wraps the surface), which this stage cannot
/// represent, and the solid is left out with why. A loop whose area says nothing (a seam joined
/// at poles alone) leaves the face as its flags say. An edge keeps its curve (a LINE at its
/// VECTOR's magnitude, a CIRCLE, an ELLIPSE, a B-spline) over the range its vertices invert to,
/// the way its same_sense says, a closed edge a whole period. Each half-edge's pcurve is built
/// (geo::pcurveOn), never read. Tolerances: each pcurve is fitted to the model resolution of the
/// solid's size, or to its curve's own stand-off from its surface where that is larger (measured,
/// never chased); each edge's and vertex's tolerance grows to cover how far the file has it
/// stray (its gaps, its stand-off), as per-entity tolerances do, up to a thousandth of the
/// solid's size, and the report says how exact a solid so read is. The file's
/// UNCERTAINTY_MEASURE says only how near a closed B-spline curve's ends must be to join (and a
/// vertex to be at the join), no coarser than a thousandth of the curve. A solid whose exact form
/// cannot be made, strays beyond the limit, or does not hold together (topo::validateExact) is
/// left out with why: nothing faceted comes in for it (StepReadOptions::keepWhenNotExact aside,
/// for a native file's embedded STEP). A solid's shell written inside out (an outer shell whose
/// faces face into it, consistently: the signed volume it encloses, integrated on its exact
/// faces, is negative; or a void's facing out of the void) is turned round as it is read, its
/// loops run the other way and its faces reversed, so that every solid comes in facing out and
/// every void into itself.
///
/// Faces keep their holes (FACE_BOUND). Solids come in in millimetres: the
/// LENGTH_UNIT of each shape representation's context — an SI unit with any
/// prefix, or a conversion-based unit such as the inch — scales them. A file
/// that names none, or one this reader cannot read, is read only in a unit
/// its reader is told (StepReadOptions): a guess would give every length
/// the wrong size. A solid that cannot be rebuilt is reported and the rest
/// still come in. A solid with voids (BREP_WITH_VOIDS) comes in as a body
/// with cavities, each a shell facing into its void, and goes out as one.
///
/// Known limitations (documented, by design of this first slice):
/// - Faces on analytic surfaces import with an untrimmed carrier patch sized
///   to the loop extent (PLANE, spanning the boundary-curve control hulls) or
///   the full closed surface (CYLINDRICAL_SURFACE); non-rectangular planar
///   faces and partial cylindrical faces therefore over-cover visually until
///   surface trimming lands.
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
    /// millimetres whatever the file's length unit, each with its exact form
    /// (see above). A solid that cannot be rebuilt, or made exact, is left out
    /// and named in @p report; the others still come in.
    /// Returns an empty vector when none can be read (see lastError()).
    ///
    /// @p cancelled, when given, is looked at all the way through: once it
    /// is set, reading stops and nothing is returned (lastError() is
    /// "cancelled"). An import on a worker thread passes its Cancel flag.
    static std::vector<std::unique_ptr<topo::Solid>> load(
        const std::string& filePath, ImportReport* report = nullptr,
        const std::atomic<bool>* cancelled = nullptr, const StepReadOptions& options = {});

    /// Parse Part-21 text and reconstruct the solids it holds, as load().
    /// An assembly's parts come in placed, each as many times as the
    /// assembly places it (Phase 153).
    static std::vector<std::unique_ptr<topo::Solid>> fromString(
        const std::string& text, ImportReport* report = nullptr,
        const std::atomic<bool>* cancelled = nullptr, const StepReadOptions& options = {});

    /// Read a file's parts, each once, and where its assembly places them
    /// (Phase 153). A file of parts alone places each once, where drawn.
    /// A part none of whose solids can be rebuilt is left out, with its
    /// placements, and named in @p report. No parts when none can be read
    /// (see lastError()); @p cancelled as for load().
    static StepAssembly loadAssembly(const std::string& filePath, ImportReport* report = nullptr,
                                     const std::atomic<bool>* cancelled = nullptr,
                                     const StepReadOptions& options = {});
    static StepAssembly assemblyFromString(const std::string& text, ImportReport* report = nullptr,
                                           const std::atomic<bool>* cancelled = nullptr,
                                           const StepReadOptions& options = {});

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
    /// Whether this thread's last read failed for its length unit alone: the
    /// file names none, or one this reader cannot read, and no
    /// StepReadOptions::unknownLengthUnit was given. Read again with one.
    static bool lastLengthUnitUnknown();
};

}  // namespace hz::io
