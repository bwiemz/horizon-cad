#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "horizon/fileio/ImportReport.h"
#include "horizon/topology/Solid.h"

namespace hz::io {

/// STEP AP242 (ISO 10303-21) import/export of B-Rep solids.
///
/// The writer emits every Horizon solid as one MANIFOLD_SOLID_BREP per shell
/// (grouped in a shared ADVANCED_BREP_SHAPE_REPRESENTATION) whose faces and
/// edges carry exact (rational) B-spline geometry — Horizon's kernel is
/// uniformly NURBS-backed, so the export is lossless.  The reader parses a
/// practical AP242 subset: B-spline curves/surfaces (polynomial and rational),
/// LINE and CIRCLE edge geometry (including OCC-style SURFACE_CURVE /
/// SEAM_CURVE wrappers), PLANE / CYLINDRICAL_SURFACE face geometry, and the
/// ADVANCED_FACE same_sense flag (baked into the surface orientation), which
/// covers Horizon round-trips plus typical advanced-B-Rep exports from other
/// CAD systems.
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
/// - BREP_WITH_VOIDS (inner cavities) and assembly product structure are not
///   yet mapped.
/// - Curved faces are measured by their vertex polygons (see
///   MassProperties), so a round hole's wall, bounded by a few arcs, adds
///   little to a volume, although the faces around it are drawn with it.
class StepFormat {
public:
    /// Write solids to an AP242 Part-21 file. Returns false on I/O failure or
    /// when @p solids is empty (see lastError()).
    static bool save(const std::string& filePath, const std::vector<const topo::Solid*>& solids);

    /// Serialize solids to Part-21 text (the exact bytes save() would write).
    static std::string toString(const std::vector<const topo::Solid*>& solids);

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
    static std::vector<std::unique_ptr<topo::Solid>> fromString(
        const std::string& text, ImportReport* report = nullptr,
        const std::atomic<bool>* cancelled = nullptr);

    /// Human-readable description of the most recent load/save failure on this
    /// thread's last call (empty when the call succeeded).
    static const std::string& lastError();
};

}  // namespace hz::io
