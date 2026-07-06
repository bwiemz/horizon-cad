#pragma once

#include <memory>
#include <string>
#include <vector>

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
/// Known limitations (documented, by design of this first slice):
/// - Faces on analytic surfaces import with an untrimmed carrier patch sized
///   to the loop extent (PLANE, spanning the boundary-curve control hulls) or
///   the full closed surface (CYLINDRICAL_SURFACE); non-rectangular planar
///   faces and partial cylindrical faces therefore over-cover visually until
///   surface trimming lands.
/// - BREP_WITH_VOIDS (inner cavities) and assembly product structure are not
///   yet mapped.
class StepFormat {
public:
    /// Write solids to an AP242 Part-21 file. Returns false on I/O failure or
    /// when @p solids is empty (see lastError()).
    static bool save(const std::string& filePath, const std::vector<const topo::Solid*>& solids);

    /// Serialize solids to Part-21 text (the exact bytes save() would write).
    static std::string toString(const std::vector<const topo::Solid*>& solids);

    /// Read all MANIFOLD_SOLID_BREP solids from a Part-21 file. Returns an
    /// empty vector on failure (see lastError()).
    static std::vector<std::unique_ptr<topo::Solid>> load(const std::string& filePath);

    /// Parse Part-21 text and reconstruct all contained solids.
    static std::vector<std::unique_ptr<topo::Solid>> fromString(const std::string& text);

    /// Human-readable description of the most recent load/save failure on this
    /// thread's last call (empty when the call succeeded).
    static const std::string& lastError();
};

}  // namespace hz::io
