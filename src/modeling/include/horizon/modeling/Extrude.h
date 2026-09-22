#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// Creates a 3D solid by extruding a closed 2D profile along a direction.
///
/// Supported profiles:
///   - Rectangle (4 DraftLine loop) → box topology (8V, 12E, 6F)
///   - Any other closed loop of lines and arcs, or a single circle → an
///     N-sided prism (2N V, 3N E, N+2 F)
///
/// Arcs and circles are faceted: each is followed along its curve at
/// `segments` chords per full turn (or at the count its own radius needs to
/// keep every chord within `chordTolerance`, when that is positive), so the
/// volume converges to the exact one from below.  Every face is planar and
/// carries the surface its loop lies on; the lateral facets of an arc record
/// the cylinder they approximate on `topo::Face::analyticSurface` (for an
/// extrusion along the sketch normal), and each chord of an arc records its
/// circle on `topo::Edge::analyticCurve`.
///
/// All faces receive TopologyIDs and NURBS surface bindings.
class Extrude {
public:
    /// Chords per full turn of an arc or circle in the profile.
    static constexpr int kDefaultSegments = 32;

    /// Extrude a 2D profile into a 3D solid.
    ///
    /// @param profile    Draft entities forming a closed loop on the sketch plane.
    /// @param plane      The 2D sketch plane (provides localToWorld transform).
    /// @param direction  Extrusion direction (unit vector, typically plane.normal()).
    /// @param distance   Extrusion distance (positive = along direction).
    /// @param featureID  Base name for TopologyID generation (e.g. "extrude_1").
    /// @param segments   Chords per full turn of profile arcs (>= 3).
    /// @param chordTolerance  When positive, overrides @p segments per arc.
    /// @return The extruded solid, or nullptr if the profile is invalid.
    static std::unique_ptr<topo::Solid> execute(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
        const draft::SketchPlane& plane, const math::Vec3& direction, double distance,
        const std::string& featureID, int segments = kDefaultSegments, double chordTolerance = 0.0);
};

}  // namespace hz::model
