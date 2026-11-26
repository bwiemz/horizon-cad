#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/Naming.h"
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
    /// @param reason     When given, receives why the extrusion failed.
    /// @param naming     How the side faces and edges are named (see
    ///                   NamingScheme): by position, as before persistent
    ///                   naming, or from the sketch entities they come from.
    /// @return The extruded solid, or nullptr if the profile is invalid.
    static std::unique_ptr<topo::Solid> execute(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
        const draft::SketchPlane& plane, const math::Vec3& direction, double distance,
        const std::string& featureID, int segments = kDefaultSegments, double chordTolerance = 0.0,
        std::string* reason = nullptr, NamingScheme naming = NamingScheme::Positional);

    /// The extrusion built exactly (stage 3), from whole faces carrying exact geometry
    /// (topo::Face::exactSurface, pcurves, topo::Edge::exactCurve) and no NURBS carrier: one face
    /// for each curve of the profile, a plane for a line, and for an arc or a circle a cylinder
    /// when it is swept along the sketch's normal (within the model's angular resolution), a NURBS
    /// extrusion (NurbsSurface::makeExtrusion) along any other direction. The caps are planes, a
    /// hole's loop an inner loop of each; each region of the profile is a shell of its own, an
    /// island inside a hole among them. Nothing is a Boolean.
    ///
    /// Its faces have master's Stable logical names: `<featureID>/cap_bottom` (at the sketch),
    /// `<featureID>/cap_top` and `<featureID>/side:<entity>` (ProfileValidationResult's
    /// edgeSources), a hole's sides `<featureID>~hole<k>/side:<entity>` (k counted through the
    /// regions) and the faces of a region after the first `<featureID>~region<r>/...`. Its edges
    /// are named after them (model::nameEdgesLogically). Its vertices are where master's faceted
    /// extrude put each loop's corners; where the sketch's curves miss one (by no more than the
    /// profile's join tolerance), its vertices and the edge along the offset there carry the gap
    /// as their tolerance.
    ///
    /// @return nullptr, with @p reason, where master refuses the profile, the distance or the
    ///         direction (in master's words), or with "could not be built exactly: <why>" where
    ///         only the exact construction cannot build it.
    static std::unique_ptr<topo::Solid> exact(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
        const draft::SketchPlane& plane, const math::Vec3& direction, double distance,
        const std::string& featureID, std::string* reason = nullptr);
};

}  // namespace hz::model
