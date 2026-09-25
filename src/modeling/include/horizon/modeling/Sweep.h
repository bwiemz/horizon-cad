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

/// Creates a 3D solid by sweeping a closed profile along a 3D path.
///
/// The path is a polyline (2+ points); each segment produces one level of
/// lateral faces.  The profile is carried along it by a rotation-minimizing
/// frame: at each interior path point the section is the cut of the incoming
/// prism by the miter plane bisecting the turn, so the cross-section
/// perpendicular to every segment is the profile turned by the smallest
/// rotation between consecutive segment directions, and the far cap is the
/// profile's plane carried through the same turns.  Every lateral face is
/// exactly planar, and with the profile's centroid on the path the volume is
/// exactly (profile area normal to the path) x (path length).
///
/// Refused (nullptr): a path that doubles back on itself, a profile whose
/// plane contains the initial sweep direction, a turn tight enough that the
/// profile reaches past its inside (the band would fold through itself), and
/// any result the geometric validator rejects.  Twist and guide curves are
/// not supported.
class Sweep {
public:
    /// Steps per full turn used to sample a circular arc in a path.  A path
    /// arc is swept as a chain of mitered segments, so like a revolve the
    /// volume converges to the exact one from below as this rises.
    static constexpr int kDefaultArcSegments = 32;

    /// Sweep a profile along a polyline path.
    ///
    /// @param profile     Closed 2D profile on @p plane.
    /// @param plane       The profile's sketch plane.
    /// @param pathPoints  Polyline path in world space (>= 2 distinct points).
    /// @param featureID   Base name for TopologyID generation (e.g. "sweep_1").
    /// @param profileSegments  Chords per full turn of profile arcs (>= 3).
    /// @param chordTolerance   When positive, overrides @p profileSegments per arc.
    /// @param naming      Stable: each side named after the profile element it
    ///                    sweeps, `swept:<source>`, in facets (one a path
    ///                    segment), and the edges logically. Otherwise by
    ///                    position, as before.
    /// @return The swept solid, or nullptr if the input is invalid.
    static std::unique_ptr<topo::Solid> execute(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
        const draft::SketchPlane& plane, const std::vector<math::Vec3>& pathPoints,
        const std::string& featureID, int profileSegments = kDefaultArcSegments,
        double chordTolerance = 0.0, std::string* reason = nullptr,
        NamingScheme naming = NamingScheme::Positional);
};

}  // namespace hz::model
