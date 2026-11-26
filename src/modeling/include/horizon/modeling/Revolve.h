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

/// Creates a 3D solid by revolving a closed 2D profile around an axis.
///
/// The revolution is faceted: the profile is swept through @c segments angular
/// steps and each profile edge becomes a band of planar quads.  The exact
/// surface of revolution each band approximates — a cylinder, a cone frustum
/// or a planar annulus, depending on the profile edge — is kept on
/// @c topo::Face::analyticSurface, and the circle each profile vertex traces
/// is kept on @c topo::Edge::analyticCurve.  The faces themselves carry the
/// planar carriers their loops actually lie on, so every loop-based consumer
/// (Booleans, mass properties, tessellation, interference) sees the same shape
/// the renderer draws.
///
/// Accepts any closed profile of three or more vertices and any angle in
/// (0, 2*pi].  A partial revolution is capped at both ends; a full revolution
/// closes on itself and is uncapped.
///
/// The profile may touch the axis but may not cross it: a profile straddling
/// the axis sweeps through itself, and the result would not be a solid.
///
/// A full revolution of a profile that stays clear of the axis is a torus-like
/// shell of genus 1.
class Revolve {
public:
    /// Angular steps around the axis for a full revolution.  A partial
    /// revolution uses the same angular resolution, proportionally fewer.
    static constexpr int kDefaultSegments = 32;

    /// Angular steps per full turn needed to keep the chord sagitta of a
    /// circle of @p radius within @p tolerance.
    static int segmentsForTolerance(double radius, double tolerance);

    /// The largest distance from the axis of any profile vertex: the radius
    /// whose chord sag governs the revolve's accuracy.  Returns 0 when the
    /// profile is not a closed loop.
    static double profileRadius(const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
                                const draft::SketchPlane& plane, const math::Vec3& axisPoint,
                                const math::Vec3& axisDirection);

    /// Revolve a closed 2D profile around an axis.
    ///
    /// @param profile       Draft entities forming a closed loop on the sketch plane.
    /// @param plane         The 2D sketch plane (provides localToWorld transform).
    /// @param axisPoint     A point on the revolution axis (in world space).
    /// @param axisDirection Direction of the revolution axis (will be normalized).
    /// @param angle         Revolution angle in radians, in (0, 2*pi].
    /// @param featureID     Base name for TopologyID generation (e.g. "revolve_1").
    /// @param segments      Angular steps for a full turn (>= 3); also the
    ///                      chords per turn used to facet profile arcs.
    /// @param chordTolerance When positive, profile arcs are instead faceted at
    ///                      the count their own radius needs to keep every
    ///                      chord within it.
    /// @param naming        Stable: each band is named after the profile element
    ///                      it sweeps, `revolved:<source>`, in facets, and the
    ///                      edges logically. Otherwise by position, as before.
    /// @return The revolved solid, or nullptr if the profile or angle is invalid.
    static std::unique_ptr<topo::Solid> execute(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
        const draft::SketchPlane& plane, const math::Vec3& axisPoint,
        const math::Vec3& axisDirection, double angle, const std::string& featureID,
        int segments = kDefaultSegments, double chordTolerance = 0.0, std::string* reason = nullptr,
        NamingScheme naming = NamingScheme::Positional);

    /// The revolve built exactly (stage 3), from whole faces carrying exact geometry
    /// (topo::Face::exactSurface, pcurves, topo::Edge::exactCurve) and no NURBS carrier. Each
    /// curve of the profile (ExactProfile's loops) sweeps one face about the axis: a line square
    /// to it a plane (a disc or a ring through a whole turn, a sector of one through part), a line
    /// along it a cylinder, a line on it nothing, any other line a cone to the apex where it
    /// meets the axis; an arc centred on the axis a sphere, an arc whose circle clears the axis a
    /// torus, and one whose circle reaches it a NURBS surface of revolution (Ruling 2). Where the
    /// profile meets the axis there is no edge: a face comes to a pole or an apex there, its loop
    /// jumping along it in (u, v). Through a whole turn each curved face has a seam where the
    /// profile lies, at u = 0 along the profile's side of the axis (where master's ring starts);
    /// through part of one the profile caps it at each end, and the caps meet along the axis
    /// where the profile ran along it. A hole in the profile is a cavity (a shell of its own)
    /// through a whole turn, and an inner loop of each cap through part of one; each region is a
    /// shell of its own. Nothing is a Boolean.
    ///
    /// Its faces have master's Stable logical names: `<featureID>/revolved:<entity>`
    /// (ProfileValidationResult's edgeSources), `<featureID>/cap_start` and `/cap_end`, a hole's
    /// faces `<featureID>~hole<k>/...` (k counted through the regions) and the faces of a region
    /// after the first `<featureID>~region<r>/...`. Its edges are named after them
    /// (model::nameEdgesLogically). Its vertices are where master's faceted revolve put each
    /// loop's corners, at u = 0 and turned by the angle; where the sketch's curves miss a corner
    /// (by no more than the profile's join tolerance), its vertices and the rim it traces carry
    /// the gap as their tolerance.
    ///
    /// @return nullptr, with @p reason, where master refuses the profile, the angle or the axis
    ///         (in master's words, as Revolve::execute gives them), or with "could not be built
    ///         exactly: <why>" where only the exact construction cannot build it (an arc that
    ///         reaches the axis between its ends, a horn's cusp; regions on both sides of the
    ///         axis).
    static std::unique_ptr<topo::Solid> exact(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
        const draft::SketchPlane& plane, const math::Vec3& axisPoint,
        const math::Vec3& axisDirection, double angle, const std::string& featureID,
        std::string* reason = nullptr);
};

}  // namespace hz::model
