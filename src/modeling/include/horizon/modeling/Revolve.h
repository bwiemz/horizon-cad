#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Vec3.h"
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
/// shell of genus 1.  It is manifold and closed, but @c checkEulerFormula()
/// carries no genus term and reports such a shell as invalid; see
/// PrimitiveFactory::makeTorus for the same caveat.
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
    /// @return The revolved solid, or nullptr if the profile or angle is invalid.
    static std::unique_ptr<topo::Solid> execute(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
        const draft::SketchPlane& plane, const math::Vec3& axisPoint,
        const math::Vec3& axisDirection, double angle, const std::string& featureID,
        int segments = kDefaultSegments, double chordTolerance = 0.0);
};

}  // namespace hz::model
