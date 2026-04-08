#pragma once

#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"

#include <memory>
#include <string>
#include <vector>

namespace hz::model {

/// Creates a 3D solid by revolving a closed 2D profile around an axis.
///
/// Supported profiles:
///   - Rectangle (4 DraftLine loop) revolving 360 degrees -> torus-like topology (8V, 12E, 6F)
///
/// All faces receive TopologyIDs and NURBS surface bindings.
class Revolve {
public:
    /// Revolve a closed 2D profile around an axis.
    ///
    /// @param profile       Draft entities forming a closed loop on the sketch plane.
    /// @param plane         The 2D sketch plane (provides localToWorld transform).
    /// @param axisPoint     A point on the revolution axis (in world space).
    /// @param axisDirection Direction of the revolution axis (will be normalized).
    /// @param angle         Revolution angle in radians (up to 2*pi for full revolution).
    /// @param featureID     Base name for TopologyID generation (e.g. "revolve_1").
    /// @return The revolved solid, or nullptr if the profile is invalid.
    static std::unique_ptr<topo::Solid> execute(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
        const draft::SketchPlane& plane,
        const math::Vec3& axisPoint,
        const math::Vec3& axisDirection,
        double angle,
        const std::string& featureID);
};

}  // namespace hz::model
