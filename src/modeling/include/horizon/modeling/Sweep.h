#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// Creates a 3D solid by sweeping a closed profile along a 3D path.
///
/// Era-2 scope (matches the roadmap): translation transport — the profile
/// keeps its orientation as it moves along the path. Frenet-frame rotation
/// and twist are deferred with guide-curve support. The path is a polyline
/// (2+ points); each segment produces a ring in the swept solid.
class Sweep {
public:
    /// Sweep a profile along a polyline path.
    ///
    /// @param profile     Closed 2D profile on @p plane.
    /// @param plane       The profile's sketch plane.
    /// @param pathPoints  Polyline path in world space (>= 2 distinct points).
    /// @param featureID   Base name for TopologyID generation (e.g. "sweep_1").
    /// @return The swept solid, or nullptr if the input is invalid.
    static std::unique_ptr<topo::Solid> execute(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
        const draft::SketchPlane& plane, const std::vector<math::Vec3>& pathPoints,
        const std::string& featureID);
};

}  // namespace hz::model
