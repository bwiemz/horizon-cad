#pragma once

#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"

#include <memory>
#include <string>
#include <vector>

namespace hz::model {

/// Creates a 3D solid by extruding a closed 2D profile along a direction.
///
/// Supported profiles:
///   - Rectangle (4 DraftLine loop) → box topology (8V, 12E, 6F)
///   - Circle (single DraftCircle) → cylinder topology (8V, 12E, 6F)
///   - General N-gon (N DraftLine/DraftArc loop) → prism topology (2N V, 3N E, N+2 F)
///
/// All faces receive TopologyIDs and NURBS surface bindings.
class Extrude {
public:
    /// Extrude a 2D profile into a 3D solid.
    ///
    /// @param profile    Draft entities forming a closed loop on the sketch plane.
    /// @param plane      The 2D sketch plane (provides localToWorld transform).
    /// @param direction  Extrusion direction (unit vector, typically plane.normal()).
    /// @param distance   Extrusion distance (positive = along direction).
    /// @param featureID  Base name for TopologyID generation (e.g. "extrude_1").
    /// @return The extruded solid, or nullptr if the profile is invalid.
    static std::unique_ptr<topo::Solid> execute(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
        const draft::SketchPlane& plane,
        const math::Vec3& direction,
        double distance,
        const std::string& featureID);
};

}  // namespace hz::model
