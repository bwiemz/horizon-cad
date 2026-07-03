#pragma once

#include <memory>

#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// Applies a draft (taper) to a solid's lateral faces.
///
/// Each lateral face tilts about the neutral plane by @p angleRad relative to
/// the pull direction: a vertex at signed height h above the neutral plane
/// moves outward by h*tan(angle). The offset is applied along the mitered
/// combination of the vertex's two incident lateral-face normals, which keeps
/// every lateral face planar. Caps and connectivity are unchanged.
///
/// The operation consumes and returns the solid (feature-tree semantics).
class Draft {
public:
    /// @param solid        The solid to draft (ownership consumed).
    /// @param pullDir      Pull/mold-open direction (unit vector).
    /// @param neutralPoint A point on the neutral plane (the plane where the
    ///                     draft has zero effect), with normal = pullDir.
    /// @param angleRad     Draft angle in radians (positive = taper outward
    ///                     above the neutral plane).
    /// @return The drafted solid, or nullptr on invalid input.
    static std::unique_ptr<topo::Solid> execute(std::unique_ptr<topo::Solid> solid,
                                                const math::Vec3& pullDir,
                                                const math::Vec3& neutralPoint, double angleRad);
};

}  // namespace hz::model
