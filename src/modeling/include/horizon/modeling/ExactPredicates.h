#pragma once

#include "horizon/math/Vec3.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::model {

/// Geometric predicates for Boolean operations on B-Rep solids.
///
/// orient3D classifies a point relative to a plane; classifyPoint uses
/// ray casting through the tessellated faces to determine inside/outside.
class ExactPredicates {
public:
    /// Point vs plane classification.
    /// @return +1 (positive / front), -1 (negative / back), 0 (on plane).
    static int orient3D(const math::Vec3& planePoint, const math::Vec3& planeNormal,
                        const math::Vec3& testPoint, double tolerance = 1e-10);

    /// Point vs solid classification via ray casting.
    /// @return +1 (outside), -1 (inside), 0 (on boundary).
    static int classifyPoint(const math::Vec3& point, const topo::Solid& solid,
                             double tolerance = 1e-8);
};

}  // namespace hz::model
