#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// Linear and circular geometry patterns.
///
/// Geometry-pattern strategy (the roadmap default): the source solid's B-Rep
/// is deep-cloned once per instance with a rigid transform and all instances
/// coexist in one result solid as separate bodies (shells). Non-overlapping
/// instances — the common pattern case (bosses, spaced features) — need no
/// Boolean. Overlapping-instance merge is deferred.
///
/// Pattern TopologyIDs follow genealogy: instance 0 keeps the source IDs; each
/// copy k gets `sourceId.child("pattern", k)`.
class Pattern {
public:
    /// Linear pattern: @p count instances spaced @p spacing apart along
    /// @p direction (instance 0 is the source at its original location).
    /// @p suppressed lists instance indices to skip.
    static std::unique_ptr<topo::Solid> linear(const topo::Solid& source,
                                               const math::Vec3& direction, double spacing,
                                               int count, const std::vector<int>& suppressed = {});

    /// Circular pattern: @p count instances rotated @p angleStepRad apart
    /// about the axis through @p axisPoint along @p axisDir.
    static std::unique_ptr<topo::Solid> circular(const topo::Solid& source,
                                                 const math::Vec3& axisPoint,
                                                 const math::Vec3& axisDir, double angleStepRad,
                                                 int count,
                                                 const std::vector<int>& suppressed = {});
};

}  // namespace hz::model
