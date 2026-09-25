#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// Linear and circular geometry patterns.
///
/// Geometry-pattern strategy (the roadmap default): the source solid's B-Rep
/// is deep-cloned once per instance with a rigid transform, carrying each
/// face's and edge's ideal geometry (analyticSurface / analyticCurve) moved
/// with it. Instances that stay apart — the common pattern case (bosses,
/// spaced features) — coexist in one result solid as separate bodies
/// (shells) with no Boolean. Instances whose bounds touch or overlap are
/// merged with BooleanOp::Union, so shared material is counted once; if a
/// merge fails the pattern is refused (nullptr) rather than returned as
/// interpenetrating shells.
///
/// Pattern TopologyIDs follow genealogy: instance 0 keeps the source IDs; each
/// copy k gets `sourceId.child("pattern", k)`.
class Pattern {
public:
    /// Linear pattern: @p count instances spaced @p spacing apart along
    /// @p direction (instance 0 is the source at its original location).
    /// @p suppressed lists instance indices to skip.
    /// Instances that meet are joined, at @p naming's names: Stable keeps
    /// the instances' names; otherwise by position, as before.
    static std::unique_ptr<topo::Solid> linear(const topo::Solid& source,
                                               const math::Vec3& direction, double spacing,
                                               int count, const std::vector<int>& suppressed = {},
                                               NamingScheme naming = NamingScheme::Positional);

    /// Both solids' shells in one solid, as separate bodies: no Boolean, so
    /// shared material is not merged (use BooleanOp for that). Topology IDs
    /// are kept.
    static std::unique_ptr<topo::Solid> collect(const topo::Solid& a, const topo::Solid& b);

    /// The reverse of `collect`: each body of @p solid as a solid of its own,
    /// with its TopologyIDs. A body is an outer shell together with the
    /// cavities it encloses — an enclosed void is a second shell of the same
    /// body, facing into the void, and stays with it. Bodies come in shell
    /// order.
    static std::vector<std::unique_ptr<topo::Solid>> separate(const topo::Solid& solid);

    /// A deep copy of @p source moved by the rigid transform @p xform, with
    /// every carrier and ideal moved with it and every TopologyID kept.  This
    /// is how a component is placed in an assembly's world space.
    static std::unique_ptr<topo::Solid> transformed(const topo::Solid& source,
                                                    const math::Mat4& xform);

    /// Circular pattern: @p count instances rotated @p angleStepRad apart
    /// about the axis through @p axisPoint along @p axisDir.
    static std::unique_ptr<topo::Solid> circular(const topo::Solid& source,
                                                 const math::Vec3& axisPoint,
                                                 const math::Vec3& axisDir, double angleStepRad,
                                                 int count, const std::vector<int>& suppressed = {},
                                                 NamingScheme naming = NamingScheme::Positional);
};

}  // namespace hz::model
