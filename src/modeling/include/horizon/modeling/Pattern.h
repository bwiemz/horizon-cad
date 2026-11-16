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
    /// the instances' names; otherwise by position, as before. With
    /// @p together (for this and grid() and circular()), in rounds of
    /// instances clear of one another, each round one Boolean, and a copy
    /// where an earlier one is is left out: the same body, in a fraction of
    /// the time for many copies, but with the faces they split cut into
    /// other pieces, and so named otherwise. @p reason, when given, says why
    /// there is no result: why the Boolean joining two instances failed.
    static std::unique_ptr<topo::Solid> linear(const topo::Solid& source,
                                               const math::Vec3& direction, double spacing,
                                               int count, const std::vector<int>& suppressed = {},
                                               NamingScheme naming = NamingScheme::Positional,
                                               bool together = false,
                                               std::string* reason = nullptr);

    /// Linear pattern in two directions (SolidWorks' Direction 1 and 2): a
    /// grid of @p count by @p count2 instances, @p spacing apart along
    /// @p direction and @p spacing2 along @p direction2. Instance k is
    /// column k % count of row k / count, as @p suppressed numbers them.
    static std::unique_ptr<topo::Solid> grid(const topo::Solid& source, const math::Vec3& direction,
                                             double spacing, int count,
                                             const math::Vec3& direction2, double spacing2,
                                             int count2, const std::vector<int>& suppressed = {},
                                             NamingScheme naming = NamingScheme::Positional,
                                             bool together = false, std::string* reason = nullptr);

    /// Both solids' shells in one solid, as separate bodies: no Boolean, so
    /// shared material is not merged (use BooleanOp for that). Topology IDs
    /// are kept. Each is copied wound facing out, as `append` and a
    /// pattern's bodies are: the kernel's builders wind either way (a box
    /// primitive inward, a cylinder outward), and a solid of both measured
    /// the one less the other.
    static std::unique_ptr<topo::Solid> collect(const topo::Solid& a, const topo::Solid& b);

    /// @p source's bodies added to @p into, moved by the rigid @p xform, with
    /// every carrier and ideal moved and every TopologyID kept: `collect`
    /// for many solids, each copied once (collecting pairwise copied what
    /// was gathered again for each one added).
    static void append(topo::Solid& into, const topo::Solid& source, const math::Mat4& xform);

    /// The reverse of `collect`: each body of @p solid as a solid of its own,
    /// with its TopologyIDs. A body is an outer shell together with the
    /// cavities it encloses — an enclosed void is a second shell of the same
    /// body, facing into the void, and stays with it. Bodies come in shell
    /// order.
    static std::vector<std::unique_ptr<topo::Solid>> separate(const topo::Solid& solid);

    /// The shells of each of @p solid's bodies, as `separate` makes them:
    /// its outer shell first, then the cavities it encloses. A cavity that
    /// nothing encloses is a body of its own. For a writer that must say
    /// which shell is which (STEP's BREP_WITH_VOIDS).
    static std::vector<std::vector<const topo::Shell*>> bodyShells(const topo::Solid& solid);

    /// A deep copy of @p source moved by the rigid transform @p xform, with
    /// every carrier and ideal moved with it and every TopologyID kept.  This
    /// is how a component is placed in an assembly's world space. A mirror
    /// (Phase 162: a transform whose determinant is negative) reverses every
    /// loop and surface as it copies them, so the copy faces out as the
    /// original does; `append` does the same.
    static std::unique_ptr<topo::Solid> transformed(const topo::Solid& source,
                                                    const math::Mat4& xform);

    /// Circular pattern: @p count instances rotated @p angleStepRad apart
    /// about the axis through @p axisPoint along @p axisDir.
    static std::unique_ptr<topo::Solid> circular(const topo::Solid& source,
                                                 const math::Vec3& axisPoint,
                                                 const math::Vec3& axisDir, double angleStepRad,
                                                 int count, const std::vector<int>& suppressed = {},
                                                 NamingScheme naming = NamingScheme::Positional,
                                                 bool together = false,
                                                 std::string* reason = nullptr);
};

}  // namespace hz::model
