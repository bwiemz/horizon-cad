#pragma once

#include <cstddef>
#include <vector>

#include "horizon/math/BoundingBox.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// One interfering pair reported by the checker.
struct InterferencePair {
    size_t indexA;
    size_t indexB;
    math::BoundingBox overlapBounds;  ///< AABB of the two solids' overlap region.
};

/// Detects overlapping solids in an assembly (world-space input).
///
/// Broad phase: an R*-tree of AABBs yields O(n log n) candidate pairs.
///
/// Narrow phase: a robust triangle-mesh overlap test on each candidate — an
/// edge of one mesh crossing a face of the other, or one solid being contained
/// in the other (point-in-solid by ray parity). This deliberately avoids the
/// volume-intersection Boolean, which is not yet robust enough to gate a
/// correctness result; the precise interference *volume* is a later follow-up.
class InterferenceChecker {
public:
    /// Find every interfering pair among @p solids (each already in world space).
    static std::vector<InterferencePair> check(const std::vector<const topo::Solid*>& solids);

    /// True if two world-space solids share interior volume.
    static bool solidsInterfere(const topo::Solid& a, const topo::Solid& b);

    /// Axis-aligned bounds of a solid's vertices.
    static math::BoundingBox solidBounds(const topo::Solid& solid);
};

}  // namespace hz::model
