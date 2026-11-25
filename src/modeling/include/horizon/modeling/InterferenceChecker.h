#pragma once

#include <atomic>
#include <cstddef>
#include <string>
#include <vector>

#include "horizon/math/BoundingBox.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// One interfering pair reported by the checker.
struct InterferencePair {
    size_t indexA;
    size_t indexB;
    math::BoundingBox overlapBounds;  ///< AABB of the two solids' overlap region.
    /// Volume of material the two solids share: their Boolean intersection,
    /// integrated.  How much two parts clash is what decides whether it is a
    /// press fit or a design error.
    double volume = 0.0;
    /// False when the intersection Boolean could not resolve the overlap
    /// (a degenerate, near-touching configuration); the pair is still
    /// reported, because the narrow phase found it, but @c volume is 0.
    bool volumeResolved = false;
    /// Why the pair could not be worked out, when it could not: a face of
    /// either that the read point cannot mesh, which face and why
    /// (BoundaryMesh's reason). Such a pair is reported, to be looked at, not
    /// passed as clear, with no volume. Empty otherwise.
    std::string problem;
};

/// Detects overlapping solids in an assembly (world-space input).
///
/// Broad phase: an R*-tree of AABBs yields O(n log n) candidate pairs.
///
/// Narrow phase: a robust triangle-mesh overlap test on each candidate — an
/// edge of one mesh crossing a face of the other, or one solid being contained
/// in the other (point-in-solid by ray parity). The test, not the Boolean,
/// decides *whether* a pair interferes; the Boolean intersection then
/// measures *how much* (InterferencePair::volume).  Solids that only touch —
/// sharing a face or tangent along a line — do not interfere.
class InterferenceChecker {
public:
    /// Find every interfering pair among @p solids (each already in world
    /// space). Once @p cancelled is set, the check stops before the next
    /// candidate pair and returns the pairs found by then.
    static std::vector<InterferencePair> check(const std::vector<const topo::Solid*>& solids,
                                               const std::atomic<bool>* cancelled = nullptr);

    /// True if two world-space solids share interior volume. True too where
    /// that cannot be worked out (a face the read point cannot mesh), with
    /// why in @p problem: reported, to be looked at, not passed as clear.
    static bool solidsInterfere(const topo::Solid& a, const topo::Solid& b,
                                std::string* problem = nullptr);

    /// Axis-aligned bounds of a solid's vertices.
    static math::BoundingBox solidBounds(const topo::Solid& solid);
};

}  // namespace hz::model
