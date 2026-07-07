#pragma once

#include <memory>

#include "horizon/topology/Solid.h"

namespace hz::model {

enum class BooleanType { Union, Subtract, Intersect };

/// Boolean operations on B-Rep solids via BSP-tree CSG.
///
/// Pipeline: each solid's trimmed boundary is extracted from its face loops
/// (BoundaryMesh), triangulated, and run through a double-precision BSP CSG
/// (MeshCsg) that splits faces along the other solid's face planes and
/// classifies the fragments exactly — including coplanar-face resolution by
/// normal orientation.  Selected fragments are welded, T-junction-freed, and
/// sewn back into a half-edge Solid (SolidSewer) with fragment provenance
/// carried through TopologyIDs.
///
/// Guarantees and limitations:
/// - Any returned solid passes Solid::checkManifold() (enforced on every
///   path, including the disjoint fast paths); on hard degeneracies the
///   operation returns nullptr instead of emitting broken topology.
/// - Genus-0 results additionally satisfy checkEulerFormula().  Through-hole
///   (genus ≥ 1) results are manifold but report Euler-invalid because the
///   topology module's Euler check has no genus term.
/// - Curved faces participate via their loop polygons (the MassProperties
///   convention), so a prism-topology cylinder acts as its inscribed square
///   prism.  Analytic surface–surface intersection is future work.  Solids
///   built on coarse box topology whose face loops are near-coplanar — the
///   revolve/torus primitives, whose eight ring corners are all coplanar —
///   enclose ~zero loop volume, so Booleans against them degenerate (remove
///   nothing / return empty); they are not currently supported operands.
/// - Face holes (inner loops) on the input solids are not honored: only the
///   outer loop of each face is taken.  Extrude/primitive/prior-Boolean
///   inputs never carry inner loops, so this affects only externally
///   imported faces-with-holes.
/// - Faces crossing the intersection are returned as planar fragments; no
///   coplanar-fragment re-merging is performed yet.
class BooleanOp {
public:
    /// Execute a Boolean operation on two solids.
    /// @return The result solid, or nullptr when the result is empty
    ///         (e.g. disjoint Intersect, A−A) or the inputs degenerate.
    static std::unique_ptr<topo::Solid> execute(const topo::Solid& solidA,
                                                const topo::Solid& solidB, BooleanType type);
};

}  // namespace hz::model
