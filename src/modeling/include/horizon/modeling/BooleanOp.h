#pragma once

#include <memory>
#include <string>

#include "horizon/modeling/Naming.h"
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
/// - Results also satisfy checkEulerFormula(), through-hole (genus ≥ 1)
///   results included.
/// - Curved geometry participates as its facets (the MassProperties
///   convention): curved primitives, revolves and arc profiles are faceted at
///   construction, so results are correct to the operands' facet error.
///   Analytic surface–surface intersection is future work.
/// - Each result face keeps its source face's ideal surface
///   (topo::Face::analyticSurface, looked up per operand by TopologyID) and
///   each result edge lying along a source edge keeps that edge's ideal curve,
///   so a bore still resolves to its cylinder for mates and dimensions.
/// - Face holes (inner loops) on the input solids are not honored: only the
///   outer loop of each face is taken.  Extrude/primitive/prior-Boolean
///   inputs never carry inner loops, so this affects only externally
///   imported faces-with-holes.
/// - Faces crossing the intersection are returned as planar fragments; no
///   coplanar-fragment re-merging is performed yet.
class BooleanOp {
public:
    /// Execute a Boolean operation on two solids.
    /// @param reason  When given, receives why there is no result.
    /// @param naming  `FromGeometry` names each piece of a split face and
    ///                every edge after its faces (see Naming.h); `Positional`
    ///                keeps the sewer's numbering, which references in files
    ///                saved before persistent naming were made against.
    /// @return The result solid, or nullptr when the result is empty
    ///         (e.g. disjoint Intersect, A−A) or the inputs degenerate.
    static std::unique_ptr<topo::Solid> execute(const topo::Solid& solidA,
                                                const topo::Solid& solidB, BooleanType type,
                                                std::string* reason = nullptr,
                                                NamingScheme naming = NamingScheme::Positional);
};

}  // namespace hz::model
