#pragma once

#include <atomic>
#include <cstddef>
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
/// - Face holes (inner loops) are honored: a face's outer loop and its holes
///   are bridged into one outline (BoundaryMesh::keyholePolygon) before
///   triangulation. A hole that no bridge can reach without crossing an edge
///   is left out.
/// - With `FromGeometry` naming, the fragments of each split face are merged
///   back into one face (FragmentMerge). They stay as fragments with
///   `Positional` naming, and where merging gives up: a face with more than
///   60 holes, or fragments that pinch or overlap.
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

/// Limits on the work of each Boolean run on this thread while it lives, in
/// place of the defaults: for a preview that should give up sooner, and for
/// tests. A Boolean over its limits fails, saying so, rather than going on
/// for an hour or until memory runs out. @p work: how many polygon corners
/// it may test against a plane; @p splits: how many times it may cut a
/// polygon in two. Zero leaves that limit as it was.
class BooleanWorkLimits {
public:
    BooleanWorkLimits(std::size_t work, std::size_t splits);
    ~BooleanWorkLimits();
    BooleanWorkLimits(const BooleanWorkLimits&) = delete;
    BooleanWorkLimits& operator=(const BooleanWorkLimits&) = delete;

private:
    std::size_t m_savedWork;
    std::size_t m_savedSplits;
};

/// While it lives, the Booleans run on this thread share one budget of work
/// as well as each having its own: a pattern of 10,000 copies that overlap
/// one another joins them one at a time, each join well within its limits,
/// and all of them for hours. Past it, a Boolean fails saying the copies are
/// too many to combine. Each Boolean is charged for the polygons it is given
/// as well as for the ones it cuts, for a join to a body grown to a million
/// polygons reads, merges and sews all of them again. @p work: how many
/// polygon corners they may test against a plane together, zero for the
/// default. Within another series, it is that series that counts.
class BooleanSeries {
public:
    explicit BooleanSeries(std::size_t work = 0);
    ~BooleanSeries();
    BooleanSeries(const BooleanSeries&) = delete;
    BooleanSeries& operator=(const BooleanSeries&) = delete;

    /// The work charged to it so far (none within another series).
    [[nodiscard]] std::size_t work() const { return m_work; }

private:
    std::size_t m_work = 0;
    bool m_outermost = false;
};

/// While it lives, each Boolean run on this thread stops as soon as
/// @p cancelled is set, failing with the reason "cancelled", rather than at
/// its end: a rebuild of a part of tens of thousands of facets could run
/// for minutes after the user had moved on, and the window waited for it.
class BooleanCancellation {
public:
    explicit BooleanCancellation(const std::atomic<bool>* cancelled);
    ~BooleanCancellation();
    BooleanCancellation(const BooleanCancellation&) = delete;
    BooleanCancellation& operator=(const BooleanCancellation&) = delete;

    /// Whether the build this thread runs for has been cancelled: for work
    /// between Booleans and around them (copying a pattern's instances,
    /// merging and sewing a result), which ran on for seconds after the
    /// cancel in a debug build, to stop at too.
    [[nodiscard]] static bool requested();

private:
    const std::atomic<bool>* m_saved;
};

}  // namespace hz::model
