#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/topology/TopologyID.h"

namespace hz::geo {
class NurbsSurface;
}  // namespace hz::geo

namespace hz::model {

/// Distance from a face plane within which a point counts as lying on it —
/// the BSP splitter's on-plane band — for a part 100 across (CsgTolerance).
/// Downstream sewing welds at a tolerance >= the band: any two points the
/// splitter treated as coincident on a shared plane must be reconcilable, or
/// seams the splitter is allowed to open could never be closed.
constexpr double kCsgPlaneEps = 1e-6;

/// The tolerances of one Boolean, from the size of its operands (Phase 142).
/// They were absolute, and 1e-6 on a part a hundredth of a millimetre
/// across is a ten-thousandth of it: its fragments' seams did not sew.
struct CsgTolerance {
    /// The on-plane band, also the weld of the fragments' sewing and of
    /// their merging: 1e-8 of the operands' extent, but never under 64 ulps
    /// of their largest coordinate, which is all the precision there is a
    /// million out.
    double plane = kCsgPlaneEps;

    static CsgTolerance of(const math::Vec3& low, const math::Vec3& high);
};

/// A convex polygon fragment flowing through the CSG pipeline.
struct CsgPolygon {
    std::vector<math::Vec3> points;              ///< Convex, consistently wound loop.
    topo::TopologyID topoId;                     ///< Provenance: source face's topology ID.
    std::shared_ptr<geo::NurbsSurface> surface;  ///< Source surface (null once split).
    bool fromA = true;
};

/// BSP-tree CSG on convex polygon soups (double precision).
///
/// Faces are split along the other solid's face planes (the planar analogue
/// of surface–surface intersection), fragments are classified inside/outside
/// by BSP traversal instead of per-face centroid sampling, and coplanar
/// faces are resolved by normal orientation.  Fragments of B kept by a
/// Subtract come back with reversed winding (flipped normals).
///
/// Input polygons must be convex and outward-oriented; both solids' boundary
/// triangulations from BoundaryMesh satisfy this.
///
/// The tree is a flat array walked with explicit stacks, so no traversal
/// recurses. Each node splits by its first polygon's plane, as csg.js does. A
/// convex solid's own planes each have all its other faces behind them, so
/// its tree is a chain whatever the choice, and building it is quadratic in
/// its facet count: a cylinder of 2,048 facets takes seconds in a debug
/// build. (A plane chosen from a sample to split least and balance most was
/// tried: 8% slower on that cylinder, and no faster on a plate of 81 pins.)
///
/// With CsgFragments::Fewest, for a caller that merges the fragments of each
/// face back together (FragmentMerge) or wants only the volume, the work and
/// the pieces that make no difference to the result are left out: a polygon
/// clear of the other operand's box is not clipped to it, and what is left
/// of B is not built into A's tree, which only cut it along A's planes. The
/// same surface comes back, in fewer pieces. AsTree keeps csg.js's pieces,
/// which references made by position were made against. Fewest also keeps
/// whole, or leaves out whole, every polygon clear of each polygon of the
/// other operand, and cuts non-convex faces into few convex pieces, not
/// triangles.
enum class CsgFragments { AsTree, Fewest };

/// A CSG that would take far more work, or leave far more pieces, than any
/// part needs: thrown by csgExecute rather than going on for an hour, or
/// until memory runs out. Two bodies of tens of thousands of facets that
/// overlap through and through (a pattern of finely faceted cylinders and
/// its mirror image) cut every facet of each along the planes of thousands
/// of the other's. The limits are many times what the largest cut in the
/// tests takes.
class CsgTooComplex : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// How many polygon corners csgExecute may test against a plane, at least:
/// about fifteen times the most a cut in the tests takes (a prism of 25,000
/// sides less a box), and a minute or two of work. More for more polygons.
inline constexpr std::size_t kCsgWorkLimit = 4'000'000'000;
inline constexpr std::size_t kCsgWorkPerPolygon = 20'000;
/// How many times it may split a polygon in two, at least: a few gigabytes
/// of pieces. More for more polygons.
inline constexpr std::size_t kCsgSplitLimit = 4'000'000;
inline constexpr std::size_t kCsgSplitsPerPolygon = 20;

/// A CSG stopped because what it runs for was cancelled (a rebuild the
/// user has moved on from): thrown by csgExecute, never for anything about
/// the bodies.
class CsgCancelled : public std::runtime_error {
public:
    CsgCancelled() : std::runtime_error("cancelled") {}
};

/// The limits set on this thread in place of those above (BooleanWorkLimits),
/// zero where none is, and the flag that cancels it (BooleanCancellation).
struct CsgLimits {
    std::size_t work = 0;
    std::size_t splits = 0;
    const std::atomic<bool>* cancelled = nullptr;
};
CsgLimits& csgLimitsOnThisThread();

/// Throws CsgTooComplex beyond the limits above, and CsgCancelled once the
/// cancellation flag set on this thread is set.
std::vector<CsgPolygon> csgExecute(const std::vector<CsgPolygon>& a,
                                   const std::vector<CsgPolygon>& b, BooleanType type,
                                   double planeEps = kCsgPlaneEps,
                                   CsgFragments fragments = CsgFragments::AsTree);

/// Triangulate boundary polygons into the convex fragments the BSP needs.
std::vector<CsgPolygon> csgTriangles(const std::vector<BoundaryPolygon>& polygons, bool fromA);

/// The convex fragments the BSP needs, with each face that is already convex
/// and flat (to well within @p planeEps) and has no hole taken whole, and
/// the rest triangulated. A face of many sides (a cylinder's cap of 4,096)
/// fanned into slivers was cut by every plane of the other operand into
/// millions of fragments, which then had to be merged back: a hole through
/// such a cylinder took most of a minute. The pieces differ from
/// csgTriangles' only in how a face is cut up, so they are for a Boolean
/// that merges the fragments of each face back together (FragmentMerge);
/// one that keeps them, for references made by position, triangulates.
std::vector<CsgPolygon> csgPieces(const std::vector<BoundaryPolygon>& polygons, bool fromA,
                                  double planeEps);

/// Signed volume enclosed by a fragment set (divergence theorem over fan
/// triangles).  Positive for an outward-oriented closed boundary.
double csgVolume(const std::vector<CsgPolygon>& polygons);

}  // namespace hz::model
