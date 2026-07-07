#pragma once

#include <memory>
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
/// the BSP splitter's on-plane band.  Exported so downstream sewing can weld
/// at a tolerance >= this: any two points the splitter treated as coincident
/// on a shared plane must be reconcilable, or seams the splitter is allowed
/// to open could never be closed.
constexpr double kCsgPlaneEps = 1e-6;

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
/// The BSP tree is built without a balancing heuristic, so its traversals
/// (build/clip/invert and node destruction) recurse to tree depth, which for
/// the axis-aligned prismatic solids typical here is O(triangle count).  That
/// is fine for the part sizes this kernel targets; a very large mesh
/// (tens of thousands of triangles) could approach stack limits, at which
/// point a plane-selection heuristic or an iterative traversal would be the
/// fix.
std::vector<CsgPolygon> csgExecute(const std::vector<CsgPolygon>& a,
                                   const std::vector<CsgPolygon>& b, BooleanType type);

/// Triangulate boundary polygons into the convex fragments the BSP needs.
std::vector<CsgPolygon> csgTriangles(const std::vector<BoundaryPolygon>& polygons, bool fromA);

/// Signed volume enclosed by a fragment set (divergence theorem over fan
/// triangles).  Positive for an outward-oriented closed boundary.
double csgVolume(const std::vector<CsgPolygon>& polygons);

}  // namespace hz::model
