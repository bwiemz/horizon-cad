#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "horizon/math/Vec2.h"

namespace hz::geo {

struct Triangulation2D {
    /// The boundary points first: points[i], for every i below the total boundary count, is
    /// the i-th input boundary point (the outer loop, then each hole, in input order). A
    /// boundary point merged into an earlier one stays here but no triangle uses it. The
    /// interior points that were kept follow, in input order.
    std::vector<math::Vec2> points;
    /// Counter-clockwise triangles, indices into points.
    std::vector<std::array<std::uint32_t, 3>> triangles;
    /// For each boundary index i, the index its triangles use: i itself, or the lowest index
    /// of the boundary points it was merged into. Map an input boundary sample through this
    /// to find its vertex in the result.
    std::vector<std::uint32_t> boundaryVertex;
    /// Empty when every boundary segment is an edge of the result and the region is well
    /// defined: the outer loop has 3 or more distinct points, the loops do not cross or touch,
    /// every hole lies inside the outer loop and outside every other hole, and the inside and
    /// outside labelling is consistent. Otherwise it names the first fault (a segment by loop
    /// and input index, a point where loops touch, a misplaced hole, or where the labelling
    /// disagrees) and why. The triangles are then best effort: callers treat a non-empty
    /// problem as failure. An input point that is not finite gives that problem and nothing else.
    std::string problem;
};

/// The constrained Delaunay triangulation of the region inside @p outer and outside each of
/// @p holes (closed polylines, no closing duplicate, either winding), with @p interior points
/// added where they fall strictly inside it.
///
/// Every boundary segment is an edge of the result, unsplit (callers rely on it to keep a
/// mesh watertight), and the result is Delaunay except across those segments. Boundary
/// points within @p merge of an earlier one (distance 0 included, so exact duplicates always
/// merge) are one point; see Triangulation2D::points for how that shows in the indices. An
/// interior point within @p merge of another point, on a boundary segment, or not strictly
/// inside the region is dropped. Decisions use the exact predicates of math/RobustPredicates.
///
/// The region is the triangles an odd number of walls away from outside the convex hull (a
/// flood fill over the triangulation that flips at each wall), so no floating-point
/// classification is involved. A boundary segment is a wall unless the loops run along it an
/// even number of times (a slit, or a bridge to an inner loop, in opposite directions): that
/// stays an edge but divides nothing, as in a B-rep face with a bridge edge. Loops that cross
/// or touch are found when constraint recovery fails (a segment would have to cross another
/// or pass through a vertex), not by testing every pair of segments; a hole outside the outer
/// loop or inside another hole, and any labelling that disagrees between neighbours, are
/// reported too.
Triangulation2D triangulateRegion(const std::vector<math::Vec2>& outer,
                                  const std::vector<std::vector<math::Vec2>>& holes,
                                  const std::vector<math::Vec2>& interior = {}, double merge = 0.0);

}  // namespace hz::geo
