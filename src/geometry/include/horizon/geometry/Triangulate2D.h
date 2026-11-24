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
    /// Empty when every boundary segment is an edge of the result and the loops are simple.
    /// Otherwise the first boundary segment that could not be kept (by loop and input index),
    /// or the first point where two loops touch, and why. The triangles are then best effort:
    /// callers treat a non-empty problem as failure.
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
/// The region is the triangles an odd number of boundary segments away from outside the
/// convex hull (a flood fill over the triangulation that flips at each boundary segment), so
/// no floating-point classification is involved. That needs simple loops that neither cross
/// nor touch; when they do, Triangulation2D::problem says where. A point shared by two loops
/// is reported at once; a crossing is found when constraint recovery fails (a segment would
/// have to cross another or pass through a vertex), not by testing every pair of segments.
Triangulation2D triangulateRegion(const std::vector<math::Vec2>& outer,
                                  const std::vector<std::vector<math::Vec2>>& holes,
                                  const std::vector<math::Vec2>& interior = {}, double merge = 0.0);

}  // namespace hz::geo
