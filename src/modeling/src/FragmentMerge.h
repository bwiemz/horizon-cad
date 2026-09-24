#pragma once

#include <vector>

#include "MeshCsg.h"

namespace hz::model {

/// Put back together the fragments the CSG leaves of each source face.
///
/// The CSG triangulates both operands and splits the triangles along the
/// other operand's planes, then keeps the pieces apart: left alone, a Boolean
/// result is a triangle soup with every face in several pieces — even a face
/// the cut never reached. Here the fragments of one source face (the same
/// TopologyID, from the same operand) that lie in one plane are merged into
/// the fewest simple polygons: shared edges cancel, and vertices left
/// standing in a straight run go.
///
/// A face here has a single loop, so a merged region with a hole in it is
/// first cut through each hole, along a line through the hole's middle
/// parallel to the region's longest outer edge. The cuts meet the outer
/// boundary part-way along an edge, so the corners of the region stay corners
/// of that face alone — a plate with a hole keeps box-like corners.
///
/// Fragments that do not merge cleanly (regions that touch at a single
/// point, a hole no cut passes through) are returned as they came.
std::vector<CsgPolygon> mergeFragments(std::vector<CsgPolygon> fragments, double tol);

}  // namespace hz::model
