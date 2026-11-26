// Ruling 21: a bridge copy's facets, chords and seams are numbered as master's were. Master's
// Stable facets and edges were already `<face>/facet:<k>`, `<edge>/chord:<k>` and
// `<face>/seam:<k>`, and a saved reference binds by its whole name before its logical one
// (FilletOp's edges, MateGeometry's faces): a name that is still there but on another facet or
// chord would bind it silently. So each name master gave that the bridge still has names the same
// place, and each master gave that the bridge has not is accounted for.
//
// The tables are master's Stable output at 1709c556 (before the extrude was exact; its primitives'
// Stable path was PrimitiveFactory, scopeToFeature, nameFacetsLogically and nameEdgesLogically, and
// the extrude's its faceted prism, or its Booleans for a profile of several loops), at 8 segments
// a turn: each face's name with the centroid of its outer loop, each edge's with its midpoint.
// "{k}" in a name is the k-th entity of the profile ("e<id>" in the tag).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "horizon/document/FeatureTree.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

using hz::doc::PrimitiveFeature;
using hz::math::kPi;
using hz::math::Vec2;
using hz::math::Vec3;
using Profile = std::vector<std::shared_ptr<hz::draft::DraftEntity>>;

namespace {

/// A face's ("F <name>") outer loop's centroid, or an edge's ("E <name>") midpoint.
struct Place {
    const char* tag;
    double x, y, z;
};

// clang-format off
const Place kCylinder[] = {
    {"E primitive_1/edge:bottom|side/chord:0", 0.707106781187, -1.70710678119, 0},
    {"E primitive_1/edge:bottom|side/chord:1", -0.707106781187, -1.70710678119, 0},
    {"E primitive_1/edge:bottom|side/chord:2", -1.70710678119, -0.707106781187, 0},
    {"E primitive_1/edge:bottom|side/chord:3", -1.70710678119, 0.707106781187, 0},
    {"E primitive_1/edge:bottom|side/chord:4", -0.707106781187, 1.70710678119, 0},
    {"E primitive_1/edge:bottom|side/chord:5", 0.707106781187, 1.70710678119, 0},
    {"E primitive_1/edge:bottom|side/chord:6", 1.70710678119, 0.707106781187, 0},
    {"E primitive_1/edge:bottom|side/chord:7", 1.70710678119, -0.707106781187, 0},
    {"E primitive_1/edge:side|top/chord:0", 1.70710678119, 0.707106781187, 5},
    {"E primitive_1/edge:side|top/chord:1", 0.707106781187, 1.70710678119, 5},
    {"E primitive_1/edge:side|top/chord:2", -0.707106781187, 1.70710678119, 5},
    {"E primitive_1/edge:side|top/chord:3", -1.70710678119, 0.707106781187, 5},
    {"E primitive_1/edge:side|top/chord:4", -1.70710678119, -0.707106781187, 5},
    {"E primitive_1/edge:side|top/chord:5", -0.707106781187, -1.70710678119, 5},
    {"E primitive_1/edge:side|top/chord:6", 0.707106781187, -1.70710678119, 5},
    {"E primitive_1/edge:side|top/chord:7", 1.70710678119, -0.707106781187, 5},
    {"E primitive_1/side/seam:0", 1.41421356237, 1.41421356237, 2.5},
    {"E primitive_1/side/seam:1", 2, 0, 2.5},
    {"E primitive_1/side/seam:2", 0, 2, 2.5},
    {"E primitive_1/side/seam:3", -1.41421356237, 1.41421356237, 2.5},
    {"E primitive_1/side/seam:4", -2, 0, 2.5},
    {"E primitive_1/side/seam:5", -1.41421356237, -1.41421356237, 2.5},
    {"E primitive_1/side/seam:6", 0, -2, 2.5},
    {"E primitive_1/side/seam:7", 1.41421356237, -1.41421356237, 2.5},
    {"F primitive_1/bottom", 0, 0, 0},
    {"F primitive_1/side/facet:0", 1.70710678119, 0.707106781187, 2.5},
    {"F primitive_1/side/facet:1", 0.707106781187, 1.70710678119, 2.5},
    {"F primitive_1/side/facet:2", -0.707106781187, 1.70710678119, 2.5},
    {"F primitive_1/side/facet:3", -1.70710678119, 0.707106781187, 2.5},
    {"F primitive_1/side/facet:4", -1.70710678119, -0.707106781187, 2.5},
    {"F primitive_1/side/facet:5", -0.707106781187, -1.70710678119, 2.5},
    {"F primitive_1/side/facet:6", 0.707106781187, -1.70710678119, 2.5},
    {"F primitive_1/side/facet:7", 1.70710678119, -0.707106781187, 2.5},
    {"F primitive_1/top", 0, 0, 5},
};
const Place kPlacedCylinder[] = {
    {"E primitive_1/edge:bottom|side/chord:0", 1.91354789371, -3.01896973943, 4.2415089522},
    {"E primitive_1/edge:bottom|side/chord:1", 0.557920970703, -2.82368094155, 4.59384108391},
    {"E primitive_1/edge:bottom|side/chord:2", -0.538742052585, -2.14589101918, 4.01252272493},
    {"E primitive_1/edge:bottom|side/chord:3", -0.734030850463, -1.3826401164, 2.8380822859},
    {"E primitive_1/edge:bottom|side/chord:4", 0.0864521062872, -0.981030260568, 1.7584910478},
    {"E primitive_1/edge:bottom|side/chord:5", 1.4420790293, -1.17631905845, 1.40615891609},
    {"E primitive_1/edge:bottom|side/chord:6", 2.53874205259, -1.85410898082, 1.98747727507},
    {"E primitive_1/edge:bottom|side/chord:7", 2.73403085046, -2.6173598836, 3.1619177141},
    {"E primitive_1/edge:side|top/chord:0", 3.78442425039, 2.29816501187, 4.47884167068},
    {"E primitive_1/edge:side|top/chord:1", 2.6877612271, 2.97595493424, 3.89752331171},
    {"E primitive_1/edge:side|top/chord:2", 1.33213430409, 3.17124373212, 4.24985544341},
    {"E primitive_1/edge:side|top/chord:3", 0.511651347343, 2.76963387629, 5.32944668151},
    {"E primitive_1/edge:side|top/chord:4", 0.706940145221, 2.0063829735, 6.50388712054},
    {"E primitive_1/edge:side|top/chord:5", 1.80360316851, 1.32859305113, 7.08520547952},
    {"E primitive_1/edge:side|top/chord:6", 3.15923009152, 1.13330425325, 6.73287334781},
    {"E primitive_1/edge:side|top/chord:7", 3.97971304827, 1.53491410909, 5.65328210971},
    {"E primitive_1/side/seam:0", 2.78317922404, 0.644099101247, 2.71890962707},
    {"E primitive_1/side/seam:1", 3.53998707894, -0.200043070195, 3.74740931868},
    {"E primitive_1/side/seam:2", 1.34666103236, 1.15553677455, 2.58477260073},
    {"E primitive_1/side/seam:3", 0.0719253780156, 1.034676697, 3.42357389049},
    {"E primitive_1/side/seam:4", -0.294304881136, 0.352317062882, 4.74395507693},
    {"E primitive_1/side/seam:5", 0.462502973771, -0.49182510856, 5.77245476854},
    {"E primitive_1/side/seam:6", 1.89902116544, -1.00326278186, 5.90659179488},
    {"E primitive_1/side/seam:7", 3.17375681979, -0.882402704315, 5.06779050513},
    {"F primitive_1/bottom", 1, -2, 3},
    {"F primitive_1/side/facet:0", 3.16158315149, 0.222028015526, 3.23315947288},
    {"F primitive_1/side/facet:1", 2.0649201282, 0.899817937898, 2.6518411139},
    {"F primitive_1/side/facet:2", 0.70929320519, 1.09510673578, 3.00417324561},
    {"F primitive_1/side/facet:3", -0.11118975156, 0.693496879942, 4.08376448371},
    {"F primitive_1/side/facet:4", 0.0840990463177, -0.0697540228391, 5.25820492274},
    {"F primitive_1/side/facet:5", 1.18076206961, -0.747543945211, 5.83952328171},
    {"F primitive_1/side/facet:6", 2.53638899262, -0.942832743089, 5.48719115},
    {"F primitive_1/side/facet:7", 3.35687194937, -0.541222887255, 4.40759991191},
    {"F primitive_1/top", 2.24568219781, 2.15227399269, 5.49136439561},
};
const Place kSphere[] = {
    {"E primitive_1/band/seam:0", 1.81066017178, 0.75, 2.12132034356},
    {"E primitive_1/band/seam:1", 0.75, 0.75, 2.56066017178},
    {"E primitive_1/band/seam:10", -0.75, -0.75, 2.56066017178},
    {"E primitive_1/band/seam:11", -0.75, -1.81066017178, 2.12132034356},
    {"E primitive_1/band/seam:12", 0, -1.06066017178, 2.56066017178},
    {"E primitive_1/band/seam:13", 0.75, -1.81066017178, 2.12132034356},
    {"E primitive_1/band/seam:14", 0.75, -0.75, 2.56066017178},
    {"E primitive_1/band/seam:15", 1.81066017178, -0.75, 2.12132034356},
    {"E primitive_1/band/seam:16", 2.56066017178, 1.06066017178, 0},
    {"E primitive_1/band/seam:17", 1.81066017178, 1.81066017178, 1.06066017178},
    {"E primitive_1/band/seam:18", 2.56066017178, 0, 1.06066017178},
    {"E primitive_1/band/seam:19", 1.06066017178, 2.56066017178, 0},
    {"E primitive_1/band/seam:2", 1.06066017178, 0, 2.56066017178},
    {"E primitive_1/band/seam:20", 0, 2.56066017178, 1.06066017178},
    {"E primitive_1/band/seam:21", -1.06066017178, 2.56066017178, 0},
    {"E primitive_1/band/seam:22", -1.81066017178, 1.81066017178, 1.06066017178},
    {"E primitive_1/band/seam:23", -2.56066017178, 1.06066017178, 0},
    {"E primitive_1/band/seam:24", -2.56066017178, 0, 1.06066017178},
    {"E primitive_1/band/seam:25", -2.56066017178, -1.06066017178, 0},
    {"E primitive_1/band/seam:26", -1.81066017178, -1.81066017178, 1.06066017178},
    {"E primitive_1/band/seam:27", -1.06066017178, -2.56066017178, 0},
    {"E primitive_1/band/seam:28", 0, -2.56066017178, 1.06066017178},
    {"E primitive_1/band/seam:29", 1.06066017178, -2.56066017178, 0},
    {"E primitive_1/band/seam:3", 0.75, 1.81066017178, 2.12132034356},
    {"E primitive_1/band/seam:30", 1.81066017178, -1.81066017178, 1.06066017178},
    {"E primitive_1/band/seam:31", 2.56066017178, -1.06066017178, 0},
    {"E primitive_1/band/seam:32", 1.81066017178, 0.75, -2.12132034356},
    {"E primitive_1/band/seam:33", 1.81066017178, 1.81066017178, -1.06066017178},
    {"E primitive_1/band/seam:34", 2.56066017178, 0, -1.06066017178},
    {"E primitive_1/band/seam:35", 0.75, 1.81066017178, -2.12132034356},
    {"E primitive_1/band/seam:36", 0, 2.56066017178, -1.06066017178},
    {"E primitive_1/band/seam:37", -0.75, 1.81066017178, -2.12132034356},
    {"E primitive_1/band/seam:38", -1.81066017178, 1.81066017178, -1.06066017178},
    {"E primitive_1/band/seam:39", -1.81066017178, 0.75, -2.12132034356},
    {"E primitive_1/band/seam:4", 0, 1.06066017178, 2.56066017178},
    {"E primitive_1/band/seam:40", -2.56066017178, 0, -1.06066017178},
    {"E primitive_1/band/seam:41", -1.81066017178, -0.75, -2.12132034356},
    {"E primitive_1/band/seam:42", -1.81066017178, -1.81066017178, -1.06066017178},
    {"E primitive_1/band/seam:43", -0.75, -1.81066017178, -2.12132034356},
    {"E primitive_1/band/seam:44", 0, -2.56066017178, -1.06066017178},
    {"E primitive_1/band/seam:45", 0.75, -1.81066017178, -2.12132034356},
    {"E primitive_1/band/seam:46", 1.81066017178, -1.81066017178, -1.06066017178},
    {"E primitive_1/band/seam:47", 1.81066017178, -0.75, -2.12132034356},
    {"E primitive_1/band/seam:48", 0.75, 0.75, -2.56066017178},
    {"E primitive_1/band/seam:49", 1.06066017178, 0, -2.56066017178},
    {"E primitive_1/band/seam:5", -0.75, 1.81066017178, 2.12132034356},
    {"E primitive_1/band/seam:50", 0, 1.06066017178, -2.56066017178},
    {"E primitive_1/band/seam:51", -0.75, 0.75, -2.56066017178},
    {"E primitive_1/band/seam:52", -1.06066017178, 0, -2.56066017178},
    {"E primitive_1/band/seam:53", -0.75, -0.75, -2.56066017178},
    {"E primitive_1/band/seam:54", 0, -1.06066017178, -2.56066017178},
    {"E primitive_1/band/seam:55", 0.75, -0.75, -2.56066017178},
    {"E primitive_1/band/seam:6", -0.75, 0.75, 2.56066017178},
    {"E primitive_1/band/seam:7", -1.81066017178, 0.75, 2.12132034356},
    {"E primitive_1/band/seam:8", -1.06066017178, 0, 2.56066017178},
    {"E primitive_1/band/seam:9", -1.81066017178, -0.75, 2.12132034356},
    {"F primitive_1/band/facet:0", 1.20710678119, 0.5, 2.41421356237},
    {"F primitive_1/band/facet:1", 0.5, 1.20710678119, 2.41421356237},
    {"F primitive_1/band/facet:10", -0.914213562373, 2.20710678119, 1},
    {"F primitive_1/band/facet:11", -2.20710678119, 0.914213562373, 1},
    {"F primitive_1/band/facet:12", -2.20710678119, -0.914213562373, 1},
    {"F primitive_1/band/facet:13", -0.914213562373, -2.20710678119, 1},
    {"F primitive_1/band/facet:14", 0.914213562373, -2.20710678119, 1},
    {"F primitive_1/band/facet:15", 2.20710678119, -0.914213562373, 1},
    {"F primitive_1/band/facet:16", 2.20710678119, 0.914213562373, -1},
    {"F primitive_1/band/facet:17", 0.914213562373, 2.20710678119, -1},
    {"F primitive_1/band/facet:18", -0.914213562373, 2.20710678119, -1},
    {"F primitive_1/band/facet:19", -2.20710678119, 0.914213562373, -1},
    {"F primitive_1/band/facet:2", -0.5, 1.20710678119, 2.41421356237},
    {"F primitive_1/band/facet:20", -2.20710678119, -0.914213562373, -1},
    {"F primitive_1/band/facet:21", -0.914213562373, -2.20710678119, -1},
    {"F primitive_1/band/facet:22", 0.914213562373, -2.20710678119, -1},
    {"F primitive_1/band/facet:23", 2.20710678119, -0.914213562373, -1},
    {"F primitive_1/band/facet:24", 1.20710678119, 0.5, -2.41421356237},
    {"F primitive_1/band/facet:25", 0.5, 1.20710678119, -2.41421356237},
    {"F primitive_1/band/facet:26", -0.5, 1.20710678119, -2.41421356237},
    {"F primitive_1/band/facet:27", -1.20710678119, 0.5, -2.41421356237},
    {"F primitive_1/band/facet:28", -1.20710678119, -0.5, -2.41421356237},
    {"F primitive_1/band/facet:29", -0.5, -1.20710678119, -2.41421356237},
    {"F primitive_1/band/facet:3", -1.20710678119, 0.5, 2.41421356237},
    {"F primitive_1/band/facet:30", 0.5, -1.20710678119, -2.41421356237},
    {"F primitive_1/band/facet:31", 1.20710678119, -0.5, -2.41421356237},
    {"F primitive_1/band/facet:4", -1.20710678119, -0.5, 2.41421356237},
    {"F primitive_1/band/facet:5", -0.5, -1.20710678119, 2.41421356237},
    {"F primitive_1/band/facet:6", 0.5, -1.20710678119, 2.41421356237},
    {"F primitive_1/band/facet:7", 1.20710678119, -0.5, 2.41421356237},
    {"F primitive_1/band/facet:8", 2.20710678119, 0.914213562373, 1},
    {"F primitive_1/band/facet:9", 0.914213562373, 2.20710678119, 1},
};
const Place kFrustum[] = {
    {"E primitive_1/edge:bottom|side/chord:0", 1.06066017178, -2.56066017178, 0},
    {"E primitive_1/edge:bottom|side/chord:1", -1.06066017178, -2.56066017178, 0},
    {"E primitive_1/edge:bottom|side/chord:2", -2.56066017178, -1.06066017178, 0},
    {"E primitive_1/edge:bottom|side/chord:3", -2.56066017178, 1.06066017178, 0},
    {"E primitive_1/edge:bottom|side/chord:4", -1.06066017178, 2.56066017178, 0},
    {"E primitive_1/edge:bottom|side/chord:5", 1.06066017178, 2.56066017178, 0},
    {"E primitive_1/edge:bottom|side/chord:6", 2.56066017178, 1.06066017178, 0},
    {"E primitive_1/edge:bottom|side/chord:7", 2.56066017178, -1.06066017178, 0},
    {"E primitive_1/edge:side|top/chord:0", 0.853553390593, 0.353553390593, 4},
    {"E primitive_1/edge:side|top/chord:1", 0.353553390593, 0.853553390593, 4},
    {"E primitive_1/edge:side|top/chord:2", -0.353553390593, 0.853553390593, 4},
    {"E primitive_1/edge:side|top/chord:3", -0.853553390593, 0.353553390593, 4},
    {"E primitive_1/edge:side|top/chord:4", -0.853553390593, -0.353553390593, 4},
    {"E primitive_1/edge:side|top/chord:5", -0.353553390593, -0.853553390593, 4},
    {"E primitive_1/edge:side|top/chord:6", 0.353553390593, -0.853553390593, 4},
    {"E primitive_1/edge:side|top/chord:7", 0.853553390593, -0.353553390593, 4},
    {"E primitive_1/side/seam:0", 1.41421356237, 1.41421356237, 2},
    {"E primitive_1/side/seam:1", 2, 0, 2},
    {"E primitive_1/side/seam:2", 0, 2, 2},
    {"E primitive_1/side/seam:3", -1.41421356237, 1.41421356237, 2},
    {"E primitive_1/side/seam:4", -2, 0, 2},
    {"E primitive_1/side/seam:5", -1.41421356237, -1.41421356237, 2},
    {"E primitive_1/side/seam:6", 0, -2, 2},
    {"E primitive_1/side/seam:7", 1.41421356237, -1.41421356237, 2},
    {"F primitive_1/bottom", 0, 0, 0},
    {"F primitive_1/side/facet:0", 1.84936567962, 0.766032346285, 1.66666666667},
    {"F primitive_1/side/facet:1", 0.766032346285, 1.84936567962, 1.66666666667},
    {"F primitive_1/side/facet:2", -0.766032346285, 1.84936567962, 1.66666666667},
    {"F primitive_1/side/facet:3", -1.84936567962, 0.766032346285, 1.66666666667},
    {"F primitive_1/side/facet:4", -1.84936567962, -0.766032346285, 1.66666666667},
    {"F primitive_1/side/facet:5", -0.766032346285, -1.84936567962, 1.66666666667},
    {"F primitive_1/side/facet:6", 0.766032346285, -1.84936567962, 1.66666666667},
    {"F primitive_1/side/facet:7", 1.84936567962, -0.766032346285, 1.66666666667},
    {"F primitive_1/top", 0, 0, 4},
};
const Place kConeToItsTop[] = {
    {"E primitive_1/edge:bottom|side/chord:0", 0.707106781187, -1.70710678119, 0},
    {"E primitive_1/edge:bottom|side/chord:1", -0.707106781187, -1.70710678119, 0},
    {"E primitive_1/edge:bottom|side/chord:2", -1.70710678119, -0.707106781187, 0},
    {"E primitive_1/edge:bottom|side/chord:3", -1.70710678119, 0.707106781187, 0},
    {"E primitive_1/edge:bottom|side/chord:4", -0.707106781187, 1.70710678119, 0},
    {"E primitive_1/edge:bottom|side/chord:5", 0.707106781187, 1.70710678119, 0},
    {"E primitive_1/edge:bottom|side/chord:6", 1.70710678119, 0.707106781187, 0},
    {"E primitive_1/edge:bottom|side/chord:7", 1.70710678119, -0.707106781187, 0},
    {"E primitive_1/side/seam:0", 0.707106781187, 0.707106781187, 1.5},
    {"E primitive_1/side/seam:1", 1, 0, 1.5},
    {"E primitive_1/side/seam:2", 0, 1, 1.5},
    {"E primitive_1/side/seam:3", -0.707106781187, 0.707106781187, 1.5},
    {"E primitive_1/side/seam:4", -1, 0, 1.5},
    {"E primitive_1/side/seam:5", -0.707106781187, -0.707106781187, 1.5},
    {"E primitive_1/side/seam:6", 0, -1, 1.5},
    {"E primitive_1/side/seam:7", 0.707106781187, -0.707106781187, 1.5},
    {"F primitive_1/bottom", 0, 0, 0},
    {"F primitive_1/side/facet:0", 1.13807118746, 0.471404520791, 1},
    {"F primitive_1/side/facet:1", 0.471404520791, 1.13807118746, 1},
    {"F primitive_1/side/facet:2", -0.471404520791, 1.13807118746, 1},
    {"F primitive_1/side/facet:3", -1.13807118746, 0.471404520791, 1},
    {"F primitive_1/side/facet:4", -1.13807118746, -0.471404520791, 1},
    {"F primitive_1/side/facet:5", -0.471404520791, -1.13807118746, 1},
    {"F primitive_1/side/facet:6", 0.471404520791, -1.13807118746, 1},
    {"F primitive_1/side/facet:7", 1.13807118746, -0.471404520791, 1},
};
const Place kConeFromItsBottom[] = {
    {"E primitive_1/edge:side|top/chord:0", 1.70710678119, 0.707106781187, 3},
    {"E primitive_1/edge:side|top/chord:1", 0.707106781187, 1.70710678119, 3},
    {"E primitive_1/edge:side|top/chord:2", -0.707106781187, 1.70710678119, 3},
    {"E primitive_1/edge:side|top/chord:3", -1.70710678119, 0.707106781187, 3},
    {"E primitive_1/edge:side|top/chord:4", -1.70710678119, -0.707106781187, 3},
    {"E primitive_1/edge:side|top/chord:5", -0.707106781187, -1.70710678119, 3},
    {"E primitive_1/edge:side|top/chord:6", 0.707106781187, -1.70710678119, 3},
    {"E primitive_1/edge:side|top/chord:7", 1.70710678119, -0.707106781187, 3},
    {"E primitive_1/side/seam:0", 0.707106781187, 0.707106781187, 1.5},
    {"E primitive_1/side/seam:1", 1, 0, 1.5},
    {"E primitive_1/side/seam:2", 0, 1, 1.5},
    {"E primitive_1/side/seam:3", -0.707106781187, 0.707106781187, 1.5},
    {"E primitive_1/side/seam:4", -1, 0, 1.5},
    {"E primitive_1/side/seam:5", -0.707106781187, -0.707106781187, 1.5},
    {"E primitive_1/side/seam:6", 0, -1, 1.5},
    {"E primitive_1/side/seam:7", 0.707106781187, -0.707106781187, 1.5},
    {"F primitive_1/side/facet:0", 1.13807118746, 0.471404520791, 2},
    {"F primitive_1/side/facet:1", 0.471404520791, 1.13807118746, 2},
    {"F primitive_1/side/facet:2", -0.471404520791, 1.13807118746, 2},
    {"F primitive_1/side/facet:3", -1.13807118746, 0.471404520791, 2},
    {"F primitive_1/side/facet:4", -1.13807118746, -0.471404520791, 2},
    {"F primitive_1/side/facet:5", -0.471404520791, -1.13807118746, 2},
    {"F primitive_1/side/facet:6", 0.471404520791, -1.13807118746, 2},
    {"F primitive_1/side/facet:7", 1.13807118746, -0.471404520791, 2},
    {"F primitive_1/top", 0, 0, 3},
};
const Place kTorus[] = {
    {"E primitive_1/patch/seam:0", 5.54809703886, 2.29809703886, 0},
    {"E primitive_1/patch/seam:1", 4.06586399182, 4.06586399182, 0.75},
    {"E primitive_1/patch/seam:10", 4.06586399182, 4.06586399182, -0.75},
    {"E primitive_1/patch/seam:11", 5.75, 0, -0.75},
    {"E primitive_1/patch/seam:12", 2.29809703886, 5.54809703886, 0},
    {"E primitive_1/patch/seam:13", 0, 5.75, 0.75},
    {"E primitive_1/patch/seam:14", 1.76776695297, 4.26776695297, 1.5},
    {"E primitive_1/patch/seam:15", 0, 4.25, 0.75},
    {"E primitive_1/patch/seam:16", 1.23743686708, 2.98743686708, 0},
    {"E primitive_1/patch/seam:17", 0, 4.25, -0.75},
    {"E primitive_1/patch/seam:18", 1.76776695297, 4.26776695297, -1.5},
    {"E primitive_1/patch/seam:19", 0, 5.75, -0.75},
    {"E primitive_1/patch/seam:2", 4.26776695297, 1.76776695297, 1.5},
    {"E primitive_1/patch/seam:20", -2.29809703886, 5.54809703886, 0},
    {"E primitive_1/patch/seam:21", -4.06586399182, 4.06586399182, 0.75},
    {"E primitive_1/patch/seam:22", -1.76776695297, 4.26776695297, 1.5},
    {"E primitive_1/patch/seam:23", -3.00520382004, 3.00520382004, 0.75},
    {"E primitive_1/patch/seam:24", -1.23743686708, 2.98743686708, 0},
    {"E primitive_1/patch/seam:25", -3.00520382004, 3.00520382004, -0.75},
    {"E primitive_1/patch/seam:26", -1.76776695297, 4.26776695297, -1.5},
    {"E primitive_1/patch/seam:27", -4.06586399182, 4.06586399182, -0.75},
    {"E primitive_1/patch/seam:28", -5.54809703886, 2.29809703886, 0},
    {"E primitive_1/patch/seam:29", -5.75, 0, 0.75},
    {"E primitive_1/patch/seam:3", 5.75, 0, 0.75},
    {"E primitive_1/patch/seam:30", -4.26776695297, 1.76776695297, 1.5},
    {"E primitive_1/patch/seam:31", -4.25, 0, 0.75},
    {"E primitive_1/patch/seam:32", -2.98743686708, 1.23743686708, 0},
    {"E primitive_1/patch/seam:33", -4.25, 0, -0.75},
    {"E primitive_1/patch/seam:34", -4.26776695297, 1.76776695297, -1.5},
    {"E primitive_1/patch/seam:35", -5.75, 0, -0.75},
    {"E primitive_1/patch/seam:36", -5.54809703886, -2.29809703886, 0},
    {"E primitive_1/patch/seam:37", -4.06586399182, -4.06586399182, 0.75},
    {"E primitive_1/patch/seam:38", -4.26776695297, -1.76776695297, 1.5},
    {"E primitive_1/patch/seam:39", -3.00520382004, -3.00520382004, 0.75},
    {"E primitive_1/patch/seam:4", 3.00520382004, 3.00520382004, 0.75},
    {"E primitive_1/patch/seam:40", -2.98743686708, -1.23743686708, 0},
    {"E primitive_1/patch/seam:41", -3.00520382004, -3.00520382004, -0.75},
    {"E primitive_1/patch/seam:42", -4.26776695297, -1.76776695297, -1.5},
    {"E primitive_1/patch/seam:43", -4.06586399182, -4.06586399182, -0.75},
    {"E primitive_1/patch/seam:44", -2.29809703886, -5.54809703886, 0},
    {"E primitive_1/patch/seam:45", 0, -5.75, 0.75},
    {"E primitive_1/patch/seam:46", -1.76776695297, -4.26776695297, 1.5},
    {"E primitive_1/patch/seam:47", 0, -4.25, 0.75},
    {"E primitive_1/patch/seam:48", -1.23743686708, -2.98743686708, 0},
    {"E primitive_1/patch/seam:49", 0, -4.25, -0.75},
    {"E primitive_1/patch/seam:5", 2.98743686708, 1.23743686708, 0},
    {"E primitive_1/patch/seam:50", -1.76776695297, -4.26776695297, -1.5},
    {"E primitive_1/patch/seam:51", 0, -5.75, -0.75},
    {"E primitive_1/patch/seam:52", 2.29809703886, -5.54809703886, 0},
    {"E primitive_1/patch/seam:53", 4.06586399182, -4.06586399182, 0.75},
    {"E primitive_1/patch/seam:54", 1.76776695297, -4.26776695297, 1.5},
    {"E primitive_1/patch/seam:55", 3.00520382004, -3.00520382004, 0.75},
    {"E primitive_1/patch/seam:56", 1.23743686708, -2.98743686708, 0},
    {"E primitive_1/patch/seam:57", 3.00520382004, -3.00520382004, -0.75},
    {"E primitive_1/patch/seam:58", 1.76776695297, -4.26776695297, -1.5},
    {"E primitive_1/patch/seam:59", 4.06586399182, -4.06586399182, -0.75},
    {"E primitive_1/patch/seam:6", 4.25, 0, 0.75},
    {"E primitive_1/patch/seam:60", 5.54809703886, -2.29809703886, 0},
    {"E primitive_1/patch/seam:61", 4.26776695297, -1.76776695297, 1.5},
    {"E primitive_1/patch/seam:62", 2.98743686708, -1.23743686708, 0},
    {"E primitive_1/patch/seam:63", 4.26776695297, -1.76776695297, -1.5},
    {"E primitive_1/patch/seam:7", 3.00520382004, 3.00520382004, -0.75},
    {"E primitive_1/patch/seam:8", 4.26776695297, 1.76776695297, -1.5},
    {"E primitive_1/patch/seam:9", 4.25, 0, -0.75},
    {"F primitive_1/patch/facet:0", 4.93576525865, 2.04446091082, 0.717391304348},
    {"F primitive_1/patch/facet:1", 3.66525867725, 1.51819985372, 0.794117647059},
    {"F primitive_1/patch/facet:10", -1.51819985372, 3.66525867725, -0.794117647059},
    {"F primitive_1/patch/facet:11", -2.04446091082, 4.93576525865, -0.717391304348},
    {"F primitive_1/patch/facet:12", -4.93576525865, 2.04446091082, 0.717391304348},
    {"F primitive_1/patch/facet:13", -3.66525867725, 1.51819985372, 0.794117647059},
    {"F primitive_1/patch/facet:14", -3.66525867725, 1.51819985372, -0.794117647059},
    {"F primitive_1/patch/facet:15", -4.93576525865, 2.04446091082, -0.717391304348},
    {"F primitive_1/patch/facet:16", -4.93576525865, -2.04446091082, 0.717391304348},
    {"F primitive_1/patch/facet:17", -3.66525867725, -1.51819985372, 0.794117647059},
    {"F primitive_1/patch/facet:18", -3.66525867725, -1.51819985372, -0.794117647059},
    {"F primitive_1/patch/facet:19", -4.93576525865, -2.04446091082, -0.717391304348},
    {"F primitive_1/patch/facet:2", 3.66525867725, 1.51819985372, -0.794117647059},
    {"F primitive_1/patch/facet:20", -2.04446091082, -4.93576525865, 0.717391304348},
    {"F primitive_1/patch/facet:21", -1.51819985372, -3.66525867725, 0.794117647059},
    {"F primitive_1/patch/facet:22", -1.51819985372, -3.66525867725, -0.794117647059},
    {"F primitive_1/patch/facet:23", -2.04446091082, -4.93576525865, -0.717391304348},
    {"F primitive_1/patch/facet:24", 2.04446091082, -4.93576525865, 0.717391304348},
    {"F primitive_1/patch/facet:25", 1.51819985372, -3.66525867725, 0.794117647059},
    {"F primitive_1/patch/facet:26", 1.51819985372, -3.66525867725, -0.794117647059},
    {"F primitive_1/patch/facet:27", 2.04446091082, -4.93576525865, -0.717391304348},
    {"F primitive_1/patch/facet:28", 4.93576525865, -2.04446091082, 0.717391304348},
    {"F primitive_1/patch/facet:29", 3.66525867725, -1.51819985372, 0.794117647059},
    {"F primitive_1/patch/facet:3", 4.93576525865, 2.04446091082, -0.717391304348},
    {"F primitive_1/patch/facet:30", 3.66525867725, -1.51819985372, -0.794117647059},
    {"F primitive_1/patch/facet:31", 4.93576525865, -2.04446091082, -0.717391304348},
    {"F primitive_1/patch/facet:4", 2.04446091082, 4.93576525865, 0.717391304348},
    {"F primitive_1/patch/facet:5", 1.51819985372, 3.66525867725, 0.794117647059},
    {"F primitive_1/patch/facet:6", 1.51819985372, 3.66525867725, -0.794117647059},
    {"F primitive_1/patch/facet:7", 2.04446091082, 4.93576525865, -0.717391304348},
    {"F primitive_1/patch/facet:8", -2.04446091082, 4.93576525865, 0.717391304348},
    {"F primitive_1/patch/facet:9", -1.51819985372, 3.66525867725, 0.794117647059},
};
const Place kBox[] = {
    {"E primitive_1/edge:back|bottom", 1, 3, 0},
    {"E primitive_1/edge:back|left", 0, 3, 2},
    {"E primitive_1/edge:back|right", 2, 3, 2},
    {"E primitive_1/edge:back|top", 1, 3, 4},
    {"E primitive_1/edge:bottom|front", 1, 0, 0},
    {"E primitive_1/edge:bottom|left", 0, 1.5, 0},
    {"E primitive_1/edge:bottom|right", 2, 1.5, 0},
    {"E primitive_1/edge:front|left", 0, 0, 2},
    {"E primitive_1/edge:front|right", 2, 0, 2},
    {"E primitive_1/edge:front|top", 1, 0, 4},
    {"E primitive_1/edge:left|top", 0, 1.5, 4},
    {"E primitive_1/edge:right|top", 2, 1.5, 4},
    {"F primitive_1/back", 1, 3, 2},
    {"F primitive_1/bottom", 1, 1.5, 0},
    {"F primitive_1/front", 1, 0, 2},
    {"F primitive_1/left", 0, 1.5, 2},
    {"F primitive_1/right", 2, 1.5, 2},
    {"F primitive_1/top", 1, 1.5, 4},
};
const Place kSlotUp[] = {
    {"E x/edge:cap_bottom|side:{0}", 10, -5, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:0", 21.767766953, -4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:1", 24.267766953, -1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:2", 24.267766953, 1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:3", 21.767766953, 4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{2}", 10, 5, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:0", -1.76776695297, 4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:1", -4.26776695297, 1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:2", -4.26776695297, -1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:3", -1.76776695297, -4.26776695297, 0},
    {"E x/edge:cap_top|side:{0}", 10, -5, 3},
    {"E x/edge:cap_top|side:{1}/chord:0", 21.767766953, -4.26776695297, 3},
    {"E x/edge:cap_top|side:{1}/chord:1", 24.267766953, -1.76776695297, 3},
    {"E x/edge:cap_top|side:{1}/chord:2", 24.267766953, 1.76776695297, 3},
    {"E x/edge:cap_top|side:{1}/chord:3", 21.767766953, 4.26776695297, 3},
    {"E x/edge:cap_top|side:{2}", 10, 5, 3},
    {"E x/edge:cap_top|side:{3}/chord:0", -1.76776695297, 4.26776695297, 3},
    {"E x/edge:cap_top|side:{3}/chord:1", -4.26776695297, 1.76776695297, 3},
    {"E x/edge:cap_top|side:{3}/chord:2", -4.26776695297, -1.76776695297, 3},
    {"E x/edge:cap_top|side:{3}/chord:3", -1.76776695297, -4.26776695297, 3},
    {"E x/edge:side:{0}|side:{1}", 20, -5, 1.5},
    {"E x/edge:side:{0}|side:{3}", 0, -5, 1.5},
    {"E x/edge:side:{1}|side:{2}", 20, 5, 1.5},
    {"E x/edge:side:{2}|side:{3}", 0, 5, 1.5},
    {"E x/side:{1}/seam:0", 23.5355339059, -3.53553390593, 1.5},
    {"E x/side:{1}/seam:1", 25, 0, 1.5},
    {"E x/side:{1}/seam:2", 23.5355339059, 3.53553390593, 1.5},
    {"E x/side:{3}/seam:0", -3.53553390593, 3.53553390593, 1.5},
    {"E x/side:{3}/seam:1", -5, 0, 1.5},
    {"E x/side:{3}/seam:2", -3.53553390593, -3.53553390593, 1.5},
    {"F x/cap_bottom", 10, 0, 0},
    {"F x/cap_top", 10, 0, 3},
    {"F x/side:{0}", 10, -5, 1.5},
    {"F x/side:{1}/facet:0", 21.767766953, -4.26776695297, 1.5},
    {"F x/side:{1}/facet:1", 24.267766953, -1.76776695297, 1.5},
    {"F x/side:{1}/facet:2", 24.267766953, 1.76776695297, 1.5},
    {"F x/side:{1}/facet:3", 21.767766953, 4.26776695297, 1.5},
    {"F x/side:{2}", 10, 5, 1.5},
    {"F x/side:{3}/facet:0", -1.76776695297, 4.26776695297, 1.5},
    {"F x/side:{3}/facet:1", -4.26776695297, 1.76776695297, 1.5},
    {"F x/side:{3}/facet:2", -4.26776695297, -1.76776695297, 1.5},
    {"F x/side:{3}/facet:3", -1.76776695297, -4.26776695297, 1.5},
};
const Place kSlotDown[] = {
    {"E x/edge:cap_bottom|side:{0}", 10, -5, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:0", 21.767766953, -4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:1", 24.267766953, -1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:2", 24.267766953, 1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:3", 21.767766953, 4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{2}", 10, 5, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:0", -1.76776695297, 4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:1", -4.26776695297, 1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:2", -4.26776695297, -1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:3", -1.76776695297, -4.26776695297, 0},
    {"E x/edge:cap_top|side:{0}", 10, -5, -3},
    {"E x/edge:cap_top|side:{1}/chord:0", 21.767766953, -4.26776695297, -3},
    {"E x/edge:cap_top|side:{1}/chord:1", 24.267766953, -1.76776695297, -3},
    {"E x/edge:cap_top|side:{1}/chord:2", 24.267766953, 1.76776695297, -3},
    {"E x/edge:cap_top|side:{1}/chord:3", 21.767766953, 4.26776695297, -3},
    {"E x/edge:cap_top|side:{2}", 10, 5, -3},
    {"E x/edge:cap_top|side:{3}/chord:0", -1.76776695297, 4.26776695297, -3},
    {"E x/edge:cap_top|side:{3}/chord:1", -4.26776695297, 1.76776695297, -3},
    {"E x/edge:cap_top|side:{3}/chord:2", -4.26776695297, -1.76776695297, -3},
    {"E x/edge:cap_top|side:{3}/chord:3", -1.76776695297, -4.26776695297, -3},
    {"E x/edge:side:{0}|side:{1}", 20, -5, -1.5},
    {"E x/edge:side:{0}|side:{3}", 0, -5, -1.5},
    {"E x/edge:side:{1}|side:{2}", 20, 5, -1.5},
    {"E x/edge:side:{2}|side:{3}", 0, 5, -1.5},
    {"E x/side:{1}/seam:0", 23.5355339059, -3.53553390593, -1.5},
    {"E x/side:{1}/seam:1", 25, 0, -1.5},
    {"E x/side:{1}/seam:2", 23.5355339059, 3.53553390593, -1.5},
    {"E x/side:{3}/seam:0", -3.53553390593, 3.53553390593, -1.5},
    {"E x/side:{3}/seam:1", -5, 0, -1.5},
    {"E x/side:{3}/seam:2", -3.53553390593, -3.53553390593, -1.5},
    {"F x/cap_bottom", 10, 0, 0},
    {"F x/cap_top", 10, 0, -3},
    {"F x/side:{0}", 10, -5, -1.5},
    {"F x/side:{1}/facet:0", 21.767766953, -4.26776695297, -1.5},
    {"F x/side:{1}/facet:1", 24.267766953, -1.76776695297, -1.5},
    {"F x/side:{1}/facet:2", 24.267766953, 1.76776695297, -1.5},
    {"F x/side:{1}/facet:3", 21.767766953, 4.26776695297, -1.5},
    {"F x/side:{2}", 10, 5, -1.5},
    {"F x/side:{3}/facet:0", -1.76776695297, 4.26776695297, -1.5},
    {"F x/side:{3}/facet:1", -4.26776695297, 1.76776695297, -1.5},
    {"F x/side:{3}/facet:2", -4.26776695297, -1.76776695297, -1.5},
    {"F x/side:{3}/facet:3", -1.76776695297, -4.26776695297, -1.5},
};
const Place kClockwiseSlotUp[] = {
    {"E x/edge:cap_bottom|side:{0}", 10, 5, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:0", 21.767766953, 4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:1", 24.267766953, 1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:2", 24.267766953, -1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:3", 21.767766953, -4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{2}", 10, -5, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:0", -1.76776695297, -4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:1", -4.26776695297, -1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:2", -4.26776695297, 1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:3", -1.76776695297, 4.26776695297, 0},
    {"E x/edge:cap_top|side:{0}", 10, 5, 3},
    {"E x/edge:cap_top|side:{1}/chord:0", 21.767766953, 4.26776695297, 3},
    {"E x/edge:cap_top|side:{1}/chord:1", 24.267766953, 1.76776695297, 3},
    {"E x/edge:cap_top|side:{1}/chord:2", 24.267766953, -1.76776695297, 3},
    {"E x/edge:cap_top|side:{1}/chord:3", 21.767766953, -4.26776695297, 3},
    {"E x/edge:cap_top|side:{2}", 10, -5, 3},
    {"E x/edge:cap_top|side:{3}/chord:0", -1.76776695297, -4.26776695297, 3},
    {"E x/edge:cap_top|side:{3}/chord:1", -4.26776695297, -1.76776695297, 3},
    {"E x/edge:cap_top|side:{3}/chord:2", -4.26776695297, 1.76776695297, 3},
    {"E x/edge:cap_top|side:{3}/chord:3", -1.76776695297, 4.26776695297, 3},
    {"E x/edge:side:{0}|side:{1}", 20, 5, 1.5},
    {"E x/edge:side:{0}|side:{3}", 0, 5, 1.5},
    {"E x/edge:side:{1}|side:{2}", 20, -5, 1.5},
    {"E x/edge:side:{2}|side:{3}", 0, -5, 1.5},
    {"E x/side:{1}/seam:0", 23.5355339059, 3.53553390593, 1.5},
    {"E x/side:{1}/seam:1", 25, 0, 1.5},
    {"E x/side:{1}/seam:2", 23.5355339059, -3.53553390593, 1.5},
    {"E x/side:{3}/seam:0", -3.53553390593, -3.53553390593, 1.5},
    {"E x/side:{3}/seam:1", -5, 0, 1.5},
    {"E x/side:{3}/seam:2", -3.53553390593, 3.53553390593, 1.5},
    {"F x/cap_bottom", 10, 0, 0},
    {"F x/cap_top", 10, 0, 3},
    {"F x/side:{0}", 10, 5, 1.5},
    {"F x/side:{1}/facet:0", 21.767766953, -4.26776695297, 1.5},
    {"F x/side:{1}/facet:1", 24.267766953, -1.76776695297, 1.5},
    {"F x/side:{1}/facet:2", 24.267766953, 1.76776695297, 1.5},
    {"F x/side:{1}/facet:3", 21.767766953, 4.26776695297, 1.5},
    {"F x/side:{2}", 10, -5, 1.5},
    {"F x/side:{3}/facet:0", -1.76776695297, 4.26776695297, 1.5},
    {"F x/side:{3}/facet:1", -4.26776695297, 1.76776695297, 1.5},
    {"F x/side:{3}/facet:2", -4.26776695297, -1.76776695297, 1.5},
    {"F x/side:{3}/facet:3", -1.76776695297, -4.26776695297, 1.5},
};
const Place kClockwiseSlotDown[] = {
    {"E x/edge:cap_bottom|side:{0}", 10, 5, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:0", 21.767766953, 4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:1", 24.267766953, 1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:2", 24.267766953, -1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:3", 21.767766953, -4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{2}", 10, -5, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:0", -1.76776695297, -4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:1", -4.26776695297, -1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:2", -4.26776695297, 1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:3", -1.76776695297, 4.26776695297, 0},
    {"E x/edge:cap_top|side:{0}", 10, 5, -3},
    {"E x/edge:cap_top|side:{1}/chord:0", 21.767766953, 4.26776695297, -3},
    {"E x/edge:cap_top|side:{1}/chord:1", 24.267766953, 1.76776695297, -3},
    {"E x/edge:cap_top|side:{1}/chord:2", 24.267766953, -1.76776695297, -3},
    {"E x/edge:cap_top|side:{1}/chord:3", 21.767766953, -4.26776695297, -3},
    {"E x/edge:cap_top|side:{2}", 10, -5, -3},
    {"E x/edge:cap_top|side:{3}/chord:0", -1.76776695297, -4.26776695297, -3},
    {"E x/edge:cap_top|side:{3}/chord:1", -4.26776695297, -1.76776695297, -3},
    {"E x/edge:cap_top|side:{3}/chord:2", -4.26776695297, 1.76776695297, -3},
    {"E x/edge:cap_top|side:{3}/chord:3", -1.76776695297, 4.26776695297, -3},
    {"E x/edge:side:{0}|side:{1}", 20, 5, -1.5},
    {"E x/edge:side:{0}|side:{3}", 0, 5, -1.5},
    {"E x/edge:side:{1}|side:{2}", 20, -5, -1.5},
    {"E x/edge:side:{2}|side:{3}", 0, -5, -1.5},
    {"E x/side:{1}/seam:0", 23.5355339059, 3.53553390593, -1.5},
    {"E x/side:{1}/seam:1", 25, 0, -1.5},
    {"E x/side:{1}/seam:2", 23.5355339059, -3.53553390593, -1.5},
    {"E x/side:{3}/seam:0", -3.53553390593, -3.53553390593, -1.5},
    {"E x/side:{3}/seam:1", -5, 0, -1.5},
    {"E x/side:{3}/seam:2", -3.53553390593, 3.53553390593, -1.5},
    {"F x/cap_bottom", 10, 0, 0},
    {"F x/cap_top", 10, 0, -3},
    {"F x/side:{0}", 10, 5, -1.5},
    {"F x/side:{1}/facet:0", 21.767766953, -4.26776695297, -1.5},
    {"F x/side:{1}/facet:1", 24.267766953, -1.76776695297, -1.5},
    {"F x/side:{1}/facet:2", 24.267766953, 1.76776695297, -1.5},
    {"F x/side:{1}/facet:3", 21.767766953, 4.26776695297, -1.5},
    {"F x/side:{2}", 10, -5, -1.5},
    {"F x/side:{3}/facet:0", -1.76776695297, 4.26776695297, -1.5},
    {"F x/side:{3}/facet:1", -4.26776695297, 1.76776695297, -1.5},
    {"F x/side:{3}/facet:2", -4.26776695297, -1.76776695297, -1.5},
    {"F x/side:{3}/facet:3", -1.76776695297, -4.26776695297, -1.5},
};
const Place kDisc[] = {
    {"E x/edge:cap_bottom|side:{0}/chord:0", 8.12132034356, 6.12132034356, 0},
    {"E x/edge:cap_bottom|side:{0}/chord:1", 5.12132034356, 9.12132034356, 0},
    {"E x/edge:cap_bottom|side:{0}/chord:2", 0.87867965644, 9.12132034356, 0},
    {"E x/edge:cap_bottom|side:{0}/chord:3", -2.12132034356, 6.12132034356, 0},
    {"E x/edge:cap_bottom|side:{0}/chord:4", -2.12132034356, 1.87867965644, 0},
    {"E x/edge:cap_bottom|side:{0}/chord:5", 0.87867965644, -1.12132034356, 0},
    {"E x/edge:cap_bottom|side:{0}/chord:6", 5.12132034356, -1.12132034356, 0},
    {"E x/edge:cap_bottom|side:{0}/chord:7", 8.12132034356, 1.87867965644, 0},
    {"E x/edge:cap_top|side:{0}/chord:0", 8.12132034356, 6.12132034356, 3},
    {"E x/edge:cap_top|side:{0}/chord:1", 5.12132034356, 9.12132034356, 3},
    {"E x/edge:cap_top|side:{0}/chord:2", 0.87867965644, 9.12132034356, 3},
    {"E x/edge:cap_top|side:{0}/chord:3", -2.12132034356, 6.12132034356, 3},
    {"E x/edge:cap_top|side:{0}/chord:4", -2.12132034356, 1.87867965644, 3},
    {"E x/edge:cap_top|side:{0}/chord:5", 0.87867965644, -1.12132034356, 3},
    {"E x/edge:cap_top|side:{0}/chord:6", 5.12132034356, -1.12132034356, 3},
    {"E x/edge:cap_top|side:{0}/chord:7", 8.12132034356, 1.87867965644, 3},
    {"E x/side:{0}/seam:0", 9, 4, 1.5},
    {"E x/side:{0}/seam:1", 7.24264068712, 8.24264068712, 1.5},
    {"E x/side:{0}/seam:2", 3, 10, 1.5},
    {"E x/side:{0}/seam:3", -1.24264068712, 8.24264068712, 1.5},
    {"E x/side:{0}/seam:4", -3, 4, 1.5},
    {"E x/side:{0}/seam:5", -1.24264068712, -0.242640687119, 1.5},
    {"E x/side:{0}/seam:6", 3, -2, 1.5},
    {"E x/side:{0}/seam:7", 7.24264068712, -0.242640687119, 1.5},
    {"F x/cap_bottom", 3, 4, 0},
    {"F x/cap_top", 3, 4, 3},
    {"F x/side:{0}/facet:0", 8.12132034356, 6.12132034356, 1.5},
    {"F x/side:{0}/facet:1", 5.12132034356, 9.12132034356, 1.5},
    {"F x/side:{0}/facet:2", 0.87867965644, 9.12132034356, 1.5},
    {"F x/side:{0}/facet:3", -2.12132034356, 6.12132034356, 1.5},
    {"F x/side:{0}/facet:4", -2.12132034356, 1.87867965644, 1.5},
    {"F x/side:{0}/facet:5", 0.87867965644, -1.12132034356, 1.5},
    {"F x/side:{0}/facet:6", 5.12132034356, -1.12132034356, 1.5},
    {"F x/side:{0}/facet:7", 8.12132034356, 1.87867965644, 1.5},
};
const Place kHalfDisc[] = {
    {"E x/edge:cap_bottom|side:{0}", 0, 0, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:0", 4.26776695297, 1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:1", 1.76776695297, 4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:2", -1.76776695297, 4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:3", -4.26776695297, 1.76776695297, 0},
    {"E x/edge:cap_top|side:{0}", 0, 0, 2},
    {"E x/edge:cap_top|side:{1}/chord:0", 4.26776695297, 1.76776695297, 2},
    {"E x/edge:cap_top|side:{1}/chord:1", 1.76776695297, 4.26776695297, 2},
    {"E x/edge:cap_top|side:{1}/chord:2", -1.76776695297, 4.26776695297, 2},
    {"E x/edge:cap_top|side:{1}/chord:3", -4.26776695297, 1.76776695297, 2},
    {"E x/edge:side:{0}|side:{1}/chord:0", -5, 0, 1},
    {"E x/edge:side:{0}|side:{1}/chord:1", 5, 0, 1},
    {"E x/side:{1}/seam:0", 3.53553390593, 3.53553390593, 1},
    {"E x/side:{1}/seam:1", 0, 5, 1},
    {"E x/side:{1}/seam:2", -3.53553390593, 3.53553390593, 1},
    {"F x/cap_bottom", 0, 2.01184463531, 0},
    {"F x/cap_top", 0, 2.01184463531, 2},
    {"F x/side:{0}", 0, 0, 1},
    {"F x/side:{1}/facet:0", 4.26776695297, 1.76776695297, 1},
    {"F x/side:{1}/facet:1", 1.76776695297, 4.26776695297, 1},
    {"F x/side:{1}/facet:2", -1.76776695297, 4.26776695297, 1},
    {"F x/side:{1}/facet:3", -4.26776695297, 1.76776695297, 1},
};
const Place kPlate[] = {
    {"E edge:x/cap_bottom/piece:0|x~hole0/side:{4}/chord:0", 15.732233047, 28.232233047, 0},
    {"E edge:x/cap_bottom/piece:0|x~hole0/side:{4}/chord:1", 18.232233047, 25.732233047, 0},
    {"E edge:x/cap_bottom/piece:0|x~hole0/side:{4}/chord:2", 21.767766953, 25.732233047, 0},
    {"E edge:x/cap_bottom/piece:0|x~hole0/side:{4}/chord:3", 24.267766953, 28.232233047, 0},
    {"E edge:x/cap_bottom/piece:0|x~hole1/side:{5}/chord:0", 63.1715728753, 27.1715728753, 0},
    {"E edge:x/cap_bottom/piece:0|x~hole1/side:{5}/chord:1", 67.1715728753, 23.1715728753, 0},
    {"E edge:x/cap_bottom/piece:0|x~hole1/side:{5}/chord:2", 72.8284271247, 23.1715728753, 0},
    {"E edge:x/cap_bottom/piece:0|x~hole1/side:{5}/chord:3", 76.8284271247, 27.1715728753, 0},
    {"E edge:x/cap_bottom/piece:1|x~hole0/side:{4}/chord:0", 24.267766953, 31.767766953, 0},
    {"E edge:x/cap_bottom/piece:1|x~hole0/side:{4}/chord:1", 21.767766953, 34.267766953, 0},
    {"E edge:x/cap_bottom/piece:1|x~hole0/side:{4}/chord:2", 18.232233047, 34.267766953, 0},
    {"E edge:x/cap_bottom/piece:1|x~hole0/side:{4}/chord:3", 15.732233047, 31.767766953, 0},
    {"E edge:x/cap_bottom/piece:1|x~hole1/side:{5}/chord:0", 76.8284271247, 32.8284271247, 0},
    {"E edge:x/cap_bottom/piece:1|x~hole1/side:{5}/chord:1", 72.8284271247, 36.8284271247, 0},
    {"E edge:x/cap_bottom/piece:1|x~hole1/side:{5}/chord:2", 67.1715728753, 36.8284271247, 0},
    {"E edge:x/cap_bottom/piece:1|x~hole1/side:{5}/chord:3", 63.1715728753, 32.8284271247, 0},
    {"E edge:x/cap_top/piece:0|x~hole0/side:{4}/chord:0", 15.732233047, 31.767766953, 10},
    {"E edge:x/cap_top/piece:0|x~hole0/side:{4}/chord:1", 18.232233047, 34.267766953, 10},
    {"E edge:x/cap_top/piece:0|x~hole0/side:{4}/chord:2", 21.767766953, 34.267766953, 10},
    {"E edge:x/cap_top/piece:0|x~hole0/side:{4}/chord:3", 24.267766953, 31.767766953, 10},
    {"E edge:x/cap_top/piece:0|x~hole1/side:{5}/chord:0", 63.1715728753, 32.8284271247, 10},
    {"E edge:x/cap_top/piece:0|x~hole1/side:{5}/chord:1", 67.1715728753, 36.8284271247, 10},
    {"E edge:x/cap_top/piece:0|x~hole1/side:{5}/chord:2", 72.8284271247, 36.8284271247, 10},
    {"E edge:x/cap_top/piece:0|x~hole1/side:{5}/chord:3", 76.8284271247, 32.8284271247, 10},
    {"E edge:x/cap_top/piece:1|x~hole0/side:{4}/chord:0", 24.267766953, 28.232233047, 10},
    {"E edge:x/cap_top/piece:1|x~hole0/side:{4}/chord:1", 21.767766953, 25.732233047, 10},
    {"E edge:x/cap_top/piece:1|x~hole0/side:{4}/chord:2", 18.232233047, 25.732233047, 10},
    {"E edge:x/cap_top/piece:1|x~hole0/side:{4}/chord:3", 15.732233047, 28.232233047, 10},
    {"E edge:x/cap_top/piece:1|x~hole1/side:{5}/chord:0", 76.8284271247, 27.1715728753, 10},
    {"E edge:x/cap_top/piece:1|x~hole1/side:{5}/chord:1", 72.8284271247, 23.1715728753, 10},
    {"E edge:x/cap_top/piece:1|x~hole1/side:{5}/chord:2", 67.1715728753, 23.1715728753, 10},
    {"E edge:x/cap_top/piece:1|x~hole1/side:{5}/chord:3", 63.1715728753, 27.1715728753, 10},
    {"E x/edge:cap_bottom/piece:0|cap_bottom/piece:1/chord:0", 89, 30, 0},
    {"E x/edge:cap_bottom/piece:0|cap_bottom/piece:1/chord:1", 7.5, 30, 0},
    {"E x/edge:cap_bottom/piece:0|cap_bottom/piece:1/chord:2", 43.5, 30, 0},
    {"E x/edge:cap_bottom/piece:0|side:{0}", 50, 0, 0},
    {"E x/edge:cap_bottom/piece:0|side:{1}", 100, 15, 0},
    {"E x/edge:cap_bottom/piece:0|side:{3}", 0, 15, 0},
    {"E x/edge:cap_bottom/piece:1|side:{1}", 100, 45, 0},
    {"E x/edge:cap_bottom/piece:1|side:{2}", 50, 60, 0},
    {"E x/edge:cap_bottom/piece:1|side:{3}", 0, 45, 0},
    {"E x/edge:cap_top/piece:0|cap_top/piece:1/chord:0", 89, 30, 10},
    {"E x/edge:cap_top/piece:0|cap_top/piece:1/chord:1", 7.5, 30, 10},
    {"E x/edge:cap_top/piece:0|cap_top/piece:1/chord:2", 43.5, 30, 10},
    {"E x/edge:cap_top/piece:0|side:{1}", 100, 45, 10},
    {"E x/edge:cap_top/piece:0|side:{2}", 50, 60, 10},
    {"E x/edge:cap_top/piece:0|side:{3}", 0, 45, 10},
    {"E x/edge:cap_top/piece:1|side:{0}", 50, 0, 10},
    {"E x/edge:cap_top/piece:1|side:{1}", 100, 15, 10},
    {"E x/edge:cap_top/piece:1|side:{3}", 0, 15, 10},
    {"E x/edge:side:{0}|side:{1}", 100, 0, 5},
    {"E x/edge:side:{0}|side:{3}", 0, 0, 5},
    {"E x/edge:side:{1}|side:{2}", 100, 60, 5},
    {"E x/edge:side:{2}|side:{3}", 0, 60, 5},
    {"E x~hole0/side:{4}/seam:0", 25, 30, 5},
    {"E x~hole0/side:{4}/seam:1", 23.5355339059, 33.5355339059, 5},
    {"E x~hole0/side:{4}/seam:2", 20, 35, 5},
    {"E x~hole0/side:{4}/seam:3", 16.4644660941, 33.5355339059, 5},
    {"E x~hole0/side:{4}/seam:4", 15, 30, 5},
    {"E x~hole0/side:{4}/seam:5", 16.4644660941, 26.4644660941, 5},
    {"E x~hole0/side:{4}/seam:6", 20, 25, 5},
    {"E x~hole0/side:{4}/seam:7", 23.5355339059, 26.4644660941, 5},
    {"E x~hole1/side:{5}/seam:0", 78, 30, 5},
    {"E x~hole1/side:{5}/seam:1", 75.6568542495, 35.6568542495, 5},
    {"E x~hole1/side:{5}/seam:2", 70, 38, 5},
    {"E x~hole1/side:{5}/seam:3", 64.3431457505, 35.6568542495, 5},
    {"E x~hole1/side:{5}/seam:4", 62, 30, 5},
    {"E x~hole1/side:{5}/seam:5", 64.3431457505, 24.3431457505, 5},
    {"E x~hole1/side:{5}/seam:6", 70, 22, 5},
    {"E x~hole1/side:{5}/seam:7", 75.6568542495, 24.3431457505, 5},
    {"F x/cap_bottom/piece:0", 49.599388981, 16.4592372, 0},
    {"F x/cap_bottom/piece:1", 50.3682529339, 45.0098118875, 0},
    {"F x/cap_top/piece:0", 49.599388981, 43.5407628, 10},
    {"F x/cap_top/piece:1", 50.3682529339, 14.9901881125, 10},
    {"F x/side:{0}", 50, 0, 5},
    {"F x/side:{1}", 100, 30, 5},
    {"F x/side:{2}", 50, 60, 5},
    {"F x/side:{3}", 0, 30, 5},
    {"F x~hole0/side:{4}/facet:0", 24.267766953, 31.767766953, 5},
    {"F x~hole0/side:{4}/facet:1", 21.767766953, 34.267766953, 5},
    {"F x~hole0/side:{4}/facet:2", 18.232233047, 34.267766953, 5},
    {"F x~hole0/side:{4}/facet:3", 15.732233047, 31.767766953, 5},
    {"F x~hole0/side:{4}/facet:4", 15.732233047, 28.232233047, 5},
    {"F x~hole0/side:{4}/facet:5", 18.232233047, 25.732233047, 5},
    {"F x~hole0/side:{4}/facet:6", 21.767766953, 25.732233047, 5},
    {"F x~hole0/side:{4}/facet:7", 24.267766953, 28.232233047, 5},
    {"F x~hole1/side:{5}/facet:0", 76.8284271247, 32.8284271247, 5},
    {"F x~hole1/side:{5}/facet:1", 72.8284271247, 36.8284271247, 5},
    {"F x~hole1/side:{5}/facet:2", 67.1715728753, 36.8284271247, 5},
    {"F x~hole1/side:{5}/facet:3", 63.1715728753, 32.8284271247, 5},
    {"F x~hole1/side:{5}/facet:4", 63.1715728753, 27.1715728753, 5},
    {"F x~hole1/side:{5}/facet:5", 67.1715728753, 23.1715728753, 5},
    {"F x~hole1/side:{5}/facet:6", 72.8284271247, 23.1715728753, 5},
    {"F x~hole1/side:{5}/facet:7", 76.8284271247, 27.1715728753, 5},
};
const Place kIsland[] = {
    {"E edge:x/cap_bottom/piece:0|x~hole0/side:{4}", 20, 10, 0},
    {"E edge:x/cap_bottom/piece:0|x~hole0/side:{5}", 30, 15, 0},
    {"E edge:x/cap_bottom/piece:0|x~hole0/side:{7}", 10, 15, 0},
    {"E edge:x/cap_bottom/piece:1|x~hole0/side:{5}", 30, 25, 0},
    {"E edge:x/cap_bottom/piece:1|x~hole0/side:{6}", 20, 30, 0},
    {"E edge:x/cap_bottom/piece:1|x~hole0/side:{7}", 10, 25, 0},
    {"E edge:x/cap_top/piece:0|x~hole0/side:{4}", 25, 10, 4},
    {"E edge:x/cap_top/piece:0|x~hole0/side:{5}", 30, 20, 4},
    {"E edge:x/cap_top/piece:0|x~hole0/side:{6}", 25, 30, 4},
    {"E edge:x/cap_top/piece:1|x~hole0/side:{4}", 15, 10, 4},
    {"E edge:x/cap_top/piece:1|x~hole0/side:{6}", 15, 30, 4},
    {"E edge:x/cap_top/piece:1|x~hole0/side:{7}", 10, 20, 4},
    {"E x/edge:cap_bottom/piece:0|cap_bottom/piece:1/chord:0", 5, 20, 0},
    {"E x/edge:cap_bottom/piece:0|cap_bottom/piece:1/chord:1", 35, 20, 0},
    {"E x/edge:cap_bottom/piece:0|side:{0}", 20, 0, 0},
    {"E x/edge:cap_bottom/piece:0|side:{1}", 40, 10, 0},
    {"E x/edge:cap_bottom/piece:0|side:{3}", 0, 10, 0},
    {"E x/edge:cap_bottom/piece:1|side:{1}", 40, 30, 0},
    {"E x/edge:cap_bottom/piece:1|side:{2}", 20, 40, 0},
    {"E x/edge:cap_bottom/piece:1|side:{3}", 0, 30, 0},
    {"E x/edge:cap_top/piece:0|cap_top/piece:1/chord:0", 20, 35, 4},
    {"E x/edge:cap_top/piece:0|cap_top/piece:1/chord:1", 20, 5, 4},
    {"E x/edge:cap_top/piece:0|side:{0}", 30, 0, 4},
    {"E x/edge:cap_top/piece:0|side:{1}", 40, 20, 4},
    {"E x/edge:cap_top/piece:0|side:{2}", 30, 40, 4},
    {"E x/edge:cap_top/piece:1|side:{0}", 10, 0, 4},
    {"E x/edge:cap_top/piece:1|side:{2}", 10, 40, 4},
    {"E x/edge:cap_top/piece:1|side:{3}", 0, 20, 4},
    {"E x/edge:side:{0}|side:{1}", 40, 0, 2},
    {"E x/edge:side:{0}|side:{3}", 0, 0, 2},
    {"E x/edge:side:{1}|side:{2}", 40, 40, 2},
    {"E x/edge:side:{2}|side:{3}", 0, 40, 2},
    {"E x~hole0/edge:side:{4}|side:{5}", 30, 10, 2},
    {"E x~hole0/edge:side:{4}|side:{7}", 10, 10, 2},
    {"E x~hole0/edge:side:{5}|side:{6}", 30, 30, 2},
    {"E x~hole0/edge:side:{6}|side:{7}", 10, 30, 2},
    {"E x~region1/edge:cap_bottom|side:{8}/chord:0", 18.232233047, 15.732233047, 0},
    {"E x~region1/edge:cap_bottom|side:{8}/chord:1", 15.732233047, 18.232233047, 0},
    {"E x~region1/edge:cap_bottom|side:{8}/chord:2", 15.732233047, 21.767766953, 0},
    {"E x~region1/edge:cap_bottom|side:{8}/chord:3", 18.232233047, 24.267766953, 0},
    {"E x~region1/edge:cap_bottom|side:{8}/chord:4", 21.767766953, 24.267766953, 0},
    {"E x~region1/edge:cap_bottom|side:{8}/chord:5", 24.267766953, 21.767766953, 0},
    {"E x~region1/edge:cap_bottom|side:{8}/chord:6", 24.267766953, 18.232233047, 0},
    {"E x~region1/edge:cap_bottom|side:{8}/chord:7", 21.767766953, 15.732233047, 0},
    {"E x~region1/edge:cap_top|side:{8}/chord:0", 21.767766953, 24.267766953, 4},
    {"E x~region1/edge:cap_top|side:{8}/chord:1", 18.232233047, 24.267766953, 4},
    {"E x~region1/edge:cap_top|side:{8}/chord:2", 15.732233047, 21.767766953, 4},
    {"E x~region1/edge:cap_top|side:{8}/chord:3", 15.732233047, 18.232233047, 4},
    {"E x~region1/edge:cap_top|side:{8}/chord:4", 18.232233047, 15.732233047, 4},
    {"E x~region1/edge:cap_top|side:{8}/chord:5", 21.767766953, 15.732233047, 4},
    {"E x~region1/edge:cap_top|side:{8}/chord:6", 24.267766953, 18.232233047, 4},
    {"E x~region1/edge:cap_top|side:{8}/chord:7", 24.267766953, 21.767766953, 4},
    {"E x~region1/side:{8}/seam:0", 23.5355339059, 23.5355339059, 2},
    {"E x~region1/side:{8}/seam:1", 25, 20, 2},
    {"E x~region1/side:{8}/seam:2", 20, 25, 2},
    {"E x~region1/side:{8}/seam:3", 16.4644660941, 23.5355339059, 2},
    {"E x~region1/side:{8}/seam:4", 15, 20, 2},
    {"E x~region1/side:{8}/seam:5", 16.4644660941, 16.4644660941, 2},
    {"E x~region1/side:{8}/seam:6", 20, 15, 2},
    {"E x~region1/side:{8}/seam:7", 23.5355339059, 16.4644660941, 2},
    {"F x/cap_bottom/piece:0", 18.7878787879, 11.5151515152, 0},
    {"F x/cap_bottom/piece:1", 21.2121212121, 28.4848484848, 0},
    {"F x/cap_top/piece:0", 28.4848484848, 21.2121212121, 4},
    {"F x/cap_top/piece:1", 11.5151515152, 18.7878787879, 4},
    {"F x/side:{0}", 20, 0, 2},
    {"F x/side:{1}", 40, 20, 2},
    {"F x/side:{2}", 20, 40, 2},
    {"F x/side:{3}", 0, 20, 2},
    {"F x~hole0/side:{4}", 20, 10, 2},
    {"F x~hole0/side:{5}", 30, 20, 2},
    {"F x~hole0/side:{6}", 20, 30, 2},
    {"F x~hole0/side:{7}", 10, 20, 2},
    {"F x~region1/cap_bottom", 20, 20, 0},
    {"F x~region1/cap_top", 20, 20, 4},
    {"F x~region1/side:{8}/facet:0", 24.267766953, 21.767766953, 2},
    {"F x~region1/side:{8}/facet:1", 21.767766953, 24.267766953, 2},
    {"F x~region1/side:{8}/facet:2", 18.232233047, 24.267766953, 2},
    {"F x~region1/side:{8}/facet:3", 15.732233047, 21.767766953, 2},
    {"F x~region1/side:{8}/facet:4", 15.732233047, 18.232233047, 2},
    {"F x~region1/side:{8}/facet:5", 18.232233047, 15.732233047, 2},
    {"F x~region1/side:{8}/facet:6", 21.767766953, 15.732233047, 2},
    {"F x~region1/side:{8}/facet:7", 24.267766953, 18.232233047, 2},
};
const Place kObliqueSlot[] = {
    {"E x/edge:cap_bottom|side:{0}", 10, -5, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:0", 21.767766953, -4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:1", 24.267766953, -1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:2", 24.267766953, 1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{1}/chord:3", 21.767766953, 4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{2}", 10, 5, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:0", -1.76776695297, 4.26776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:1", -4.26776695297, 1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:2", -4.26776695297, -1.76776695297, 0},
    {"E x/edge:cap_bottom|side:{3}/chord:3", -1.76776695297, -4.26776695297, 0},
    {"E x/edge:cap_top|side:{0}", 10.8466487815, -5.56443252103, 2.82216260515},
    {"E x/edge:cap_top|side:{1}/chord:0", 22.6144157345, -4.832199474, 2.82216260515},
    {"E x/edge:cap_top|side:{1}/chord:1", 25.1144157345, -2.332199474, 2.82216260515},
    {"E x/edge:cap_top|side:{1}/chord:2", 25.1144157345, 1.20333443194, 2.82216260515},
    {"E x/edge:cap_top|side:{1}/chord:3", 22.6144157345, 3.70333443194, 2.82216260515},
    {"E x/edge:cap_top|side:{2}", 10.8466487815, 4.43556747897, 2.82216260515},
    {"E x/edge:cap_top|side:{3}/chord:0", -0.921118171421, 3.70333443194, 2.82216260515},
    {"E x/edge:cap_top|side:{3}/chord:1", -3.42111817142, 1.20333443194, 2.82216260515},
    {"E x/edge:cap_top|side:{3}/chord:2", -3.42111817142, -2.332199474, 2.82216260515},
    {"E x/edge:cap_top|side:{3}/chord:3", -0.921118171421, -4.832199474, 2.82216260515},
    {"E x/edge:side:{0}|side:{1}", 20.4233243908, -5.28221626052, 1.41108130258},
    {"E x/edge:side:{0}|side:{3}", 0.423324390773, -5.28221626052, 1.41108130258},
    {"E x/edge:side:{1}|side:{2}", 20.4233243908, 4.71778373948, 1.41108130258},
    {"E x/edge:side:{2}|side:{3}", 0.423324390773, 4.71778373948, 1.41108130258},
    {"E x/side:{1}/seam:0", 23.9588582967, -3.81775016645, 1.41108130258},
    {"E x/side:{1}/seam:1", 25.4233243908, -0.282216260515, 1.41108130258},
    {"E x/side:{1}/seam:2", 23.9588582967, 3.25331764542, 1.41108130258},
    {"E x/side:{3}/seam:0", -3.11220951516, 3.25331764542, 1.41108130258},
    {"E x/side:{3}/seam:1", -4.57667560923, -0.282216260515, 1.41108130258},
    {"E x/side:{3}/seam:2", -3.11220951516, -3.81775016645, 1.41108130258},
    {"F x/cap_bottom", 10, 0, 0},
    {"F x/cap_top", 10.8466487815, -0.56443252103, 2.82216260515},
    {"F x/side:{0}", 10.4233243908, -5.28221626052, 1.41108130258},
    {"F x/side:{1}/facet:0", 22.1910913437, -4.54998321348, 1.41108130258},
    {"F x/side:{1}/facet:1", 24.6910913437, -2.04998321348, 1.41108130258},
    {"F x/side:{1}/facet:2", 24.6910913437, 1.48555069245, 1.41108130258},
    {"F x/side:{1}/facet:3", 22.1910913437, 3.98555069245, 1.41108130258},
    {"F x/side:{2}", 10.4233243908, 4.71778373948, 1.41108130258},
    {"F x/side:{3}/facet:0", -1.34444256219, 3.98555069245, 1.41108130258},
    {"F x/side:{3}/facet:1", -3.84444256219, 1.48555069245, 1.41108130258},
    {"F x/side:{3}/facet:2", -3.84444256219, -2.04998321348, 1.41108130258},
    {"F x/side:{3}/facet:3", -1.34444256219, -4.54998321348, 1.41108130258},
};

// clang-format on

/// The centroid of @p face's outer loop, by area: what a face is, whatever corners stand in its
/// sides (a Boolean's T-junctions).
Vec3 centroidOf(const hz::topo::Face& face) {
    std::vector<Vec3> points;
    const hz::topo::HalfEdge* he = face.outerLoop->halfEdge;
    do {
        points.push_back(he->origin->point);
        he = he->next;
    } while (he != face.outerLoop->halfEdge);
    Vec3 sum;
    double total = 0.0;
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        const double area = (points[i] - points[0]).cross(points[i + 1] - points[0]).length();
        sum = sum + (points[0] + points[i] + points[i + 1]) * (area / 3.0);
        total += area;
    }
    return total > 0.0 ? sum / total : points.front();
}

/// @p tag with the two faces an edge `<source>/edge:<a>|<b>` is named after in sorted order:
/// nameEdgesLogically sorts them as they are named, and "{k}" stands for entities whose ids sort
/// otherwise in another run ("e9" after "e10").
std::string canonical(const std::string& tag) {
    const std::size_t edge = tag.find("/edge:");
    if (edge == std::string::npos) return tag;
    const std::size_t a = edge + 6;
    const std::size_t bar = tag.find('|', a);
    if (bar == std::string::npos || tag.find('/', a) < bar) return tag;
    const std::size_t end = std::min(tag.find('/', bar), tag.size());
    std::string first = tag.substr(a, bar - a);
    std::string second = tag.substr(bar + 1, end - bar - 1);
    if (second < first) std::swap(first, second);
    return tag.substr(0, a) + first + "|" + second + tag.substr(end);
}

/// @p solid's places by name: "F <face>" and "E <edge>" (canonical()).
std::map<std::string, Vec3> placesOf(const hz::topo::Solid& solid) {
    std::map<std::string, Vec3> places;
    for (const auto& face : solid.faces()) {
        if (face.alive) places["F " + face.topoId.tag()] = centroidOf(face);
    }
    for (const auto& edge : solid.edges()) {
        if (!edge.alive) continue;
        places["E " + canonical(edge.topoId.tag())] =
            (edge.halfEdge->origin->point + edge.halfEdge->twin->origin->point) * 0.5;
    }
    return places;
}

/// @p recorded with each "{k}" the k-th entity of @p profile's name, "e<id>".
std::string named(std::string recorded, const Profile* profile) {
    if (profile == nullptr) return recorded;
    for (std::size_t k = 0; k < profile->size(); ++k) {
        const std::string mark = "{" + std::to_string(k) + "}";
        const std::string id = "e" + std::to_string((*profile)[k]->id());
        for (std::size_t at = recorded.find(mark); at != std::string::npos;
             at = recorded.find(mark)) {
            recorded.replace(at, mark.size(), id);
        }
    }
    return recorded;
}

/// The size of @p solid: its vertices' box's diagonal.
double sizeOf(const hz::topo::Solid& solid) {
    Vec3 lo(1e300, 1e300, 1e300);
    Vec3 hi(-1e300, -1e300, -1e300);
    for (const auto& v : solid.vertices()) {
        lo = Vec3(std::min(lo.x, v.point.x), std::min(lo.y, v.point.y), std::min(lo.z, v.point.z));
        hi = Vec3(std::max(hi.x, v.point.x), std::max(hi.y, v.point.y), std::max(hi.z, v.point.z));
    }
    return (hi - lo).length();
}

/// Each name of @p masters that @p bridge has names the same place, within 1e-9 of the size, but
/// those @p incomparable says to pass over; the names @p bridge has not are @p gone.
void expectNumberedAsMasters(const hz::topo::Solid& bridge, const Place* masters, std::size_t count,
                             const Profile* profile, const std::set<std::string>& gone,
                             const std::function<bool(const std::string&)>& incomparable = {}) {
    const auto places = placesOf(bridge);
    const double within = 1e-9 * sizeOf(bridge);
    std::set<std::string> missing;
    for (std::size_t i = 0; i < count; ++i) {
        const Place& place = masters[i];
        const std::string tag = canonical(named(place.tag, profile));
        const auto found = places.find(tag);
        if (found == places.end()) {
            missing.insert(tag);
            continue;
        }
        if (incomparable && incomparable(tag)) continue;
        EXPECT_LT((found->second - Vec3(place.x, place.y, place.z)).length(), within) << tag;
    }
    std::set<std::string> expected;
    for (const std::string& tag : gone) expected.insert(canonical(named(tag, profile)));
    EXPECT_EQ(missing, expected);
}

template <std::size_t N>
void expectNumberedAsMasters(const hz::topo::Solid& bridge, const Place (&masters)[N],
                             const Profile* profile, const std::set<std::string>& gone,
                             const std::function<bool(const std::string&)>& incomparable = {}) {
    expectNumberedAsMasters(bridge, masters, N, profile, gone, incomparable);
}

/// Every edge of @p bridge named for an edge of @p exact (its name, or `<name>/chord:<k>`) lies on
/// that edge's curve: no line of a grid has an exact edge's name (Ruling 18).
void expectChordsOnTheirEdges(const hz::topo::Solid& bridge, const hz::topo::Solid& exact) {
    std::map<std::string, const hz::topo::Edge*> byName;
    for (const auto& e : exact.edges()) byName[e.topoId.tag()] = &e;
    const double within = 1e-9 * sizeOf(bridge);
    for (const auto& e : bridge.edges()) {
        auto found = byName.find(e.topoId.tag());
        if (found == byName.end()) found = byName.find(hz::model::logicalEdge(e.topoId.tag()));
        if (found == byName.end()) continue;
        const hz::geo::Curve3& curve = *found->second->exactCurve;
        for (const hz::topo::Vertex* v : {e.halfEdge->origin, e.halfEdge->twin->origin}) {
            const Vec3 on = curve.evaluate(curve.closestParameter(v->point));
            EXPECT_LT((on - v->point).length(), within) << e.topoId.tag();
        }
    }
}

std::shared_ptr<hz::draft::DraftEntity> line(Vec2 a, Vec2 b) {
    return std::make_shared<hz::draft::DraftLine>(a, b);
}

std::shared_ptr<hz::draft::DraftEntity> arc(Vec2 c, double r, double a0, double a1) {
    return std::make_shared<hz::draft::DraftArc>(c, r, a0, a1);
}

std::shared_ptr<hz::draft::DraftEntity> circle(Vec2 c, double r) {
    return std::make_shared<hz::draft::DraftCircle>(c, r);
}

/// A primitive named "primitive_1" at 8 segments a turn, built: its bridge copy and its exact
/// body.
hz::doc::FeatureOutput built(std::unique_ptr<PrimitiveFeature> primitive) {
    primitive->restoreFeatureID("primitive_1");
    if (primitive->isFaceted()) {
        EXPECT_TRUE(primitive->setParameter("segments", 8));
    }
    std::string why;
    auto out = primitive->executeWithExact({}, nullptr, &why);
    EXPECT_NE(out.faceted, nullptr) << why;
    EXPECT_NE(out.exact, nullptr) << why;
    return out;
}

/// @p profile extruded by @p distance along @p direction at 8 segments a turn, "x": its exact
/// form and its bridge copy.
hz::model::Extrude::ExactAndBridge extruded(const Profile& profile, const Vec3& direction,
                                            double distance) {
    std::string why;
    auto out = hz::model::Extrude::exactAndBridge(profile, hz::draft::SketchPlane(), direction,
                                                  distance, "x", 8, 0.0, &why);
    EXPECT_NE(out.faceted, nullptr) << why;
    // Extrude::execute gives the same bridge, named alike.
    const auto executed =
        hz::model::Extrude::execute(profile, hz::draft::SketchPlane(), direction, distance, "x", 8,
                                    0.0, &why, hz::model::NamingScheme::Stable);
    EXPECT_NE(executed, nullptr) << why;
    if (executed && out.faceted) {
        std::set<std::string> a;
        std::set<std::string> b;
        for (const auto& [tag, place] : placesOf(*executed)) a.insert(tag);
        for (const auto& [tag, place] : placesOf(*out.faceted)) b.insert(tag);
        EXPECT_EQ(a, b);
    }
    return out;
}

Profile slot() {
    return {line({0, -5}, {20, -5}), arc({20, 0}, 5, -kPi / 2, kPi / 2), line({20, 5}, {0, 5}),
            arc({0, 0}, 5, kPi / 2, 3 * kPi / 2)};
}

/// The slot chained from its top line drawn left to right: clockwise.
Profile clockwiseSlot() {
    return {line({0, 5}, {20, 5}), arc({20, 0}, 5, -kPi / 2, kPi / 2), line({20, -5}, {0, -5}),
            arc({0, 0}, 5, kPi / 2, 3 * kPi / 2)};
}

}  // namespace

// The curved primitives, placed or not, have master's facets, chords and seams. Where an exact
// seam has several chords (a sphere's meridian at u = 0, a torus's outer equator and its tube at
// u = 0) its chords are `<seam>/chord:<k>`, the seam named after master's edge at its first
// chord: master's names of those chords, each a seam of its own, are gone.
TEST(BridgeNumbering, PrimitivesAreNumberedAsMastersWere) {
    struct Case {
        const char* what;
        std::unique_ptr<PrimitiveFeature> primitive;
        const Place* table;
        std::size_t size;
        std::set<std::string> gone;
        bool placed = false;
    };
    std::vector<Case> cases;
    cases.push_back({"a cylinder",
                     PrimitiveFeature::makeCylinder(2.0, 5.0),
                     kCylinder,
                     std::size(kCylinder),
                     {}});
    cases.push_back({"a cylinder stood at (1, -2, 3) along (0.3, 1, 0.6)",
                     PrimitiveFeature::makeCylinder(2.0, 5.0),
                     kPlacedCylinder,
                     std::size(kPlacedCylinder),
                     {},
                     true});
    // The meridian at u = 0, from pole to pole: named after master's chord at the south pole.
    cases.push_back({"a sphere",
                     PrimitiveFeature::makeSphere(3.0),
                     kSphere,
                     std::size(kSphere),
                     {"E primitive_1/band/seam:2", "E primitive_1/band/seam:18",
                      "E primitive_1/band/seam:34", "E primitive_1/band/seam:49"}});
    cases.push_back({"a frustum",
                     PrimitiveFeature::makeCone(3.0, 1.0, 4.0),
                     kFrustum,
                     std::size(kFrustum),
                     {}});
    cases.push_back({"a cone to its top",
                     PrimitiveFeature::makeCone(2.0, 0.0, 3.0),
                     kConeToItsTop,
                     std::size(kConeToItsTop),
                     {}});
    cases.push_back({"a cone from its bottom",
                     PrimitiveFeature::makeCone(0.0, 2.0, 3.0),
                     kConeFromItsBottom,
                     std::size(kConeFromItsBottom),
                     {}});
    // The outer equator (8 chords) and the tube's circle at u = 0 (4).
    cases.push_back({"a torus",
                     PrimitiveFeature::makeTorus(5.0, 1.5),
                     kTorus,
                     std::size(kTorus),
                     {"E primitive_1/patch/seam:0", "E primitive_1/patch/seam:3",
                      "E primitive_1/patch/seam:6", "E primitive_1/patch/seam:9",
                      "E primitive_1/patch/seam:11", "E primitive_1/patch/seam:12",
                      "E primitive_1/patch/seam:20", "E primitive_1/patch/seam:28",
                      "E primitive_1/patch/seam:36", "E primitive_1/patch/seam:44",
                      "E primitive_1/patch/seam:52", "E primitive_1/patch/seam:60"}});
    cases.push_back({"a box", PrimitiveFeature::makeBox(2.0, 3.0, 4.0), kBox, std::size(kBox), {}});
    for (Case& c : cases) {
        SCOPED_TRACE(c.what);
        if (c.placed) {
            ASSERT_TRUE(c.primitive->setVector("basePoint", Vec3(1.0, -2.0, 3.0)));
            ASSERT_TRUE(c.primitive->setVector("axisDirection", Vec3(0.3, 1.0, 0.6)));
        }
        const auto out = built(std::move(c.primitive));
        ASSERT_NE(out.faceted, nullptr);
        ASSERT_NE(out.exact, nullptr);
        expectNumberedAsMasters(*out.faceted, c.table, c.size, nullptr, c.gone);
        expectChordsOnTheirEdges(*out.faceted, *out.exact);
    }
}

TEST(BridgeNumbering, ExtrusionsAreNumberedAsMastersWere) {
    const Vec3 up(0, 0, 1);
    {
        SCOPED_TRACE("a slot, up");
        const Profile p = slot();
        const auto out = extruded(p, up, 3.0);
        ASSERT_NE(out.faceted, nullptr);
        expectNumberedAsMasters(*out.faceted, kSlotUp, &p, {});
        expectChordsOnTheirEdges(*out.faceted, *out.exact);
    }
    {
        SCOPED_TRACE("a slot, down");
        const Profile p = slot();
        const auto out = extruded(p, up, -3.0);
        ASSERT_NE(out.faceted, nullptr);
        expectNumberedAsMasters(*out.faceted, kSlotDown, &p, {});
        expectChordsOnTheirEdges(*out.faceted, *out.exact);
    }
    {
        SCOPED_TRACE("a clockwise slot, up");
        const Profile p = clockwiseSlot();
        const auto out = extruded(p, up, 3.0);
        ASSERT_NE(out.faceted, nullptr);
        expectNumberedAsMasters(*out.faceted, kClockwiseSlotUp, &p, {});
        expectChordsOnTheirEdges(*out.faceted, *out.exact);
    }
    {
        SCOPED_TRACE("a clockwise slot, down");
        const Profile p = clockwiseSlot();
        const auto out = extruded(p, up, -3.0);
        ASSERT_NE(out.faceted, nullptr);
        expectNumberedAsMasters(*out.faceted, kClockwiseSlotDown, &p, {});
        expectChordsOnTheirEdges(*out.faceted, *out.exact);
    }
    {
        SCOPED_TRACE("a disc");
        const Profile p = {circle({3, 4}, 6)};
        const auto out = extruded(p, up, 3.0);
        ASSERT_NE(out.faceted, nullptr);
        expectNumberedAsMasters(*out.faceted, kDisc, &p, {});
        expectChordsOnTheirEdges(*out.faceted, *out.exact);
    }
    {
        // Its two lines along the offset part the same two faces: `.../chord:0` and `/chord:1`,
        // in master's order, the exact edges renamed with them.
        SCOPED_TRACE("a half disc");
        const Profile p = {line({-5, 0}, {5, 0}), arc({0, 0}, 5, 0, kPi)};
        const auto out = extruded(p, up, 2.0);
        ASSERT_NE(out.faceted, nullptr);
        expectNumberedAsMasters(*out.faceted, kHalfDisc, &p, {});
        expectChordsOnTheirEdges(*out.faceted, *out.exact);
    }
    {
        SCOPED_TRACE("a slot at a slant");
        const Profile p = slot();
        const auto out = extruded(p, Vec3(0.3, -0.2, 1.0).normalized(), 3.0);
        ASSERT_NE(out.faceted, nullptr);
        expectNumberedAsMasters(*out.faceted, kObliqueSlot, &p, {});
        expectChordsOnTheirEdges(*out.faceted, *out.exact);
    }
}

// A profile of several loops was master's Booleans: what they made is not the bridge's, on
// purpose (Task 5). A plate with two holes: master's Boolean split each cap in two, so every name
// of a cap's piece, and of the edges between a piece and a hole's wall or the other piece, is gone
// (the bridge's caps are one face each, with the holes); everything else is master's, the hole
// walls' facets and seams included.
TEST(BridgeNumbering, APlateWithHolesIsNumberedAsMastersWasWhereItCanBe) {
    const Profile p = {line({0, 0}, {100, 0}), line({100, 0}, {100, 60}), line({100, 60}, {0, 60}),
                       line({0, 60}, {0, 0}),  circle({20, 30}, 5),       circle({70, 30}, 8)};
    const auto out = extruded(p, Vec3(0, 0, 1), 10.0);
    ASSERT_NE(out.faceted, nullptr);
    std::set<std::string> gone;
    for (const Place& place : kPlate) {
        if (std::string(place.tag).find("/piece:") != std::string::npos) gone.insert(place.tag);
    }
    EXPECT_EQ(gone.size(), 54u);
    expectNumberedAsMasters(*out.faceted, kPlate, &p, gone);
    expectChordsOnTheirEdges(*out.faceted, *out.exact);
}

// A square with a square hole and a disc inside it: master joined the disc to the square with a
// Boolean union, which merged and sewed its facets again, and numbered the disc's chords and
// seams (`x~region1/...`) in the order that merge left them: not comparable, and not compared
// (the chords and seams of `x~region1` are those names, at other places; Task 11 rewrites a saved
// chord's name to its curve's). The square's caps were split in two (`/piece:`), gone. Every face,
// and every other edge, is master's.
TEST(BridgeNumbering, AnIslandIsNumberedAsMastersWasWhereItCanBe) {
    const Profile p = {line({0, 0}, {40, 0}),    line({40, 0}, {40, 40}),  line({40, 40}, {0, 40}),
                       line({0, 40}, {0, 0}),    line({10, 10}, {30, 10}), line({30, 10}, {30, 30}),
                       line({30, 30}, {10, 30}), line({10, 30}, {10, 10}), circle({20, 20}, 5)};
    const auto out = extruded(p, Vec3(0, 0, 1), 4.0);
    ASSERT_NE(out.faceted, nullptr);
    std::set<std::string> gone;
    for (const Place& place : kIsland) {
        if (std::string(place.tag).find("/piece:") != std::string::npos) gone.insert(place.tag);
    }
    const auto merged = [](const std::string& tag) { return tag.rfind("E x~region1/", 0) == 0; };
    expectNumberedAsMasters(*out.faceted, kIsland, &p, gone, merged);
    expectChordsOnTheirEdges(*out.faceted, *out.exact);
}
