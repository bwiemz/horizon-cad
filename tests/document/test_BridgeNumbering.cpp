// Ruling 21: a bridge copy's facets, chords and seams are numbered as master's were. Master's
// Stable facets and edges were already `<face>/facet:<k>`, `<edge>/chord:<k>` and
// `<face>/seam:<k>`, and a saved reference binds by its whole name before its logical one
// (FilletOp's edges, MateGeometry's faces): a name that is still there but on another facet or
// chord would bind it silently. So each name master gave that the bridge still has names the same
// place, and each master gave that the bridge has not is accounted for.
//
// The tables are master's Stable output at 1709c556 (before the extrude was exact; its primitives'
// Stable path was PrimitiveFactory, scopeToFeature, nameFacetsLogically and nameEdgesLogically, and
// the extrude's its faceted prism, or its Booleans for a profile of several loops), and, for the
// revolves, at 9314271e (before the revolve was exact: its faceted rings sewn, or their Booleans
// for a profile of several loops), at 8 segments a turn: each face's name with the centroid of its
// outer loop, each edge's with its midpoint. "{k}" in a name is the k-th entity of the profile
// ("e<id>" in the tag). Every revolve turns its profile, drawn at x >= 0, about the sketch's y
// axis through its origin; a lemon, an apple and an ogive are recorded at 12 segments too
// (master's code path, re-enabled on this branch for the recording, reproduced its 8-segment
// lemon exactly). The
// hole tools are each type of hole 5 across and 6 deep into the top of a 20 x 20 x 10 box at its
// middle (a counterbore 9 across and 3 deep, a countersink 10 across at 90 degrees), as HoleFeature
// built its tool to cut.

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
#include "horizon/modeling/Revolve.h"
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

const Place kTurnedRectangle[] = {
    {"E x/edge:revolved:{0}|revolved:{1}/chord:0", 2.56066017178, 0, -1.06066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:1", 1.06066017178, 0, -2.56066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:2", -1.06066017178, 0, -2.56066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:3", -2.56066017178, 0, -1.06066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:4", -2.56066017178, 0, 1.06066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:5", -1.06066017178, 0, 2.56066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:6", 1.06066017178, 0, 2.56066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:7", 2.56066017178, 0, 1.06066017178},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:0", 2.56066017178, 5, -1.06066017178},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:1", 1.06066017178, 5, -2.56066017178},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:2", -1.06066017178, 5, -2.56066017178},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:3", -2.56066017178, 5, -1.06066017178},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:4", -2.56066017178, 5, 1.06066017178},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:5", -1.06066017178, 5, 2.56066017178},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:6", 1.06066017178, 5, 2.56066017178},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:7", 2.56066017178, 5, 1.06066017178},
    {"E x/revolved:{1}/seam:0", 2.12132034356, 2.5, -2.12132034356},
    {"E x/revolved:{1}/seam:1", 3, 2.5, 0},
    {"E x/revolved:{1}/seam:2", 1.83697019872e-16, 2.5, -3},
    {"E x/revolved:{1}/seam:3", -2.12132034356, 2.5, -2.12132034356},
    {"E x/revolved:{1}/seam:4", -3, 2.5, -3.67394039744e-16},
    {"E x/revolved:{1}/seam:5", -2.12132034356, 2.5, 2.12132034356},
    {"E x/revolved:{1}/seam:6", -5.51091059616e-16, 2.5, 3},
    {"E x/revolved:{1}/seam:7", 2.12132034356, 2.5, 2.12132034356},
    {"F x/revolved:{0}", 3.48909435263e-17, 0, -2.40517170169e-17},
    {"F x/revolved:{1}/facet:0", 2.56066017178, 2.5, -1.06066017178},
    {"F x/revolved:{1}/facet:1", 1.06066017178, 2.5, -2.56066017178},
    {"F x/revolved:{1}/facet:2", -1.06066017178, 2.5, -2.56066017178},
    {"F x/revolved:{1}/facet:3", -2.56066017178, 2.5, -1.06066017178},
    {"F x/revolved:{1}/facet:4", -2.56066017178, 2.5, 1.06066017178},
    {"F x/revolved:{1}/facet:5", -1.06066017178, 2.5, 2.56066017178},
    {"F x/revolved:{1}/facet:6", 1.06066017178, 2.5, 2.56066017178},
    {"F x/revolved:{1}/facet:7", 2.56066017178, 2.5, 1.06066017178},
    {"F x/revolved:{2}", -8.72273588158e-17, 5, 3.48909435263e-17},
};
const Place kTurnedRectangleQuarter[] = {
    {"E x/edge:cap_end|cap_start", 0, 2.5, 0},
    {"E x/edge:cap_end|revolved:{0}", 9.18485099361e-17, 0, -1.5},
    {"E x/edge:cap_end|revolved:{1}", 1.83697019872e-16, 2.5, -3},
    {"E x/edge:cap_end|revolved:{2}", 9.18485099361e-17, 5, -1.5},
    {"E x/edge:cap_start|revolved:{0}", 1.5, 0, 0},
    {"E x/edge:cap_start|revolved:{1}", 3, 2.5, 0},
    {"E x/edge:cap_start|revolved:{2}", 1.5, 5, 0},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:0", 2.56066017178, 0, -1.06066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:1", 1.06066017178, 0, -2.56066017178},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:0", 2.56066017178, 5, -1.06066017178},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:1", 1.06066017178, 5, -2.56066017178},
    {"E x/revolved:{0}/seam:0", 1.06066017178, 0, -1.06066017178},
    {"E x/revolved:{1}/seam:0", 2.12132034356, 2.5, -2.12132034356},
    {"E x/revolved:{2}/seam:0", 1.06066017178, 5, -1.06066017178},
    {"F x/cap_end", 9.18485099361e-17, 2.5, -1.5},
    {"F x/cap_start", 1.5, 2.5, 0},
    {"F x/revolved:{0}/facet:0", 1.70710678119, 0, -0.707106781187},
    {"F x/revolved:{0}/facet:1", 0.707106781187, 0, -1.70710678119},
    {"F x/revolved:{1}/facet:0", 2.56066017178, 2.5, -1.06066017178},
    {"F x/revolved:{1}/facet:1", 1.06066017178, 2.5, -2.56066017178},
    {"F x/revolved:{2}/facet:0", 1.70710678119, 5, -0.707106781187},
    {"F x/revolved:{2}/facet:1", 0.707106781187, 5, -1.70710678119},
};
const Place kTurnedTriangle[] = {
    {"E x/edge:revolved:{1}|revolved:{0}/chord:0", 2.56066017178, 0, -1.06066017178},
    {"E x/edge:revolved:{1}|revolved:{0}/chord:1", 1.06066017178, 0, -2.56066017178},
    {"E x/edge:revolved:{1}|revolved:{0}/chord:2", -1.06066017178, 0, -2.56066017178},
    {"E x/edge:revolved:{1}|revolved:{0}/chord:3", -2.56066017178, 0, -1.06066017178},
    {"E x/edge:revolved:{1}|revolved:{0}/chord:4", -2.56066017178, 0, 1.06066017178},
    {"E x/edge:revolved:{1}|revolved:{0}/chord:5", -1.06066017178, 0, 2.56066017178},
    {"E x/edge:revolved:{1}|revolved:{0}/chord:6", 1.06066017178, 0, 2.56066017178},
    {"E x/edge:revolved:{1}|revolved:{0}/chord:7", 2.56066017178, 0, 1.06066017178},
    {"E x/revolved:{1}/seam:0", 1.06066017178, 2, -1.06066017178},
    {"E x/revolved:{1}/seam:1", 1.5, 2, 0},
    {"E x/revolved:{1}/seam:2", 9.18485099361e-17, 2, -1.5},
    {"E x/revolved:{1}/seam:3", -1.06066017178, 2, -1.06066017178},
    {"E x/revolved:{1}/seam:4", -1.5, 2, -1.83697019872e-16},
    {"E x/revolved:{1}/seam:5", -1.06066017178, 2, 1.06066017178},
    {"E x/revolved:{1}/seam:6", -2.75545529808e-16, 2, 1.5},
    {"E x/revolved:{1}/seam:7", 1.06066017178, 2, 1.06066017178},
    {"F x/revolved:{0}", 3.48909435263e-17, 0, -2.40517170169e-17},
    {"F x/revolved:{1}/facet:0", 1.70710678119, 1.33333333333, -0.707106781187},
    {"F x/revolved:{1}/facet:1", 0.707106781187, 1.33333333333, -1.70710678119},
    {"F x/revolved:{1}/facet:2", -0.707106781187, 1.33333333333, -1.70710678119},
    {"F x/revolved:{1}/facet:3", -1.70710678119, 1.33333333333, -0.707106781187},
    {"F x/revolved:{1}/facet:4", -1.70710678119, 1.33333333333, 0.707106781187},
    {"F x/revolved:{1}/facet:5", -0.707106781187, 1.33333333333, 1.70710678119},
    {"F x/revolved:{1}/facet:6", 0.707106781187, 1.33333333333, 1.70710678119},
    {"F x/revolved:{1}/facet:7", 1.70710678119, 1.33333333333, 0.707106781187},
};
const Place kTurnedSemicircle[] = {
    {"E x/revolved:{0}/seam:0", 1.20710678119, -1.41421356237, -0.5},
    {"E x/revolved:{0}/seam:1", 0.707106781187, -1.70710678119, 0},
    {"E x/revolved:{0}/seam:10", 0.707106781187, 1.70710678119, 0},
    {"E x/revolved:{0}/seam:11", 0.5, -1.41421356237, -1.20710678119},
    {"E x/revolved:{0}/seam:12", -1.4039921706e-16, -1.70710678119, -0.707106781187},
    {"E x/revolved:{0}/seam:13", 1.04530142769e-16, -0.707106781187, -1.70710678119},
    {"E x/revolved:{0}/seam:14", 0.707106781187, -4.89858719659e-16, -1.70710678119},
    {"E x/revolved:{0}/seam:15", 1.04530142769e-16, 0.707106781187, -1.70710678119},
    {"E x/revolved:{0}/seam:16", 0.5, 1.41421356237, -1.20710678119},
    {"E x/revolved:{0}/seam:17", 1.04530142769e-16, 1.70710678119, -0.707106781187},
    {"E x/revolved:{0}/seam:18", -0.5, -1.41421356237, -1.20710678119},
    {"E x/revolved:{0}/seam:19", -0.5, -1.70710678119, -0.5},
    {"E x/revolved:{0}/seam:2", 0.5, -1.70710678119, -0.5},
    {"E x/revolved:{0}/seam:20", -1.20710678119, -0.707106781187, -1.20710678119},
    {"E x/revolved:{0}/seam:21", -0.707106781187, -4.89858719659e-16, -1.70710678119},
    {"E x/revolved:{0}/seam:22", -1.20710678119, 0.707106781187, -1.20710678119},
    {"E x/revolved:{0}/seam:23", -0.5, 1.41421356237, -1.20710678119},
    {"E x/revolved:{0}/seam:24", -0.5, 1.70710678119, -0.5},
    {"E x/revolved:{0}/seam:25", -1.20710678119, -1.41421356237, -0.5},
    {"E x/revolved:{0}/seam:26", -0.707106781187, -1.70710678119, -8.65956056235e-17},
    {"E x/revolved:{0}/seam:27", -1.70710678119, -0.707106781187, -2.09060285538e-16},
    {"E x/revolved:{0}/seam:28", -1.70710678119, -4.89858719659e-16, -0.707106781187},
    {"E x/revolved:{0}/seam:29", -1.70710678119, 0.707106781187, -2.09060285538e-16},
    {"E x/revolved:{0}/seam:3", 1.20710678119, -0.707106781187, -1.20710678119},
    {"E x/revolved:{0}/seam:30", -1.20710678119, 1.41421356237, -0.5},
    {"E x/revolved:{0}/seam:31", -0.707106781187, 1.70710678119, -8.65956056235e-17},
    {"E x/revolved:{0}/seam:32", -1.20710678119, -1.41421356237, 0.5},
    {"E x/revolved:{0}/seam:33", -0.5, -1.70710678119, 0.5},
    {"E x/revolved:{0}/seam:34", -1.20710678119, -0.707106781187, 1.20710678119},
    {"E x/revolved:{0}/seam:35", -1.70710678119, -4.89858719659e-16, 0.707106781187},
    {"E x/revolved:{0}/seam:36", -1.20710678119, 0.707106781187, 1.20710678119},
    {"E x/revolved:{0}/seam:37", -1.20710678119, 1.41421356237, 0.5},
    {"E x/revolved:{0}/seam:38", -0.5, 1.70710678119, 0.5},
    {"E x/revolved:{0}/seam:39", -0.5, -1.41421356237, 1.20710678119},
    {"E x/revolved:{0}/seam:4", 1.70710678119, -4.89858719659e-16, -0.707106781187},
    {"E x/revolved:{0}/seam:40", -3.13590428307e-16, -1.70710678119, 0.707106781187},
    {"E x/revolved:{0}/seam:41", -3.13590428307e-16, -0.707106781187, 1.70710678119},
    {"E x/revolved:{0}/seam:42", -0.707106781187, -4.89858719659e-16, 1.70710678119},
    {"E x/revolved:{0}/seam:43", -3.13590428307e-16, 0.707106781187, 1.70710678119},
    {"E x/revolved:{0}/seam:44", -0.5, 1.41421356237, 1.20710678119},
    {"E x/revolved:{0}/seam:45", -6.8661068478e-17, 1.70710678119, 0.707106781187},
    {"E x/revolved:{0}/seam:46", 0.5, -1.41421356237, 1.20710678119},
    {"E x/revolved:{0}/seam:47", 0.5, -1.70710678119, 0.5},
    {"E x/revolved:{0}/seam:48", 1.20710678119, -0.707106781187, 1.20710678119},
    {"E x/revolved:{0}/seam:49", 0.707106781187, -4.89858719659e-16, 1.70710678119},
    {"E x/revolved:{0}/seam:5", 1.70710678119, -0.707106781187, 0},
    {"E x/revolved:{0}/seam:50", 1.20710678119, 0.707106781187, 1.20710678119},
    {"E x/revolved:{0}/seam:51", 0.5, 1.41421356237, 1.20710678119},
    {"E x/revolved:{0}/seam:52", 0.5, 1.70710678119, 0.5},
    {"E x/revolved:{0}/seam:53", 1.20710678119, -1.41421356237, 0.5},
    {"E x/revolved:{0}/seam:54", 1.70710678119, -4.89858719659e-16, 0.707106781187},
    {"E x/revolved:{0}/seam:55", 1.20710678119, 1.41421356237, 0.5},
    {"E x/revolved:{0}/seam:6", 1.20710678119, 0.707106781187, -1.20710678119},
    {"E x/revolved:{0}/seam:7", 1.20710678119, 1.41421356237, -0.5},
    {"E x/revolved:{0}/seam:8", 1.70710678119, 0.707106781187, 0},
    {"E x/revolved:{0}/seam:9", 0.5, 1.70710678119, -0.5},
    {"F x/revolved:{0}/facet:0.0", 0.804737854124, -1.60947570825, -0.333333333333},
    {"F x/revolved:{0}/facet:0.1", 0.333333333333, -1.60947570825, -0.804737854124},
    {"F x/revolved:{0}/facet:0.2", -0.333333333333, -1.60947570825, -0.804737854124},
    {"F x/revolved:{0}/facet:0.3", -0.804737854124, -1.60947570825, -0.333333333333},
    {"F x/revolved:{0}/facet:0.4", -0.804737854124, -1.60947570825, 0.333333333333},
    {"F x/revolved:{0}/facet:0.5", -0.333333333333, -1.60947570825, 0.804737854124},
    {"F x/revolved:{0}/facet:0.6", 0.333333333333, -1.60947570825, 0.804737854124},
    {"F x/revolved:{0}/facet:0.7", 0.804737854124, -1.60947570825, 0.333333333333},
    {"F x/revolved:{0}/facet:1.0", 1.47140452079, -0.666666666667, -0.609475708249},
    {"F x/revolved:{0}/facet:1.1", 0.609475708249, -0.666666666667, -1.47140452079},
    {"F x/revolved:{0}/facet:1.2", -0.609475708249, -0.666666666667, -1.47140452079},
    {"F x/revolved:{0}/facet:1.3", -1.47140452079, -0.666666666667, -0.609475708249},
    {"F x/revolved:{0}/facet:1.4", -1.47140452079, -0.666666666667, 0.609475708249},
    {"F x/revolved:{0}/facet:1.5", -0.609475708249, -0.666666666667, 1.47140452079},
    {"F x/revolved:{0}/facet:1.6", 0.609475708249, -0.666666666667, 1.47140452079},
    {"F x/revolved:{0}/facet:1.7", 1.47140452079, -0.666666666667, 0.609475708249},
    {"F x/revolved:{0}/facet:2.0", 1.47140452079, 0.666666666667, -0.609475708249},
    {"F x/revolved:{0}/facet:2.1", 0.609475708249, 0.666666666667, -1.47140452079},
    {"F x/revolved:{0}/facet:2.2", -0.609475708249, 0.666666666667, -1.47140452079},
    {"F x/revolved:{0}/facet:2.3", -1.47140452079, 0.666666666667, -0.609475708249},
    {"F x/revolved:{0}/facet:2.4", -1.47140452079, 0.666666666667, 0.609475708249},
    {"F x/revolved:{0}/facet:2.5", -0.609475708249, 0.666666666667, 1.47140452079},
    {"F x/revolved:{0}/facet:2.6", 0.609475708249, 0.666666666667, 1.47140452079},
    {"F x/revolved:{0}/facet:2.7", 1.47140452079, 0.666666666667, 0.609475708249},
    {"F x/revolved:{0}/facet:3.0", 0.804737854124, 1.60947570825, -0.333333333333},
    {"F x/revolved:{0}/facet:3.1", 0.333333333333, 1.60947570825, -0.804737854124},
    {"F x/revolved:{0}/facet:3.2", -0.333333333333, 1.60947570825, -0.804737854124},
    {"F x/revolved:{0}/facet:3.3", -0.804737854124, 1.60947570825, -0.333333333333},
    {"F x/revolved:{0}/facet:3.4", -0.804737854124, 1.60947570825, 0.333333333333},
    {"F x/revolved:{0}/facet:3.5", -0.333333333333, 1.60947570825, 0.804737854124},
    {"F x/revolved:{0}/facet:3.6", 0.333333333333, 1.60947570825, 0.804737854124},
    {"F x/revolved:{0}/facet:3.7", 0.804737854124, 1.60947570825, 0.333333333333},
};
const Place kTurnedSemicircleQuarter[] = {
    {"E x/edge:cap_end|cap_start", -1.22464679915e-16, 0, 0},
    {"E x/edge:cap_end|revolved:{0}/chord:0", -1.4039921706e-16, -1.70710678119, -0.707106781187},
    {"E x/edge:cap_end|revolved:{0}/chord:1", 1.04530142769e-16, -0.707106781187, -1.70710678119},
    {"E x/edge:cap_end|revolved:{0}/chord:2", 1.04530142769e-16, 0.707106781187, -1.70710678119},
    {"E x/edge:cap_end|revolved:{0}/chord:3", 1.04530142769e-16, 1.70710678119, -0.707106781187},
    {"E x/edge:cap_start|revolved:{0}/chord:0", 0.707106781187, -1.70710678119, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:1", 1.70710678119, -0.707106781187, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:2", 1.70710678119, 0.707106781187, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:3", 0.707106781187, 1.70710678119, 0},
    {"E x/revolved:{0}/seam:0", 1.20710678119, -1.41421356237, -0.5},
    {"E x/revolved:{0}/seam:1", 0.5, -1.70710678119, -0.5},
    {"E x/revolved:{0}/seam:2", 1.20710678119, -0.707106781187, -1.20710678119},
    {"E x/revolved:{0}/seam:3", 1.70710678119, -4.89858719659e-16, -0.707106781187},
    {"E x/revolved:{0}/seam:4", 1.20710678119, 0.707106781187, -1.20710678119},
    {"E x/revolved:{0}/seam:5", 1.20710678119, 1.41421356237, -0.5},
    {"E x/revolved:{0}/seam:6", 0.5, 1.70710678119, -0.5},
    {"E x/revolved:{0}/seam:7", 0.5, -1.41421356237, -1.20710678119},
    {"E x/revolved:{0}/seam:8", 0.707106781187, -4.89858719659e-16, -1.70710678119},
    {"E x/revolved:{0}/seam:9", 0.5, 1.41421356237, -1.20710678119},
    {"F x/cap_end", 2.88652018745e-17, -7.85046229342e-17, -0.804737854124},
    {"F x/cap_start", 0.804737854124, -1.57009245868e-16, 0},
    {"F x/revolved:{0}/facet:0.0", 0.804737854124, -1.60947570825, -0.333333333333},
    {"F x/revolved:{0}/facet:0.1", 0.333333333333, -1.60947570825, -0.804737854124},
    {"F x/revolved:{0}/facet:1.0", 1.47140452079, -0.666666666667, -0.609475708249},
    {"F x/revolved:{0}/facet:1.1", 0.609475708249, -0.666666666667, -1.47140452079},
    {"F x/revolved:{0}/facet:2.0", 1.47140452079, 0.666666666667, -0.609475708249},
    {"F x/revolved:{0}/facet:2.1", 0.609475708249, 0.666666666667, -1.47140452079},
    {"F x/revolved:{0}/facet:3.0", 0.804737854124, 1.60947570825, -0.333333333333},
    {"F x/revolved:{0}/facet:3.1", 0.333333333333, 1.60947570825, -0.804737854124},
};
const Place kTurnedTorus[] = {
    {"E x/revolved:{0}/seam:0", 4.13908729653, 0.353553390593, -4.13908729653},
    {"E x/revolved:{0}/seam:1", 4.87132034356, 0.707106781187, -2.01776695297},
    {"E x/revolved:{0}/seam:10", 2.93198051534, 0.353553390593, -2.93198051534},
    {"E x/revolved:{0}/seam:100", -1.76776695297, -1, 4.26776695297},
    {"E x/revolved:{0}/seam:101", -9.83431803578e-16, -0.853553390593, 5.35355339059},
    {"E x/revolved:{0}/seam:102", -2.01776695297, -0.707106781187, 4.87132034356},
    {"E x/revolved:{0}/seam:103", -1.07528031351e-15, -0.353553390593, 5.85355339059},
    {"E x/revolved:{0}/seam:104", 4.13908729653, 0.353553390593, 4.13908729653},
    {"E x/revolved:{0}/seam:105", 2.01776695297, 0.707106781187, 4.87132034356},
    {"E x/revolved:{0}/seam:106", 2.12132034356, 0, 5.12132034356},
    {"E x/revolved:{0}/seam:107", 3.78553390593, 0.853553390593, 3.78553390593},
    {"E x/revolved:{0}/seam:108", 1.76776695297, 1, 4.26776695297},
    {"E x/revolved:{0}/seam:109", 3.28553390593, 0.853553390593, 3.28553390593},
    {"E x/revolved:{0}/seam:11", 3.41421356237, 1.22464679915e-16, -1.41421356237},
    {"E x/revolved:{0}/seam:110", 1.51776695297, 0.707106781187, 3.66421356237},
    {"E x/revolved:{0}/seam:111", 2.93198051534, 0.353553390593, 2.93198051534},
    {"E x/revolved:{0}/seam:112", 1.41421356237, 1.22464679915e-16, 3.41421356237},
    {"E x/revolved:{0}/seam:113", 2.93198051534, -0.353553390593, 2.93198051534},
    {"E x/revolved:{0}/seam:114", 1.51776695297, -0.707106781187, 3.66421356237},
    {"E x/revolved:{0}/seam:115", 3.28553390593, -0.853553390593, 3.28553390593},
    {"E x/revolved:{0}/seam:116", 1.76776695297, -1, 4.26776695297},
    {"E x/revolved:{0}/seam:117", 3.78553390593, -0.853553390593, 3.78553390593},
    {"E x/revolved:{0}/seam:118", 2.01776695297, -0.707106781187, 4.87132034356},
    {"E x/revolved:{0}/seam:119", 4.13908729653, -0.353553390593, 4.13908729653},
    {"E x/revolved:{0}/seam:12", 4.14644660941, 0.353553390593, 0},
    {"E x/revolved:{0}/seam:120", 4.87132034356, 0.707106781187, 2.01776695297},
    {"E x/revolved:{0}/seam:121", 5.12132034356, 0, 2.12132034356},
    {"E x/revolved:{0}/seam:122", 4.26776695297, 1, 1.76776695297},
    {"E x/revolved:{0}/seam:123", 3.66421356237, 0.707106781187, 1.51776695297},
    {"E x/revolved:{0}/seam:124", 3.41421356237, 1.22464679915e-16, 1.41421356237},
    {"E x/revolved:{0}/seam:125", 3.66421356237, -0.707106781187, 1.51776695297},
    {"E x/revolved:{0}/seam:126", 4.26776695297, -1, 1.76776695297},
    {"E x/revolved:{0}/seam:127", 4.87132034356, -0.707106781187, 2.01776695297},
    {"E x/revolved:{0}/seam:13", 2.93198051534, -0.353553390593, -2.93198051534},
    {"E x/revolved:{0}/seam:14", 3.66421356237, -0.707106781187, -1.51776695297},
    {"E x/revolved:{0}/seam:15", 4.14644660941, -0.353553390593, 0},
    {"E x/revolved:{0}/seam:16", 3.28553390593, -0.853553390593, -3.28553390593},
    {"E x/revolved:{0}/seam:17", 4.26776695297, -1, -1.76776695297},
    {"E x/revolved:{0}/seam:18", 4.64644660941, -0.853553390593, 0},
    {"E x/revolved:{0}/seam:19", 3.78553390593, -0.853553390593, -3.78553390593},
    {"E x/revolved:{0}/seam:2", 5.85355339059, 0.353553390593, 0},
    {"E x/revolved:{0}/seam:20", 4.87132034356, -0.707106781187, -2.01776695297},
    {"E x/revolved:{0}/seam:21", 5.35355339059, -0.853553390593, 0},
    {"E x/revolved:{0}/seam:22", 4.13908729653, -0.353553390593, -4.13908729653},
    {"E x/revolved:{0}/seam:23", 5.85355339059, -0.353553390593, 0},
    {"E x/revolved:{0}/seam:24", 3.58426771171e-16, 0.353553390593, -5.85355339059},
    {"E x/revolved:{0}/seam:25", 2.01776695297, 0.707106781187, -4.87132034356},
    {"E x/revolved:{0}/seam:26", 2.12132034356, 0, -5.12132034356},
    {"E x/revolved:{0}/seam:27", 3.27810601193e-16, 0.853553390593, -5.35355339059},
    {"E x/revolved:{0}/seam:28", 1.76776695297, 1, -4.26776695297},
    {"E x/revolved:{0}/seam:29", 2.84512798381e-16, 0.853553390593, -4.64644660941},
    {"E x/revolved:{0}/seam:3", 5.12132034356, 0, -2.12132034356},
    {"E x/revolved:{0}/seam:30", 1.51776695297, 0.707106781187, -3.66421356237},
    {"E x/revolved:{0}/seam:31", 2.53896628402e-16, 0.353553390593, -4.14644660941},
    {"E x/revolved:{0}/seam:32", 1.41421356237, 1.22464679915e-16, -3.41421356237},
    {"E x/revolved:{0}/seam:33", 2.53896628402e-16, -0.353553390593, -4.14644660941},
    {"E x/revolved:{0}/seam:34", 1.51776695297, -0.707106781187, -3.66421356237},
    {"E x/revolved:{0}/seam:35", 2.84512798381e-16, -0.853553390593, -4.64644660941},
    {"E x/revolved:{0}/seam:36", 1.76776695297, -1, -4.26776695297},
    {"E x/revolved:{0}/seam:37", 3.27810601193e-16, -0.853553390593, -5.35355339059},
    {"E x/revolved:{0}/seam:38", 2.01776695297, -0.707106781187, -4.87132034356},
    {"E x/revolved:{0}/seam:39", 3.58426771171e-16, -0.353553390593, -5.85355339059},
    {"E x/revolved:{0}/seam:4", 3.78553390593, 0.853553390593, -3.78553390593},
    {"E x/revolved:{0}/seam:40", -4.13908729653, 0.353553390593, -4.13908729653},
    {"E x/revolved:{0}/seam:41", -2.01776695297, 0.707106781187, -4.87132034356},
    {"E x/revolved:{0}/seam:42", -2.12132034356, 0, -5.12132034356},
    {"E x/revolved:{0}/seam:43", -3.78553390593, 0.853553390593, -3.78553390593},
    {"E x/revolved:{0}/seam:44", -1.76776695297, 1, -4.26776695297},
    {"E x/revolved:{0}/seam:45", -3.28553390593, 0.853553390593, -3.28553390593},
    {"E x/revolved:{0}/seam:46", -1.51776695297, 0.707106781187, -3.66421356237},
    {"E x/revolved:{0}/seam:47", -2.93198051534, 0.353553390593, -2.93198051534},
    {"E x/revolved:{0}/seam:48", -1.41421356237, 1.22464679915e-16, -3.41421356237},
    {"E x/revolved:{0}/seam:49", -2.93198051534, -0.353553390593, -2.93198051534},
    {"E x/revolved:{0}/seam:5", 4.26776695297, 1, -1.76776695297},
    {"E x/revolved:{0}/seam:50", -1.51776695297, -0.707106781187, -3.66421356237},
    {"E x/revolved:{0}/seam:51", -3.28553390593, -0.853553390593, -3.28553390593},
    {"E x/revolved:{0}/seam:52", -1.76776695297, -1, -4.26776695297},
    {"E x/revolved:{0}/seam:53", -3.78553390593, -0.853553390593, -3.78553390593},
    {"E x/revolved:{0}/seam:54", -2.01776695297, -0.707106781187, -4.87132034356},
    {"E x/revolved:{0}/seam:55", -4.13908729653, -0.353553390593, -4.13908729653},
    {"E x/revolved:{0}/seam:56", -5.85355339059, 0.353553390593, -7.16853542343e-16},
    {"E x/revolved:{0}/seam:57", -4.87132034356, 0.707106781187, -2.01776695297},
    {"E x/revolved:{0}/seam:58", -5.12132034356, 0, -2.12132034356},
    {"E x/revolved:{0}/seam:59", -5.35355339059, 0.853553390593, -6.55621202385e-16},
    {"E x/revolved:{0}/seam:6", 5.35355339059, 0.853553390593, 0},
    {"E x/revolved:{0}/seam:60", -4.26776695297, 1, -1.76776695297},
    {"E x/revolved:{0}/seam:61", -4.64644660941, 0.853553390593, -5.69025596762e-16},
    {"E x/revolved:{0}/seam:62", -3.66421356237, 0.707106781187, -1.51776695297},
    {"E x/revolved:{0}/seam:63", -4.14644660941, 0.353553390593, -5.07793256805e-16},
    {"E x/revolved:{0}/seam:64", -3.41421356237, 1.22464679915e-16, -1.41421356237},
    {"E x/revolved:{0}/seam:65", -4.14644660941, -0.353553390593, -5.07793256805e-16},
    {"E x/revolved:{0}/seam:66", -3.66421356237, -0.707106781187, -1.51776695297},
    {"E x/revolved:{0}/seam:67", -4.64644660941, -0.853553390593, -5.69025596762e-16},
    {"E x/revolved:{0}/seam:68", -4.26776695297, -1, -1.76776695297},
    {"E x/revolved:{0}/seam:69", -5.35355339059, -0.853553390593, -6.55621202385e-16},
    {"E x/revolved:{0}/seam:7", 3.28553390593, 0.853553390593, -3.28553390593},
    {"E x/revolved:{0}/seam:70", -4.87132034356, -0.707106781187, -2.01776695297},
    {"E x/revolved:{0}/seam:71", -5.85355339059, -0.353553390593, -7.16853542343e-16},
    {"E x/revolved:{0}/seam:72", -4.13908729653, 0.353553390593, 4.13908729653},
    {"E x/revolved:{0}/seam:73", -4.87132034356, 0.707106781187, 2.01776695297},
    {"E x/revolved:{0}/seam:74", -5.12132034356, 0, 2.12132034356},
    {"E x/revolved:{0}/seam:75", -3.78553390593, 0.853553390593, 3.78553390593},
    {"E x/revolved:{0}/seam:76", -4.26776695297, 1, 1.76776695297},
    {"E x/revolved:{0}/seam:77", -3.28553390593, 0.853553390593, 3.28553390593},
    {"E x/revolved:{0}/seam:78", -3.66421356237, 0.707106781187, 1.51776695297},
    {"E x/revolved:{0}/seam:79", -2.93198051534, 0.353553390593, 2.93198051534},
    {"E x/revolved:{0}/seam:8", 3.66421356237, 0.707106781187, -1.51776695297},
    {"E x/revolved:{0}/seam:80", -3.41421356237, 1.22464679915e-16, 1.41421356237},
    {"E x/revolved:{0}/seam:81", -2.93198051534, -0.353553390593, 2.93198051534},
    {"E x/revolved:{0}/seam:82", -3.66421356237, -0.707106781187, 1.51776695297},
    {"E x/revolved:{0}/seam:83", -3.28553390593, -0.853553390593, 3.28553390593},
    {"E x/revolved:{0}/seam:84", -4.26776695297, -1, 1.76776695297},
    {"E x/revolved:{0}/seam:85", -3.78553390593, -0.853553390593, 3.78553390593},
    {"E x/revolved:{0}/seam:86", -4.87132034356, -0.707106781187, 2.01776695297},
    {"E x/revolved:{0}/seam:87", -4.13908729653, -0.353553390593, 4.13908729653},
    {"E x/revolved:{0}/seam:88", -1.07528031351e-15, 0.353553390593, 5.85355339059},
    {"E x/revolved:{0}/seam:89", -2.01776695297, 0.707106781187, 4.87132034356},
    {"E x/revolved:{0}/seam:9", 4.64644660941, 0.853553390593, 0},
    {"E x/revolved:{0}/seam:90", -2.12132034356, 0, 5.12132034356},
    {"E x/revolved:{0}/seam:91", -9.83431803578e-16, 0.853553390593, 5.35355339059},
    {"E x/revolved:{0}/seam:92", -1.76776695297, 1, 4.26776695297},
    {"E x/revolved:{0}/seam:93", -8.53538395143e-16, 0.853553390593, 4.64644660941},
    {"E x/revolved:{0}/seam:94", -1.51776695297, 0.707106781187, 3.66421356237},
    {"E x/revolved:{0}/seam:95", -7.61689885207e-16, 0.353553390593, 4.14644660941},
    {"E x/revolved:{0}/seam:96", -1.41421356237, 1.22464679915e-16, 3.41421356237},
    {"E x/revolved:{0}/seam:97", -7.61689885207e-16, -0.353553390593, 4.14644660941},
    {"E x/revolved:{0}/seam:98", -1.51776695297, -0.707106781187, 3.66421356237},
    {"E x/revolved:{0}/seam:99", -8.53538395143e-16, -0.853553390593, 4.64644660941},
    {"F x/revolved:{0}/facet:0.0", 4.99736277739, 0.350604942483, -2.06997543849},
    {"F x/revolved:{0}/facet:0.1", 2.06997543849, 0.350604942483, -4.99736277739},
    {"F x/revolved:{0}/facet:0.2", -2.06997543849, 0.350604942483, -4.99736277739},
    {"F x/revolved:{0}/facet:0.3", -4.99736277739, 0.350604942483, -2.06997543849},
    {"F x/revolved:{0}/facet:0.4", -4.99736277739, 0.350604942483, 2.06997543849},
    {"F x/revolved:{0}/facet:0.5", -2.06997543849, 0.350604942483, 4.99736277739},
    {"F x/revolved:{0}/facet:0.6", 2.06997543849, 0.350604942483, 4.99736277739},
    {"F x/revolved:{0}/facet:0.7", 4.99736277739, 0.350604942483, 2.06997543849},
    {"F x/revolved:{0}/facet:1.0", 4.57618684802, 0.850329569484, -1.89551865641},
    {"F x/revolved:{0}/facet:1.1", 1.89551865641, 0.850329569484, -4.57618684802},
    {"F x/revolved:{0}/facet:1.2", -1.89551865641, 0.850329569484, -4.57618684802},
    {"F x/revolved:{0}/facet:1.3", -4.57618684802, 0.850329569484, -1.89551865641},
    {"F x/revolved:{0}/facet:1.4", -4.57618684802, 0.850329569484, 1.89551865641},
    {"F x/revolved:{0}/facet:1.5", -1.89551865641, 0.850329569484, 4.57618684802},
    {"F x/revolved:{0}/facet:1.6", 1.89551865641, 0.850329569484, 4.57618684802},
    {"F x/revolved:{0}/facet:1.7", 4.57618684802, 0.850329569484, 1.89551865641},
    {"F x/revolved:{0}/facet:2.0", 3.97364443463, 0.85726782011, -1.64593741687},
    {"F x/revolved:{0}/facet:2.1", 1.64593741687, 0.85726782011, -3.97364443463},
    {"F x/revolved:{0}/facet:2.2", -1.64593741687, 0.85726782011, -3.97364443463},
    {"F x/revolved:{0}/facet:2.3", -3.97364443463, 0.85726782011, -1.64593741687},
    {"F x/revolved:{0}/facet:2.4", -3.97364443463, 0.85726782011, 1.64593741687},
    {"F x/revolved:{0}/facet:2.5", -1.64593741687, 0.85726782011, 3.97364443463},
    {"F x/revolved:{0}/facet:2.6", 1.64593741687, 0.85726782011, 3.97364443463},
    {"F x/revolved:{0}/facet:2.7", 3.97364443463, 0.85726782011, 1.64593741687},
    {"F x/revolved:{0}/facet:3.0", 3.5406851699, 0.357715725252, -1.46659981747},
    {"F x/revolved:{0}/facet:3.1", 1.46659981747, 0.357715725252, -3.5406851699},
    {"F x/revolved:{0}/facet:3.2", -1.46659981747, 0.357715725252, -3.5406851699},
    {"F x/revolved:{0}/facet:3.3", -3.5406851699, 0.357715725252, -1.46659981747},
    {"F x/revolved:{0}/facet:3.4", -3.5406851699, 0.357715725252, 1.46659981747},
    {"F x/revolved:{0}/facet:3.5", -1.46659981747, 0.357715725252, 3.5406851699},
    {"F x/revolved:{0}/facet:3.6", 1.46659981747, 0.357715725252, 3.5406851699},
    {"F x/revolved:{0}/facet:3.7", 3.5406851699, 0.357715725252, 1.46659981747},
    {"F x/revolved:{0}/facet:4.0", 3.5406851699, -0.357715725252, -1.46659981747},
    {"F x/revolved:{0}/facet:4.1", 1.46659981747, -0.357715725252, -3.5406851699},
    {"F x/revolved:{0}/facet:4.2", -1.46659981747, -0.357715725252, -3.5406851699},
    {"F x/revolved:{0}/facet:4.3", -3.5406851699, -0.357715725252, -1.46659981747},
    {"F x/revolved:{0}/facet:4.4", -3.5406851699, -0.357715725252, 1.46659981747},
    {"F x/revolved:{0}/facet:4.5", -1.46659981747, -0.357715725252, 3.5406851699},
    {"F x/revolved:{0}/facet:4.6", 1.46659981747, -0.357715725252, 3.5406851699},
    {"F x/revolved:{0}/facet:4.7", 3.5406851699, -0.357715725252, 1.46659981747},
    {"F x/revolved:{0}/facet:5.0", 3.97364443463, -0.85726782011, -1.64593741687},
    {"F x/revolved:{0}/facet:5.1", 1.64593741687, -0.85726782011, -3.97364443463},
    {"F x/revolved:{0}/facet:5.2", -1.64593741687, -0.85726782011, -3.97364443463},
    {"F x/revolved:{0}/facet:5.3", -3.97364443463, -0.85726782011, -1.64593741687},
    {"F x/revolved:{0}/facet:5.4", -3.97364443463, -0.85726782011, 1.64593741687},
    {"F x/revolved:{0}/facet:5.5", -1.64593741687, -0.85726782011, 3.97364443463},
    {"F x/revolved:{0}/facet:5.6", 1.64593741687, -0.85726782011, 3.97364443463},
    {"F x/revolved:{0}/facet:5.7", 3.97364443463, -0.85726782011, 1.64593741687},
    {"F x/revolved:{0}/facet:6.0", 4.57618684802, -0.850329569484, -1.89551865641},
    {"F x/revolved:{0}/facet:6.1", 1.89551865641, -0.850329569484, -4.57618684802},
    {"F x/revolved:{0}/facet:6.2", -1.89551865641, -0.850329569484, -4.57618684802},
    {"F x/revolved:{0}/facet:6.3", -4.57618684802, -0.850329569484, -1.89551865641},
    {"F x/revolved:{0}/facet:6.4", -4.57618684802, -0.850329569484, 1.89551865641},
    {"F x/revolved:{0}/facet:6.5", -1.89551865641, -0.850329569484, 4.57618684802},
    {"F x/revolved:{0}/facet:6.6", 1.89551865641, -0.850329569484, 4.57618684802},
    {"F x/revolved:{0}/facet:6.7", 4.57618684802, -0.850329569484, 1.89551865641},
    {"F x/revolved:{0}/facet:7.0", 4.99736277739, -0.350604942483, -2.06997543849},
    {"F x/revolved:{0}/facet:7.1", 2.06997543849, -0.350604942483, -4.99736277739},
    {"F x/revolved:{0}/facet:7.2", -2.06997543849, -0.350604942483, -4.99736277739},
    {"F x/revolved:{0}/facet:7.3", -4.99736277739, -0.350604942483, -2.06997543849},
    {"F x/revolved:{0}/facet:7.4", -4.99736277739, -0.350604942483, 2.06997543849},
    {"F x/revolved:{0}/facet:7.5", -2.06997543849, -0.350604942483, 4.99736277739},
    {"F x/revolved:{0}/facet:7.6", 2.06997543849, -0.350604942483, 4.99736277739},
    {"F x/revolved:{0}/facet:7.7", 4.99736277739, -0.350604942483, 2.06997543849},
};
const Place kTurnedTorusQuarter[] = {
    {"E x/edge:cap_end|revolved:{0}/chord:0", 3.58426771171e-16, 0.353553390593, -5.85355339059},
    {"E x/edge:cap_end|revolved:{0}/chord:1", 3.27810601193e-16, 0.853553390593, -5.35355339059},
    {"E x/edge:cap_end|revolved:{0}/chord:2", 2.84512798381e-16, 0.853553390593, -4.64644660941},
    {"E x/edge:cap_end|revolved:{0}/chord:3", 2.53896628402e-16, 0.353553390593, -4.14644660941},
    {"E x/edge:cap_end|revolved:{0}/chord:4", 2.53896628402e-16, -0.353553390593, -4.14644660941},
    {"E x/edge:cap_end|revolved:{0}/chord:5", 2.84512798381e-16, -0.853553390593, -4.64644660941},
    {"E x/edge:cap_end|revolved:{0}/chord:6", 3.27810601193e-16, -0.853553390593, -5.35355339059},
    {"E x/edge:cap_end|revolved:{0}/chord:7", 3.58426771171e-16, -0.353553390593, -5.85355339059},
    {"E x/edge:cap_start|revolved:{0}/chord:0", 5.85355339059, 0.353553390593, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:1", 5.35355339059, 0.853553390593, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:2", 4.64644660941, 0.853553390593, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:3", 4.14644660941, 0.353553390593, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:4", 4.14644660941, -0.353553390593, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:5", 4.64644660941, -0.853553390593, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:6", 5.35355339059, -0.853553390593, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:7", 5.85355339059, -0.353553390593, 0},
    {"E x/revolved:{0}/seam:0", 4.13908729653, 0.353553390593, -4.13908729653},
    {"E x/revolved:{0}/seam:1", 4.87132034356, 0.707106781187, -2.01776695297},
    {"E x/revolved:{0}/seam:10", 3.66421356237, -0.707106781187, -1.51776695297},
    {"E x/revolved:{0}/seam:11", 3.28553390593, -0.853553390593, -3.28553390593},
    {"E x/revolved:{0}/seam:12", 4.26776695297, -1, -1.76776695297},
    {"E x/revolved:{0}/seam:13", 3.78553390593, -0.853553390593, -3.78553390593},
    {"E x/revolved:{0}/seam:14", 4.87132034356, -0.707106781187, -2.01776695297},
    {"E x/revolved:{0}/seam:15", 4.13908729653, -0.353553390593, -4.13908729653},
    {"E x/revolved:{0}/seam:16", 2.01776695297, 0.707106781187, -4.87132034356},
    {"E x/revolved:{0}/seam:17", 2.12132034356, 0, -5.12132034356},
    {"E x/revolved:{0}/seam:18", 1.76776695297, 1, -4.26776695297},
    {"E x/revolved:{0}/seam:19", 1.51776695297, 0.707106781187, -3.66421356237},
    {"E x/revolved:{0}/seam:2", 5.12132034356, 0, -2.12132034356},
    {"E x/revolved:{0}/seam:20", 1.41421356237, 1.22464679915e-16, -3.41421356237},
    {"E x/revolved:{0}/seam:21", 1.51776695297, -0.707106781187, -3.66421356237},
    {"E x/revolved:{0}/seam:22", 1.76776695297, -1, -4.26776695297},
    {"E x/revolved:{0}/seam:23", 2.01776695297, -0.707106781187, -4.87132034356},
    {"E x/revolved:{0}/seam:3", 3.78553390593, 0.853553390593, -3.78553390593},
    {"E x/revolved:{0}/seam:4", 4.26776695297, 1, -1.76776695297},
    {"E x/revolved:{0}/seam:5", 3.28553390593, 0.853553390593, -3.28553390593},
    {"E x/revolved:{0}/seam:6", 3.66421356237, 0.707106781187, -1.51776695297},
    {"E x/revolved:{0}/seam:7", 2.93198051534, 0.353553390593, -2.93198051534},
    {"E x/revolved:{0}/seam:8", 3.41421356237, 1.22464679915e-16, -1.41421356237},
    {"E x/revolved:{0}/seam:9", 2.93198051534, -0.353553390593, -2.93198051534},
    {"F x/cap_end", 3.06161699787e-16, 7.30850096795e-17, -5},
    {"F x/cap_start", 5, 8.34111618676e-17, 0},
    {"F x/revolved:{0}/facet:0.0", 4.99736277739, 0.350604942483, -2.06997543849},
    {"F x/revolved:{0}/facet:0.1", 2.06997543849, 0.350604942483, -4.99736277739},
    {"F x/revolved:{0}/facet:1.0", 4.57618684802, 0.850329569484, -1.89551865641},
    {"F x/revolved:{0}/facet:1.1", 1.89551865641, 0.850329569484, -4.57618684802},
    {"F x/revolved:{0}/facet:2.0", 3.97364443463, 0.85726782011, -1.64593741687},
    {"F x/revolved:{0}/facet:2.1", 1.64593741687, 0.85726782011, -3.97364443463},
    {"F x/revolved:{0}/facet:3.0", 3.5406851699, 0.357715725252, -1.46659981747},
    {"F x/revolved:{0}/facet:3.1", 1.46659981747, 0.357715725252, -3.5406851699},
    {"F x/revolved:{0}/facet:4.0", 3.5406851699, -0.357715725252, -1.46659981747},
    {"F x/revolved:{0}/facet:4.1", 1.46659981747, -0.357715725252, -3.5406851699},
    {"F x/revolved:{0}/facet:5.0", 3.97364443463, -0.85726782011, -1.64593741687},
    {"F x/revolved:{0}/facet:5.1", 1.64593741687, -0.85726782011, -3.97364443463},
    {"F x/revolved:{0}/facet:6.0", 4.57618684802, -0.850329569484, -1.89551865641},
    {"F x/revolved:{0}/facet:6.1", 1.89551865641, -0.850329569484, -4.57618684802},
    {"F x/revolved:{0}/facet:7.0", 4.99736277739, -0.350604942483, -2.06997543849},
    {"F x/revolved:{0}/facet:7.1", 2.06997543849, -0.350604942483, -4.99736277739},
};
const Place kTurnedLemon[] = {
    {"E x/revolved:{0}/seam:0", 0.750602254581, -0.684040286651, -0.310909633795},
    {"E x/revolved:{0}/seam:1", 0.439692620786, -1.20804554711, 0},
    {"E x/revolved:{0}/seam:10", 5.38468160654e-17, -4.99600361081e-16, -0.879385241572},
    {"E x/revolved:{0}/seam:11", 0.310909633795, 0.684040286651, -0.750602254581},
    {"E x/revolved:{0}/seam:12", 1.37945710495e-16, 1.20804554711, -0.439692620786},
    {"E x/revolved:{0}/seam:13", -0.310909633795, -0.684040286651, -0.750602254581},
    {"E x/revolved:{0}/seam:14", -0.310909633795, -1.20804554711, -0.310909633795},
    {"E x/revolved:{0}/seam:15", -0.621819267591, -4.4408920985e-16, -0.621819267591},
    {"E x/revolved:{0}/seam:16", -0.310909633795, 0.684040286651, -0.750602254581},
    {"E x/revolved:{0}/seam:17", -0.310909633795, 1.20804554711, -0.310909633795},
    {"E x/revolved:{0}/seam:18", -0.750602254581, -0.684040286651, -0.310909633795},
    {"E x/revolved:{0}/seam:19", -0.439692620786, -1.20804554711, -5.38468160654e-17},
    {"E x/revolved:{0}/seam:2", 0.310909633795, -1.20804554711, -0.310909633795},
    {"E x/revolved:{0}/seam:20", -0.879385241572, -4.99600361081e-16, -1.07693632131e-16},
    {"E x/revolved:{0}/seam:21", -0.750602254581, 0.684040286651, -0.310909633795},
    {"E x/revolved:{0}/seam:22", -0.439692620786, 1.20804554711, -5.38468160654e-17},
    {"E x/revolved:{0}/seam:23", -0.750602254581, -0.684040286651, 0.310909633795},
    {"E x/revolved:{0}/seam:24", -0.310909633795, -1.20804554711, 0.310909633795},
    {"E x/revolved:{0}/seam:25", -0.621819267591, -4.99600361081e-16, 0.621819267591},
    {"E x/revolved:{0}/seam:26", -0.750602254581, 0.684040286651, 0.310909633795},
    {"E x/revolved:{0}/seam:27", -0.310909633795, 1.20804554711, 0.310909633795},
    {"E x/revolved:{0}/seam:28", -0.310909633795, -0.684040286651, 0.750602254581},
    {"E x/revolved:{0}/seam:29", 3.02520783644e-17, -1.20804554711, 0.439692620786},
    {"E x/revolved:{0}/seam:3", 0.621819267591, -4.99600361081e-16, -0.621819267591},
    {"E x/revolved:{0}/seam:30", -1.61540448196e-16, -4.99600361081e-16, 0.879385241572},
    {"E x/revolved:{0}/seam:31", -0.310909633795, 0.684040286651, 0.750602254581},
    {"E x/revolved:{0}/seam:32", 3.02520783644e-17, 1.20804554711, 0.439692620786},
    {"E x/revolved:{0}/seam:33", 0.310909633795, -0.684040286651, 0.750602254581},
    {"E x/revolved:{0}/seam:34", 0.310909633795, -1.20804554711, 0.310909633795},
    {"E x/revolved:{0}/seam:35", 0.621819267591, -4.99600361081e-16, 0.621819267591},
    {"E x/revolved:{0}/seam:36", 0.310909633795, 0.684040286651, 0.750602254581},
    {"E x/revolved:{0}/seam:37", 0.310909633795, 1.20804554711, 0.310909633795},
    {"E x/revolved:{0}/seam:38", 0.750602254581, -0.684040286651, 0.310909633795},
    {"E x/revolved:{0}/seam:39", 0.750602254581, 0.684040286651, 0.310909633795},
    {"E x/revolved:{0}/seam:4", 0.750602254581, 0.684040286651, -0.310909633795},
    {"E x/revolved:{0}/seam:5", 0.879385241572, -4.99600361081e-16, 0},
    {"E x/revolved:{0}/seam:6", 0.310909633795, 1.20804554711, -0.310909633795},
    {"E x/revolved:{0}/seam:7", 0.439692620786, 1.20804554711, 0},
    {"E x/revolved:{0}/seam:8", 0.310909633795, -0.684040286651, -0.750602254581},
    {"E x/revolved:{0}/seam:9", 1.37945710495e-16, -1.20804554711, -0.439692620786},
    {"F x/revolved:{0}/facet:0.0", 0.500401503054, -1.03337712696, -0.207273089197},
    {"F x/revolved:{0}/facet:0.1", 0.207273089197, -1.03337712696, -0.500401503054},
    {"F x/revolved:{0}/facet:0.2", -0.207273089197, -1.03337712696, -0.500401503054},
    {"F x/revolved:{0}/facet:0.3", -0.500401503054, -1.03337712696, -0.207273089197},
    {"F x/revolved:{0}/facet:0.4", -0.500401503054, -1.03337712696, 0.207273089197},
    {"F x/revolved:{0}/facet:0.5", -0.207273089197, -1.03337712696, 0.500401503054},
    {"F x/revolved:{0}/facet:0.6", 0.207273089197, -1.03337712696, 0.500401503054},
    {"F x/revolved:{0}/facet:0.7", 0.500401503054, -1.03337712696, 0.207273089197},
    {"F x/revolved:{0}/facet:1.0", 0.750602254581, -4.52148393994e-16, -0.310909633795},
    {"F x/revolved:{0}/facet:1.1", 0.310909633795, -4.52148393994e-16, -0.750602254581},
    {"F x/revolved:{0}/facet:1.2", -0.310909633795, -4.37076780861e-16, -0.750602254581},
    {"F x/revolved:{0}/facet:1.3", -0.750602254581, -4.22005167728e-16, -0.310909633795},
    {"F x/revolved:{0}/facet:1.4", -0.750602254581, -4.37076780861e-16, 0.310909633795},
    {"F x/revolved:{0}/facet:1.5", -0.310909633795, -4.67220007127e-16, 0.750602254581},
    {"F x/revolved:{0}/facet:1.6", 0.310909633795, -4.52148393994e-16, 0.750602254581},
    {"F x/revolved:{0}/facet:1.7", 0.750602254581, -4.37076780861e-16, 0.310909633795},
    {"F x/revolved:{0}/facet:2.0", 0.500401503054, 1.03337712696, -0.207273089197},
    {"F x/revolved:{0}/facet:2.1", 0.207273089197, 1.03337712696, -0.500401503054},
    {"F x/revolved:{0}/facet:2.2", -0.207273089197, 1.03337712696, -0.500401503054},
    {"F x/revolved:{0}/facet:2.3", -0.500401503054, 1.03337712696, -0.207273089197},
    {"F x/revolved:{0}/facet:2.4", -0.500401503054, 1.03337712696, 0.207273089197},
    {"F x/revolved:{0}/facet:2.5", -0.207273089197, 1.03337712696, 0.500401503054},
    {"F x/revolved:{0}/facet:2.6", 0.207273089197, 1.03337712696, 0.500401503054},
    {"F x/revolved:{0}/facet:2.7", 0.500401503054, 1.03337712696, 0.207273089197},
};
const Place kTurnedPoleApex[] = {
    {"E x/edge:revolved:{0}|revolved:{1}/chord:0", 2.56066017178, 0, -1.06066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:1", 1.06066017178, 0, -2.56066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:2", -1.06066017178, 0, -2.56066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:3", -2.56066017178, 0, -1.06066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:4", -2.56066017178, 0, 1.06066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:5", -1.06066017178, 0, 2.56066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:6", 1.06066017178, 0, 2.56066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:7", 2.56066017178, 0, 1.06066017178},
    {"E x/revolved:{0}/seam:0", 1.81066017178, -2.12132034356, -0.75},
    {"E x/revolved:{0}/seam:1", 1.06066017178, -2.56066017178, 0},
    {"E x/revolved:{0}/seam:10", -1.81066017178, -1.06066017178, -1.81066017178},
    {"E x/revolved:{0}/seam:11", -1.81066017178, -2.12132034356, -0.75},
    {"E x/revolved:{0}/seam:12", -1.06066017178, -2.56066017178, -1.29893408435e-16},
    {"E x/revolved:{0}/seam:13", -2.56066017178, -1.06066017178, -3.13590428307e-16},
    {"E x/revolved:{0}/seam:14", -1.81066017178, -2.12132034356, 0.75},
    {"E x/revolved:{0}/seam:15", -0.75, -2.56066017178, 0.75},
    {"E x/revolved:{0}/seam:16", -1.81066017178, -1.06066017178, 1.81066017178},
    {"E x/revolved:{0}/seam:17", -0.75, -2.12132034356, 1.81066017178},
    {"E x/revolved:{0}/seam:18", -4.70385642461e-16, -2.56066017178, 1.06066017178},
    {"E x/revolved:{0}/seam:19", -4.70385642461e-16, -1.06066017178, 2.56066017178},
    {"E x/revolved:{0}/seam:2", 0.75, -2.56066017178, -0.75},
    {"E x/revolved:{0}/seam:20", 0.75, -2.12132034356, 1.81066017178},
    {"E x/revolved:{0}/seam:21", 0.75, -2.56066017178, 0.75},
    {"E x/revolved:{0}/seam:22", 1.81066017178, -1.06066017178, 1.81066017178},
    {"E x/revolved:{0}/seam:23", 1.81066017178, -2.12132034356, 0.75},
    {"E x/revolved:{0}/seam:3", 1.81066017178, -1.06066017178, -1.81066017178},
    {"E x/revolved:{0}/seam:4", 2.56066017178, -1.06066017178, 0},
    {"E x/revolved:{0}/seam:5", 0.75, -2.12132034356, -1.81066017178},
    {"E x/revolved:{0}/seam:6", -2.1059882559e-16, -2.56066017178, -1.06066017178},
    {"E x/revolved:{0}/seam:7", 1.56795214154e-16, -1.06066017178, -2.56066017178},
    {"E x/revolved:{0}/seam:8", -0.75, -2.12132034356, -1.81066017178},
    {"E x/revolved:{0}/seam:9", -0.75, -2.56066017178, -0.75},
    {"E x/revolved:{1}/seam:0", 1.06066017178, 2, -1.06066017178},
    {"E x/revolved:{1}/seam:1", 1.5, 2, 0},
    {"E x/revolved:{1}/seam:2", 9.18485099361e-17, 2, -1.5},
    {"E x/revolved:{1}/seam:3", -1.06066017178, 2, -1.06066017178},
    {"E x/revolved:{1}/seam:4", -1.5, 2, -1.83697019872e-16},
    {"E x/revolved:{1}/seam:5", -1.06066017178, 2, 1.06066017178},
    {"E x/revolved:{1}/seam:6", -2.75545529808e-16, 2, 1.5},
    {"E x/revolved:{1}/seam:7", 1.06066017178, 2, 1.06066017178},
    {"F x/revolved:{0}/facet:0.0", 1.20710678119, -2.41421356237, -0.5},
    {"F x/revolved:{0}/facet:0.1", 0.5, -2.41421356237, -1.20710678119},
    {"F x/revolved:{0}/facet:0.2", -0.5, -2.41421356237, -1.20710678119},
    {"F x/revolved:{0}/facet:0.3", -1.20710678119, -2.41421356237, -0.5},
    {"F x/revolved:{0}/facet:0.4", -1.20710678119, -2.41421356237, 0.5},
    {"F x/revolved:{0}/facet:0.5", -0.5, -2.41421356237, 1.20710678119},
    {"F x/revolved:{0}/facet:0.6", 0.5, -2.41421356237, 1.20710678119},
    {"F x/revolved:{0}/facet:0.7", 1.20710678119, -2.41421356237, 0.5},
    {"F x/revolved:{0}/facet:1.0", 2.20710678119, -1, -0.914213562373},
    {"F x/revolved:{0}/facet:1.1", 0.914213562373, -1, -2.20710678119},
    {"F x/revolved:{0}/facet:1.2", -0.914213562373, -1, -2.20710678119},
    {"F x/revolved:{0}/facet:1.3", -2.20710678119, -1, -0.914213562373},
    {"F x/revolved:{0}/facet:1.4", -2.20710678119, -1, 0.914213562373},
    {"F x/revolved:{0}/facet:1.5", -0.914213562373, -1, 2.20710678119},
    {"F x/revolved:{0}/facet:1.6", 0.914213562373, -1, 2.20710678119},
    {"F x/revolved:{0}/facet:1.7", 2.20710678119, -1, 0.914213562373},
    {"F x/revolved:{1}/facet:0", 1.70710678119, 1.33333333333, -0.707106781187},
    {"F x/revolved:{1}/facet:1", 0.707106781187, 1.33333333333, -1.70710678119},
    {"F x/revolved:{1}/facet:2", -0.707106781187, 1.33333333333, -1.70710678119},
    {"F x/revolved:{1}/facet:3", -1.70710678119, 1.33333333333, -0.707106781187},
    {"F x/revolved:{1}/facet:4", -1.70710678119, 1.33333333333, 0.707106781187},
    {"F x/revolved:{1}/facet:5", -0.707106781187, 1.33333333333, 1.70710678119},
    {"F x/revolved:{1}/facet:6", 0.707106781187, 1.33333333333, 1.70710678119},
    {"F x/revolved:{1}/facet:7", 1.70710678119, 1.33333333333, 0.707106781187},
};
const Place kTurnedPoleApexQuarter[] = {
    {"E x/edge:cap_end|cap_start", -2.75545529808e-16, 0.5, 0},
    {"E x/edge:cap_end|revolved:{0}/chord:0", -2.1059882559e-16, -2.56066017178, -1.06066017178},
    {"E x/edge:cap_end|revolved:{0}/chord:1", 1.56795214154e-16, -1.06066017178, -2.56066017178},
    {"E x/edge:cap_end|revolved:{1}", 9.18485099361e-17, 2, -1.5},
    {"E x/edge:cap_start|revolved:{0}/chord:0", 1.06066017178, -2.56066017178, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:1", 2.56066017178, -1.06066017178, 0},
    {"E x/edge:cap_start|revolved:{1}", 1.5, 2, 0},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:0", 2.56066017178, 0, -1.06066017178},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:1", 1.06066017178, 0, -2.56066017178},
    {"E x/revolved:{0}/seam:0", 1.81066017178, -2.12132034356, -0.75},
    {"E x/revolved:{0}/seam:1", 0.75, -2.56066017178, -0.75},
    {"E x/revolved:{0}/seam:2", 1.81066017178, -1.06066017178, -1.81066017178},
    {"E x/revolved:{0}/seam:3", 0.75, -2.12132034356, -1.81066017178},
    {"E x/revolved:{1}/seam:0", 1.06066017178, 2, -1.06066017178},
    {"F x/cap_end", -4.25511779809e-17, 0.0257214887585, -1.1066017178},
    {"F x/cap_start", 1.1066017178, 0.0257214887585, 0},
    {"F x/revolved:{0}/facet:0.0", 1.20710678119, -2.41421356237, -0.5},
    {"F x/revolved:{0}/facet:0.1", 0.5, -2.41421356237, -1.20710678119},
    {"F x/revolved:{0}/facet:1.0", 2.20710678119, -1, -0.914213562373},
    {"F x/revolved:{0}/facet:1.1", 0.914213562373, -1, -2.20710678119},
    {"F x/revolved:{1}/facet:0", 1.70710678119, 1.33333333333, -0.707106781187},
    {"F x/revolved:{1}/facet:1", 0.707106781187, 1.33333333333, -1.70710678119},
};
const Place kTurnedPoleApexAlmost[] = {
    {"E x/edge:cap_end|cap_start", -2.75545529808e-16, 0.5, 0},
    {"E x/edge:cap_end|revolved:{0}/chord:0", 1.0606585563, -2.56066017178, 0.00185120028439},
    {"E x/edge:cap_end|revolved:{0}/chord:1", 2.56065627167, -1.06066017178, 0.00446919283323},
    {"E x/edge:cap_end|revolved:{1}", 1.49999771537, 2, 0.00261799254885},
    {"E x/edge:cap_start|revolved:{0}/chord:0", 1.06066017178, -2.56066017178, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:1", 2.56066017178, -1.06066017178, 0},
    {"E x/edge:cap_start|revolved:{1}", 1.5, 2, 0},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:0", 2.56089154669, 0, -1.06042874639},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:1", 1.06154604514, 0, -2.5604286036},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:2", -1.05931124575, 0, -2.56135400222},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:3", -2.55996517304, 0, -1.06266314179},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:4", -2.56181597011, 0, 1.05819354343},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:5", -1.06378003612, 0, 2.55950125509},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:6", 1.05707563964, 0, 2.56227745027},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:7", 2.55903684986, 0, 1.06489672792},
    {"E x/revolved:{0}/seam:0", 1.81082377855, -2.12132034356, -0.749836357535},
    {"E x/revolved:{0}/seam:1", 0.750163606767, -2.56066017178, -0.749836357535},
    {"E x/revolved:{0}/seam:10", -1.06065976791, -2.56066017178, -0.000925600494636},
    {"E x/revolved:{0}/seam:11", -2.56065919675, -1.06066017178, -0.00223459726749},
    {"E x/revolved:{0}/seam:12", -1.81147744462, -2.12132034356, 0.748255830364},
    {"E x/revolved:{0}/seam:13", -0.750817676708, -2.56066017178, 0.749181430858},
    {"E x/revolved:{0}/seam:14", -1.81263421798, -1.06066017178, 1.80868397106},
    {"E x/revolved:{0}/seam:15", -0.752206077229, -2.12132034356, 1.80984069393},
    {"E x/revolved:{0}/seam:16", -0.00138840052168, -2.56066017178, 1.06065926307},
    {"E x/revolved:{0}/seam:17", -0.00335189536944, -1.06066017178, 2.56065797797},
    {"E x/revolved:{0}/seam:18", 0.747465353017, -2.12132034356, 1.81180376037},
    {"E x/revolved:{0}/seam:19", 0.748853753539, -2.56066017178, 0.751144497291},
    {"E x/revolved:{0}/seam:2", 1.81105515346, -1.06066017178, -1.81026510392},
    {"E x/revolved:{0}/seam:20", 1.80789288803, -1.06066017178, 1.81342323266},
    {"E x/revolved:{0}/seam:21", 1.80951230984, -2.12132034356, 0.752995697576},
    {"E x/revolved:{0}/seam:3", 0.750626407059, -2.12132034356, -1.81049642835},
    {"E x/revolved:{0}/seam:4", 0.000462800291373, -2.56066017178, -1.06066007081},
    {"E x/revolved:{0}/seam:5", 0.0011172987401, -1.06066017178, -2.56065992802},
    {"E x/revolved:{0}/seam:6", -0.749046165253, -2.12132034356, -1.81115078399},
    {"E x/revolved:{0}/seam:7", -0.749508965545, -2.56066017178, -0.750490713179},
    {"E x/revolved:{0}/seam:8", -1.80947470974, -1.06066017178, -1.81184485819},
    {"E x/revolved:{0}/seam:9", -1.81016873346, -2.12132034356, -0.751416313674},
    {"E x/revolved:{1}/seam:0", 1.06089154669, 2, -1.06042874639},
    {"E x/revolved:{1}/seam:1", 0.00065449844873, 2, -1.49999985721},
    {"E x/revolved:{1}/seam:2", -1.05996574419, 2, -1.06135414501},
    {"E x/revolved:{1}/seam:3", -1.49999942884, 2, -0.00130899677285},
    {"E x/revolved:{1}/seam:4", -1.06181654127, 2, 1.0595025402},
    {"E x/revolved:{1}/seam:5", -0.00196349484776, 2, 1.4999987149},
    {"E x/revolved:{1}/seam:6", 1.05903913449, 2, 1.06227873537},
    {"F x/cap_end", 1.10660003235, 0.0257214887585, 0.00193138336783},
    {"F x/cap_start", 1.1066017178, 0.0257214887585, 0},
    {"F x/revolved:{0}/facet:0.0", 1.20721585236, -2.41421356237, -0.499890905023},
    {"F x/revolved:{0}/facet:0.1", 0.500417604706, -2.41421356237, -1.2069976189},
    {"F x/revolved:{0}/facet:0.2", -0.499364110169, -2.41421356237, -1.20743385599},
    {"F x/revolved:{0}/facet:0.3", -1.20677915564, -2.41421356237, -0.500944209116},
    {"F x/revolved:{0}/facet:0.4", -1.20765162975, -2.41421356237, 0.498837220243},
    {"F x/revolved:{0}/facet:0.5", -0.501470718153, -2.41421356237, 1.20656046262},
    {"F x/revolved:{0}/facet:0.6", 0.498310235345, -2.41421356237, 1.20786917358},
    {"F x/revolved:{0}/facet:0.7", 1.20634153989, -2.41421356237, 0.501997131717},
    {"F x/revolved:{0}/facet:1.0", 2.20730620989, -1, -0.914014090159},
    {"F x/revolved:{0}/facet:1.1", 0.914977122145, -1, -2.2069071859},
    {"F x/revolved:{0}/facet:1.2", -0.913050884158, -1, -2.20770481364},
    {"F x/revolved:{0}/facet:1.3", -2.20650774174, -1, -0.915939979932},
    {"F x/revolved:{0}/facet:1.4", -2.20810299707, -1, 0.912087504325},
    {"F x/revolved:{0}/facet:1.5", -0.916902663337, -1, 2.2061078775},
    {"F x/revolved:{0}/facet:1.6", 0.911123950843, -1, 2.20850076011},
    {"F x/revolved:{0}/facet:1.7", 2.20570759325, -1, 0.917865172177},
    {"F x/revolved:{1}/facet:0", 1.70726103113, 1.33333333333, -0.706952497591},
    {"F x/revolved:{1}/facet:1", 0.707697363425, 1.33333333333, -1.7069524024},
    {"F x/revolved:{1}/facet:2", -0.706207497163, 1.33333333333, -1.70756933482},
    {"F x/revolved:{1}/facet:3", -1.70664344869, 1.33333333333, -0.708442094524},
    {"F x/revolved:{1}/facet:4", -1.70787731341, 1.33333333333, 0.705462362284},
    {"F x/revolved:{1}/facet:5", -0.709186690745, 1.33333333333, 1.70633417006},
    {"F x/revolved:{1}/facet:6", 0.704717093094, 1.33333333333, 1.70818496684},
    {"F x/revolved:{1}/facet:7", 1.70602456657, 1.33333333333, 0.709931151947},
};
const Place kTurnedWasher[] = {
    {"E x/edge:revolved:{0}|revolved:{1}/chord:0", 3.41421356237, 0, -1.41421356237},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:1", 1.41421356237, 0, -3.41421356237},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:2", -1.41421356237, 0, -3.41421356237},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:3", -3.41421356237, 0, -1.41421356237},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:4", -3.41421356237, 0, 1.41421356237},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:5", -1.41421356237, 0, 3.41421356237},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:6", 1.41421356237, 0, 3.41421356237},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:7", 3.41421356237, 0, 1.41421356237},
    {"E x/edge:revolved:{0}|revolved:{3}/chord:0", 1.70710678119, 0, -0.707106781187},
    {"E x/edge:revolved:{0}|revolved:{3}/chord:1", 0.707106781187, 0, -1.70710678119},
    {"E x/edge:revolved:{0}|revolved:{3}/chord:2", -0.707106781187, 0, -1.70710678119},
    {"E x/edge:revolved:{0}|revolved:{3}/chord:3", -1.70710678119, 0, -0.707106781187},
    {"E x/edge:revolved:{0}|revolved:{3}/chord:4", -1.70710678119, 0, 0.707106781187},
    {"E x/edge:revolved:{0}|revolved:{3}/chord:5", -0.707106781187, 0, 1.70710678119},
    {"E x/edge:revolved:{0}|revolved:{3}/chord:6", 0.707106781187, 0, 1.70710678119},
    {"E x/edge:revolved:{0}|revolved:{3}/chord:7", 1.70710678119, 0, 0.707106781187},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:0", 3.41421356237, 3, -1.41421356237},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:1", 1.41421356237, 3, -3.41421356237},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:2", -1.41421356237, 3, -3.41421356237},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:3", -3.41421356237, 3, -1.41421356237},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:4", -3.41421356237, 3, 1.41421356237},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:5", -1.41421356237, 3, 3.41421356237},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:6", 1.41421356237, 3, 3.41421356237},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:7", 3.41421356237, 3, 1.41421356237},
    {"E x/edge:revolved:{2}|revolved:{3}/chord:0", 1.70710678119, 3, -0.707106781187},
    {"E x/edge:revolved:{2}|revolved:{3}/chord:1", 0.707106781187, 3, -1.70710678119},
    {"E x/edge:revolved:{2}|revolved:{3}/chord:2", -0.707106781187, 3, -1.70710678119},
    {"E x/edge:revolved:{2}|revolved:{3}/chord:3", -1.70710678119, 3, -0.707106781187},
    {"E x/edge:revolved:{2}|revolved:{3}/chord:4", -1.70710678119, 3, 0.707106781187},
    {"E x/edge:revolved:{2}|revolved:{3}/chord:5", -0.707106781187, 3, 1.70710678119},
    {"E x/edge:revolved:{2}|revolved:{3}/chord:6", 0.707106781187, 3, 1.70710678119},
    {"E x/edge:revolved:{2}|revolved:{3}/chord:7", 1.70710678119, 3, 0.707106781187},
    {"E x/revolved:{1}/seam:0", 2.82842712475, 1.5, -2.82842712475},
    {"E x/revolved:{1}/seam:1", 4, 1.5, 0},
    {"E x/revolved:{1}/seam:2", 2.44929359829e-16, 1.5, -4},
    {"E x/revolved:{1}/seam:3", -2.82842712475, 1.5, -2.82842712475},
    {"E x/revolved:{1}/seam:4", -4, 1.5, -4.89858719659e-16},
    {"E x/revolved:{1}/seam:5", -2.82842712475, 1.5, 2.82842712475},
    {"E x/revolved:{1}/seam:6", -7.34788079488e-16, 1.5, 4},
    {"E x/revolved:{1}/seam:7", 2.82842712475, 1.5, 2.82842712475},
    {"E x/revolved:{3}/seam:0", 1.41421356237, 1.5, -1.41421356237},
    {"E x/revolved:{3}/seam:1", 2, 1.5, 0},
    {"E x/revolved:{3}/seam:2", 1.22464679915e-16, 1.5, -2},
    {"E x/revolved:{3}/seam:3", -1.41421356237, 1.5, -1.41421356237},
    {"E x/revolved:{3}/seam:4", -2, 1.5, -2.44929359829e-16},
    {"E x/revolved:{3}/seam:5", -1.41421356237, 1.5, 1.41421356237},
    {"E x/revolved:{3}/seam:6", -3.67394039744e-16, 1.5, 2},
    {"E x/revolved:{3}/seam:7", 1.41421356237, 1.5, 1.41421356237},
    {"F x/revolved:{0}", -1.57009245868e-16, 0, -5.68261699153e-17},
    {"F x/revolved:{1}/facet:0", 3.41421356237, 1.5, -1.41421356237},
    {"F x/revolved:{1}/facet:1", 1.41421356237, 1.5, -3.41421356237},
    {"F x/revolved:{1}/facet:2", -1.41421356237, 1.5, -3.41421356237},
    {"F x/revolved:{1}/facet:3", -3.41421356237, 1.5, -1.41421356237},
    {"F x/revolved:{1}/facet:4", -3.41421356237, 1.5, 1.41421356237},
    {"F x/revolved:{1}/facet:5", -1.41421356237, 1.5, 3.41421356237},
    {"F x/revolved:{1}/facet:6", 1.41421356237, 1.5, 3.41421356237},
    {"F x/revolved:{1}/facet:7", 3.41421356237, 1.5, 1.41421356237},
    {"F x/revolved:{2}", -1.57009245868e-16, 3, 1.96261557335e-17},
    {"F x/revolved:{3}/facet:0", 1.70710678119, 1.5, -0.707106781187},
    {"F x/revolved:{3}/facet:1", 0.707106781187, 1.5, -1.70710678119},
    {"F x/revolved:{3}/facet:2", -0.707106781187, 1.5, -1.70710678119},
    {"F x/revolved:{3}/facet:3", -1.70710678119, 1.5, -0.707106781187},
    {"F x/revolved:{3}/facet:4", -1.70710678119, 1.5, 0.707106781187},
    {"F x/revolved:{3}/facet:5", -0.707106781187, 1.5, 1.70710678119},
    {"F x/revolved:{3}/facet:6", 0.707106781187, 1.5, 1.70710678119},
    {"F x/revolved:{3}/facet:7", 1.70710678119, 1.5, 0.707106781187},
};
const Place kTurnedWasherQuarter[] = {
    {"E x/edge:cap_end|revolved:{0}", 1.83697019872e-16, 0, -3},
    {"E x/edge:cap_end|revolved:{1}", 2.44929359829e-16, 1.5, -4},
    {"E x/edge:cap_end|revolved:{2}", 1.83697019872e-16, 3, -3},
    {"E x/edge:cap_end|revolved:{3}", 1.22464679915e-16, 1.5, -2},
    {"E x/edge:cap_start|revolved:{0}", 3, 0, 0},
    {"E x/edge:cap_start|revolved:{1}", 4, 1.5, 0},
    {"E x/edge:cap_start|revolved:{2}", 3, 3, 0},
    {"E x/edge:cap_start|revolved:{3}", 2, 1.5, 0},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:0", 3.41421356237, 0, -1.41421356237},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:1", 1.41421356237, 0, -3.41421356237},
    {"E x/edge:revolved:{0}|revolved:{3}/chord:0", 1.70710678119, 0, -0.707106781187},
    {"E x/edge:revolved:{0}|revolved:{3}/chord:1", 0.707106781187, 0, -1.70710678119},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:0", 3.41421356237, 3, -1.41421356237},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:1", 1.41421356237, 3, -3.41421356237},
    {"E x/edge:revolved:{2}|revolved:{3}/chord:0", 1.70710678119, 3, -0.707106781187},
    {"E x/edge:revolved:{2}|revolved:{3}/chord:1", 0.707106781187, 3, -1.70710678119},
    {"E x/revolved:{0}/seam:0", 2.12132034356, 0, -2.12132034356},
    {"E x/revolved:{1}/seam:0", 2.82842712475, 1.5, -2.82842712475},
    {"E x/revolved:{2}/seam:0", 2.12132034356, 3, -2.12132034356},
    {"E x/revolved:{3}/seam:0", 1.41421356237, 1.5, -1.41421356237},
    {"F x/cap_end", 1.83697019872e-16, 1.5, -3},
    {"F x/cap_start", 3, 1.5, 0},
    {"F x/revolved:{0}/facet:0", 2.6554994374, 0, -1.09994388185},
    {"F x/revolved:{0}/facet:1", 1.09994388185, 0, -2.6554994374},
    {"F x/revolved:{1}/facet:0", 3.41421356237, 1.5, -1.41421356237},
    {"F x/revolved:{1}/facet:1", 1.41421356237, 1.5, -3.41421356237},
    {"F x/revolved:{2}/facet:0", 2.6554994374, 3, -1.09994388185},
    {"F x/revolved:{2}/facet:1", 1.09994388185, 3, -2.6554994374},
    {"F x/revolved:{3}/facet:0", 1.70710678119, 1.5, -0.707106781187},
    {"F x/revolved:{3}/facet:1", 0.707106781187, 1.5, -1.70710678119},
};
const Place kTurnedHoled[] = {
    {"E x/edge:revolved:{0}/piece:0|revolved:{0}/piece:1/chord:0", -5.51091059616e-16, 0, 3},
    {"E x/edge:revolved:{0}/piece:0|revolved:{0}/piece:1/chord:1", 1.83697019872e-16, 0, -3},
    {"E x/edge:revolved:{0}/piece:0|revolved:{1}/chord:0", 4.26776695297, 0, 1.76776695297},
    {"E x/edge:revolved:{0}/piece:0|revolved:{1}/chord:1", 1.76776695297, 0, 4.26776695297},
    {"E x/edge:revolved:{0}/piece:0|revolved:{1}/chord:2", 1.76776695297, 0, -4.26776695297},
    {"E x/edge:revolved:{0}/piece:0|revolved:{1}/chord:3", 4.26776695297, 0, -1.76776695297},
    {"E x/edge:revolved:{0}/piece:0|revolved:{3}/chord:0", 0.353553390593, 0, 0.853553390593},
    {"E x/edge:revolved:{0}/piece:0|revolved:{3}/chord:1", 0.853553390593, 0, 0.353553390593},
    {"E x/edge:revolved:{0}/piece:0|revolved:{3}/chord:2", 0.853553390593, 0, -0.353553390593},
    {"E x/edge:revolved:{0}/piece:0|revolved:{3}/chord:3", 0.353553390593, 0, -0.853553390593},
    {"E x/edge:revolved:{0}/piece:1|revolved:{1}/chord:0", -1.76776695297, 0, 4.26776695297},
    {"E x/edge:revolved:{0}/piece:1|revolved:{1}/chord:1", -4.26776695297, 0, 1.76776695297},
    {"E x/edge:revolved:{0}/piece:1|revolved:{1}/chord:2", -4.26776695297, 0, -1.76776695297},
    {"E x/edge:revolved:{0}/piece:1|revolved:{1}/chord:3", -1.76776695297, 0, -4.26776695297},
    {"E x/edge:revolved:{0}/piece:1|revolved:{3}/chord:0", -0.353553390593, 0, -0.853553390593},
    {"E x/edge:revolved:{0}/piece:1|revolved:{3}/chord:1", -0.853553390593, 0, -0.353553390593},
    {"E x/edge:revolved:{0}/piece:1|revolved:{3}/chord:2", -0.853553390593, 0, 0.353553390593},
    {"E x/edge:revolved:{0}/piece:1|revolved:{3}/chord:3", -0.353553390593, 0, 0.853553390593},
    {"E x/edge:revolved:{1}|revolved:{2}/piece:0/chord:0", 4.26776695297, 4, -1.76776695297},
    {"E x/edge:revolved:{1}|revolved:{2}/piece:0/chord:1", -1.76776695297, 4, 4.26776695297},
    {"E x/edge:revolved:{1}|revolved:{2}/piece:0/chord:2", 1.76776695297, 4, 4.26776695297},
    {"E x/edge:revolved:{1}|revolved:{2}/piece:0/chord:3", 4.26776695297, 4, 1.76776695297},
    {"E x/edge:revolved:{1}|revolved:{2}/piece:1/chord:0", 1.76776695297, 4, -4.26776695297},
    {"E x/edge:revolved:{1}|revolved:{2}/piece:1/chord:1", -1.76776695297, 4, -4.26776695297},
    {"E x/edge:revolved:{1}|revolved:{2}/piece:1/chord:2", -4.26776695297, 4, -1.76776695297},
    {"E x/edge:revolved:{1}|revolved:{2}/piece:1/chord:3", -4.26776695297, 4, 1.76776695297},
    {"E x/edge:revolved:{2}/piece:0|revolved:{2}/piece:1/chord:0", 2.12132034356, 4, -2.12132034356},
    {"E x/edge:revolved:{2}/piece:0|revolved:{2}/piece:1/chord:1", -2.12132034356, 4, 2.12132034356},
    {"E x/edge:revolved:{2}/piece:0|revolved:{3}/chord:0", 0.853553390593, 4, -0.353553390593},
    {"E x/edge:revolved:{2}/piece:0|revolved:{3}/chord:1", 0.853553390593, 4, 0.353553390593},
    {"E x/edge:revolved:{2}/piece:0|revolved:{3}/chord:2", 0.353553390593, 4, 0.853553390593},
    {"E x/edge:revolved:{2}/piece:0|revolved:{3}/chord:3", -0.353553390593, 4, 0.853553390593},
    {"E x/edge:revolved:{2}/piece:1|revolved:{3}/chord:0", -0.853553390593, 4, 0.353553390593},
    {"E x/edge:revolved:{2}/piece:1|revolved:{3}/chord:1", -0.853553390593, 4, -0.353553390593},
    {"E x/edge:revolved:{2}/piece:1|revolved:{3}/chord:2", -0.353553390593, 4, -0.853553390593},
    {"E x/edge:revolved:{2}/piece:1|revolved:{3}/chord:3", 0.353553390593, 4, -0.853553390593},
    {"E x/revolved:{1}/seam:0", 3.53553390593, 2, -3.53553390593},
    {"E x/revolved:{1}/seam:1", 5, 2, 0},
    {"E x/revolved:{1}/seam:2", 3.06161699787e-16, 2, -5},
    {"E x/revolved:{1}/seam:3", -3.53553390593, 2, -3.53553390593},
    {"E x/revolved:{1}/seam:4", -5, 2, -6.12323399574e-16},
    {"E x/revolved:{1}/seam:5", -3.53553390593, 2, 3.53553390593},
    {"E x/revolved:{1}/seam:6", -9.18485099361e-16, 2, 5},
    {"E x/revolved:{1}/seam:7", 3.53553390593, 2, 3.53553390593},
    {"E x/revolved:{3}/seam:0", 0.707106781187, 2, -0.707106781187},
    {"E x/revolved:{3}/seam:1", 1, 2, 0},
    {"E x/revolved:{3}/seam:2", 6.12323399574e-17, 2, -1},
    {"E x/revolved:{3}/seam:3", -0.707106781187, 2, -0.707106781187},
    {"E x/revolved:{3}/seam:4", -1, 2, -1.22464679915e-16},
    {"E x/revolved:{3}/seam:5", -0.707106781187, 2, 0.707106781187},
    {"E x/revolved:{3}/seam:6", -1.83697019872e-16, 2, 1},
    {"E x/revolved:{3}/seam:7", 0.707106781187, 2, 0.707106781187},
    {"E x~hole0/revolved:{4}/seam:0", 3.85355339059, 2.35355339059, 0},
    {"E x~hole0/revolved:{4}/seam:1", 3.16421356237, 2.70710678119, -1.31066017178},
    {"E x~hole0/revolved:{4}/seam:10", -3.16421356237, 2.70710678119, -1.31066017178},
    {"E x~hole0/revolved:{4}/seam:100", -4.86144355399e-16, 1.14644660941, 2.64644660941},
    {"E x~hole0/revolved:{4}/seam:101", 1.06066017178, 1, 2.56066017178},
    {"E x~hole0/revolved:{4}/seam:102", 1.87132034356, 1.14644660941, 1.87132034356},
    {"E x~hole0/revolved:{4}/seam:103", 2.56066017178, 1, 1.06066017178},
    {"E x~hole0/revolved:{4}/seam:104", 3.35355339059, 1.14644660941, 0},
    {"E x~hole0/revolved:{4}/seam:105", 3.16421356237, 1.29289321881, -1.31066017178},
    {"E x~hole0/revolved:{4}/seam:106", 2.37132034356, 1.14644660941, -2.37132034356},
    {"E x~hole0/revolved:{4}/seam:107", 1.31066017178, 1.29289321881, -3.16421356237},
    {"E x~hole0/revolved:{4}/seam:108", 2.05345921278e-16, 1.14644660941, -3.35355339059},
    {"E x~hole0/revolved:{4}/seam:109", -1.31066017178, 1.29289321881, -3.16421356237},
    {"E x~hole0/revolved:{4}/seam:11", -3.85355339059, 2.35355339059, -4.71924182513e-16},
    {"E x~hole0/revolved:{4}/seam:110", -2.37132034356, 1.14644660941, -2.37132034356},
    {"E x~hole0/revolved:{4}/seam:111", -3.16421356237, 1.29289321881, -1.31066017178},
    {"E x~hole0/revolved:{4}/seam:112", -3.35355339059, 1.14644660941, -4.10691842556e-16},
    {"E x~hole0/revolved:{4}/seam:113", -3.16421356237, 1.29289321881, 1.31066017178},
    {"E x~hole0/revolved:{4}/seam:114", -2.37132034356, 1.14644660941, 2.37132034356},
    {"E x~hole0/revolved:{4}/seam:115", -1.31066017178, 1.29289321881, 3.16421356237},
    {"E x~hole0/revolved:{4}/seam:116", -6.16037763834e-16, 1.14644660941, 3.35355339059},
    {"E x~hole0/revolved:{4}/seam:117", 1.31066017178, 1.29289321881, 3.16421356237},
    {"E x~hole0/revolved:{4}/seam:118", 2.37132034356, 1.14644660941, 2.37132034356},
    {"E x~hole0/revolved:{4}/seam:119", 3.16421356237, 1.29289321881, 1.31066017178},
    {"E x~hole0/revolved:{4}/seam:12", -3.41421356237, 2, -1.41421356237},
    {"E x~hole0/revolved:{4}/seam:120", 3.85355339059, 1.64644660941, 0},
    {"E x~hole0/revolved:{4}/seam:121", 2.72487373415, 1.64644660941, -2.72487373415},
    {"E x~hole0/revolved:{4}/seam:122", 2.35962091257e-16, 1.64644660941, -3.85355339059},
    {"E x~hole0/revolved:{4}/seam:123", -2.72487373415, 1.64644660941, -2.72487373415},
    {"E x~hole0/revolved:{4}/seam:124", -3.85355339059, 1.64644660941, -4.71924182513e-16},
    {"E x~hole0/revolved:{4}/seam:125", -2.72487373415, 1.64644660941, 2.72487373415},
    {"E x~hole0/revolved:{4}/seam:126", -7.0788627377e-16, 1.64644660941, 3.85355339059},
    {"E x~hole0/revolved:{4}/seam:127", 2.72487373415, 1.64644660941, 2.72487373415},
    {"E x~hole0/revolved:{4}/seam:13", -3.16421356237, 2.70710678119, 1.31066017178},
    {"E x~hole0/revolved:{4}/seam:14", -2.72487373415, 2.35355339059, 2.72487373415},
    {"E x~hole0/revolved:{4}/seam:15", -3.41421356237, 2, 1.41421356237},
    {"E x~hole0/revolved:{4}/seam:16", -1.31066017178, 2.70710678119, 3.16421356237},
    {"E x~hole0/revolved:{4}/seam:17", -7.0788627377e-16, 2.35355339059, 3.85355339059},
    {"E x~hole0/revolved:{4}/seam:18", -1.41421356237, 2, 3.41421356237},
    {"E x~hole0/revolved:{4}/seam:19", 1.31066017178, 2.70710678119, 3.16421356237},
    {"E x~hole0/revolved:{4}/seam:2", 2.72487373415, 2.35355339059, -2.72487373415},
    {"E x~hole0/revolved:{4}/seam:20", 2.72487373415, 2.35355339059, 2.72487373415},
    {"E x~hole0/revolved:{4}/seam:21", 1.41421356237, 2, 3.41421356237},
    {"E x~hole0/revolved:{4}/seam:22", 3.16421356237, 2.70710678119, 1.31066017178},
    {"E x~hole0/revolved:{4}/seam:23", 3.41421356237, 2, 1.41421356237},
    {"E x~hole0/revolved:{4}/seam:24", 3.35355339059, 2.85355339059, 0},
    {"E x~hole0/revolved:{4}/seam:25", 2.56066017178, 3, -1.06066017178},
    {"E x~hole0/revolved:{4}/seam:26", 2.37132034356, 2.85355339059, -2.37132034356},
    {"E x~hole0/revolved:{4}/seam:27", 1.06066017178, 3, -2.56066017178},
    {"E x~hole0/revolved:{4}/seam:28", 2.05345921278e-16, 2.85355339059, -3.35355339059},
    {"E x~hole0/revolved:{4}/seam:29", -1.06066017178, 3, -2.56066017178},
    {"E x~hole0/revolved:{4}/seam:3", 3.41421356237, 2, -1.41421356237},
    {"E x~hole0/revolved:{4}/seam:30", -2.37132034356, 2.85355339059, -2.37132034356},
    {"E x~hole0/revolved:{4}/seam:31", -2.56066017178, 3, -1.06066017178},
    {"E x~hole0/revolved:{4}/seam:32", -3.35355339059, 2.85355339059, -4.10691842556e-16},
    {"E x~hole0/revolved:{4}/seam:33", -2.56066017178, 3, 1.06066017178},
    {"E x~hole0/revolved:{4}/seam:34", -2.37132034356, 2.85355339059, 2.37132034356},
    {"E x~hole0/revolved:{4}/seam:35", -1.06066017178, 3, 2.56066017178},
    {"E x~hole0/revolved:{4}/seam:36", -6.16037763834e-16, 2.85355339059, 3.35355339059},
    {"E x~hole0/revolved:{4}/seam:37", 1.06066017178, 3, 2.56066017178},
    {"E x~hole0/revolved:{4}/seam:38", 2.37132034356, 2.85355339059, 2.37132034356},
    {"E x~hole0/revolved:{4}/seam:39", 2.56066017178, 3, 1.06066017178},
    {"E x~hole0/revolved:{4}/seam:4", 1.31066017178, 2.70710678119, -3.16421356237},
    {"E x~hole0/revolved:{4}/seam:40", 2.64644660941, 2.85355339059, 0},
    {"E x~hole0/revolved:{4}/seam:41", 1.95710678119, 2.70710678119, -0.81066017178},
    {"E x~hole0/revolved:{4}/seam:42", 1.87132034356, 2.85355339059, -1.87132034356},
    {"E x~hole0/revolved:{4}/seam:43", 0.81066017178, 2.70710678119, -1.95710678119},
    {"E x~hole0/revolved:{4}/seam:44", 1.62048118466e-16, 2.85355339059, -2.64644660941},
    {"E x~hole0/revolved:{4}/seam:45", -0.81066017178, 2.70710678119, -1.95710678119},
    {"E x~hole0/revolved:{4}/seam:46", -1.87132034356, 2.85355339059, -1.87132034356},
    {"E x~hole0/revolved:{4}/seam:47", -1.95710678119, 2.70710678119, -0.81066017178},
    {"E x~hole0/revolved:{4}/seam:48", -2.64644660941, 2.85355339059, -3.24096236932e-16},
    {"E x~hole0/revolved:{4}/seam:49", -1.95710678119, 2.70710678119, 0.81066017178},
    {"E x~hole0/revolved:{4}/seam:5", 2.35962091257e-16, 2.35355339059, -3.85355339059},
    {"E x~hole0/revolved:{4}/seam:50", -1.87132034356, 2.85355339059, 1.87132034356},
    {"E x~hole0/revolved:{4}/seam:51", -0.81066017178, 2.70710678119, 1.95710678119},
    {"E x~hole0/revolved:{4}/seam:52", -4.86144355399e-16, 2.85355339059, 2.64644660941},
    {"E x~hole0/revolved:{4}/seam:53", 0.81066017178, 2.70710678119, 1.95710678119},
    {"E x~hole0/revolved:{4}/seam:54", 1.87132034356, 2.85355339059, 1.87132034356},
    {"E x~hole0/revolved:{4}/seam:55", 1.95710678119, 2.70710678119, 0.81066017178},
    {"E x~hole0/revolved:{4}/seam:56", 2.14644660941, 2.35355339059, 0},
    {"E x~hole0/revolved:{4}/seam:57", 1.70710678119, 2, -0.707106781187},
    {"E x~hole0/revolved:{4}/seam:58", 1.51776695297, 2.35355339059, -1.51776695297},
    {"E x~hole0/revolved:{4}/seam:59", 0.707106781187, 2, -1.70710678119},
    {"E x~hole0/revolved:{4}/seam:6", 1.41421356237, 2, -3.41421356237},
    {"E x~hole0/revolved:{4}/seam:60", 1.31431948488e-16, 2.35355339059, -2.14644660941},
    {"E x~hole0/revolved:{4}/seam:61", -0.707106781187, 2, -1.70710678119},
    {"E x~hole0/revolved:{4}/seam:62", -1.51776695297, 2.35355339059, -1.51776695297},
    {"E x~hole0/revolved:{4}/seam:63", -1.70710678119, 2, -0.707106781187},
    {"E x~hole0/revolved:{4}/seam:64", -2.14644660941, 2.35355339059, -2.62863896975e-16},
    {"E x~hole0/revolved:{4}/seam:65", -1.70710678119, 2, 0.707106781187},
    {"E x~hole0/revolved:{4}/seam:66", -1.51776695297, 2.35355339059, 1.51776695297},
    {"E x~hole0/revolved:{4}/seam:67", -0.707106781187, 2, 1.70710678119},
    {"E x~hole0/revolved:{4}/seam:68", -3.94295845463e-16, 2.35355339059, 2.14644660941},
    {"E x~hole0/revolved:{4}/seam:69", 0.707106781187, 2, 1.70710678119},
    {"E x~hole0/revolved:{4}/seam:7", -1.31066017178, 2.70710678119, -3.16421356237},
    {"E x~hole0/revolved:{4}/seam:70", 1.51776695297, 2.35355339059, 1.51776695297},
    {"E x~hole0/revolved:{4}/seam:71", 1.70710678119, 2, 0.707106781187},
    {"E x~hole0/revolved:{4}/seam:72", 2.14644660941, 1.64644660941, 0},
    {"E x~hole0/revolved:{4}/seam:73", 1.95710678119, 1.29289321881, -0.81066017178},
    {"E x~hole0/revolved:{4}/seam:74", 1.51776695297, 1.64644660941, -1.51776695297},
    {"E x~hole0/revolved:{4}/seam:75", 0.81066017178, 1.29289321881, -1.95710678119},
    {"E x~hole0/revolved:{4}/seam:76", 1.31431948488e-16, 1.64644660941, -2.14644660941},
    {"E x~hole0/revolved:{4}/seam:77", -0.81066017178, 1.29289321881, -1.95710678119},
    {"E x~hole0/revolved:{4}/seam:78", -1.51776695297, 1.64644660941, -1.51776695297},
    {"E x~hole0/revolved:{4}/seam:79", -1.95710678119, 1.29289321881, -0.81066017178},
    {"E x~hole0/revolved:{4}/seam:8", -2.72487373415, 2.35355339059, -2.72487373415},
    {"E x~hole0/revolved:{4}/seam:80", -2.14644660941, 1.64644660941, -2.62863896975e-16},
    {"E x~hole0/revolved:{4}/seam:81", -1.95710678119, 1.29289321881, 0.81066017178},
    {"E x~hole0/revolved:{4}/seam:82", -1.51776695297, 1.64644660941, 1.51776695297},
    {"E x~hole0/revolved:{4}/seam:83", -0.81066017178, 1.29289321881, 1.95710678119},
    {"E x~hole0/revolved:{4}/seam:84", -3.94295845463e-16, 1.64644660941, 2.14644660941},
    {"E x~hole0/revolved:{4}/seam:85", 0.81066017178, 1.29289321881, 1.95710678119},
    {"E x~hole0/revolved:{4}/seam:86", 1.51776695297, 1.64644660941, 1.51776695297},
    {"E x~hole0/revolved:{4}/seam:87", 1.95710678119, 1.29289321881, 0.81066017178},
    {"E x~hole0/revolved:{4}/seam:88", 2.64644660941, 1.14644660941, 0},
    {"E x~hole0/revolved:{4}/seam:89", 2.56066017178, 1, -1.06066017178},
    {"E x~hole0/revolved:{4}/seam:9", -1.41421356237, 2, -3.41421356237},
    {"E x~hole0/revolved:{4}/seam:90", 1.87132034356, 1.14644660941, -1.87132034356},
    {"E x~hole0/revolved:{4}/seam:91", 1.06066017178, 1, -2.56066017178},
    {"E x~hole0/revolved:{4}/seam:92", 1.62048118466e-16, 1.14644660941, -2.64644660941},
    {"E x~hole0/revolved:{4}/seam:93", -1.06066017178, 1, -2.56066017178},
    {"E x~hole0/revolved:{4}/seam:94", -1.87132034356, 1.14644660941, -1.87132034356},
    {"E x~hole0/revolved:{4}/seam:95", -2.56066017178, 1, -1.06066017178},
    {"E x~hole0/revolved:{4}/seam:96", -2.64644660941, 1.14644660941, -3.24096236932e-16},
    {"E x~hole0/revolved:{4}/seam:97", -2.56066017178, 1, 1.06066017178},
    {"E x~hole0/revolved:{4}/seam:98", -1.87132034356, 1.14644660941, 1.87132034356},
    {"E x~hole0/revolved:{4}/seam:99", -1.06066017178, 1, 2.56066017178},
    {"F x/revolved:{0}/piece:0", 2.07890612315, 0, 3.14018491737e-16},
    {"F x/revolved:{0}/piece:1", -1.80387818761, 0, 0.192398277913},
    {"F x/revolved:{1}/facet:0", 4.26776695297, 2, -1.76776695297},
    {"F x/revolved:{1}/facet:1", 1.76776695297, 2, -4.26776695297},
    {"F x/revolved:{1}/facet:2", -1.76776695297, 2, -4.26776695297},
    {"F x/revolved:{1}/facet:3", -4.26776695297, 2, -1.76776695297},
    {"F x/revolved:{1}/facet:4", -4.26776695297, 2, 1.76776695297},
    {"F x/revolved:{1}/facet:5", -1.76776695297, 2, 4.26776695297},
    {"F x/revolved:{1}/facet:6", 1.76776695297, 2, 4.26776695297},
    {"F x/revolved:{1}/facet:7", 4.26776695297, 2, 1.76776695297},
    {"F x/revolved:{2}/piece:0", 1.47000861713, 4, 1.47000861713},
    {"F x/revolved:{2}/piece:1", -1.13948837189, 4, -1.4115806259},
    {"F x/revolved:{3}/facet:0", 0.853553390593, 2, -0.353553390593},
    {"F x/revolved:{3}/facet:1", 0.353553390593, 2, -0.853553390593},
    {"F x/revolved:{3}/facet:2", -0.353553390593, 2, -0.853553390593},
    {"F x/revolved:{3}/facet:3", -0.853553390593, 2, -0.353553390593},
    {"F x/revolved:{3}/facet:4", -0.853553390593, 2, 0.353553390593},
    {"F x/revolved:{3}/facet:5", -0.353553390593, 2, 0.853553390593},
    {"F x/revolved:{3}/facet:6", 0.353553390593, 2, 0.853553390593},
    {"F x/revolved:{3}/facet:7", 0.853553390593, 2, 0.353553390593},
    {"F x~hole0/revolved:{4}/facet:0.0", 3.29079702092, 2.34907469348, -1.36309275708},
    {"F x~hole0/revolved:{4}/facet:0.1", 1.36309275708, 2.34907469348, -3.29079702092},
    {"F x~hole0/revolved:{4}/facet:0.2", -1.36309275708, 2.34907469348, -3.29079702092},
    {"F x~hole0/revolved:{4}/facet:0.3", -3.29079702092, 2.34907469348, -1.36309275708},
    {"F x~hole0/revolved:{4}/facet:0.4", -3.29079702092, 2.34907469348, 1.36309275708},
    {"F x~hole0/revolved:{4}/facet:0.5", -1.36309275708, 2.34907469348, 3.29079702092},
    {"F x~hole0/revolved:{4}/facet:0.6", 1.36309275708, 2.34907469348, 3.29079702092},
    {"F x~hole0/revolved:{4}/facet:0.7", 3.29079702092, 2.34907469348, 1.36309275708},
    {"F x~hole0/revolved:{4}/facet:1.0", 2.87304195382, 2.84840693952, -1.19005294254},
    {"F x~hole0/revolved:{4}/facet:1.1", 1.19005294254, 2.84840693952, -2.87304195382},
    {"F x~hole0/revolved:{4}/facet:1.2", -1.19005294254, 2.84840693952, -2.87304195382},
    {"F x~hole0/revolved:{4}/facet:1.3", -2.87304195382, 2.84840693952, -1.19005294254},
    {"F x~hole0/revolved:{4}/facet:1.4", -2.87304195382, 2.84840693952, 1.19005294254},
    {"F x~hole0/revolved:{4}/facet:1.5", -1.19005294254, 2.84840693952, 2.87304195382},
    {"F x~hole0/revolved:{4}/facet:1.6", 1.19005294254, 2.84840693952, 2.87304195382},
    {"F x~hole0/revolved:{4}/facet:1.7", 2.87304195382, 2.84840693952, 1.19005294254},
    {"F x~hole0/revolved:{4}/facet:2.0", 2.27232214722, 2.8600749272, -0.94122665146},
    {"F x~hole0/revolved:{4}/facet:2.1", 0.94122665146, 2.8600749272, -2.27232214722},
    {"F x~hole0/revolved:{4}/facet:2.2", -0.94122665146, 2.8600749272, -2.27232214722},
    {"F x~hole0/revolved:{4}/facet:2.3", -2.27232214722, 2.8600749272, -0.94122665146},
    {"F x~hole0/revolved:{4}/facet:2.4", -2.27232214722, 2.8600749272, 0.94122665146},
    {"F x~hole0/revolved:{4}/facet:2.5", -0.94122665146, 2.8600749272, 2.27232214722},
    {"F x~hole0/revolved:{4}/facet:2.6", 0.94122665146, 2.8600749272, 2.27232214722},
    {"F x~hole0/revolved:{4}/facet:2.7", 2.27232214722, 2.8600749272, 0.94122665146},
    {"F x~hole0/revolved:{4}/facet:3.0", 1.8349495922, 2.36159407437, -0.76006100736},
    {"F x~hole0/revolved:{4}/facet:3.1", 0.76006100736, 2.36159407437, -1.8349495922},
    {"F x~hole0/revolved:{4}/facet:3.2", -0.76006100736, 2.36159407437, -1.8349495922},
    {"F x~hole0/revolved:{4}/facet:3.3", -1.8349495922, 2.36159407437, -0.76006100736},
    {"F x~hole0/revolved:{4}/facet:3.4", -1.8349495922, 2.36159407437, 0.76006100736},
    {"F x~hole0/revolved:{4}/facet:3.5", -0.76006100736, 2.36159407437, 1.8349495922},
    {"F x~hole0/revolved:{4}/facet:3.6", 0.76006100736, 2.36159407437, 1.8349495922},
    {"F x~hole0/revolved:{4}/facet:3.7", 1.8349495922, 2.36159407437, 0.76006100736},
    {"F x~hole0/revolved:{4}/facet:4.0", 1.8349495922, 1.63840592563, -0.76006100736},
    {"F x~hole0/revolved:{4}/facet:4.1", 0.76006100736, 1.63840592563, -1.8349495922},
    {"F x~hole0/revolved:{4}/facet:4.2", -0.76006100736, 1.63840592563, -1.8349495922},
    {"F x~hole0/revolved:{4}/facet:4.3", -1.8349495922, 1.63840592563, -0.76006100736},
    {"F x~hole0/revolved:{4}/facet:4.4", -1.8349495922, 1.63840592563, 0.76006100736},
    {"F x~hole0/revolved:{4}/facet:4.5", -0.76006100736, 1.63840592563, 1.8349495922},
    {"F x~hole0/revolved:{4}/facet:4.6", 0.76006100736, 1.63840592563, 1.8349495922},
    {"F x~hole0/revolved:{4}/facet:4.7", 1.8349495922, 1.63840592563, 0.76006100736},
    {"F x~hole0/revolved:{4}/facet:5.0", 2.27232214722, 1.1399250728, -0.94122665146},
    {"F x~hole0/revolved:{4}/facet:5.1", 0.94122665146, 1.1399250728, -2.27232214722},
    {"F x~hole0/revolved:{4}/facet:5.2", -0.94122665146, 1.1399250728, -2.27232214722},
    {"F x~hole0/revolved:{4}/facet:5.3", -2.27232214722, 1.1399250728, -0.94122665146},
    {"F x~hole0/revolved:{4}/facet:5.4", -2.27232214722, 1.1399250728, 0.94122665146},
    {"F x~hole0/revolved:{4}/facet:5.5", -0.94122665146, 1.1399250728, 2.27232214722},
    {"F x~hole0/revolved:{4}/facet:5.6", 0.94122665146, 1.1399250728, 2.27232214722},
    {"F x~hole0/revolved:{4}/facet:5.7", 2.27232214722, 1.1399250728, 0.94122665146},
    {"F x~hole0/revolved:{4}/facet:6.0", 2.87304195382, 1.15159306048, -1.19005294254},
    {"F x~hole0/revolved:{4}/facet:6.1", 1.19005294254, 1.15159306048, -2.87304195382},
    {"F x~hole0/revolved:{4}/facet:6.2", -1.19005294254, 1.15159306048, -2.87304195382},
    {"F x~hole0/revolved:{4}/facet:6.3", -2.87304195382, 1.15159306048, -1.19005294254},
    {"F x~hole0/revolved:{4}/facet:6.4", -2.87304195382, 1.15159306048, 1.19005294254},
    {"F x~hole0/revolved:{4}/facet:6.5", -1.19005294254, 1.15159306048, 2.87304195382},
    {"F x~hole0/revolved:{4}/facet:6.6", 1.19005294254, 1.15159306048, 2.87304195382},
    {"F x~hole0/revolved:{4}/facet:6.7", 2.87304195382, 1.15159306048, 1.19005294254},
    {"F x~hole0/revolved:{4}/facet:7.0", 3.29079702092, 1.65092530652, -1.36309275708},
    {"F x~hole0/revolved:{4}/facet:7.1", 1.36309275708, 1.65092530652, -3.29079702092},
    {"F x~hole0/revolved:{4}/facet:7.2", -1.36309275708, 1.65092530652, -3.29079702092},
    {"F x~hole0/revolved:{4}/facet:7.3", -3.29079702092, 1.65092530652, -1.36309275708},
    {"F x~hole0/revolved:{4}/facet:7.4", -3.29079702092, 1.65092530652, 1.36309275708},
    {"F x~hole0/revolved:{4}/facet:7.5", -1.36309275708, 1.65092530652, 3.29079702092},
    {"F x~hole0/revolved:{4}/facet:7.6", 1.36309275708, 1.65092530652, 3.29079702092},
    {"F x~hole0/revolved:{4}/facet:7.7", 3.29079702092, 1.65092530652, 1.36309275708},
};
const Place kTurnedHoledQuarter[] = {
    {"E edge:x/cap_end/piece:0|x~hole0/revolved:{4}/chord:0", 2.01288355304e-16, 1.14644660941, -3.28728830948},
    {"E edge:x/cap_end/piece:0|x~hole0/revolved:{4}/chord:1", 2.31299560115e-16, 1.64644660941, -3.77740847853},
    {"E edge:x/cap_end/piece:0|x~hole0/revolved:{4}/chord:2", 2.31299560115e-16, 2.35355339059, -3.77740847853},
    {"E edge:x/cap_end/piece:0|x~hole0/revolved:{4}/chord:3", 2.01288355304e-16, 2.85355339059, -3.28728830948},
    {"E edge:x/cap_end/piece:1|x~hole0/revolved:{4}/chord:0", 1.58846102436e-16, 2.85355339059, -2.5941537192},
    {"E edge:x/cap_end/piece:1|x~hole0/revolved:{4}/chord:1", 1.28834897624e-16, 2.35355339059, -2.10403355015},
    {"E edge:x/cap_end/piece:1|x~hole0/revolved:{4}/chord:2", 1.28834897624e-16, 1.64644660941, -2.10403355015},
    {"E edge:x/cap_end/piece:1|x~hole0/revolved:{4}/chord:3", 1.58846102436e-16, 1.14644660941, -2.5941537192},
    {"E edge:x/cap_start/piece:0|x~hole0/revolved:{4}/chord:0", 3.28728830948, 2.85355339059, 0},
    {"E edge:x/cap_start/piece:0|x~hole0/revolved:{4}/chord:1", 3.77740847853, 2.35355339059, 0},
    {"E edge:x/cap_start/piece:0|x~hole0/revolved:{4}/chord:2", 3.77740847853, 1.64644660941, 0},
    {"E edge:x/cap_start/piece:0|x~hole0/revolved:{4}/chord:3", 3.28728830948, 1.14644660941, 0},
    {"E edge:x/cap_start/piece:1|x~hole0/revolved:{4}/chord:0", 2.5941537192, 1.14644660941, 0},
    {"E edge:x/cap_start/piece:1|x~hole0/revolved:{4}/chord:1", 2.10403355015, 1.64644660941, 0},
    {"E edge:x/cap_start/piece:1|x~hole0/revolved:{4}/chord:2", 2.10403355015, 2.35355339059, 0},
    {"E edge:x/cap_start/piece:1|x~hole0/revolved:{4}/chord:3", 2.5941537192, 2.85355339059, 0},
    {"E x/edge:cap_end/piece:0|cap_end/piece:1/chord:0", 1.8006722887e-16, 0.5, -2.94072101434},
    {"E x/edge:cap_end/piece:0|cap_end/piece:1/chord:1", 1.8006722887e-16, 3.5, -2.94072101434},
    {"E x/edge:cap_end/piece:0|revolved:{0}", 2.43114464328e-16, 0, -3.97036050717},
    {"E x/edge:cap_end/piece:0|revolved:{1}", 3.06161699787e-16, 2, -5},
    {"E x/edge:cap_end/piece:0|revolved:{2}", 2.43114464328e-16, 4, -3.97036050717},
    {"E x/edge:cap_end/piece:1|revolved:{0}", 1.20649784414e-16, 0, -1.97036050717},
    {"E x/edge:cap_end/piece:1|revolved:{2}", 1.20649784414e-16, 4, -1.97036050717},
    {"E x/edge:cap_end/piece:1|revolved:{3}", 6.12323399574e-17, 2, -1},
    {"E x/edge:cap_start/piece:0|cap_start/piece:1/chord:0", 2.94072101434, 3.5, 0},
    {"E x/edge:cap_start/piece:0|cap_start/piece:1/chord:1", 2.94072101434, 0.5, 0},
    {"E x/edge:cap_start/piece:0|revolved:{0}", 3.97036050717, 0, 0},
    {"E x/edge:cap_start/piece:0|revolved:{1}", 5, 2, 0},
    {"E x/edge:cap_start/piece:0|revolved:{2}", 3.97036050717, 4, 0},
    {"E x/edge:cap_start/piece:1|revolved:{0}", 1.97036050717, 0, 0},
    {"E x/edge:cap_start/piece:1|revolved:{2}", 1.97036050717, 4, 0},
    {"E x/edge:cap_start/piece:1|revolved:{3}", 1, 2, 0},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:0", 4.26776695297, 0, -1.76776695297},
    {"E x/edge:revolved:{0}|revolved:{1}/chord:1", 1.76776695297, 0, -4.26776695297},
    {"E x/edge:revolved:{0}|revolved:{3}/chord:0", 0.853553390593, 0, -0.353553390593},
    {"E x/edge:revolved:{0}|revolved:{3}/chord:1", 0.353553390593, 0, -0.853553390593},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:0", 4.26776695297, 4, -1.76776695297},
    {"E x/edge:revolved:{1}|revolved:{2}/chord:1", 1.76776695297, 4, -4.26776695297},
    {"E x/edge:revolved:{2}|revolved:{3}/chord:0", 0.853553390593, 4, -0.353553390593},
    {"E x/edge:revolved:{2}|revolved:{3}/chord:1", 0.353553390593, 4, -0.853553390593},
    {"E x/revolved:{0}/seam:0", 2.12132034356, 0, -2.12132034356},
    {"E x/revolved:{1}/seam:0", 3.53553390593, 2, -3.53553390593},
    {"E x/revolved:{2}/seam:0", 2.12132034356, 4, -2.12132034356},
    {"E x/revolved:{3}/seam:0", 0.707106781187, 2, -0.707106781187},
    {"E x~hole0/revolved:{4}/seam:0", 3.44586224119, 2.70710678119, -0.88443923793},
    {"E x~hole0/revolved:{4}/seam:1", 3.38656866421, 2.35355339059, -1.83875675845},
    {"E x~hole0/revolved:{4}/seam:10", 2.03396380938, 3, -2.03396380938},
    {"E x~hole0/revolved:{4}/seam:11", 1.60017737832, 2.85355339059, -2.94716010788},
    {"E x~hole0/revolved:{4}/seam:12", 0.715738140389, 3, -2.78858617616},
    {"E x~hole0/revolved:{4}/seam:13", 2.13131011113, 2.70710678119, -0.547037042848},
    {"E x~hole0/revolved:{4}/seam:14", 2.32574256809, 2.85355339059, -1.26277518324},
    {"E x~hole0/revolved:{4}/seam:15", 1.55455394195, 2.70710678119, -1.55455394195},
    {"E x~hole0/revolved:{4}/seam:16", 1.26277518324, 2.85355339059, -2.32574256809},
    {"E x~hole0/revolved:{4}/seam:17", 0.547037042848, 2.70710678119, -2.13131011113},
    {"E x~hole0/revolved:{4}/seam:18", 1.85905745078, 2, -0.47715876026},
    {"E x~hole0/revolved:{4}/seam:19", 1.88633401176, 2.35355339059, -1.02419580311},
    {"E x~hole0/revolved:{4}/seam:2", 3.71811490155, 2, -0.954317520519},
    {"E x~hole0/revolved:{4}/seam:20", 1.35597587292, 2, -1.35597587292},
    {"E x~hole0/revolved:{4}/seam:21", 1.02419580311, 2.35355339059, -1.88633401176},
    {"E x~hole0/revolved:{4}/seam:22", 0.47715876026, 2, -1.85905745078},
    {"E x~hole0/revolved:{4}/seam:23", 2.13131011113, 1.29289321881, -0.547037042848},
    {"E x~hole0/revolved:{4}/seam:24", 1.88633401176, 1.64644660941, -1.02419580311},
    {"E x~hole0/revolved:{4}/seam:25", 1.55455394195, 1.29289321881, -1.55455394195},
    {"E x~hole0/revolved:{4}/seam:26", 1.02419580311, 1.64644660941, -1.88633401176},
    {"E x~hole0/revolved:{4}/seam:27", 0.547037042848, 1.29289321881, -2.13131011113},
    {"E x~hole0/revolved:{4}/seam:28", 2.78858617616, 1, -0.715738140389},
    {"E x~hole0/revolved:{4}/seam:29", 2.32574256809, 1.14644660941, -1.26277518324},
    {"E x~hole0/revolved:{4}/seam:3", 2.51337367682, 2.70710678119, -2.51337367682},
    {"E x~hole0/revolved:{4}/seam:30", 2.03396380938, 1, -2.03396380938},
    {"E x~hole0/revolved:{4}/seam:31", 1.26277518324, 1.14644660941, -2.32574256809},
    {"E x~hole0/revolved:{4}/seam:32", 0.715738140389, 1, -2.78858617616},
    {"E x~hole0/revolved:{4}/seam:33", 3.44586224119, 1.29289321881, -0.88443923793},
    {"E x~hole0/revolved:{4}/seam:34", 2.94716010788, 1.14644660941, -1.60017737832},
    {"E x~hole0/revolved:{4}/seam:35", 2.51337367682, 1.29289321881, -2.51337367682},
    {"E x~hole0/revolved:{4}/seam:36", 1.60017737832, 1.14644660941, -2.94716010788},
    {"E x~hole0/revolved:{4}/seam:37", 0.88443923793, 1.29289321881, -3.44586224119},
    {"E x~hole0/revolved:{4}/seam:38", 3.38656866421, 1.64644660941, -1.83875675845},
    {"E x~hole0/revolved:{4}/seam:39", 1.83875675845, 1.64644660941, -3.38656866421},
    {"E x~hole0/revolved:{4}/seam:4", 1.83875675845, 2.35355339059, -3.38656866421},
    {"E x~hole0/revolved:{4}/seam:5", 2.71195174584, 2, -2.71195174584},
    {"E x~hole0/revolved:{4}/seam:6", 0.954317520519, 2, -3.71811490155},
    {"E x~hole0/revolved:{4}/seam:7", 0.88443923793, 2.70710678119, -3.44586224119},
    {"E x~hole0/revolved:{4}/seam:8", 2.78858617616, 3, -0.715738140389},
    {"E x~hole0/revolved:{4}/seam:9", 2.94716010788, 2.85355339059, -1.60017737832},
    {"F x/cap_end/piece:0", 2.34602427459e-16, 2.08302295772, -3.83134839567},
    {"F x/cap_end/piece:1", 1.28805082904e-16, 2.08702326074, -2.10354663882},
    {"F x/cap_start/piece:0", 4.05622455071, 1.98269165464, 0},
    {"F x/cap_start/piece:1", 1.88683904768, 1.98129739409, 0},
    {"F x/revolved:{0}/facet:0", 2.94001723427, 0, -1.21779501204},
    {"F x/revolved:{0}/facet:1", 1.21779501204, 0, -2.94001723427},
    {"F x/revolved:{1}/facet:0", 4.26776695297, 2, -1.76776695297},
    {"F x/revolved:{1}/facet:1", 1.76776695297, 2, -4.26776695297},
    {"F x/revolved:{2}/facet:0", 2.94001723427, 4, -1.21779501204},
    {"F x/revolved:{2}/facet:1", 1.21779501204, 4, -2.94001723427},
    {"F x/revolved:{3}/facet:0", 0.853553390593, 2, -0.353553390593},
    {"F x/revolved:{3}/facet:1", 0.353553390593, 2, -0.853553390593},
    {"F x~hole0/revolved:{4}/facet:0.0", 3.58371297458, 2.34907469348, -0.919820976681},
    {"F x~hole0/revolved:{4}/facet:0.1", 2.61392047189, 2.34907469348, -2.61392047189},
    {"F x~hole0/revolved:{4}/facet:0.2", 0.919820976681, 2.34907469348, -3.58371297458},
    {"F x~hole0/revolved:{4}/facet:1.0", 3.128773261, 2.84840693952, -0.803052950153},
    {"F x~hole0/revolved:{4}/facet:1.1", 2.28209249369, 2.84840693952, -2.28209249369},
    {"F x~hole0/revolved:{4}/facet:1.2", 0.803052950153, 2.84840693952, -3.128773261},
    {"F x~hole0/revolved:{4}/facet:2.0", 2.47458299909, 2.8600749272, -0.635143876545},
    {"F x~hole0/revolved:{4}/facet:2.1", 1.80493337681, 2.8600749272, -1.80493337681},
    {"F x~hole0/revolved:{4}/facet:2.2", 0.635143876545, 2.8600749272, -2.47458299909},
    {"F x~hole0/revolved:{4}/facet:3.0", 1.9982796324, 2.36159407437, -0.512892504559},
    {"F x~hole0/revolved:{4}/facet:3.1", 1.45752298712, 2.36159407437, -1.45752298712},
    {"F x~hole0/revolved:{4}/facet:3.2", 0.512892504559, 2.36159407437, -1.9982796324},
    {"F x~hole0/revolved:{4}/facet:4.0", 1.9982796324, 1.63840592563, -0.512892504559},
    {"F x~hole0/revolved:{4}/facet:4.1", 1.45752298712, 1.63840592563, -1.45752298712},
    {"F x~hole0/revolved:{4}/facet:4.2", 0.512892504559, 1.63840592563, -1.9982796324},
    {"F x~hole0/revolved:{4}/facet:5.0", 2.47458299909, 1.1399250728, -0.635143876545},
    {"F x~hole0/revolved:{4}/facet:5.1", 1.80493337681, 1.1399250728, -1.80493337681},
    {"F x~hole0/revolved:{4}/facet:5.2", 0.635143876545, 1.1399250728, -2.47458299909},
    {"F x~hole0/revolved:{4}/facet:6.0", 3.128773261, 1.15159306048, -0.803052950153},
    {"F x~hole0/revolved:{4}/facet:6.1", 2.28209249369, 1.15159306048, -2.28209249369},
    {"F x~hole0/revolved:{4}/facet:6.2", 0.803052950153, 1.15159306048, -3.128773261},
    {"F x~hole0/revolved:{4}/facet:7.0", 3.58371297458, 1.65092530652, -0.919820976681},
    {"F x~hole0/revolved:{4}/facet:7.1", 2.61392047189, 1.65092530652, -2.61392047189},
    {"F x~hole0/revolved:{4}/facet:7.2", 0.919820976681, 1.65092530652, -3.58371297458},
};

const Place kSimpleHoleTool[] = {
    {"E hole_1/bottom/seam:0", 10.8838834765, 9.11611652352, 3.24892422622},
    {"E hole_1/bottom/seam:1", 11.25, 10, 3.24892422622},
    {"E hole_1/bottom/seam:2", 10, 8.75, 3.24892422622},
    {"E hole_1/bottom/seam:3", 9.11611652352, 9.11611652352, 3.24892422622},
    {"E hole_1/bottom/seam:4", 8.75, 10, 3.24892422622},
    {"E hole_1/bottom/seam:5", 9.11611652352, 10.8838834765, 3.24892422622},
    {"E hole_1/bottom/seam:6", 10, 11.25, 3.24892422622},
    {"E hole_1/bottom/seam:7", 10.8838834765, 10.8838834765, 3.24892422622},
    {"E hole_1/edge:top|wall/chord:0", 12.1338834765, 9.11611652352, 10.120001},
    {"E hole_1/edge:top|wall/chord:1", 10.8838834765, 7.86611652352, 10.120001},
    {"E hole_1/edge:top|wall/chord:2", 9.11611652352, 7.86611652352, 10.120001},
    {"E hole_1/edge:top|wall/chord:3", 7.86611652352, 9.11611652352, 10.120001},
    {"E hole_1/edge:top|wall/chord:4", 7.86611652352, 10.8838834765, 10.120001},
    {"E hole_1/edge:top|wall/chord:5", 9.11611652352, 12.1338834765, 10.120001},
    {"E hole_1/edge:top|wall/chord:6", 10.8838834765, 12.1338834765, 10.120001},
    {"E hole_1/edge:top|wall/chord:7", 12.1338834765, 10.8838834765, 10.120001},
    {"E hole_1/edge:wall|bottom/chord:0", 12.1338834765, 9.11611652352, 4},
    {"E hole_1/edge:wall|bottom/chord:1", 10.8838834765, 7.86611652352, 4},
    {"E hole_1/edge:wall|bottom/chord:2", 9.11611652352, 7.86611652352, 4},
    {"E hole_1/edge:wall|bottom/chord:3", 7.86611652352, 9.11611652352, 4},
    {"E hole_1/edge:wall|bottom/chord:4", 7.86611652352, 10.8838834765, 4},
    {"E hole_1/edge:wall|bottom/chord:5", 9.11611652352, 12.1338834765, 4},
    {"E hole_1/edge:wall|bottom/chord:6", 10.8838834765, 12.1338834765, 4},
    {"E hole_1/edge:wall|bottom/chord:7", 12.1338834765, 10.8838834765, 4},
    {"E hole_1/wall/seam:0", 11.767766953, 8.23223304703, 7.0600005},
    {"E hole_1/wall/seam:1", 12.5, 10, 7.0600005},
    {"E hole_1/wall/seam:2", 10, 7.5, 7.0600005},
    {"E hole_1/wall/seam:3", 8.23223304703, 8.23223304703, 7.0600005},
    {"E hole_1/wall/seam:4", 7.5, 10, 7.0600005},
    {"E hole_1/wall/seam:5", 8.23223304703, 11.767766953, 7.0600005},
    {"E hole_1/wall/seam:6", 10, 12.5, 7.0600005},
    {"E hole_1/wall/seam:7", 11.767766953, 11.767766953, 7.0600005},
    {"F hole_1/bottom/facet:0", 11.4225889843, 9.41074434901, 3.49928281748},
    {"F hole_1/bottom/facet:1", 10.589255651, 8.57741101568, 3.49928281748},
    {"F hole_1/bottom/facet:2", 9.41074434901, 8.57741101568, 3.49928281748},
    {"F hole_1/bottom/facet:3", 8.57741101568, 9.41074434901, 3.49928281748},
    {"F hole_1/bottom/facet:4", 8.57741101568, 10.589255651, 3.49928281748},
    {"F hole_1/bottom/facet:5", 9.41074434901, 11.4225889843, 3.49928281748},
    {"F hole_1/bottom/facet:6", 10.589255651, 11.4225889843, 3.49928281748},
    {"F hole_1/bottom/facet:7", 11.4225889843, 10.589255651, 3.49928281748},
    {"F hole_1/top", 10, 10, 10.120001},
    {"F hole_1/wall/facet:0", 12.1338834765, 9.11611652352, 7.0600005},
    {"F hole_1/wall/facet:1", 10.8838834765, 7.86611652352, 7.0600005},
    {"F hole_1/wall/facet:2", 9.11611652352, 7.86611652352, 7.0600005},
    {"F hole_1/wall/facet:3", 7.86611652352, 9.11611652352, 7.0600005},
    {"F hole_1/wall/facet:4", 7.86611652352, 10.8838834765, 7.0600005},
    {"F hole_1/wall/facet:5", 9.11611652352, 12.1338834765, 7.0600005},
    {"F hole_1/wall/facet:6", 10.8838834765, 12.1338834765, 7.0600005},
    {"F hole_1/wall/facet:7", 12.1338834765, 10.8838834765, 7.0600005},
};
const Place kCounterboreHoleTool[] = {
    {"E hole_1/bore/seam:0", 13.1819805153, 6.81801948466, 8.5600005},
    {"E hole_1/bore/seam:1", 14.5, 10, 8.5600005},
    {"E hole_1/bore/seam:2", 10, 5.5, 8.5600005},
    {"E hole_1/bore/seam:3", 6.81801948466, 6.81801948466, 8.5600005},
    {"E hole_1/bore/seam:4", 5.5, 10, 8.5600005},
    {"E hole_1/bore/seam:5", 6.81801948466, 13.1819805153, 8.5600005},
    {"E hole_1/bore/seam:6", 10, 14.5, 8.5600005},
    {"E hole_1/bore/seam:7", 13.1819805153, 13.1819805153, 8.5600005},
    {"E hole_1/bottom/seam:0", 10.8838834765, 9.11611652352, 3.24892422622},
    {"E hole_1/bottom/seam:1", 11.25, 10, 3.24892422622},
    {"E hole_1/bottom/seam:2", 10, 8.75, 3.24892422622},
    {"E hole_1/bottom/seam:3", 9.11611652352, 9.11611652352, 3.24892422622},
    {"E hole_1/bottom/seam:4", 8.75, 10, 3.24892422622},
    {"E hole_1/bottom/seam:5", 9.11611652352, 10.8838834765, 3.24892422622},
    {"E hole_1/bottom/seam:6", 10, 11.25, 3.24892422622},
    {"E hole_1/bottom/seam:7", 10.8838834765, 10.8838834765, 3.24892422622},
    {"E hole_1/edge:bore|boreFloor/chord:0", 13.8409902577, 8.40900974233, 7},
    {"E hole_1/edge:bore|boreFloor/chord:1", 11.5909902577, 6.15900974233, 7},
    {"E hole_1/edge:bore|boreFloor/chord:2", 8.40900974233, 6.15900974233, 7},
    {"E hole_1/edge:bore|boreFloor/chord:3", 6.15900974233, 8.40900974233, 7},
    {"E hole_1/edge:bore|boreFloor/chord:4", 6.15900974233, 11.5909902577, 7},
    {"E hole_1/edge:bore|boreFloor/chord:5", 8.40900974233, 13.8409902577, 7},
    {"E hole_1/edge:bore|boreFloor/chord:6", 11.5909902577, 13.8409902577, 7},
    {"E hole_1/edge:bore|boreFloor/chord:7", 13.8409902577, 11.5909902577, 7},
    {"E hole_1/edge:top|bore/chord:0", 13.8409902577, 8.40900974233, 10.120001},
    {"E hole_1/edge:top|bore/chord:1", 11.5909902577, 6.15900974233, 10.120001},
    {"E hole_1/edge:top|bore/chord:2", 8.40900974233, 6.15900974233, 10.120001},
    {"E hole_1/edge:top|bore/chord:3", 6.15900974233, 8.40900974233, 10.120001},
    {"E hole_1/edge:top|bore/chord:4", 6.15900974233, 11.5909902577, 10.120001},
    {"E hole_1/edge:top|bore/chord:5", 8.40900974233, 13.8409902577, 10.120001},
    {"E hole_1/edge:top|bore/chord:6", 11.5909902577, 13.8409902577, 10.120001},
    {"E hole_1/edge:top|bore/chord:7", 13.8409902577, 11.5909902577, 10.120001},
    {"E hole_1/edge:wall|boreFloor/chord:0", 12.1338834765, 9.11611652352, 7},
    {"E hole_1/edge:wall|boreFloor/chord:1", 10.8838834765, 7.86611652352, 7},
    {"E hole_1/edge:wall|boreFloor/chord:2", 9.11611652352, 7.86611652352, 7},
    {"E hole_1/edge:wall|boreFloor/chord:3", 7.86611652352, 9.11611652352, 7},
    {"E hole_1/edge:wall|boreFloor/chord:4", 7.86611652352, 10.8838834765, 7},
    {"E hole_1/edge:wall|boreFloor/chord:5", 9.11611652352, 12.1338834765, 7},
    {"E hole_1/edge:wall|boreFloor/chord:6", 10.8838834765, 12.1338834765, 7},
    {"E hole_1/edge:wall|boreFloor/chord:7", 12.1338834765, 10.8838834765, 7},
    {"E hole_1/edge:wall|bottom/chord:0", 12.1338834765, 9.11611652352, 4},
    {"E hole_1/edge:wall|bottom/chord:1", 10.8838834765, 7.86611652352, 4},
    {"E hole_1/edge:wall|bottom/chord:2", 9.11611652352, 7.86611652352, 4},
    {"E hole_1/edge:wall|bottom/chord:3", 7.86611652352, 9.11611652352, 4},
    {"E hole_1/edge:wall|bottom/chord:4", 7.86611652352, 10.8838834765, 4},
    {"E hole_1/edge:wall|bottom/chord:5", 9.11611652352, 12.1338834765, 4},
    {"E hole_1/edge:wall|bottom/chord:6", 10.8838834765, 12.1338834765, 4},
    {"E hole_1/edge:wall|bottom/chord:7", 12.1338834765, 10.8838834765, 4},
    {"E hole_1/wall/seam:0", 11.767766953, 8.23223304703, 5.5},
    {"E hole_1/wall/seam:1", 12.5, 10, 5.5},
    {"E hole_1/wall/seam:2", 10, 7.5, 5.5},
    {"E hole_1/wall/seam:3", 8.23223304703, 8.23223304703, 5.5},
    {"E hole_1/wall/seam:4", 7.5, 10, 5.5},
    {"E hole_1/wall/seam:5", 8.23223304703, 11.767766953, 5.5},
    {"E hole_1/wall/seam:6", 10, 12.5, 5.5},
    {"E hole_1/wall/seam:7", 11.767766953, 11.767766953, 5.5},
    {"F hole_1/bore/facet:0", 13.8409902577, 8.40900974233, 8.5600005},
    {"F hole_1/bore/facet:1", 11.5909902577, 6.15900974233, 8.5600005},
    {"F hole_1/bore/facet:2", 8.40900974233, 6.15900974233, 8.5600005},
    {"F hole_1/bore/facet:3", 6.15900974233, 8.40900974233, 8.5600005},
    {"F hole_1/bore/facet:4", 6.15900974233, 11.5909902577, 8.5600005},
    {"F hole_1/bore/facet:5", 8.40900974233, 13.8409902577, 8.5600005},
    {"F hole_1/bore/facet:6", 11.5909902577, 13.8409902577, 8.5600005},
    {"F hole_1/bore/facet:7", 13.8409902577, 11.5909902577, 8.5600005},
    {"F hole_1/boreFloor", 10, 10, 7},
    {"F hole_1/bottom/facet:0", 11.4225889843, 9.41074434901, 3.49928281748},
    {"F hole_1/bottom/facet:1", 10.589255651, 8.57741101568, 3.49928281748},
    {"F hole_1/bottom/facet:2", 9.41074434901, 8.57741101568, 3.49928281748},
    {"F hole_1/bottom/facet:3", 8.57741101568, 9.41074434901, 3.49928281748},
    {"F hole_1/bottom/facet:4", 8.57741101568, 10.589255651, 3.49928281748},
    {"F hole_1/bottom/facet:5", 9.41074434901, 11.4225889843, 3.49928281748},
    {"F hole_1/bottom/facet:6", 10.589255651, 11.4225889843, 3.49928281748},
    {"F hole_1/bottom/facet:7", 11.4225889843, 10.589255651, 3.49928281748},
    {"F hole_1/top", 10, 10, 10.120001},
    {"F hole_1/wall/facet:0", 12.1338834765, 9.11611652352, 5.5},
    {"F hole_1/wall/facet:1", 10.8838834765, 7.86611652352, 5.5},
    {"F hole_1/wall/facet:2", 9.11611652352, 7.86611652352, 5.5},
    {"F hole_1/wall/facet:3", 7.86611652352, 9.11611652352, 5.5},
    {"F hole_1/wall/facet:4", 7.86611652352, 10.8838834765, 5.5},
    {"F hole_1/wall/facet:5", 9.11611652352, 12.1338834765, 5.5},
    {"F hole_1/wall/facet:6", 10.8838834765, 12.1338834765, 5.5},
    {"F hole_1/wall/facet:7", 12.1338834765, 10.8838834765, 5.5},
};
const Place kCountersinkHoleTool[] = {
    {"E hole_1/bottom/seam:0", 10.8838834765, 9.11611652352, 3.24892422622},
    {"E hole_1/bottom/seam:1", 11.25, 10, 3.24892422622},
    {"E hole_1/bottom/seam:2", 10, 8.75, 3.24892422622},
    {"E hole_1/bottom/seam:3", 9.11611652352, 9.11611652352, 3.24892422622},
    {"E hole_1/bottom/seam:4", 8.75, 10, 3.24892422622},
    {"E hole_1/bottom/seam:5", 9.11611652352, 10.8838834765, 3.24892422622},
    {"E hole_1/bottom/seam:6", 10, 11.25, 3.24892422622},
    {"E hole_1/bottom/seam:7", 10.8838834765, 10.8838834765, 3.24892422622},
    {"E hole_1/edge:top|sink/chord:0", 14.3701942134, 8.18980628661, 10.120001},
    {"E hole_1/edge:top|sink/chord:1", 11.8101937134, 5.62980578661, 10.120001},
    {"E hole_1/edge:top|sink/chord:2", 8.18980628661, 5.62980578661, 10.120001},
    {"E hole_1/edge:top|sink/chord:3", 5.62980578661, 8.18980628661, 10.120001},
    {"E hole_1/edge:top|sink/chord:4", 5.62980578661, 11.8101937134, 10.120001},
    {"E hole_1/edge:top|sink/chord:5", 8.18980628661, 14.3701942134, 10.120001},
    {"E hole_1/edge:top|sink/chord:6", 11.8101937134, 14.3701942134, 10.120001},
    {"E hole_1/edge:top|sink/chord:7", 14.3701942134, 11.8101937134, 10.120001},
    {"E hole_1/edge:wall|bottom/chord:0", 12.1338834765, 9.11611652352, 4},
    {"E hole_1/edge:wall|bottom/chord:1", 10.8838834765, 7.86611652352, 4},
    {"E hole_1/edge:wall|bottom/chord:2", 9.11611652352, 7.86611652352, 4},
    {"E hole_1/edge:wall|bottom/chord:3", 7.86611652352, 9.11611652352, 4},
    {"E hole_1/edge:wall|bottom/chord:4", 7.86611652352, 10.8838834765, 4},
    {"E hole_1/edge:wall|bottom/chord:5", 9.11611652352, 12.1338834765, 4},
    {"E hole_1/edge:wall|bottom/chord:6", 10.8838834765, 12.1338834765, 4},
    {"E hole_1/edge:wall|bottom/chord:7", 12.1338834765, 10.8838834765, 4},
    {"E hole_1/edge:wall|sink/chord:0", 12.1338834765, 9.11611652352, 7.5},
    {"E hole_1/edge:wall|sink/chord:1", 10.8838834765, 7.86611652352, 7.5},
    {"E hole_1/edge:wall|sink/chord:2", 9.11611652352, 7.86611652352, 7.5},
    {"E hole_1/edge:wall|sink/chord:3", 7.86611652352, 9.11611652352, 7.5},
    {"E hole_1/edge:wall|sink/chord:4", 7.86611652352, 10.8838834765, 7.5},
    {"E hole_1/edge:wall|sink/chord:5", 9.11611652352, 12.1338834765, 7.5},
    {"E hole_1/edge:wall|sink/chord:6", 10.8838834765, 12.1338834765, 7.5},
    {"E hole_1/edge:wall|sink/chord:7", 12.1338834765, 10.8838834765, 7.5},
    {"E hole_1/sink/seam:0", 12.6940771899, 7.30592281013, 8.8100005},
    {"E hole_1/sink/seam:1", 13.8100005, 10, 8.8100005},
    {"E hole_1/sink/seam:2", 10, 6.1899995, 8.8100005},
    {"E hole_1/sink/seam:3", 7.30592281013, 7.30592281013, 8.8100005},
    {"E hole_1/sink/seam:4", 6.1899995, 10, 8.8100005},
    {"E hole_1/sink/seam:5", 7.30592281013, 12.6940771899, 8.8100005},
    {"E hole_1/sink/seam:6", 10, 13.8100005, 8.8100005},
    {"E hole_1/sink/seam:7", 12.6940771899, 12.6940771899, 8.8100005},
    {"E hole_1/wall/seam:0", 11.767766953, 8.23223304703, 5.75},
    {"E hole_1/wall/seam:1", 12.5, 10, 5.75},
    {"E hole_1/wall/seam:2", 10, 7.5, 5.75},
    {"E hole_1/wall/seam:3", 8.23223304703, 8.23223304703, 5.75},
    {"E hole_1/wall/seam:4", 7.5, 10, 5.75},
    {"E hole_1/wall/seam:5", 8.23223304703, 11.767766953, 5.75},
    {"E hole_1/wall/seam:6", 10, 12.5, 5.75},
    {"E hole_1/wall/seam:7", 11.767766953, 11.767766953, 5.75},
    {"F hole_1/bottom/facet:0", 11.4225889843, 9.41074434901, 3.49928281748},
    {"F hole_1/bottom/facet:1", 10.589255651, 8.57741101568, 3.49928281748},
    {"F hole_1/bottom/facet:2", 9.41074434901, 8.57741101568, 3.49928281748},
    {"F hole_1/bottom/facet:3", 8.57741101568, 9.41074434901, 3.49928281748},
    {"F hole_1/bottom/facet:4", 8.57741101568, 10.589255651, 3.49928281748},
    {"F hole_1/bottom/facet:5", 9.41074434901, 11.4225889843, 3.49928281748},
    {"F hole_1/bottom/facet:6", 10.589255651, 11.4225889843, 3.49928281748},
    {"F hole_1/bottom/facet:7", 11.4225889843, 10.589255651, 3.49928281748},
    {"F hole_1/sink/facet:0", 13.3801914171, 8.59987887163, 8.96014057741},
    {"F hole_1/sink/facet:1", 11.4001211284, 6.61980858293, 8.96014057741},
    {"F hole_1/sink/facet:2", 8.59987887163, 6.61980858293, 8.96014057741},
    {"F hole_1/sink/facet:3", 6.61980858293, 8.59987887163, 8.96014057741},
    {"F hole_1/sink/facet:4", 6.61980858293, 11.4001211284, 8.96014057741},
    {"F hole_1/sink/facet:5", 8.59987887163, 13.3801914171, 8.96014057741},
    {"F hole_1/sink/facet:6", 11.4001211284, 13.3801914171, 8.96014057741},
    {"F hole_1/sink/facet:7", 13.3801914171, 11.4001211284, 8.96014057741},
    {"F hole_1/top", 10, 10, 10.120001},
    {"F hole_1/wall/facet:0", 12.1338834765, 9.11611652352, 5.75},
    {"F hole_1/wall/facet:1", 10.8838834765, 7.86611652352, 5.75},
    {"F hole_1/wall/facet:2", 9.11611652352, 7.86611652352, 5.75},
    {"F hole_1/wall/facet:3", 7.86611652352, 9.11611652352, 5.75},
    {"F hole_1/wall/facet:4", 7.86611652352, 10.8838834765, 5.75},
    {"F hole_1/wall/facet:5", 9.11611652352, 12.1338834765, 5.75},
    {"F hole_1/wall/facet:6", 10.8838834765, 12.1338834765, 5.75},
    {"F hole_1/wall/facet:7", 12.1338834765, 10.8838834765, 5.75},
};

const Place kTurnedLemon12[] = {
    {"E x/revolved:{0}/seam:0", 0.683012701892, -1, -0.183012701892},
    {"E x/revolved:{0}/seam:1", 0.366025403784, -1.36602540378, 0},
    {"E x/revolved:{0}/seam:10", 0.366025403784, 1.36602540378, 0},
    {"E x/revolved:{0}/seam:11", 0.5, -1, -0.5},
    {"E x/revolved:{0}/seam:12", 0.183012701892, -1.36602540378, -0.316987298108},
    {"E x/revolved:{0}/seam:13", 0.433012701892, -0.5, -0.75},
    {"E x/revolved:{0}/seam:14", 0.683012701892, -4.89858719659e-16, -0.683012701892},
    {"E x/revolved:{0}/seam:15", 0.433012701892, 0.5, -0.75},
    {"E x/revolved:{0}/seam:16", 0.5, 1, -0.5},
    {"E x/revolved:{0}/seam:17", 0.183012701892, 1.36602540378, -0.316987298108},
    {"E x/revolved:{0}/seam:18", 0.183012701892, -1, -0.683012701892},
    {"E x/revolved:{0}/seam:19", 1.3343489442e-16, -1.36602540378, -0.366025403784},
    {"E x/revolved:{0}/seam:2", 0.316987298108, -1.36602540378, -0.183012701892},
    {"E x/revolved:{0}/seam:20", 5.30287619362e-17, -0.5, -0.866025403784},
    {"E x/revolved:{0}/seam:21", 0.25, -4.89858719659e-16, -0.933012701892},
    {"E x/revolved:{0}/seam:22", 5.30287619362e-17, 0.5, -0.866025403784},
    {"E x/revolved:{0}/seam:23", 0.183012701892, 1, -0.683012701892},
    {"E x/revolved:{0}/seam:24", 1.3343489442e-16, 1.36602540378, -0.366025403784},
    {"E x/revolved:{0}/seam:25", -0.183012701892, -1, -0.683012701892},
    {"E x/revolved:{0}/seam:26", -0.183012701892, -1.36602540378, -0.316987298108},
    {"E x/revolved:{0}/seam:27", -0.433012701892, -0.5, -0.75},
    {"E x/revolved:{0}/seam:28", -0.25, -4.89858719659e-16, -0.933012701892},
    {"E x/revolved:{0}/seam:29", -0.433012701892, 0.5, -0.75},
    {"E x/revolved:{0}/seam:3", 0.75, -0.5, -0.433012701892},
    {"E x/revolved:{0}/seam:30", -0.183012701892, 1, -0.683012701892},
    {"E x/revolved:{0}/seam:31", -0.183012701892, 1.36602540378, -0.316987298108},
    {"E x/revolved:{0}/seam:32", -0.5, -1, -0.5},
    {"E x/revolved:{0}/seam:33", -0.316987298108, -1.36602540378, -0.183012701892},
    {"E x/revolved:{0}/seam:34", -0.75, -0.5, -0.433012701892},
    {"E x/revolved:{0}/seam:35", -0.683012701892, -4.89858719659e-16, -0.683012701892},
    {"E x/revolved:{0}/seam:36", -0.75, 0.5, -0.433012701892},
    {"E x/revolved:{0}/seam:37", -0.5, 1, -0.5},
    {"E x/revolved:{0}/seam:38", -0.316987298108, 1.36602540378, -0.183012701892},
    {"E x/revolved:{0}/seam:39", -0.683012701892, -1, -0.183012701892},
    {"E x/revolved:{0}/seam:4", 0.933012701892, -4.89858719659e-16, -0.25},
    {"E x/revolved:{0}/seam:40", -0.366025403784, -1.36602540378, -4.48251839151e-17},
    {"E x/revolved:{0}/seam:41", -0.866025403784, -0.5, -1.06057523872e-16},
    {"E x/revolved:{0}/seam:42", -0.933012701892, -4.89858719659e-16, -0.25},
    {"E x/revolved:{0}/seam:43", -0.866025403784, 0.5, -1.06057523872e-16},
    {"E x/revolved:{0}/seam:44", -0.683012701892, 1, -0.183012701892},
    {"E x/revolved:{0}/seam:45", -0.366025403784, 1.36602540378, -4.48251839151e-17},
    {"E x/revolved:{0}/seam:46", -0.683012701892, -1, 0.183012701892},
    {"E x/revolved:{0}/seam:47", -0.316987298108, -1.36602540378, 0.183012701892},
    {"E x/revolved:{0}/seam:48", -0.75, -0.5, 0.433012701892},
    {"E x/revolved:{0}/seam:49", -0.933012701892, -4.89858719659e-16, 0.25},
    {"E x/revolved:{0}/seam:5", 0.866025403784, -0.5, 0},
    {"E x/revolved:{0}/seam:50", -0.75, 0.5, 0.433012701892},
    {"E x/revolved:{0}/seam:51", -0.683012701892, 1, 0.183012701892},
    {"E x/revolved:{0}/seam:52", -0.316987298108, 1.36602540378, 0.183012701892},
    {"E x/revolved:{0}/seam:53", -0.5, -1, 0.5},
    {"E x/revolved:{0}/seam:54", -0.183012701892, -1.36602540378, 0.316987298108},
    {"E x/revolved:{0}/seam:55", -0.433012701892, -0.5, 0.75},
    {"E x/revolved:{0}/seam:56", -0.683012701892, -4.89858719659e-16, 0.683012701892},
    {"E x/revolved:{0}/seam:57", -0.433012701892, 0.5, 0.75},
    {"E x/revolved:{0}/seam:58", -0.5, 1, 0.5},
    {"E x/revolved:{0}/seam:59", -0.183012701892, 1.36602540378, 0.316987298108},
    {"E x/revolved:{0}/seam:6", 0.75, 0.5, -0.433012701892},
    {"E x/revolved:{0}/seam:60", -0.183012701892, -1, 0.683012701892},
    {"E x/revolved:{0}/seam:61", 4.37845265898e-17, -1.36602540378, 0.366025403784},
    {"E x/revolved:{0}/seam:62", -1.59086285809e-16, -0.5, 0.866025403784},
    {"E x/revolved:{0}/seam:63", -0.25, -4.89858719659e-16, 0.933012701892},
    {"E x/revolved:{0}/seam:64", -1.59086285809e-16, 0.5, 0.866025403784},
    {"E x/revolved:{0}/seam:65", -0.183012701892, 1, 0.683012701892},
    {"E x/revolved:{0}/seam:66", 4.37845265898e-17, 1.36602540378, 0.366025403784},
    {"E x/revolved:{0}/seam:67", 0.183012701892, -1, 0.683012701892},
    {"E x/revolved:{0}/seam:68", 0.183012701892, -1.36602540378, 0.316987298108},
    {"E x/revolved:{0}/seam:69", 0.433012701892, -0.5, 0.75},
    {"E x/revolved:{0}/seam:7", 0.683012701892, 1, -0.183012701892},
    {"E x/revolved:{0}/seam:70", 0.25, -4.89858719659e-16, 0.933012701892},
    {"E x/revolved:{0}/seam:71", 0.433012701892, 0.5, 0.75},
    {"E x/revolved:{0}/seam:72", 0.183012701892, 1, 0.683012701892},
    {"E x/revolved:{0}/seam:73", 0.183012701892, 1.36602540378, 0.316987298108},
    {"E x/revolved:{0}/seam:74", 0.5, -1, 0.5},
    {"E x/revolved:{0}/seam:75", 0.316987298108, -1.36602540378, 0.183012701892},
    {"E x/revolved:{0}/seam:76", 0.75, -0.5, 0.433012701892},
    {"E x/revolved:{0}/seam:77", 0.683012701892, -4.89858719659e-16, 0.683012701892},
    {"E x/revolved:{0}/seam:78", 0.75, 0.5, 0.433012701892},
    {"E x/revolved:{0}/seam:79", 0.5, 1, 0.5},
    {"E x/revolved:{0}/seam:8", 0.866025403784, 0.5, 0},
    {"E x/revolved:{0}/seam:80", 0.316987298108, 1.36602540378, 0.183012701892},
    {"E x/revolved:{0}/seam:81", 0.683012701892, -1, 0.183012701892},
    {"E x/revolved:{0}/seam:82", 0.933012701892, -4.89858719659e-16, 0.25},
    {"E x/revolved:{0}/seam:83", 0.683012701892, 1, 0.183012701892},
    {"E x/revolved:{0}/seam:9", 0.316987298108, 1.36602540378, -0.183012701892},
    {"F x/revolved:{0}/facet:0.0", 0.455341801261, -1.24401693586, -0.122008467928},
    {"F x/revolved:{0}/facet:0.1", 0.333333333333, -1.24401693586, -0.333333333333},
    {"F x/revolved:{0}/facet:0.10", 0.333333333333, -1.24401693586, 0.333333333333},
    {"F x/revolved:{0}/facet:0.11", 0.455341801261, -1.24401693586, 0.122008467928},
    {"F x/revolved:{0}/facet:0.2", 0.122008467928, -1.24401693586, -0.455341801261},
    {"F x/revolved:{0}/facet:0.3", -0.122008467928, -1.24401693586, -0.455341801261},
    {"F x/revolved:{0}/facet:0.4", -0.333333333333, -1.24401693586, -0.333333333333},
    {"F x/revolved:{0}/facet:0.5", -0.455341801261, -1.24401693586, -0.122008467928},
    {"F x/revolved:{0}/facet:0.6", -0.455341801261, -1.24401693586, 0.122008467928},
    {"F x/revolved:{0}/facet:0.7", -0.333333333333, -1.24401693586, 0.333333333333},
    {"F x/revolved:{0}/facet:0.8", -0.122008467928, -1.24401693586, 0.455341801261},
    {"F x/revolved:{0}/facet:0.9", 0.122008467928, -1.24401693586, 0.455341801261},
    {"F x/revolved:{0}/facet:1.0", 0.814458557658, -0.474216576937, -0.218233512793},
    {"F x/revolved:{0}/facet:1.1", 0.596225044865, -0.474216576937, -0.596225044865},
    {"F x/revolved:{0}/facet:1.10", 0.596225044865, -0.474216576937, 0.596225044865},
    {"F x/revolved:{0}/facet:1.11", 0.814458557658, -0.474216576937, 0.218233512793},
    {"F x/revolved:{0}/facet:1.2", 0.218233512793, -0.474216576937, -0.814458557658},
    {"F x/revolved:{0}/facet:1.3", -0.218233512793, -0.474216576937, -0.814458557658},
    {"F x/revolved:{0}/facet:1.4", -0.596225044865, -0.474216576937, -0.596225044865},
    {"F x/revolved:{0}/facet:1.5", -0.814458557658, -0.474216576937, -0.218233512793},
    {"F x/revolved:{0}/facet:1.6", -0.814458557658, -0.474216576937, 0.218233512793},
    {"F x/revolved:{0}/facet:1.7", -0.596225044865, -0.474216576937, 0.596225044865},
    {"F x/revolved:{0}/facet:1.8", -0.218233512793, -0.474216576937, 0.814458557658},
    {"F x/revolved:{0}/facet:1.9", 0.218233512793, -0.474216576937, 0.814458557658},
    {"F x/revolved:{0}/facet:2.0", 0.814458557658, 0.474216576937, -0.218233512793},
    {"F x/revolved:{0}/facet:2.1", 0.596225044865, 0.474216576937, -0.596225044865},
    {"F x/revolved:{0}/facet:2.10", 0.596225044865, 0.474216576937, 0.596225044865},
    {"F x/revolved:{0}/facet:2.11", 0.814458557658, 0.474216576937, 0.218233512793},
    {"F x/revolved:{0}/facet:2.2", 0.218233512793, 0.474216576937, -0.814458557658},
    {"F x/revolved:{0}/facet:2.3", -0.218233512793, 0.474216576937, -0.814458557658},
    {"F x/revolved:{0}/facet:2.4", -0.596225044865, 0.474216576937, -0.596225044865},
    {"F x/revolved:{0}/facet:2.5", -0.814458557658, 0.474216576937, -0.218233512793},
    {"F x/revolved:{0}/facet:2.6", -0.814458557658, 0.474216576937, 0.218233512793},
    {"F x/revolved:{0}/facet:2.7", -0.596225044865, 0.474216576937, 0.596225044865},
    {"F x/revolved:{0}/facet:2.8", -0.218233512793, 0.474216576937, 0.814458557658},
    {"F x/revolved:{0}/facet:2.9", 0.218233512793, 0.474216576937, 0.814458557658},
    {"F x/revolved:{0}/facet:3.0", 0.455341801261, 1.24401693586, -0.122008467928},
    {"F x/revolved:{0}/facet:3.1", 0.333333333333, 1.24401693586, -0.333333333333},
    {"F x/revolved:{0}/facet:3.10", 0.333333333333, 1.24401693586, 0.333333333333},
    {"F x/revolved:{0}/facet:3.11", 0.455341801261, 1.24401693586, 0.122008467928},
    {"F x/revolved:{0}/facet:3.2", 0.122008467928, 1.24401693586, -0.455341801261},
    {"F x/revolved:{0}/facet:3.3", -0.122008467928, 1.24401693586, -0.455341801261},
    {"F x/revolved:{0}/facet:3.4", -0.333333333333, 1.24401693586, -0.333333333333},
    {"F x/revolved:{0}/facet:3.5", -0.455341801261, 1.24401693586, -0.122008467928},
    {"F x/revolved:{0}/facet:3.6", -0.455341801261, 1.24401693586, 0.122008467928},
    {"F x/revolved:{0}/facet:3.7", -0.333333333333, 1.24401693586, 0.333333333333},
    {"F x/revolved:{0}/facet:3.8", -0.122008467928, 1.24401693586, 0.455341801261},
    {"F x/revolved:{0}/facet:3.9", 0.122008467928, 1.24401693586, 0.455341801261},
};
const Place kTurnedLemonQuarter12[] = {
    {"E x/edge:cap_end|cap_start", 2.22044604925e-16, 0, 0},
    {"E x/edge:cap_end|revolved:{0}/chord:0", 1.3343489442e-16, -1.36602540378, -0.366025403784},
    {"E x/edge:cap_end|revolved:{0}/chord:1", 5.30287619362e-17, -0.5, -0.866025403784},
    {"E x/edge:cap_end|revolved:{0}/chord:2", 5.30287619362e-17, 0.5, -0.866025403784},
    {"E x/edge:cap_end|revolved:{0}/chord:3", 1.3343489442e-16, 1.36602540378, -0.366025403784},
    {"E x/edge:cap_start|revolved:{0}/chord:0", 0.366025403784, -1.36602540378, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:1", 0.866025403784, -0.5, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:2", 0.866025403784, 0.5, 0},
    {"E x/edge:cap_start|revolved:{0}/chord:3", 0.366025403784, 1.36602540378, 0},
    {"E x/revolved:{0}/seam:0", 0.683012701892, -1, -0.183012701892},
    {"E x/revolved:{0}/seam:1", 0.316987298108, -1.36602540378, -0.183012701892},
    {"E x/revolved:{0}/seam:10", 0.683012701892, -4.89858719659e-16, -0.683012701892},
    {"E x/revolved:{0}/seam:11", 0.433012701892, 0.5, -0.75},
    {"E x/revolved:{0}/seam:12", 0.5, 1, -0.5},
    {"E x/revolved:{0}/seam:13", 0.183012701892, 1.36602540378, -0.316987298108},
    {"E x/revolved:{0}/seam:14", 0.183012701892, -1, -0.683012701892},
    {"E x/revolved:{0}/seam:15", 0.25, -4.89858719659e-16, -0.933012701892},
    {"E x/revolved:{0}/seam:16", 0.183012701892, 1, -0.683012701892},
    {"E x/revolved:{0}/seam:2", 0.75, -0.5, -0.433012701892},
    {"E x/revolved:{0}/seam:3", 0.933012701892, -4.89858719659e-16, -0.25},
    {"E x/revolved:{0}/seam:4", 0.75, 0.5, -0.433012701892},
    {"E x/revolved:{0}/seam:5", 0.683012701892, 1, -0.183012701892},
    {"E x/revolved:{0}/seam:6", 0.316987298108, 1.36602540378, -0.183012701892},
    {"E x/revolved:{0}/seam:7", 0.5, -1, -0.5},
    {"E x/revolved:{0}/seam:8", 0.183012701892, -1.36602540378, -0.316987298108},
    {"E x/revolved:{0}/seam:9", 0.433012701892, -0.5, -0.75},
    {"F x/cap_end", 1.3933597553e-16, -2.44763645572e-17, -0.390992597589},
    {"F x/cap_start", 0.390992597589, -4.89527291145e-17, 0},
    {"F x/revolved:{0}/facet:0.0", 0.455341801261, -1.24401693586, -0.122008467928},
    {"F x/revolved:{0}/facet:0.1", 0.333333333333, -1.24401693586, -0.333333333333},
    {"F x/revolved:{0}/facet:0.2", 0.122008467928, -1.24401693586, -0.455341801261},
    {"F x/revolved:{0}/facet:1.0", 0.814458557658, -0.474216576937, -0.218233512793},
    {"F x/revolved:{0}/facet:1.1", 0.596225044865, -0.474216576937, -0.596225044865},
    {"F x/revolved:{0}/facet:1.2", 0.218233512793, -0.474216576937, -0.814458557658},
    {"F x/revolved:{0}/facet:2.0", 0.814458557658, 0.474216576937, -0.218233512793},
    {"F x/revolved:{0}/facet:2.1", 0.596225044865, 0.474216576937, -0.596225044865},
    {"F x/revolved:{0}/facet:2.2", 0.218233512793, 0.474216576937, -0.814458557658},
    {"F x/revolved:{0}/facet:3.0", 0.455341801261, 1.24401693586, -0.122008467928},
    {"F x/revolved:{0}/facet:3.1", 0.333333333333, 1.24401693586, -0.333333333333},
    {"F x/revolved:{0}/facet:3.2", 0.122008467928, 1.24401693586, -0.455341801261},
};
const Place kTurnedApple12[] = {
    {"E x/revolved:{0}/seam:0", 0.933012701892, -2, -0.25},
    {"E x/revolved:{0}/seam:1", 0.5, -1.86602540378, 0},
    {"E x/revolved:{0}/seam:10", 2.79903810568, -4.89858719659e-16, -0.75},
    {"E x/revolved:{0}/seam:100", -1.29903810568, -1.86602540378, 0.75},
    {"E x/revolved:{0}/seam:101", -1.86602540378, -1.73205080757, 0.5},
    {"E x/revolved:{0}/seam:102", -2.04903810568, -1.36602540378, 1.18301270189},
    {"E x/revolved:{0}/seam:103", -2.54903810568, -1, 0.683012701892},
    {"E x/revolved:{0}/seam:104", -2.48205080757, -0.5, 1.43301270189},
    {"E x/revolved:{0}/seam:105", -2.79903810568, -4.89858719659e-16, 0.75},
    {"E x/revolved:{0}/seam:106", -2.48205080757, 0.5, 1.43301270189},
    {"E x/revolved:{0}/seam:107", -2.54903810568, 1, 0.683012701892},
    {"E x/revolved:{0}/seam:108", -2.04903810568, 1.36602540378, 1.18301270189},
    {"E x/revolved:{0}/seam:109", -1.86602540378, 1.73205080757, 0.5},
    {"E x/revolved:{0}/seam:11", 2.86602540378, -0.5, 0},
    {"E x/revolved:{0}/seam:110", -1.29903810568, 1.86602540378, 0.75},
    {"E x/revolved:{0}/seam:111", -0.933012701892, 2, 0.25},
    {"E x/revolved:{0}/seam:112", -0.433012701892, 1.86602540378, 0.25},
    {"E x/revolved:{0}/seam:113", -0.683012701892, -2, 0.683012701892},
    {"E x/revolved:{0}/seam:114", -0.25, -1.86602540378, 0.433012701892},
    {"E x/revolved:{0}/seam:115", -0.75, -1.86602540378, 1.29903810568},
    {"E x/revolved:{0}/seam:116", -1.36602540378, -1.73205080757, 1.36602540378},
    {"E x/revolved:{0}/seam:117", -1.18301270189, -1.36602540378, 2.04903810568},
    {"E x/revolved:{0}/seam:118", -1.86602540378, -1, 1.86602540378},
    {"E x/revolved:{0}/seam:119", -1.43301270189, -0.5, 2.48205080757},
    {"E x/revolved:{0}/seam:12", 2.48205080757, 0.5, -1.43301270189},
    {"E x/revolved:{0}/seam:120", -2.04903810568, -4.89858719659e-16, 2.04903810568},
    {"E x/revolved:{0}/seam:121", -1.43301270189, 0.5, 2.48205080757},
    {"E x/revolved:{0}/seam:122", -1.86602540378, 1, 1.86602540378},
    {"E x/revolved:{0}/seam:123", -1.18301270189, 1.36602540378, 2.04903810568},
    {"E x/revolved:{0}/seam:124", -1.36602540378, 1.73205080757, 1.36602540378},
    {"E x/revolved:{0}/seam:125", -0.75, 1.86602540378, 1.29903810568},
    {"E x/revolved:{0}/seam:126", -0.683012701892, 2, 0.683012701892},
    {"E x/revolved:{0}/seam:127", -0.25, 1.86602540378, 0.433012701892},
    {"E x/revolved:{0}/seam:128", -0.25, -2, 0.933012701892},
    {"E x/revolved:{0}/seam:129", 2.96729548683e-16, -1.86602540378, 0.5},
    {"E x/revolved:{0}/seam:13", 2.54903810568, 1, -0.683012701892},
    {"E x/revolved:{0}/seam:130", -2.75545529808e-16, -1.86602540378, 1.5},
    {"E x/revolved:{0}/seam:131", -0.5, -1.73205080757, 1.86602540378},
    {"E x/revolved:{0}/seam:132", -4.34631815617e-16, -1.36602540378, 2.36602540378},
    {"E x/revolved:{0}/seam:133", -0.683012701892, -1, 2.54903810568},
    {"E x/revolved:{0}/seam:134", -5.26480325553e-16, -0.5, 2.86602540378},
    {"E x/revolved:{0}/seam:135", -0.75, -4.89858719659e-16, 2.79903810568},
    {"E x/revolved:{0}/seam:136", -5.26480325553e-16, 0.5, 2.86602540378},
    {"E x/revolved:{0}/seam:137", -0.683012701892, 1, 2.54903810568},
    {"E x/revolved:{0}/seam:138", -4.34631815617e-16, 1.36602540378, 2.36602540378},
    {"E x/revolved:{0}/seam:139", -0.5, 1.73205080757, 1.86602540378},
    {"E x/revolved:{0}/seam:14", 2.86602540378, 0.5, 0},
    {"E x/revolved:{0}/seam:140", -2.75545529808e-16, 1.86602540378, 1.5},
    {"E x/revolved:{0}/seam:141", -0.25, 2, 0.933012701892},
    {"E x/revolved:{0}/seam:142", 1.30196094989e-16, 1.86602540378, 0.5},
    {"E x/revolved:{0}/seam:143", 0.25, -2, 0.933012701892},
    {"E x/revolved:{0}/seam:144", 0.25, -1.86602540378, 0.433012701892},
    {"E x/revolved:{0}/seam:145", 0.75, -1.86602540378, 1.29903810568},
    {"E x/revolved:{0}/seam:146", 0.5, -1.73205080757, 1.86602540378},
    {"E x/revolved:{0}/seam:147", 1.18301270189, -1.36602540378, 2.04903810568},
    {"E x/revolved:{0}/seam:148", 0.683012701892, -1, 2.54903810568},
    {"E x/revolved:{0}/seam:149", 1.43301270189, -0.5, 2.48205080757},
    {"E x/revolved:{0}/seam:15", 2.04903810568, 1.36602540378, -1.18301270189},
    {"E x/revolved:{0}/seam:150", 0.75, -4.89858719659e-16, 2.79903810568},
    {"E x/revolved:{0}/seam:151", 1.43301270189, 0.5, 2.48205080757},
    {"E x/revolved:{0}/seam:152", 0.683012701892, 1, 2.54903810568},
    {"E x/revolved:{0}/seam:153", 1.18301270189, 1.36602540378, 2.04903810568},
    {"E x/revolved:{0}/seam:154", 0.5, 1.73205080757, 1.86602540378},
    {"E x/revolved:{0}/seam:155", 0.75, 1.86602540378, 1.29903810568},
    {"E x/revolved:{0}/seam:156", 0.25, 2, 0.933012701892},
    {"E x/revolved:{0}/seam:157", 0.25, 1.86602540378, 0.433012701892},
    {"E x/revolved:{0}/seam:158", 0.683012701892, -2, 0.683012701892},
    {"E x/revolved:{0}/seam:159", 0.433012701892, -1.86602540378, 0.25},
    {"E x/revolved:{0}/seam:16", 1.86602540378, 1.73205080757, -0.5},
    {"E x/revolved:{0}/seam:160", 1.29903810568, -1.86602540378, 0.75},
    {"E x/revolved:{0}/seam:161", 1.36602540378, -1.73205080757, 1.36602540378},
    {"E x/revolved:{0}/seam:162", 2.04903810568, -1.36602540378, 1.18301270189},
    {"E x/revolved:{0}/seam:163", 1.86602540378, -1, 1.86602540378},
    {"E x/revolved:{0}/seam:164", 2.48205080757, -0.5, 1.43301270189},
    {"E x/revolved:{0}/seam:165", 2.04903810568, -4.89858719659e-16, 2.04903810568},
    {"E x/revolved:{0}/seam:166", 2.48205080757, 0.5, 1.43301270189},
    {"E x/revolved:{0}/seam:167", 1.86602540378, 1, 1.86602540378},
    {"E x/revolved:{0}/seam:168", 2.04903810568, 1.36602540378, 1.18301270189},
    {"E x/revolved:{0}/seam:169", 1.36602540378, 1.73205080757, 1.36602540378},
    {"E x/revolved:{0}/seam:17", 2.36602540378, 1.36602540378, 0},
    {"E x/revolved:{0}/seam:170", 1.29903810568, 1.86602540378, 0.75},
    {"E x/revolved:{0}/seam:171", 0.683012701892, 2, 0.683012701892},
    {"E x/revolved:{0}/seam:172", 0.433012701892, 1.86602540378, 0.25},
    {"E x/revolved:{0}/seam:173", 0.933012701892, -2, 0.25},
    {"E x/revolved:{0}/seam:174", 1.86602540378, -1.73205080757, 0.5},
    {"E x/revolved:{0}/seam:175", 2.54903810568, -1, 0.683012701892},
    {"E x/revolved:{0}/seam:176", 2.79903810568, -4.89858719659e-16, 0.75},
    {"E x/revolved:{0}/seam:177", 2.54903810568, 1, 0.683012701892},
    {"E x/revolved:{0}/seam:178", 1.86602540378, 1.73205080757, 0.5},
    {"E x/revolved:{0}/seam:179", 0.933012701892, 2, 0.25},
    {"E x/revolved:{0}/seam:18", 1.29903810568, 1.86602540378, -0.75},
    {"E x/revolved:{0}/seam:19", 0.933012701892, 2, -0.25},
    {"E x/revolved:{0}/seam:2", 0.433012701892, -1.86602540378, -0.25},
    {"E x/revolved:{0}/seam:20", 1.5, 1.86602540378, 0},
    {"E x/revolved:{0}/seam:21", 0.433012701892, 1.86602540378, -0.25},
    {"E x/revolved:{0}/seam:22", 0.5, 1.86602540378, 0},
    {"E x/revolved:{0}/seam:23", 0.683012701892, -2, -0.683012701892},
    {"E x/revolved:{0}/seam:24", 0.25, -1.86602540378, -0.433012701892},
    {"E x/revolved:{0}/seam:25", 0.75, -1.86602540378, -1.29903810568},
    {"E x/revolved:{0}/seam:26", 1.36602540378, -1.73205080757, -1.36602540378},
    {"E x/revolved:{0}/seam:27", 1.18301270189, -1.36602540378, -2.04903810568},
    {"E x/revolved:{0}/seam:28", 1.86602540378, -1, -1.86602540378},
    {"E x/revolved:{0}/seam:29", 1.43301270189, -0.5, -2.48205080757},
    {"E x/revolved:{0}/seam:3", 1.29903810568, -1.86602540378, -0.75},
    {"E x/revolved:{0}/seam:30", 2.04903810568, -4.89858719659e-16, -2.04903810568},
    {"E x/revolved:{0}/seam:31", 1.43301270189, 0.5, -2.48205080757},
    {"E x/revolved:{0}/seam:32", 1.86602540378, 1, -1.86602540378},
    {"E x/revolved:{0}/seam:33", 1.18301270189, 1.36602540378, -2.04903810568},
    {"E x/revolved:{0}/seam:34", 1.36602540378, 1.73205080757, -1.36602540378},
    {"E x/revolved:{0}/seam:35", 0.75, 1.86602540378, -1.29903810568},
    {"E x/revolved:{0}/seam:36", 0.683012701892, 2, -0.683012701892},
    {"E x/revolved:{0}/seam:37", 0.25, 1.86602540378, -0.433012701892},
    {"E x/revolved:{0}/seam:38", 0.25, -2, -0.933012701892},
    {"E x/revolved:{0}/seam:39", 4.19194228597e-16, -1.86602540378, -0.5},
    {"E x/revolved:{0}/seam:4", 1.86602540378, -1.73205080757, -0.5},
    {"E x/revolved:{0}/seam:40", 9.18485099361e-17, -1.86602540378, -1.5},
    {"E x/revolved:{0}/seam:41", 0.5, -1.73205080757, -1.86602540378},
    {"E x/revolved:{0}/seam:42", 1.44877271872e-16, -1.36602540378, -2.36602540378},
    {"E x/revolved:{0}/seam:43", 0.683012701892, -1, -2.54903810568},
    {"E x/revolved:{0}/seam:44", 1.75493441851e-16, -0.5, -2.86602540378},
    {"E x/revolved:{0}/seam:45", 0.75, -4.89858719659e-16, -2.79903810568},
    {"E x/revolved:{0}/seam:46", 1.75493441851e-16, 0.5, -2.86602540378},
    {"E x/revolved:{0}/seam:47", 0.683012701892, 1, -2.54903810568},
    {"E x/revolved:{0}/seam:48", 1.44877271872e-16, 1.36602540378, -2.36602540378},
    {"E x/revolved:{0}/seam:49", 0.5, 1.73205080757, -1.86602540378},
    {"E x/revolved:{0}/seam:5", 1.5, -1.86602540378, 0},
    {"E x/revolved:{0}/seam:50", 9.18485099361e-17, 1.86602540378, -1.5},
    {"E x/revolved:{0}/seam:51", 0.25, 2, -0.933012701892},
    {"E x/revolved:{0}/seam:52", 2.52660774904e-16, 1.86602540378, -0.5},
    {"E x/revolved:{0}/seam:53", -0.25, -2, -0.933012701892},
    {"E x/revolved:{0}/seam:54", -0.25, -1.86602540378, -0.433012701892},
    {"E x/revolved:{0}/seam:55", -0.75, -1.86602540378, -1.29903810568},
    {"E x/revolved:{0}/seam:56", -0.5, -1.73205080757, -1.86602540378},
    {"E x/revolved:{0}/seam:57", -1.18301270189, -1.36602540378, -2.04903810568},
    {"E x/revolved:{0}/seam:58", -0.683012701892, -1, -2.54903810568},
    {"E x/revolved:{0}/seam:59", -1.43301270189, -0.5, -2.48205080757},
    {"E x/revolved:{0}/seam:6", 2.04903810568, -1.36602540378, -1.18301270189},
    {"E x/revolved:{0}/seam:60", -0.75, -4.89858719659e-16, -2.79903810568},
    {"E x/revolved:{0}/seam:61", -1.43301270189, 0.5, -2.48205080757},
    {"E x/revolved:{0}/seam:62", -0.683012701892, 1, -2.54903810568},
    {"E x/revolved:{0}/seam:63", -1.18301270189, 1.36602540378, -2.04903810568},
    {"E x/revolved:{0}/seam:64", -0.5, 1.73205080757, -1.86602540378},
    {"E x/revolved:{0}/seam:65", -0.75, 1.86602540378, -1.29903810568},
    {"E x/revolved:{0}/seam:66", -0.25, 2, -0.933012701892},
    {"E x/revolved:{0}/seam:67", -0.25, 1.86602540378, -0.433012701892},
    {"E x/revolved:{0}/seam:68", -0.683012701892, -2, -0.683012701892},
    {"E x/revolved:{0}/seam:69", -0.433012701892, -1.86602540378, -0.25},
    {"E x/revolved:{0}/seam:7", 2.54903810568, -1, -0.683012701892},
    {"E x/revolved:{0}/seam:70", -1.29903810568, -1.86602540378, -0.75},
    {"E x/revolved:{0}/seam:71", -1.36602540378, -1.73205080757, -1.36602540378},
    {"E x/revolved:{0}/seam:72", -2.04903810568, -1.36602540378, -1.18301270189},
    {"E x/revolved:{0}/seam:73", -1.86602540378, -1, -1.86602540378},
    {"E x/revolved:{0}/seam:74", -2.48205080757, -0.5, -1.43301270189},
    {"E x/revolved:{0}/seam:75", -2.04903810568, -4.89858719659e-16, -2.04903810568},
    {"E x/revolved:{0}/seam:76", -2.48205080757, 0.5, -1.43301270189},
    {"E x/revolved:{0}/seam:77", -1.86602540378, 1, -1.86602540378},
    {"E x/revolved:{0}/seam:78", -2.04903810568, 1.36602540378, -1.18301270189},
    {"E x/revolved:{0}/seam:79", -1.36602540378, 1.73205080757, -1.36602540378},
    {"E x/revolved:{0}/seam:8", 2.36602540378, -1.36602540378, 0},
    {"E x/revolved:{0}/seam:80", -1.29903810568, 1.86602540378, -0.75},
    {"E x/revolved:{0}/seam:81", -0.683012701892, 2, -0.683012701892},
    {"E x/revolved:{0}/seam:82", -0.433012701892, 1.86602540378, -0.25},
    {"E x/revolved:{0}/seam:83", -0.933012701892, -2, -0.25},
    {"E x/revolved:{0}/seam:84", -0.5, -1.86602540378, -6.12323399574e-17},
    {"E x/revolved:{0}/seam:85", -1.5, -1.86602540378, -1.83697019872e-16},
    {"E x/revolved:{0}/seam:86", -1.86602540378, -1.73205080757, -0.5},
    {"E x/revolved:{0}/seam:87", -2.36602540378, -1.36602540378, -2.89754543745e-16},
    {"E x/revolved:{0}/seam:88", -2.54903810568, -1, -0.683012701892},
    {"E x/revolved:{0}/seam:89", -2.86602540378, -0.5, -3.50986883702e-16},
    {"E x/revolved:{0}/seam:9", 2.48205080757, -0.5, -1.43301270189},
    {"E x/revolved:{0}/seam:90", -2.79903810568, -4.89858719659e-16, -0.75},
    {"E x/revolved:{0}/seam:91", -2.86602540378, 0.5, -3.50986883702e-16},
    {"E x/revolved:{0}/seam:92", -2.54903810568, 1, -0.683012701892},
    {"E x/revolved:{0}/seam:93", -2.36602540378, 1.36602540378, -2.89754543745e-16},
    {"E x/revolved:{0}/seam:94", -1.86602540378, 1.73205080757, -0.5},
    {"E x/revolved:{0}/seam:95", -1.5, 1.86602540378, -1.83697019872e-16},
    {"E x/revolved:{0}/seam:96", -0.933012701892, 2, -0.25},
    {"E x/revolved:{0}/seam:97", -0.5, 1.86602540378, -6.12323399574e-17},
    {"E x/revolved:{0}/seam:98", -0.933012701892, -2, 0.25},
    {"E x/revolved:{0}/seam:99", -0.433012701892, -1.86602540378, 0.25},
    {"F x/revolved:{0}/facet:0.0", 0.622008467928, -1.91068360252, -0.166666666667},
    {"F x/revolved:{0}/facet:0.1", 0.455341801261, -1.91068360252, -0.455341801261},
    {"F x/revolved:{0}/facet:0.10", 0.455341801261, -1.91068360252, 0.455341801261},
    {"F x/revolved:{0}/facet:0.11", 0.622008467928, -1.91068360252, 0.166666666667},
    {"F x/revolved:{0}/facet:0.2", 0.166666666667, -1.91068360252, -0.622008467928},
    {"F x/revolved:{0}/facet:0.3", -0.166666666667, -1.91068360252, -0.622008467928},
    {"F x/revolved:{0}/facet:0.4", -0.455341801261, -1.91068360252, -0.455341801261},
    {"F x/revolved:{0}/facet:0.5", -0.622008467928, -1.91068360252, -0.166666666667},
    {"F x/revolved:{0}/facet:0.6", -0.622008467928, -1.91068360252, 0.166666666667},
    {"F x/revolved:{0}/facet:0.7", -0.455341801261, -1.91068360252, 0.455341801261},
    {"F x/revolved:{0}/facet:0.8", -0.166666666667, -1.91068360252, 0.622008467928},
    {"F x/revolved:{0}/facet:0.9", 0.166666666667, -1.91068360252, 0.622008467928},
    {"F x/revolved:{0}/facet:1.0", 1.45135309183, -1.85113933754, -0.388888888889},
    {"F x/revolved:{0}/facet:1.1", 1.06246420294, -1.85113933754, -1.06246420294},
    {"F x/revolved:{0}/facet:1.10", 1.06246420294, -1.85113933754, 1.06246420294},
    {"F x/revolved:{0}/facet:1.11", 1.45135309183, -1.85113933754, 0.388888888889},
    {"F x/revolved:{0}/facet:1.2", 0.388888888889, -1.85113933754, -1.45135309183},
    {"F x/revolved:{0}/facet:1.3", -0.388888888889, -1.85113933754, -1.45135309183},
    {"F x/revolved:{0}/facet:1.4", -1.06246420294, -1.85113933754, -1.06246420294},
    {"F x/revolved:{0}/facet:1.5", -1.45135309183, -1.85113933754, -0.388888888889},
    {"F x/revolved:{0}/facet:1.6", -1.45135309183, -1.85113933754, 0.388888888889},
    {"F x/revolved:{0}/facet:1.7", -1.06246420294, -1.85113933754, 1.06246420294},
    {"F x/revolved:{0}/facet:1.8", -0.388888888889, -1.85113933754, 1.45135309183},
    {"F x/revolved:{0}/facet:1.9", 0.388888888889, -1.85113933754, 1.45135309183},
    {"F x/revolved:{0}/facet:2.0", 2.22514216018, -1.34715062811, -0.596225044865},
    {"F x/revolved:{0}/facet:2.1", 1.62891711532, -1.34715062811, -1.62891711532},
    {"F x/revolved:{0}/facet:2.10", 1.62891711532, -1.34715062811, 1.62891711532},
    {"F x/revolved:{0}/facet:2.11", 2.22514216018, -1.34715062811, 0.596225044865},
    {"F x/revolved:{0}/facet:2.2", 0.596225044865, -1.34715062811, -2.22514216018},
    {"F x/revolved:{0}/facet:2.3", -0.596225044865, -1.34715062811, -2.22514216018},
    {"F x/revolved:{0}/facet:2.4", -1.62891711532, -1.34715062811, -1.62891711532},
    {"F x/revolved:{0}/facet:2.5", -2.22514216018, -1.34715062811, -0.596225044865},
    {"F x/revolved:{0}/facet:2.6", -2.22514216018, -1.34715062811, 0.596225044865},
    {"F x/revolved:{0}/facet:2.7", -1.62891711532, -1.34715062811, 1.62891711532},
    {"F x/revolved:{0}/facet:2.8", -0.596225044865, -1.34715062811, 2.22514216018},
    {"F x/revolved:{0}/facet:2.9", 0.596225044865, -1.34715062811, 2.22514216018},
    {"F x/revolved:{0}/facet:3.0", 2.67598584656, -0.49220903648, -0.717028246542},
    {"F x/revolved:{0}/facet:3.1", 1.95895760001, -0.49220903648, -1.95895760001},
    {"F x/revolved:{0}/facet:3.10", 1.95895760001, -0.49220903648, 1.95895760001},
    {"F x/revolved:{0}/facet:3.11", 2.67598584656, -0.49220903648, 0.717028246542},
    {"F x/revolved:{0}/facet:3.2", 0.717028246542, -0.49220903648, -2.67598584656},
    {"F x/revolved:{0}/facet:3.3", -0.717028246542, -0.49220903648, -2.67598584656},
    {"F x/revolved:{0}/facet:3.4", -1.95895760001, -0.49220903648, -1.95895760001},
    {"F x/revolved:{0}/facet:3.5", -2.67598584656, -0.49220903648, -0.717028246542},
    {"F x/revolved:{0}/facet:3.6", -2.67598584656, -0.49220903648, 0.717028246542},
    {"F x/revolved:{0}/facet:3.7", -1.95895760001, -0.49220903648, 1.95895760001},
    {"F x/revolved:{0}/facet:3.8", -0.717028246542, -0.49220903648, 2.67598584656},
    {"F x/revolved:{0}/facet:3.9", 0.717028246542, -0.49220903648, 2.67598584656},
    {"F x/revolved:{0}/facet:4.0", 2.67598584656, 0.49220903648, -0.717028246542},
    {"F x/revolved:{0}/facet:4.1", 1.95895760001, 0.49220903648, -1.95895760001},
    {"F x/revolved:{0}/facet:4.10", 1.95895760001, 0.49220903648, 1.95895760001},
    {"F x/revolved:{0}/facet:4.11", 2.67598584656, 0.49220903648, 0.717028246542},
    {"F x/revolved:{0}/facet:4.2", 0.717028246542, 0.49220903648, -2.67598584656},
    {"F x/revolved:{0}/facet:4.3", -0.717028246542, 0.49220903648, -2.67598584656},
    {"F x/revolved:{0}/facet:4.4", -1.95895760001, 0.49220903648, -1.95895760001},
    {"F x/revolved:{0}/facet:4.5", -2.67598584656, 0.49220903648, -0.717028246542},
    {"F x/revolved:{0}/facet:4.6", -2.67598584656, 0.49220903648, 0.717028246542},
    {"F x/revolved:{0}/facet:4.7", -1.95895760001, 0.49220903648, 1.95895760001},
    {"F x/revolved:{0}/facet:4.8", -0.717028246542, 0.49220903648, 2.67598584656},
    {"F x/revolved:{0}/facet:4.9", 0.717028246542, 0.49220903648, 2.67598584656},
    {"F x/revolved:{0}/facet:5.0", 2.22514216018, 1.34715062811, -0.596225044865},
    {"F x/revolved:{0}/facet:5.1", 1.62891711532, 1.34715062811, -1.62891711532},
    {"F x/revolved:{0}/facet:5.10", 1.62891711532, 1.34715062811, 1.62891711532},
    {"F x/revolved:{0}/facet:5.11", 2.22514216018, 1.34715062811, 0.596225044865},
    {"F x/revolved:{0}/facet:5.2", 0.596225044865, 1.34715062811, -2.22514216018},
    {"F x/revolved:{0}/facet:5.3", -0.596225044865, 1.34715062811, -2.22514216018},
    {"F x/revolved:{0}/facet:5.4", -1.62891711532, 1.34715062811, -1.62891711532},
    {"F x/revolved:{0}/facet:5.5", -2.22514216018, 1.34715062811, -0.596225044865},
    {"F x/revolved:{0}/facet:5.6", -2.22514216018, 1.34715062811, 0.596225044865},
    {"F x/revolved:{0}/facet:5.7", -1.62891711532, 1.34715062811, 1.62891711532},
    {"F x/revolved:{0}/facet:5.8", -0.596225044865, 1.34715062811, 2.22514216018},
    {"F x/revolved:{0}/facet:5.9", 0.596225044865, 1.34715062811, 2.22514216018},
    {"F x/revolved:{0}/facet:6.0", 1.45135309183, 1.85113933754, -0.388888888889},
    {"F x/revolved:{0}/facet:6.1", 1.06246420294, 1.85113933754, -1.06246420294},
    {"F x/revolved:{0}/facet:6.10", 1.06246420294, 1.85113933754, 1.06246420294},
    {"F x/revolved:{0}/facet:6.11", 1.45135309183, 1.85113933754, 0.388888888889},
    {"F x/revolved:{0}/facet:6.2", 0.388888888889, 1.85113933754, -1.45135309183},
    {"F x/revolved:{0}/facet:6.3", -0.388888888889, 1.85113933754, -1.45135309183},
    {"F x/revolved:{0}/facet:6.4", -1.06246420294, 1.85113933754, -1.06246420294},
    {"F x/revolved:{0}/facet:6.5", -1.45135309183, 1.85113933754, -0.388888888889},
    {"F x/revolved:{0}/facet:6.6", -1.45135309183, 1.85113933754, 0.388888888889},
    {"F x/revolved:{0}/facet:6.7", -1.06246420294, 1.85113933754, 1.06246420294},
    {"F x/revolved:{0}/facet:6.8", -0.388888888889, 1.85113933754, 1.45135309183},
    {"F x/revolved:{0}/facet:6.9", 0.388888888889, 1.85113933754, 1.45135309183},
    {"F x/revolved:{0}/facet:7.0", 0.622008467928, 1.91068360252, -0.166666666667},
    {"F x/revolved:{0}/facet:7.1", 0.455341801261, 1.91068360252, -0.455341801261},
    {"F x/revolved:{0}/facet:7.10", 0.455341801261, 1.91068360252, 0.455341801261},
    {"F x/revolved:{0}/facet:7.11", 0.622008467928, 1.91068360252, 0.166666666667},
    {"F x/revolved:{0}/facet:7.2", 0.166666666667, 1.91068360252, -0.622008467928},
    {"F x/revolved:{0}/facet:7.3", -0.166666666667, 1.91068360252, -0.622008467928},
    {"F x/revolved:{0}/facet:7.4", -0.455341801261, 1.91068360252, -0.455341801261},
    {"F x/revolved:{0}/facet:7.5", -0.622008467928, 1.91068360252, -0.166666666667},
    {"F x/revolved:{0}/facet:7.6", -0.622008467928, 1.91068360252, 0.166666666667},
    {"F x/revolved:{0}/facet:7.7", -0.455341801261, 1.91068360252, 0.455341801261},
    {"F x/revolved:{0}/facet:7.8", -0.166666666667, 1.91068360252, 0.622008467928},
    {"F x/revolved:{0}/facet:7.9", 0.166666666667, 1.91068360252, 0.622008467928},
};

const Place kTurnedOgive12[] = {
    {"E x/edge:revolved:{0}|revolved:{2}/chord:0", 1.86602540378, 0, -0.5},
    {"E x/edge:revolved:{0}|revolved:{2}/chord:1", 1.36602540378, 0, -1.36602540378},
    {"E x/edge:revolved:{0}|revolved:{2}/chord:10", 1.36602540378, 0, 1.36602540378},
    {"E x/edge:revolved:{0}|revolved:{2}/chord:11", 1.86602540378, 0, 0.5},
    {"E x/edge:revolved:{0}|revolved:{2}/chord:2", 0.5, 0, -1.86602540378},
    {"E x/edge:revolved:{0}|revolved:{2}/chord:3", -0.5, 0, -1.86602540378},
    {"E x/edge:revolved:{0}|revolved:{2}/chord:4", -1.36602540378, 0, -1.36602540378},
    {"E x/edge:revolved:{0}|revolved:{2}/chord:5", -1.86602540378, 0, -0.5},
    {"E x/edge:revolved:{0}|revolved:{2}/chord:6", -1.86602540378, 0, 0.5},
    {"E x/edge:revolved:{0}|revolved:{2}/chord:7", -1.36602540378, 0, 1.36602540378},
    {"E x/edge:revolved:{0}|revolved:{2}/chord:8", -0.5, 0, 1.86602540378},
    {"E x/edge:revolved:{0}|revolved:{2}/chord:9", 0.5, 0, 1.86602540378},
    {"E x/revolved:{0}/seam:0", 1.50347897121, 1.11803398875, -0.86803398875},
    {"E x/revolved:{0}/seam:1", 1.37352154493, 2.2360679775, -0.36803398875},
    {"E x/revolved:{0}/seam:10", 2.67115769555e-16, 3.11803398875, -0.7360679775},
    {"E x/revolved:{0}/seam:11", -0.86803398875, 1.11803398875, -1.50347897121},
    {"E x/revolved:{0}/seam:12", -0.36803398875, 2.2360679775, -1.37352154493},
    {"E x/revolved:{0}/seam:13", -0.36803398875, 3.11803398875, -0.637453567427},
    {"E x/revolved:{0}/seam:14", -1.50347897121, 1.11803398875, -0.86803398875},
    {"E x/revolved:{0}/seam:15", -1.00548755618, 2.2360679775, -1.00548755618},
    {"E x/revolved:{0}/seam:16", -0.637453567427, 3.11803398875, -0.36803398875},
    {"E x/revolved:{0}/seam:17", -1.7360679775, 1.11803398875, -2.12607009175e-16},
    {"E x/revolved:{0}/seam:18", -1.37352154493, 2.2360679775, -0.36803398875},
    {"E x/revolved:{0}/seam:19", -0.7360679775, 3.11803398875, -9.014232926e-17},
    {"E x/revolved:{0}/seam:2", 1.7360679775, 1.11803398875, 0},
    {"E x/revolved:{0}/seam:20", -1.50347897121, 1.11803398875, 0.86803398875},
    {"E x/revolved:{0}/seam:21", -1.37352154493, 2.2360679775, 0.36803398875},
    {"E x/revolved:{0}/seam:22", -0.637453567427, 3.11803398875, 0.36803398875},
    {"E x/revolved:{0}/seam:23", -0.86803398875, 1.11803398875, 1.50347897121},
    {"E x/revolved:{0}/seam:24", -1.00548755618, 2.2360679775, 1.00548755618},
    {"E x/revolved:{0}/seam:25", -0.36803398875, 3.11803398875, 0.637453567427},
    {"E x/revolved:{0}/seam:26", -3.18910513762e-16, 1.11803398875, 1.7360679775},
    {"E x/revolved:{0}/seam:27", -0.36803398875, 2.2360679775, 1.37352154493},
    {"E x/revolved:{0}/seam:28", 8.6831111035e-17, 3.11803398875, 0.7360679775},
    {"E x/revolved:{0}/seam:29", 0.86803398875, 1.11803398875, 1.50347897121},
    {"E x/revolved:{0}/seam:3", 0.637453567427, 3.11803398875, -0.36803398875},
    {"E x/revolved:{0}/seam:30", 0.36803398875, 2.2360679775, 1.37352154493},
    {"E x/revolved:{0}/seam:31", 0.36803398875, 3.11803398875, 0.637453567427},
    {"E x/revolved:{0}/seam:32", 1.50347897121, 1.11803398875, 0.86803398875},
    {"E x/revolved:{0}/seam:33", 1.00548755618, 2.2360679775, 1.00548755618},
    {"E x/revolved:{0}/seam:34", 0.637453567427, 3.11803398875, 0.36803398875},
    {"E x/revolved:{0}/seam:35", 1.37352154493, 2.2360679775, 0.36803398875},
    {"E x/revolved:{0}/seam:4", 0.7360679775, 3.11803398875, 0},
    {"E x/revolved:{0}/seam:5", 0.86803398875, 1.11803398875, -1.50347897121},
    {"E x/revolved:{0}/seam:6", 1.00548755618, 2.2360679775, -1.00548755618},
    {"E x/revolved:{0}/seam:7", 0.36803398875, 3.11803398875, -0.637453567427},
    {"E x/revolved:{0}/seam:8", 1.06303504587e-16, 1.11803398875, -1.7360679775},
    {"E x/revolved:{0}/seam:9", 0.36803398875, 2.2360679775, -1.37352154493},
    {"F x/revolved:{0}/facet:0.0", 1.6322525882, 1.06137626649, -0.437360762851},
    {"F x/revolved:{0}/facet:0.1", 1.19489182535, 1.06137626649, -1.19489182535},
    {"F x/revolved:{0}/facet:0.10", 1.19489182535, 1.06137626649, 1.19489182535},
    {"F x/revolved:{0}/facet:0.11", 1.6322525882, 1.06137626649, 0.437360762851},
    {"F x/revolved:{0}/facet:0.2", 0.437360762851, 1.06137626649, -1.6322525882},
    {"F x/revolved:{0}/facet:0.3", -0.437360762851, 1.06137626649, -1.6322525882},
    {"F x/revolved:{0}/facet:0.4", -1.19489182535, 1.06137626649, -1.19489182535},
    {"F x/revolved:{0}/facet:0.5", -1.6322525882, 1.06137626649, -0.437360762851},
    {"F x/revolved:{0}/facet:0.6", -1.6322525882, 1.06137626649, 0.437360762851},
    {"F x/revolved:{0}/facet:0.7", -1.19489182535, 1.06137626649, 1.19489182535},
    {"F x/revolved:{0}/facet:0.8", -0.437360762851, 1.06137626649, 1.6322525882},
    {"F x/revolved:{0}/facet:0.9", 0.437360762851, 1.06137626649, 1.6322525882},
    {"F x/revolved:{0}/facet:1.0", 0.915681029951, 2.82404531833, -0.2453559925},
    {"F x/revolved:{0}/facet:1.1", 0.670325037451, 2.82404531833, -0.670325037451},
    {"F x/revolved:{0}/facet:1.10", 0.670325037451, 2.82404531833, 0.670325037451},
    {"F x/revolved:{0}/facet:1.11", 0.915681029951, 2.82404531833, 0.2453559925},
    {"F x/revolved:{0}/facet:1.2", 0.2453559925, 2.82404531833, -0.915681029951},
    {"F x/revolved:{0}/facet:1.3", -0.2453559925, 2.82404531833, -0.915681029951},
    {"F x/revolved:{0}/facet:1.4", -0.670325037451, 2.82404531833, -0.670325037451},
    {"F x/revolved:{0}/facet:1.5", -0.915681029951, 2.82404531833, -0.2453559925},
    {"F x/revolved:{0}/facet:1.6", -0.915681029951, 2.82404531833, 0.2453559925},
    {"F x/revolved:{0}/facet:1.7", -0.670325037451, 2.82404531833, 0.670325037451},
    {"F x/revolved:{0}/facet:1.8", -0.2453559925, 2.82404531833, 0.915681029951},
    {"F x/revolved:{0}/facet:1.9", 0.2453559925, 2.82404531833, 0.915681029951},
    {"F x/revolved:{2}", -3.23815048849e-17, 0, 6.29482353029e-17},
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

/// @p profile turned by @p angle about the sketch's y axis at @p segments a turn (8 unless said),
/// "x": its exact form and its bridge copy.
hz::model::Revolve::ExactAndBridge revolved(const Profile& profile, double angle,
                                            int segments = 8) {
    std::string why;
    auto out = hz::model::Revolve::exactAndBridge(profile, hz::draft::SketchPlane(), Vec3(),
                                                  Vec3(0, 1, 0), angle, "x", segments, 0.0, &why);
    EXPECT_NE(out.faceted, nullptr) << why;
    // Revolve::execute gives the same bridge, named alike.
    const auto executed =
        hz::model::Revolve::execute(profile, hz::draft::SketchPlane(), Vec3(), Vec3(0, 1, 0), angle,
                                    "x", segments, 0.0, &why, hz::model::NamingScheme::Stable);
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

// The revolves' profiles, as the tables were recorded from.

Profile rectangle() {
    return {line({0, 0}, {3, 0}), line({3, 0}, {3, 5}), line({3, 5}, {0, 5}), line({0, 5}, {0, 0})};
}

Profile triangle() {
    return {line({0, 0}, {3, 0}), line({3, 0}, {0, 4}), line({0, 4}, {0, 0})};
}

Profile semicircle() {
    return {arc({0, 0}, 2, -kPi / 2, kPi / 2), line({0, 2}, {0, -2})};
}

Profile lemon() {
    auto bulge = std::make_shared<hz::draft::DraftArc>(Vec2(-1, 0), 2, -kPi / 3, kPi / 3);
    const Vec2 from = bulge->startPoint();
    const Vec2 to = bulge->endPoint();
    return {bulge, line(to, from)};
}

Profile apple() {
    auto round = std::make_shared<hz::draft::DraftArc>(Vec2(1, 0), 2, -2 * kPi / 3, 2 * kPi / 3);
    const Vec2 from = round->startPoint();
    const Vec2 to = round->endPoint();
    return {round, line(to, from)};
}

/// A tangent ogive, 4 high on a base of radius 2: an arc of radius 5 about (-3, 0), across the
/// axis, from the base's rim to the tip on the axis, then down the axis and out along the base.
Profile ogive() {
    auto nose = std::make_shared<hz::draft::DraftArc>(Vec2(-3, 0), 5, 0.0, std::atan2(4.0, 3.0));
    const Vec2 tip = nose->endPoint();
    return {nose, line(tip, {0, 0}), line({0, 0}, {2, 0})};
}

Profile poleAndApex() {
    return {arc({0, 0}, 3, -kPi / 2, 0), line({3, 0}, {0, 4}), line({0, 4}, {0, -3})};
}

Profile washer() {
    return {line({2, 0}, {4, 0}), line({4, 0}, {4, 3}), line({4, 3}, {2, 3}), line({2, 3}, {2, 0})};
}

Profile holed() {
    return {line({1, 0}, {5, 0}), line({5, 0}, {5, 4}), line({5, 4}, {1, 4}), line({1, 4}, {1, 0}),
            circle({3, 2}, 1)};
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

// Revolves (Ruling 21): each face, chord and seam master's Stable revolve named, the bridge names
// alike, at the same place. Through a whole turn: a cylinder with discs, an apex cone, a ball, a
// torus, a lemon, a washer's rings and a hemisphere under a cone (a pole and an apex); through
// part of one, a ball's lune, a torus's sector and the hemisphere and cone a quarter turn and 359.9
// degrees. A curved face's exact seam of several chords (the profile's arc at u = 0, and a torus's
// rim at its corner, which its face meets on both sides) is named after master's edge at its first
// chord, its chords `<seam>/chord:<k>`, as a primitive's are: master's names of those chords, each
// a seam of its own between two of master's facets, are gone.
TEST(BridgeNumbering, RevolvesAreNumberedAsMastersWere) {
    struct Case {
        const char* what;
        Profile profile;
        double angle;
        const Place* table;
        std::size_t size;
        std::set<std::string> gone;
    };
    std::vector<Case> cases;
    cases.push_back(
        {"a rectangle", rectangle(), 2 * kPi, kTurnedRectangle, std::size(kTurnedRectangle), {}});
    cases.push_back(
        {"a triangle", triangle(), 2 * kPi, kTurnedTriangle, std::size(kTurnedTriangle), {}});
    // The meridian, its four chords.
    cases.push_back({"a half disc",
                     semicircle(),
                     2 * kPi,
                     kTurnedSemicircle,
                     std::size(kTurnedSemicircle),
                     {"E x/revolved:{0}/seam:1", "E x/revolved:{0}/seam:5",
                      "E x/revolved:{0}/seam:8", "E x/revolved:{0}/seam:10"}});
    cases.push_back({"a half disc, a quarter turn",
                     semicircle(),
                     kPi / 2,
                     kTurnedSemicircleQuarter,
                     std::size(kTurnedSemicircleQuarter),
                     {}});
    // The rim its corner turns (eight chords) and the circle at u = 0 (eight).
    cases.push_back(
        {"a torus",
         {circle({5, 0}, 1)},
         2 * kPi,
         kTurnedTorus,
         std::size(kTurnedTorus),
         {"E x/revolved:{0}/seam:2", "E x/revolved:{0}/seam:3", "E x/revolved:{0}/seam:6",
          "E x/revolved:{0}/seam:9", "E x/revolved:{0}/seam:12", "E x/revolved:{0}/seam:15",
          "E x/revolved:{0}/seam:18", "E x/revolved:{0}/seam:21", "E x/revolved:{0}/seam:23",
          "E x/revolved:{0}/seam:26", "E x/revolved:{0}/seam:42", "E x/revolved:{0}/seam:58",
          "E x/revolved:{0}/seam:74", "E x/revolved:{0}/seam:90", "E x/revolved:{0}/seam:106",
          "E x/revolved:{0}/seam:121"}});
    // The rim its corner turns, two chords.
    cases.push_back({"a torus, a quarter turn",
                     {circle({5, 0}, 1)},
                     kPi / 2,
                     kTurnedTorusQuarter,
                     std::size(kTurnedTorusQuarter),
                     {"E x/revolved:{0}/seam:2", "E x/revolved:{0}/seam:17"}});
    // Its arc, three chords.
    cases.push_back(
        {"a lemon",
         lemon(),
         2 * kPi,
         kTurnedLemon,
         std::size(kTurnedLemon),
         {"E x/revolved:{0}/seam:1", "E x/revolved:{0}/seam:5", "E x/revolved:{0}/seam:7"}});
    // The hemisphere's arc, two chords; the cone's line is one.
    cases.push_back({"a pole and an apex",
                     poleAndApex(),
                     2 * kPi,
                     kTurnedPoleApex,
                     std::size(kTurnedPoleApex),
                     {"E x/revolved:{0}/seam:1", "E x/revolved:{0}/seam:4"}});
    cases.push_back({"a pole and an apex, a quarter turn",
                     poleAndApex(),
                     kPi / 2,
                     kTurnedPoleApexQuarter,
                     std::size(kTurnedPoleApexQuarter),
                     {}});
    cases.push_back({"a pole and an apex, 359.9 degrees",
                     poleAndApex(),
                     359.9 * kPi / 180.0,
                     kTurnedPoleApexAlmost,
                     std::size(kTurnedPoleApexAlmost),
                     {}});
    cases.push_back({"a washer", washer(), 2 * kPi, kTurnedWasher, std::size(kTurnedWasher), {}});
    for (Case& c : cases) {
        SCOPED_TRACE(c.what);
        const auto out = revolved(c.profile, c.angle);
        ASSERT_NE(out.faceted, nullptr);
        ASSERT_NE(out.exact, nullptr);
        expectNumberedAsMasters(*out.faceted, c.table, c.size, &c.profile, c.gone);
        expectChordsOnTheirEdges(*out.faceted, *out.exact);
    }
}

// NURBS surfaces of revolution from pole to pole, a lemon and an apple, at 12 segments, where a
// NURBS surface's u is no angle between its rational arcs' knots (Ruling 24: their points inside
// are their first meridian turned to master's angles): every place is master's. The seam (the
// arc at u = 0) is named after master's edge at its first chord, as above: master's names of its
// chords (the lemon's four, the apple's eight) are gone; a quarter turn has no seam.
TEST(BridgeNumbering, ALemonAndAnAppleAreNumberedAsMastersWere) {
    struct Case {
        const char* what;
        Profile profile;
        double angle;
        const Place* table;
        std::size_t size;
        std::size_t gone;
    };
    std::vector<Case> cases;
    cases.push_back({"a lemon", lemon(), 2 * kPi, kTurnedLemon12, std::size(kTurnedLemon12), 4});
    cases.push_back({"a lemon, a quarter turn", lemon(), kPi / 2, kTurnedLemonQuarter12,
                     std::size(kTurnedLemonQuarter12), 0});
    cases.push_back({"an apple", apple(), 2 * kPi, kTurnedApple12, std::size(kTurnedApple12), 8});
    for (Case& c : cases) {
        SCOPED_TRACE(c.what);
        const auto out = revolved(c.profile, c.angle, 12);
        ASSERT_NE(out.faceted, nullptr);
        ASSERT_NE(out.exact, nullptr);
        // The chords of the exact seam: master's seams of the face along u = 0.
        const auto places = placesOf(*out.faceted);
        std::set<std::string> gone;
        for (std::size_t i = 0; i < c.size; ++i) {
            const std::string tag = c.table[i].tag;
            if (tag.rfind("E ", 0) == 0 && tag.find("/seam:") != std::string::npos &&
                places.count(canonical(named(tag, &c.profile))) == 0) {
                gone.insert(tag);
            }
        }
        EXPECT_EQ(gone.size(), c.gone);
        for (const std::string& tag : gone) {
            const Place* place = std::find_if(c.table, c.table + c.size,
                                              [&tag](const Place& p) { return p.tag == tag; });
            ASSERT_NE(place, c.table + c.size);
            EXPECT_LT(std::abs(place->z), 1e-9) << tag << ": on the profile's plane, at u = 0";
        }
        expectNumberedAsMasters(*out.faceted, c.table, c.size, &c.profile, gone);
        expectChordsOnTheirEdges(*out.faceted, *out.exact);
    }
}

// A NURBS surface of revolution from a rim to a pole, an ogive's nose, at 12 segments (Ruling 24:
// its points inside its first meridian turned to master's angles), and its flat base, one disc as
// master's whole turn made it: every place is master's. Its seam (the arc at u = 0, two chords)
// is named after master's edge at its first chord: master's names of its two chords are gone.
TEST(BridgeNumbering, AnOgiveIsNumberedAsMastersWas) {
    const Profile p = ogive();
    const auto out = revolved(p, 2 * kPi, 12);
    ASSERT_NE(out.faceted, nullptr);
    ASSERT_NE(out.exact, nullptr);
    const auto places = placesOf(*out.faceted);
    std::set<std::string> gone;
    for (const Place& place : kTurnedOgive12) {
        const std::string tag = place.tag;
        if (tag.rfind("E ", 0) == 0 && tag.find("/seam:") != std::string::npos &&
            places.count(canonical(named(tag, &p))) == 0) {
            gone.insert(tag);
            EXPECT_LT(std::abs(place.z), 1e-9) << tag << ": on the profile's plane, at u = 0";
        }
    }
    EXPECT_EQ(gone.size(), 2u);
    expectNumberedAsMasters(*out.faceted, kTurnedOgive12, &p, gone);
    expectChordsOnTheirEdges(*out.faceted, *out.exact);
}

// Through part of a turn a flat face of a revolve is one face, a sector of a disc or a ring, as
// every plane's bridge is: master's facets of it (`revolved:<entity>/facet:<k>`, a quad for each
// step) and the seams between them are gone. Every other name is master's, its caps', its rims'
// and the edges between the sector and the caps included.
TEST(BridgeNumbering, APartialTurnsFlatFacesAreOneFace) {
    for (const bool isWasher : {false, true}) {
        SCOPED_TRACE(isWasher ? "a washer" : "a rectangle");
        const Profile p = isWasher ? washer() : rectangle();
        const auto out = revolved(p, kPi / 2);
        ASSERT_NE(out.faceted, nullptr);
        const Place* table = isWasher ? kTurnedWasherQuarter : kTurnedRectangleQuarter;
        const std::size_t size =
            isWasher ? std::size(kTurnedWasherQuarter) : std::size(kTurnedRectangleQuarter);
        // The flat ends: the bottom ({0}) and the top ({2}), each two facets and a seam.
        std::set<std::string> gone;
        for (std::size_t i = 0; i < size; ++i) {
            const std::string tag = table[i].tag;
            for (const char* flat : {"x/revolved:{0}/", "x/revolved:{2}/"}) {
                if (tag.find(flat) != std::string::npos) gone.insert(tag);
            }
        }
        EXPECT_EQ(gone.size(), 6u);
        expectNumberedAsMasters(*out.faceted, table, size, &p, gone);
        expectChordsOnTheirEdges(*out.faceted, *out.exact);
    }
}

// A profile with a hole was master's Boolean of its square less the hole's cutter (Ruling 23).
// The bridge is named for the loops master turned, its square and its hole's cutter, as each was
// numbered before the Boolean. What the Boolean made is not the bridge's:
// - Through a whole turn the cutter was the hole's own torus, merged into the result as a cavity:
//   its facets are master's, but its chords and seams (`x~hole0/...`) were numbered by the merge,
//   not comparable and not compared (Task 11 rewrites a saved chord's name to its curve's); of
//   them, the 16 chords of its two exact seams (its rim, eight, and its circle at u = 0, eight)
//   are each seam's `<seam>/chord:<k>`, and 16 of master's names are gone. Each flat end was two
//   pieces (`/piece:<k>`), gone; it is one ring.
// - Through a quarter turn each cap was cut in two (`/piece:<k>`), gone, and the cutter was turned
//   a twentieth further at each end, so none of its faces or edges lie where the hole's do: every
//   `x~hole0/` name of master's is gone, the hole's own numbered as the bridge numbers them. The
//   flat ends are one face each, as for any partial turn (their facets and seams gone).
// Every other face and edge is master's.
TEST(BridgeNumbering, AHoledRevolveIsNumberedAsMastersWasWhereItCanBe) {
    for (const double angle : {2 * kPi, kPi / 2}) {
        SCOPED_TRACE(angle);
        const bool whole = angle > kPi;
        const Profile p = holed();
        const auto out = revolved(p, angle);
        ASSERT_NE(out.faceted, nullptr);
        const Place* table = whole ? kTurnedHoled : kTurnedHoledQuarter;
        const std::size_t size = whole ? std::size(kTurnedHoled) : std::size(kTurnedHoledQuarter);
        const auto places = placesOf(*out.faceted);
        std::set<std::string> gone;
        int goneOfTheHole = 0;
        for (std::size_t i = 0; i < size; ++i) {
            const std::string tag = table[i].tag;
            const bool piece = tag.find("/piece:") != std::string::npos;
            const bool flat = !whole && (tag.find("x/revolved:{0}/") != std::string::npos ||
                                         tag.find("x/revolved:{2}/") != std::string::npos);
            const bool ofTheHole = tag.find("x~hole0/") != std::string::npos;
            if (piece || flat) {
                gone.insert(tag);
            } else if (ofTheHole && places.count(canonical(named(tag, &p))) == 0) {
                gone.insert(tag);
                ++goneOfTheHole;
            }
        }
        if (whole) {
            EXPECT_EQ(goneOfTheHole, 16) << "the chords of the hole's two exact seams";
        } else {
            EXPECT_GT(goneOfTheHole, 0);
        }
        const auto merged = [](const std::string& tag) {
            return tag.find("x~hole0/") != std::string::npos && tag.rfind("E ", 0) == 0;
        };
        expectNumberedAsMasters(*out.faceted, table, size, &p, gone, merged);
        expectChordsOnTheirEdges(*out.faceted, *out.exact);
    }
}

// Hole tools (Ruling 21): each type's tool, a revolve of its half-section renamed for what each
// surface is, is numbered as master's was, its facets, its rims' chords and its seams, every one.
TEST(BridgeNumbering, HoleToolsAreNumberedAsMastersWere) {
    struct Case {
        const char* what;
        const Place* table;
        std::size_t size;
    };
    const std::vector<Case> cases = {
        {"simple", kSimpleHoleTool, std::size(kSimpleHoleTool)},
        {"counterbore", kCounterboreHoleTool, std::size(kCounterboreHoleTool)},
        {"countersink", kCountersinkHoleTool, std::size(kCountersinkHoleTool)}};
    for (std::size_t type = 0; type < cases.size(); ++type) {
        const Case& c = cases[type];
        SCOPED_TRACE(c.what);
        auto box = PrimitiveFeature::makeBox(20, 20, 10);
        box->restoreFeatureID("primitive_1");
        std::string why;
        const auto part = box->execute(nullptr, &why);
        ASSERT_NE(part, nullptr) << why;
        auto hole = hz::doc::HoleFeature::make("primitive_1/top", Vec3(10, 10, 10), 5.0, 6.0);
        hole->restoreFeatureID("hole_1");
        ASSERT_TRUE(hole->setParameter("segments", 8));
        if (type == 1) {
            ASSERT_TRUE(hole->setParameter("type", 1));
            ASSERT_TRUE(hole->setParameter("boreDiameter", 9.0));
            ASSERT_TRUE(hole->setParameter("boreDepth", 3.0));
        } else if (type == 2) {
            ASSERT_TRUE(hole->setParameter("type", 2));
            ASSERT_TRUE(hole->setParameter("sinkDiameter", 10.0));
            ASSERT_TRUE(hole->setParameter("sinkAngle", kPi / 2));
        }
        hz::doc::BuildContext context;
        context.part = part.get();
        const hz::doc::FeatureOutput tool = hole->executeWithExact(context, nullptr, &why);
        ASSERT_NE(tool.faceted, nullptr) << why;
        ASSERT_NE(tool.exact, nullptr) << why;
        expectNumberedAsMasters(*tool.faceted, c.table, c.size, nullptr, {});
        expectChordsOnTheirEdges(*tool.faceted, *tool.exact);
    }
}
