// Ruling 44: the faceted body of a converted feature whose master construction is one
// construction (a primitive, a single loop's extrude or revolve, a hole's tool, a sheet's flange)
// is master's own construction, so that what an operation after it numbers by the order it meets
// the body's faces (a Boolean's pieces of a face it splits, the chords and seams it names again)
// is numbered as on master: a reference saved on master names, after the operation, the place it
// named. The tables are master's at 6e822892, recorded through master's own features: each face's
// name with the centroid of its outer loop, each edge's with its midpoint, at 8 segments a turn
// ("{k}" in a name is the k-th entity of the sketch, "e<id>" in the tag).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/FacePlane.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

using hz::doc::Document;
using hz::doc::DocumentType;
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
// join: 16 faces, 40 edges, 4452.54833996.
const Place kJoin[] = {
    {"E edge:primitive_1/top/piece:0|primitive_2/side/chord:0", 11.4142135624, 13.4142135624, 10},
    {"E edge:primitive_1/top/piece:0|primitive_2/side/chord:1", 13.4142135624, 11.4142135624, 10},
    {"E edge:primitive_1/top/piece:0|primitive_2/side/chord:2", 13.4142135624, 8.58578643763, 10},
    {"E edge:primitive_1/top/piece:0|primitive_2/side/chord:3", 11.4142135624, 6.58578643763, 10},
    {"E edge:primitive_1/top/piece:1|primitive_2/side/chord:0", 8.58578643763, 6.58578643763, 10},
    {"E edge:primitive_1/top/piece:1|primitive_2/side/chord:1", 6.58578643763, 8.58578643763, 10},
    {"E edge:primitive_1/top/piece:1|primitive_2/side/chord:2", 6.58578643763, 11.4142135624, 10},
    {"E edge:primitive_1/top/piece:1|primitive_2/side/chord:3", 8.58578643763, 13.4142135624, 10},
    {"E primitive_1/edge:back|bottom", 10, 20, 0},
    {"E primitive_1/edge:back|left", 0, 20, 5},
    {"E primitive_1/edge:back|right", 20, 20, 5},
    {"E primitive_1/edge:back|top/piece:0", 15, 20, 10},
    {"E primitive_1/edge:back|top/piece:1", 5, 20, 10},
    {"E primitive_1/edge:bottom|front", 10, 0, 0},
    {"E primitive_1/edge:bottom|left", 0, 10, 0},
    {"E primitive_1/edge:bottom|right", 20, 10, 0},
    {"E primitive_1/edge:front|left", 0, 0, 5},
    {"E primitive_1/edge:front|right", 20, 0, 5},
    {"E primitive_1/edge:front|top/piece:0", 15, 0, 10},
    {"E primitive_1/edge:front|top/piece:1", 5, 0, 10},
    {"E primitive_1/edge:left|top/piece:1", 0, 10, 10},
    {"E primitive_1/edge:right|top/piece:0", 20, 10, 10},
    {"E primitive_1/edge:top/piece:0|top/piece:1/chord:0", 10, 17, 10},
    {"E primitive_1/edge:top/piece:0|top/piece:1/chord:1", 10, 3, 10},
    {"E primitive_2/edge:side|top/chord:0", 13.4142135624, 11.4142135624, 20},
    {"E primitive_2/edge:side|top/chord:1", 11.4142135624, 13.4142135624, 20},
    {"E primitive_2/edge:side|top/chord:2", 8.58578643763, 13.4142135624, 20},
    {"E primitive_2/edge:side|top/chord:3", 6.58578643763, 11.4142135624, 20},
    {"E primitive_2/edge:side|top/chord:4", 6.58578643763, 8.58578643763, 20},
    {"E primitive_2/edge:side|top/chord:5", 8.58578643763, 6.58578643763, 20},
    {"E primitive_2/edge:side|top/chord:6", 11.4142135624, 6.58578643763, 20},
    {"E primitive_2/edge:side|top/chord:7", 13.4142135624, 8.58578643763, 20},
    {"E primitive_2/side/seam:0", 12.8284271247, 12.8284271247, 15},
    {"E primitive_2/side/seam:1", 14, 10, 15},
    {"E primitive_2/side/seam:2", 10, 14, 15},
    {"E primitive_2/side/seam:3", 7.17157287525, 12.8284271247, 15},
    {"E primitive_2/side/seam:4", 6, 10, 15},
    {"E primitive_2/side/seam:5", 7.17157287525, 7.17157287525, 15},
    {"E primitive_2/side/seam:6", 10, 6, 15},
    {"E primitive_2/side/seam:7", 12.8284271247, 7.17157287525, 15},
    {"F primitive_1/back", 10, 20, 5},
    {"F primitive_1/bottom", 10, 10, 0},
    {"F primitive_1/front", 10, 0, 5},
    {"F primitive_1/left", 0, 10, 5},
    {"F primitive_1/right", 20, 10, 5},
    {"F primitive_1/top/piece:0", 15.2754430963, 9.85638894307, 10},
    {"F primitive_1/top/piece:1", 5.61111111111, 10.4038127306, 10},
    {"F primitive_2/side/facet:0", 13.4142135624, 11.4142135624, 15},
    {"F primitive_2/side/facet:1", 11.4142135624, 13.4142135624, 15},
    {"F primitive_2/side/facet:2", 8.58578643763, 13.4142135624, 15},
    {"F primitive_2/side/facet:3", 6.58578643763, 11.4142135624, 15},
    {"F primitive_2/side/facet:4", 6.58578643763, 8.58578643763, 15},
    {"F primitive_2/side/facet:5", 8.58578643763, 6.58578643763, 15},
    {"F primitive_2/side/facet:6", 11.4142135624, 6.58578643763, 15},
    {"F primitive_2/side/facet:7", 13.4142135624, 8.58578643763, 15},
    {"F primitive_2/top", 10, 10, 20},
};
// cut: 18 faces, 50 edges, 5025.44155877.
const Place kCut[] = {
    {"E edge:extrude_1/side:{0}|primitive_1/bottom/piece:0", 14, 7, 0},
    {"E edge:extrude_1/side:{0}|primitive_1/top/piece:1", 14, 7, 10},
    {"E edge:extrude_1/side:{1}|primitive_1/bottom/piece:0/chord:0", 21.0606601718, 7.43933982822, 0},
    {"E edge:extrude_1/side:{1}|primitive_1/bottom/piece:0/chord:1", 22.5606601718, 8.93933982822, 0},
    {"E edge:extrude_1/side:{1}|primitive_1/bottom/piece:1/chord:0", 22.5606601718, 11.0606601718, 0},
    {"E edge:extrude_1/side:{1}|primitive_1/bottom/piece:1/chord:1", 21.0606601718, 12.5606601718, 0},
    {"E edge:extrude_1/side:{1}|primitive_1/top/piece:0/chord:0", 22.5606601718, 11.0606601718, 10},
    {"E edge:extrude_1/side:{1}|primitive_1/top/piece:0/chord:1", 21.0606601718, 12.5606601718, 10},
    {"E edge:extrude_1/side:{1}|primitive_1/top/piece:1/chord:0", 21.0606601718, 7.43933982822, 10},
    {"E edge:extrude_1/side:{1}|primitive_1/top/piece:1/chord:1", 22.5606601718, 8.93933982822, 10},
    {"E edge:extrude_1/side:{2}|primitive_1/bottom/piece:1", 14, 13, 0},
    {"E edge:extrude_1/side:{2}|primitive_1/top/piece:0", 14, 13, 10},
    {"E edge:extrude_1/side:{3}|primitive_1/bottom/piece:0/chord:0", 5.43933982822, 8.93933982822, 0},
    {"E edge:extrude_1/side:{3}|primitive_1/bottom/piece:0/chord:1", 6.93933982822, 7.43933982822, 0},
    {"E edge:extrude_1/side:{3}|primitive_1/bottom/piece:1/chord:0", 6.93933982822, 12.5606601718, 0},
    {"E edge:extrude_1/side:{3}|primitive_1/bottom/piece:1/chord:1", 5.43933982822, 11.0606601718, 0},
    {"E edge:extrude_1/side:{3}|primitive_1/top/piece:0/chord:0", 6.93933982822, 12.5606601718, 10},
    {"E edge:extrude_1/side:{3}|primitive_1/top/piece:0/chord:1", 5.43933982822, 11.0606601718, 10},
    {"E edge:extrude_1/side:{3}|primitive_1/top/piece:1/chord:0", 5.43933982822, 8.93933982822, 10},
    {"E edge:extrude_1/side:{3}|primitive_1/top/piece:1/chord:1", 6.93933982822, 7.43933982822, 10},
    {"E extrude_1/edge:side:{0}|side:{1}", 20, 7, 5},
    {"E extrude_1/edge:side:{0}|side:{3}", 8, 7, 5},
    {"E extrude_1/edge:side:{1}|side:{2}", 20, 13, 5},
    {"E extrude_1/edge:side:{2}|side:{3}", 8, 13, 5},
    {"E extrude_1/side:{1}/seam:0", 22.1213203436, 7.87867965644, 5},
    {"E extrude_1/side:{1}/seam:1", 23, 10, 5},
    {"E extrude_1/side:{1}/seam:2", 22.1213203436, 12.1213203436, 5},
    {"E extrude_1/side:{3}/seam:0", 5.87867965644, 12.1213203436, 5},
    {"E extrude_1/side:{3}/seam:1", 5, 10, 5},
    {"E extrude_1/side:{3}/seam:2", 5.87867965644, 7.87867965644, 5},
    {"E primitive_1/edge:back|bottom/piece:1", 15, 20, 0},
    {"E primitive_1/edge:back|left", 0, 20, 5},
    {"E primitive_1/edge:back|right", 30, 20, 5},
    {"E primitive_1/edge:back|top/piece:0", 15, 20, 10},
    {"E primitive_1/edge:bottom/piece:0|bottom/piece:1/chord:0", 2.5, 10, 0},
    {"E primitive_1/edge:bottom/piece:0|bottom/piece:1/chord:1", 26.5, 10, 0},
    {"E primitive_1/edge:bottom/piece:0|front", 15, 0, 0},
    {"E primitive_1/edge:bottom/piece:0|left", 0, 5, 0},
    {"E primitive_1/edge:bottom/piece:0|right", 30, 5, 0},
    {"E primitive_1/edge:bottom/piece:1|left", 0, 15, 0},
    {"E primitive_1/edge:bottom/piece:1|right", 30, 15, 0},
    {"E primitive_1/edge:front|left", 0, 0, 5},
    {"E primitive_1/edge:front|right", 30, 0, 5},
    {"E primitive_1/edge:front|top/piece:1", 15, 0, 10},
    {"E primitive_1/edge:left|top/piece:0", 0, 15, 10},
    {"E primitive_1/edge:left|top/piece:1", 0, 5, 10},
    {"E primitive_1/edge:right|top/piece:0", 30, 15, 10},
    {"E primitive_1/edge:right|top/piece:1", 30, 5, 10},
    {"E primitive_1/edge:top/piece:0|top/piece:1/chord:0", 2.5, 10, 10},
    {"E primitive_1/edge:top/piece:0|top/piece:1/chord:1", 26.5, 10, 10},
    {"F extrude_1/side:{0}", 14, 7, 5},
    {"F extrude_1/side:{1}/facet:0", 21.0606601718, 7.43933982822, 5},
    {"F extrude_1/side:{1}/facet:1", 22.5606601718, 8.93933982822, 5},
    {"F extrude_1/side:{1}/facet:2", 22.5606601718, 11.0606601718, 5},
    {"F extrude_1/side:{1}/facet:3", 21.0606601718, 12.5606601718, 5},
    {"F extrude_1/side:{2}", 14, 13, 5},
    {"F extrude_1/side:{3}/facet:0", 6.93933982822, 12.5606601718, 5},
    {"F extrude_1/side:{3}/facet:1", 5.43933982822, 11.0606601718, 5},
    {"F extrude_1/side:{3}/facet:2", 5.43933982822, 8.93933982822, 5},
    {"F extrude_1/side:{3}/facet:3", 6.93933982822, 7.43933982822, 5},
    {"F primitive_1/back", 15, 20, 5},
    {"F primitive_1/bottom/piece:0", 14.9542283553, 4.56, 0},
    {"F primitive_1/bottom/piece:1", 15.3340669579, 14.3278688525, 0},
    {"F primitive_1/front", 15, 0, 5},
    {"F primitive_1/left", 0, 10, 5},
    {"F primitive_1/right", 30, 10, 5},
    {"F primitive_1/top/piece:0", 15.3340669579, 14.3278688525, 10},
    {"F primitive_1/top/piece:1", 14.9542283553, 4.56, 10},
};
// shell: 20 faces, 50 edges, 299.813275223.
const Place kShell[] = {
    {"E edge:primitive_1/top/piece:0|shell_1/inner:primitive_1/side/chord:0", 3.41421356237, 1.41421356237, 10},
    {"E edge:primitive_1/top/piece:0|shell_1/inner:primitive_1/side/chord:1", 3.41421356237, -1.41421356237, 10},
    {"E edge:primitive_1/top/piece:0|shell_1/inner:primitive_1/side/chord:2", 1.41421356237, -3.41421356237, 10},
    {"E edge:primitive_1/top/piece:0|shell_1/inner:primitive_1/side/chord:3", -1.41421356237, -3.41421356237, 10},
    {"E edge:primitive_1/top/piece:1|shell_1/inner:primitive_1/side/chord:0", -3.41421356237, -1.41421356237, 10},
    {"E edge:primitive_1/top/piece:1|shell_1/inner:primitive_1/side/chord:1", -3.41421356237, 1.41421356237, 10},
    {"E edge:primitive_1/top/piece:1|shell_1/inner:primitive_1/side/chord:2", -1.41421356237, 3.41421356237, 10},
    {"E edge:primitive_1/top/piece:1|shell_1/inner:primitive_1/side/chord:3", 1.41421356237, 3.41421356237, 10},
    {"E primitive_1/edge:bottom|side/chord:0", 1.76776695297, -4.26776695297, 0},
    {"E primitive_1/edge:bottom|side/chord:1", -1.76776695297, -4.26776695297, 0},
    {"E primitive_1/edge:bottom|side/chord:2", -4.26776695297, -1.76776695297, 0},
    {"E primitive_1/edge:bottom|side/chord:3", -4.26776695297, 1.76776695297, 0},
    {"E primitive_1/edge:bottom|side/chord:4", -1.76776695297, 4.26776695297, 0},
    {"E primitive_1/edge:bottom|side/chord:5", 1.76776695297, 4.26776695297, 0},
    {"E primitive_1/edge:bottom|side/chord:6", 4.26776695297, 1.76776695297, 0},
    {"E primitive_1/edge:bottom|side/chord:7", 4.26776695297, -1.76776695297, 0},
    {"E primitive_1/edge:side|top/piece:0/chord:0", 4.26776695297, 1.76776695297, 10},
    {"E primitive_1/edge:side|top/piece:0/chord:1", -1.76776695297, -4.26776695297, 10},
    {"E primitive_1/edge:side|top/piece:0/chord:2", 1.76776695297, -4.26776695297, 10},
    {"E primitive_1/edge:side|top/piece:0/chord:3", 4.26776695297, -1.76776695297, 10},
    {"E primitive_1/edge:side|top/piece:1/chord:0", 1.76776695297, 4.26776695297, 10},
    {"E primitive_1/edge:side|top/piece:1/chord:1", -1.76776695297, 4.26776695297, 10},
    {"E primitive_1/edge:side|top/piece:1/chord:2", -4.26776695297, 1.76776695297, 10},
    {"E primitive_1/edge:side|top/piece:1/chord:3", -4.26776695297, -1.76776695297, 10},
    {"E primitive_1/edge:top/piece:0|top/piece:1/chord:0", 3.18198051534, 3.18198051534, 10},
    {"E primitive_1/edge:top/piece:0|top/piece:1/chord:1", -3.18198051534, -3.18198051534, 10},
    {"E primitive_1/side/seam:0", 3.53553390593, 3.53553390593, 5},
    {"E primitive_1/side/seam:1", 5, 0, 5},
    {"E primitive_1/side/seam:2", 3.06161699787e-16, 5, 5},
    {"E primitive_1/side/seam:3", -3.53553390593, 3.53553390593, 5},
    {"E primitive_1/side/seam:4", -5, 6.12323399574e-16, 5},
    {"E primitive_1/side/seam:5", -3.53553390593, -3.53553390593, 5},
    {"E primitive_1/side/seam:6", -9.18485099361e-16, -5, 5},
    {"E primitive_1/side/seam:7", 3.53553390593, -3.53553390593, 5},
    {"E shell_1/edge:inner:primitive_1/bottom|inner:primitive_1/side/chord:0", 3.41421356237, 1.41421356237, 1},
    {"E shell_1/edge:inner:primitive_1/bottom|inner:primitive_1/side/chord:1", 1.41421356237, 3.41421356237, 1},
    {"E shell_1/edge:inner:primitive_1/bottom|inner:primitive_1/side/chord:2", -1.41421356237, 3.41421356237, 1},
    {"E shell_1/edge:inner:primitive_1/bottom|inner:primitive_1/side/chord:3", -3.41421356237, 1.41421356237, 1},
    {"E shell_1/edge:inner:primitive_1/bottom|inner:primitive_1/side/chord:4", -3.41421356237, -1.41421356237, 1},
    {"E shell_1/edge:inner:primitive_1/bottom|inner:primitive_1/side/chord:5", -1.41421356237, -3.41421356237, 1},
    {"E shell_1/edge:inner:primitive_1/bottom|inner:primitive_1/side/chord:6", 1.41421356237, -3.41421356237, 1},
    {"E shell_1/edge:inner:primitive_1/bottom|inner:primitive_1/side/chord:7", 3.41421356237, -1.41421356237, 1},
    {"E shell_1/inner:primitive_1/side/seam:0", 2.82842712475, 2.82842712475, 5.5},
    {"E shell_1/inner:primitive_1/side/seam:1", 4, -2.27777771972e-16, 5.5},
    {"E shell_1/inner:primitive_1/side/seam:2", 3.21576616653e-16, 4, 5.5},
    {"E shell_1/inner:primitive_1/side/seam:3", -2.82842712475, 2.82842712475, 5.5},
    {"E shell_1/inner:primitive_1/side/seam:4", -4, 4.61240797708e-16, 5.5},
    {"E shell_1/inner:primitive_1/side/seam:5", -2.82842712475, -2.82842712475, 5.5},
    {"E shell_1/inner:primitive_1/side/seam:6", -2.79304405468e-16, -4, 5.5},
    {"E shell_1/inner:primitive_1/side/seam:7", 2.82842712475, -2.82842712475, 5.5},
    {"F primitive_1/bottom", -5.02429586779e-17, -2.83732472918e-16, 0},
    {"F primitive_1/side/facet:0", 4.26776695297, 1.76776695297, 5},
    {"F primitive_1/side/facet:1", 1.76776695297, 4.26776695297, 5},
    {"F primitive_1/side/facet:2", -1.76776695297, 4.26776695297, 5},
    {"F primitive_1/side/facet:3", -4.26776695297, 1.76776695297, 5},
    {"F primitive_1/side/facet:4", -4.26776695297, -1.76776695297, 5},
    {"F primitive_1/side/facet:5", -1.76776695297, -4.26776695297, 5},
    {"F primitive_1/side/facet:6", 1.76776695297, -4.26776695297, 5},
    {"F primitive_1/side/facet:7", 4.26776695297, -1.76776695297, 5},
    {"F primitive_1/top/piece:0", 1.44847065761, -2.1509545443, 10},
    {"F primitive_1/top/piece:1", -1.15188890512, 1.41123653398, 10},
    {"F shell_1/inner:primitive_1/bottom", -3.3364464747e-16, 1.96261557335e-17, 1},
    {"F shell_1/inner:primitive_1/side/facet:0", 3.41421356237, 1.41421356237, 5.5},
    {"F shell_1/inner:primitive_1/side/facet:1", 1.41421356237, 3.41421356237, 5.5},
    {"F shell_1/inner:primitive_1/side/facet:2", -1.41421356237, 3.41421356237, 5.5},
    {"F shell_1/inner:primitive_1/side/facet:3", -3.41421356237, 1.41421356237, 5.5},
    {"F shell_1/inner:primitive_1/side/facet:4", -3.41421356237, -1.41421356237, 5.5},
    {"F shell_1/inner:primitive_1/side/facet:5", -1.41421356237, -3.41421356237, 5.5},
    {"F shell_1/inner:primitive_1/side/facet:6", 1.41421356237, -3.41421356237, 5.5},
    {"F shell_1/inner:primitive_1/side/facet:7", 3.41421356237, -1.41421356237, 5.5},
};
// pattern: 32 faces, 96 edges, 7660.58874503.
const Place kPattern[] = {
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/bottom/piece:0/chord:0", 19.7071067812, 9.29289321881, 0},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/bottom/piece:0/chord:1", 18.7071067812, 8.29289321881, 0},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/bottom/piece:0/chord:2", 17.2928932188, 8.29289321881, 0},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/bottom/piece:0/chord:3", 16.2928932188, 9.29289321881, 0},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/bottom/piece:1/chord:0", 16.2928932188, 10.7071067812, 0},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/bottom/piece:1/chord:1", 17.2928932188, 11.7071067812, 0},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/bottom/piece:1/chord:2", 18.7071067812, 11.7071067812, 0},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/bottom/piece:1/chord:3", 19.7071067812, 10.7071067812, 0},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/top/piece:0/chord:0", 16.2928932188, 10.7071067812, 10},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/top/piece:0/chord:1", 17.2928932188, 11.7071067812, 10},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/top/piece:0/chord:2", 18.7071067812, 11.7071067812, 10},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/top/piece:0/chord:3", 19.7071067812, 10.7071067812, 10},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/top/piece:1/chord:0", 19.7071067812, 9.29289321881, 10},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/top/piece:1/chord:1", 18.7071067812, 8.29289321881, 10},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/top/piece:1/chord:2", 17.2928932188, 8.29289321881, 10},
    {"E edge:hole_1/wall/pattern_1:1|primitive_1/top/piece:1/chord:3", 16.2928932188, 9.29289321881, 10},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/bottom/piece:0/chord:0", 29.7071067812, 9.29289321881, 0},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/bottom/piece:0/chord:1", 28.7071067812, 8.29289321881, 0},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/bottom/piece:0/chord:2", 27.2928932188, 8.29289321881, 0},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/bottom/piece:0/chord:3", 26.2928932188, 9.29289321881, 0},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/bottom/piece:1/chord:0", 26.2928932188, 10.7071067812, 0},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/bottom/piece:1/chord:1", 27.2928932188, 11.7071067812, 0},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/bottom/piece:1/chord:2", 28.7071067812, 11.7071067812, 0},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/bottom/piece:1/chord:3", 29.7071067812, 10.7071067812, 0},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/top/piece:0/chord:0", 26.2928932188, 10.7071067812, 10},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/top/piece:0/chord:1", 27.2928932188, 11.7071067812, 10},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/top/piece:0/chord:2", 28.7071067812, 11.7071067812, 10},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/top/piece:0/chord:3", 29.7071067812, 10.7071067812, 10},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/top/piece:1/chord:0", 29.7071067812, 9.29289321881, 10},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/top/piece:1/chord:1", 28.7071067812, 8.29289321881, 10},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/top/piece:1/chord:2", 27.2928932188, 8.29289321881, 10},
    {"E edge:hole_1/wall/pattern_1:2|primitive_1/top/piece:1/chord:3", 26.2928932188, 9.29289321881, 10},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:0/chord:0", 9.70710678119, 9.29289321881, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:0/chord:1", 8.70710678119, 8.29289321881, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:0/chord:2", 7.29289321881, 8.29289321881, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:0/chord:3", 6.29289321881, 9.29289321881, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:1/chord:0", 6.29289321881, 10.7071067812, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:1/chord:1", 7.29289321881, 11.7071067812, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:1/chord:2", 8.70710678119, 11.7071067812, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:1/chord:3", 9.70710678119, 10.7071067812, 0},
    {"E edge:hole_1/wall|primitive_1/top/piece:0/chord:0", 6.29289321881, 10.7071067812, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:0/chord:1", 7.29289321881, 11.7071067812, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:0/chord:2", 8.70710678119, 11.7071067812, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:0/chord:3", 9.70710678119, 10.7071067812, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:1/chord:0", 9.70710678119, 9.29289321881, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:1/chord:1", 8.70710678119, 8.29289321881, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:1/chord:2", 7.29289321881, 8.29289321881, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:1/chord:3", 6.29289321881, 9.29289321881, 10},
    {"E hole_1/wall/pattern_1:1/seam:0", 20, 10, 5},
    {"E hole_1/wall/pattern_1:1/seam:1", 19.4142135624, 8.58578643763, 5},
    {"E hole_1/wall/pattern_1:1/seam:2", 18, 8, 5},
    {"E hole_1/wall/pattern_1:1/seam:3", 16.5857864376, 8.58578643763, 5},
    {"E hole_1/wall/pattern_1:1/seam:4", 16, 10, 5},
    {"E hole_1/wall/pattern_1:1/seam:5", 16.5857864376, 11.4142135624, 5},
    {"E hole_1/wall/pattern_1:1/seam:6", 18, 12, 5},
    {"E hole_1/wall/pattern_1:1/seam:7", 19.4142135624, 11.4142135624, 5},
    {"E hole_1/wall/pattern_1:2/seam:0", 30, 10, 5},
    {"E hole_1/wall/pattern_1:2/seam:1", 29.4142135624, 8.58578643763, 5},
    {"E hole_1/wall/pattern_1:2/seam:2", 28, 8, 5},
    {"E hole_1/wall/pattern_1:2/seam:3", 26.5857864376, 8.58578643763, 5},
    {"E hole_1/wall/pattern_1:2/seam:4", 26, 10, 5},
    {"E hole_1/wall/pattern_1:2/seam:5", 26.5857864376, 11.4142135624, 5},
    {"E hole_1/wall/pattern_1:2/seam:6", 28, 12, 5},
    {"E hole_1/wall/pattern_1:2/seam:7", 29.4142135624, 11.4142135624, 5},
    {"E hole_1/wall/seam:0", 10, 10, 5},
    {"E hole_1/wall/seam:1", 9.41421356237, 8.58578643763, 5},
    {"E hole_1/wall/seam:2", 8, 8, 5},
    {"E hole_1/wall/seam:3", 6.58578643763, 8.58578643763, 5},
    {"E hole_1/wall/seam:4", 6, 10, 5},
    {"E hole_1/wall/seam:5", 6.58578643763, 11.4142135624, 5},
    {"E hole_1/wall/seam:6", 8, 12, 5},
    {"E hole_1/wall/seam:7", 9.41421356237, 11.4142135624, 5},
    {"E primitive_1/edge:back|bottom/piece:1", 20, 20, 0},
    {"E primitive_1/edge:back|left", 0, 20, 5},
    {"E primitive_1/edge:back|right", 40, 20, 5},
    {"E primitive_1/edge:back|top/piece:0", 20, 20, 10},
    {"E primitive_1/edge:bottom/piece:0|bottom/piece:1/chord:0", 35, 10, 0},
    {"E primitive_1/edge:bottom/piece:0|bottom/piece:1/chord:1", 3, 10, 0},
    {"E primitive_1/edge:bottom/piece:0|bottom/piece:1/chord:2", 13, 10, 0},
    {"E primitive_1/edge:bottom/piece:0|bottom/piece:1/chord:3", 23, 10, 0},
    {"E primitive_1/edge:bottom/piece:0|front", 20, 0, 0},
    {"E primitive_1/edge:bottom/piece:0|left", 0, 5, 0},
    {"E primitive_1/edge:bottom/piece:0|right", 40, 5, 0},
    {"E primitive_1/edge:bottom/piece:1|left", 0, 15, 0},
    {"E primitive_1/edge:bottom/piece:1|right", 40, 15, 0},
    {"E primitive_1/edge:front|left", 0, 0, 5},
    {"E primitive_1/edge:front|right", 40, 0, 5},
    {"E primitive_1/edge:front|top/piece:1", 20, 0, 10},
    {"E primitive_1/edge:left|top/piece:0", 0, 15, 10},
    {"E primitive_1/edge:left|top/piece:1", 0, 5, 10},
    {"E primitive_1/edge:right|top/piece:0", 40, 15, 10},
    {"E primitive_1/edge:right|top/piece:1", 40, 5, 10},
    {"E primitive_1/edge:top/piece:0|top/piece:1/chord:0", 35, 10, 10},
    {"E primitive_1/edge:top/piece:0|top/piece:1/chord:1", 3, 10, 10},
    {"E primitive_1/edge:top/piece:0|top/piece:1/chord:2", 13, 10, 10},
    {"E primitive_1/edge:top/piece:0|top/piece:1/chord:3", 23, 10, 10},
    {"F hole_1/wall/facet:0", 9.70710678119, 9.29289321881, 5},
    {"F hole_1/wall/facet:0/pattern_1:1", 19.7071067812, 9.29289321881, 5},
    {"F hole_1/wall/facet:0/pattern_1:2", 29.7071067812, 9.29289321881, 5},
    {"F hole_1/wall/facet:1", 8.70710678119, 8.29289321881, 5},
    {"F hole_1/wall/facet:1/pattern_1:1", 18.7071067812, 8.29289321881, 5},
    {"F hole_1/wall/facet:1/pattern_1:2", 28.7071067812, 8.29289321881, 5},
    {"F hole_1/wall/facet:2", 7.29289321881, 8.29289321881, 5},
    {"F hole_1/wall/facet:2/pattern_1:1", 17.2928932188, 8.29289321881, 5},
    {"F hole_1/wall/facet:2/pattern_1:2", 27.2928932188, 8.29289321881, 5},
    {"F hole_1/wall/facet:3", 6.29289321881, 9.29289321881, 5},
    {"F hole_1/wall/facet:3/pattern_1:1", 16.2928932188, 9.29289321881, 5},
    {"F hole_1/wall/facet:3/pattern_1:2", 26.2928932188, 9.29289321881, 5},
    {"F hole_1/wall/facet:4", 6.29289321881, 10.7071067812, 5},
    {"F hole_1/wall/facet:4/pattern_1:1", 16.2928932188, 10.7071067812, 5},
    {"F hole_1/wall/facet:4/pattern_1:2", 26.2928932188, 10.7071067812, 5},
    {"F hole_1/wall/facet:5", 7.29289321881, 11.7071067812, 5},
    {"F hole_1/wall/facet:5/pattern_1:1", 17.2928932188, 11.7071067812, 5},
    {"F hole_1/wall/facet:5/pattern_1:2", 27.2928932188, 11.7071067812, 5},
    {"F hole_1/wall/facet:6", 8.70710678119, 11.7071067812, 5},
    {"F hole_1/wall/facet:6/pattern_1:1", 18.7071067812, 11.7071067812, 5},
    {"F hole_1/wall/facet:6/pattern_1:2", 28.7071067812, 11.7071067812, 5},
    {"F hole_1/wall/facet:7", 9.70710678119, 10.7071067812, 5},
    {"F hole_1/wall/facet:7/pattern_1:1", 19.7071067812, 10.7071067812, 5},
    {"F hole_1/wall/facet:7/pattern_1:2", 29.7071067812, 10.7071067812, 5},
    {"F primitive_1/back", 20, 20, 5},
    {"F primitive_1/bottom/piece:0", 19.7441543013, 5.67221375624, 0},
    {"F primitive_1/bottom/piece:1", 20.4053631851, 14.9280584128, 0},
    {"F primitive_1/front", 20, 0, 5},
    {"F primitive_1/left", 0, 10, 5},
    {"F primitive_1/right", 40, 10, 5},
    {"F primitive_1/top/piece:0", 19.7441543013, 14.3277862438, 10},
    {"F primitive_1/top/piece:1", 20.4053631851, 5.07194158716, 10},
};
// extrude: 22 faces, 60 edges, 1586.08030253.
const Place kExtrude[] = {
    {"E edge:extrude_1/cap_bottom/piece:0|hole_1/wall/chord:0", 6.53033008589, 2.28033008589, 0},
    {"E edge:extrude_1/cap_bottom/piece:0|hole_1/wall/chord:1", 5.46966991411, 2.28033008589, 0},
    {"E edge:extrude_1/cap_bottom/piece:0|hole_1/wall/chord:2", 4.71966991411, 1.53033008589, 0},
    {"E edge:extrude_1/cap_bottom/piece:0|hole_1/wall/chord:3", 4.71966991411, 0.46966991411, 0},
    {"E edge:extrude_1/cap_bottom/piece:1|hole_1/wall/chord:0", 5.46966991411, -0.28033008589, 0},
    {"E edge:extrude_1/cap_bottom/piece:1|hole_1/wall/chord:1", 6.53033008589, -0.28033008589, 0},
    {"E edge:extrude_1/cap_bottom/piece:1|hole_1/wall/chord:2", 7.28033008589, 0.46966991411, 0},
    {"E edge:extrude_1/cap_bottom/piece:1|hole_1/wall/chord:3", 7.28033008589, 1.53033008589, 0},
    {"E edge:extrude_1/cap_top/piece:0|hole_1/wall/chord:0", 4.71966991411, 0.46966991411, 6},
    {"E edge:extrude_1/cap_top/piece:0|hole_1/wall/chord:1", 4.71966991411, 1.53033008589, 6},
    {"E edge:extrude_1/cap_top/piece:0|hole_1/wall/chord:2", 5.46966991411, 2.28033008589, 6},
    {"E edge:extrude_1/cap_top/piece:0|hole_1/wall/chord:3", 6.53033008589, 2.28033008589, 6},
    {"E edge:extrude_1/cap_top/piece:1|hole_1/wall/chord:0", 7.28033008589, 1.53033008589, 6},
    {"E edge:extrude_1/cap_top/piece:1|hole_1/wall/chord:1", 7.28033008589, 0.46966991411, 6},
    {"E edge:extrude_1/cap_top/piece:1|hole_1/wall/chord:2", 6.53033008589, -0.28033008589, 6},
    {"E edge:extrude_1/cap_top/piece:1|hole_1/wall/chord:3", 5.46966991411, -0.28033008589, 6},
    {"E extrude_1/edge:cap_bottom/piece:0|cap_bottom/piece:1/chord:0", 8.53033008589, 3.53033008589, 0},
    {"E extrude_1/edge:cap_bottom/piece:0|cap_bottom/piece:1/chord:1", 2.46966991411, -2.53033008589, 0},
    {"E extrude_1/edge:cap_bottom/piece:0|side:{2}", 5, 5, 0},
    {"E extrude_1/edge:cap_bottom/piece:0|side:{3}/chord:0", -1.76776695297, -4.26776695297, 0},
    {"E extrude_1/edge:cap_bottom/piece:0|side:{3}/chord:1", -4.26776695297, -1.76776695297, 0},
    {"E extrude_1/edge:cap_bottom/piece:0|side:{3}/chord:2", -4.26776695297, 1.76776695297, 0},
    {"E extrude_1/edge:cap_bottom/piece:0|side:{3}/chord:3", -1.76776695297, 4.26776695297, 0},
    {"E extrude_1/edge:cap_bottom/piece:1|side:{0}", 10, -5, 0},
    {"E extrude_1/edge:cap_bottom/piece:1|side:{1}/chord:0", 21.767766953, 4.26776695297, 0},
    {"E extrude_1/edge:cap_bottom/piece:1|side:{1}/chord:1", 24.267766953, 1.76776695297, 0},
    {"E extrude_1/edge:cap_bottom/piece:1|side:{1}/chord:2", 24.267766953, -1.76776695297, 0},
    {"E extrude_1/edge:cap_bottom/piece:1|side:{1}/chord:3", 21.767766953, -4.26776695297, 0},
    {"E extrude_1/edge:cap_bottom/piece:1|side:{2}", 15, 5, 0},
    {"E extrude_1/edge:cap_top/piece:0|cap_top/piece:1/chord:0", 2.46966991411, -2.53033008589, 6},
    {"E extrude_1/edge:cap_top/piece:0|cap_top/piece:1/chord:1", 8.53033008589, 3.53033008589, 6},
    {"E extrude_1/edge:cap_top/piece:0|side:{2}", 5, 5, 6},
    {"E extrude_1/edge:cap_top/piece:0|side:{3}/chord:0", -1.76776695297, 4.26776695297, 6},
    {"E extrude_1/edge:cap_top/piece:0|side:{3}/chord:1", -4.26776695297, 1.76776695297, 6},
    {"E extrude_1/edge:cap_top/piece:0|side:{3}/chord:2", -4.26776695297, -1.76776695297, 6},
    {"E extrude_1/edge:cap_top/piece:0|side:{3}/chord:3", -1.76776695297, -4.26776695297, 6},
    {"E extrude_1/edge:cap_top/piece:1|side:{0}", 10, -5, 6},
    {"E extrude_1/edge:cap_top/piece:1|side:{1}/chord:0", 21.767766953, -4.26776695297, 6},
    {"E extrude_1/edge:cap_top/piece:1|side:{1}/chord:1", 24.267766953, -1.76776695297, 6},
    {"E extrude_1/edge:cap_top/piece:1|side:{1}/chord:2", 24.267766953, 1.76776695297, 6},
    {"E extrude_1/edge:cap_top/piece:1|side:{1}/chord:3", 21.767766953, 4.26776695297, 6},
    {"E extrude_1/edge:cap_top/piece:1|side:{2}", 15, 5, 6},
    {"E extrude_1/edge:side:{0}|side:{1}", 20, -5, 3},
    {"E extrude_1/edge:side:{0}|side:{3}", 0, -5, 3},
    {"E extrude_1/edge:side:{1}|side:{2}", 20, 5, 3},
    {"E extrude_1/edge:side:{2}|side:{3}", 0, 5, 3},
    {"E extrude_1/side:{1}/seam:0", 23.5355339059, -3.53553390593, 3},
    {"E extrude_1/side:{1}/seam:1", 25, -1.22464679915e-15, 3},
    {"E extrude_1/side:{1}/seam:2", 23.5355339059, 3.53553390593, 3},
    {"E extrude_1/side:{3}/seam:0", -3.53553390593, 3.53553390593, 3},
    {"E extrude_1/side:{3}/seam:1", -5, 6.12323399574e-16, 3},
    {"E extrude_1/side:{3}/seam:2", -3.53553390593, -3.53553390593, 3},
    {"E hole_1/wall/seam:0", 7.5, 1, 3},
    {"E hole_1/wall/seam:1", 7.06066017178, -0.0606601717798, 3},
    {"E hole_1/wall/seam:2", 6, -0.5, 3},
    {"E hole_1/wall/seam:3", 4.93933982822, -0.0606601717798, 3},
    {"E hole_1/wall/seam:4", 4.5, 1, 3},
    {"E hole_1/wall/seam:5", 4.93933982822, 2.06066017178, 3},
    {"E hole_1/wall/seam:6", 6, 2.5, 3},
    {"E hole_1/wall/seam:7", 7.06066017178, 2.06066017178, 3},
    {"F extrude_1/cap_bottom/piece:0", 1.63473577175, 1.11651359074, 0},
    {"F extrude_1/cap_bottom/piece:1", 13.8022513515, -0.350248252596, 0},
    {"F extrude_1/cap_top/piece:0", 1.63473577175, 1.11651359074, 6},
    {"F extrude_1/cap_top/piece:1", 14.194021791, -0.477856566792, 6},
    {"F extrude_1/side:{0}", 10, -5, 3},
    {"F extrude_1/side:{1}/facet:0", 21.767766953, -4.26776695297, 3},
    {"F extrude_1/side:{1}/facet:1", 24.267766953, -1.76776695297, 3},
    {"F extrude_1/side:{1}/facet:2", 24.267766953, 1.76776695297, 3},
    {"F extrude_1/side:{1}/facet:3", 21.767766953, 4.26776695297, 3},
    {"F extrude_1/side:{2}", 10, 5, 3},
    {"F extrude_1/side:{3}/facet:0", -1.76776695297, 4.26776695297, 3},
    {"F extrude_1/side:{3}/facet:1", -4.26776695297, 1.76776695297, 3},
    {"F extrude_1/side:{3}/facet:2", -4.26776695297, -1.76776695297, 3},
    {"F extrude_1/side:{3}/facet:3", -1.76776695297, -4.26776695297, 3},
    {"F hole_1/wall/facet:0", 7.28033008589, 0.46966991411, 3},
    {"F hole_1/wall/facet:1", 6.53033008589, -0.28033008589, 3},
    {"F hole_1/wall/facet:2", 5.46966991411, -0.28033008589, 3},
    {"F hole_1/wall/facet:3", 4.71966991411, 0.46966991411, 3},
    {"F hole_1/wall/facet:4", 4.71966991411, 1.53033008589, 3},
    {"F hole_1/wall/facet:5", 5.46966991411, 2.28033008589, 3},
    {"F hole_1/wall/facet:6", 6.53033008589, 2.28033008589, 3},
    {"F hole_1/wall/facet:7", 7.28033008589, 1.53033008589, 3},
};
// revolve: 36 faces, 85 edges, 209.939901909.
const Place kRevolve[] = {
    {"E edge:hole_1/wall|revolve_1/revolved:{2}/piece:0/chord:0", 2.9267766953, -0.176776695297, 5},
    {"E edge:hole_1/wall|revolve_1/revolved:{2}/piece:0/chord:1", 2.6767766953, -0.426776695297, 5},
    {"E edge:hole_1/wall|revolve_1/revolved:{2}/piece:0/chord:2", 2.3232233047, -0.426776695297, 5},
    {"E edge:hole_1/wall|revolve_1/revolved:{2}/piece:0/chord:3", 2.0732233047, -0.176776695297, 5},
    {"E edge:hole_1/wall|revolve_1/revolved:{2}/piece:1/chord:0", 2.0732233047, 0.176776695297, 5},
    {"E edge:hole_1/wall|revolve_1/revolved:{2}/piece:1/chord:1", 2.3232233047, 0.426776695297, 5},
    {"E edge:hole_1/wall|revolve_1/revolved:{2}/piece:1/chord:2", 2.6767766953, 0.426776695297, 5},
    {"E edge:hole_1/wall|revolve_1/revolved:{2}/piece:1/chord:3", 2.9267766953, 0.176776695297, 5},
    {"E hole_1/bottom/seam:0", 2.75, -5.12188434238e-17, 1.84978484524},
    {"E hole_1/bottom/seam:1", 2.6767766953, -0.176776695297, 1.84978484524},
    {"E hole_1/bottom/seam:2", 2.5, -0.25, 1.84978484524},
    {"E hole_1/bottom/seam:3", 2.3232233047, -0.176776695297, 1.84978484524},
    {"E hole_1/bottom/seam:4", 2.25, -8.18350134025e-17, 1.84978484524},
    {"E hole_1/bottom/seam:5", 2.3232233047, 0.176776695297, 1.84978484524},
    {"E hole_1/bottom/seam:6", 2.5, 0.25, 1.84978484524},
    {"E hole_1/bottom/seam:7", 2.6767766953, 0.176776695297, 1.84978484524},
    {"E hole_1/edge:bottom|wall/chord:0", 2.9267766953, -0.176776695297, 2},
    {"E hole_1/edge:bottom|wall/chord:1", 2.6767766953, -0.426776695297, 2},
    {"E hole_1/edge:bottom|wall/chord:2", 2.3232233047, -0.426776695297, 2},
    {"E hole_1/edge:bottom|wall/chord:3", 2.0732233047, -0.176776695297, 2},
    {"E hole_1/edge:bottom|wall/chord:4", 2.0732233047, 0.176776695297, 2},
    {"E hole_1/edge:bottom|wall/chord:5", 2.3232233047, 0.426776695297, 2},
    {"E hole_1/edge:bottom|wall/chord:6", 2.6767766953, 0.426776695297, 2},
    {"E hole_1/edge:bottom|wall/chord:7", 2.9267766953, 0.176776695297, 2},
    {"E hole_1/wall/seam:0", 3, -2.43882596462e-17, 3.5},
    {"E hole_1/wall/seam:1", 2.85355339059, -0.353553390593, 3.5},
    {"E hole_1/wall/seam:2", 2.5, -0.5, 3.5},
    {"E hole_1/wall/seam:3", 2.14644660941, -0.353553390593, 3.5},
    {"E hole_1/wall/seam:4", 2, -8.56205996036e-17, 3.5},
    {"E hole_1/wall/seam:5", 2.14644660941, 0.353553390593, 3.5},
    {"E hole_1/wall/seam:6", 2.5, 0.5, 3.5},
    {"E hole_1/wall/seam:7", 2.85355339059, 0.353553390593, 3.5},
    {"E revolve_1/edge:revolved:{0}/piece:0|revolved:{0}/piece:1/chord:0", -4.5924254968e-16, -2.5, 0},
    {"E revolve_1/edge:revolved:{0}/piece:0|revolved:{0}/piece:1/chord:1", 1.53080849893e-16, 2.5, 0},
    {"E revolve_1/edge:revolved:{0}/piece:0|revolved:{1}/chord:0", 3.41421356237, -1.41421356237, 0},
    {"E revolve_1/edge:revolved:{0}/piece:0|revolved:{1}/chord:1", 1.41421356237, -3.41421356237, 0},
    {"E revolve_1/edge:revolved:{0}/piece:0|revolved:{1}/chord:2", 1.41421356237, 3.41421356237, 0},
    {"E revolve_1/edge:revolved:{0}/piece:0|revolved:{1}/chord:3", 3.41421356237, 1.41421356237, 0},
    {"E revolve_1/edge:revolved:{0}/piece:0|revolved:{3}/chord:0", 0.353553390593, -0.853553390593, 0},
    {"E revolve_1/edge:revolved:{0}/piece:0|revolved:{3}/chord:1", 0.853553390593, -0.353553390593, 0},
    {"E revolve_1/edge:revolved:{0}/piece:0|revolved:{3}/chord:2", 0.853553390593, 0.353553390593, 0},
    {"E revolve_1/edge:revolved:{0}/piece:0|revolved:{3}/chord:3", 0.353553390593, 0.853553390593, 0},
    {"E revolve_1/edge:revolved:{0}/piece:1|revolved:{1}/chord:0", -1.41421356237, -3.41421356237, 0},
    {"E revolve_1/edge:revolved:{0}/piece:1|revolved:{1}/chord:1", -3.41421356237, -1.41421356237, 0},
    {"E revolve_1/edge:revolved:{0}/piece:1|revolved:{1}/chord:2", -3.41421356237, 1.41421356237, 0},
    {"E revolve_1/edge:revolved:{0}/piece:1|revolved:{1}/chord:3", -1.41421356237, 3.41421356237, 0},
    {"E revolve_1/edge:revolved:{0}/piece:1|revolved:{3}/chord:0", -0.353553390593, 0.853553390593, 0},
    {"E revolve_1/edge:revolved:{0}/piece:1|revolved:{3}/chord:1", -0.853553390593, 0.353553390593, 0},
    {"E revolve_1/edge:revolved:{0}/piece:1|revolved:{3}/chord:2", -0.853553390593, -0.353553390593, 0},
    {"E revolve_1/edge:revolved:{0}/piece:1|revolved:{3}/chord:3", -0.353553390593, -0.853553390593, 0},
    {"E revolve_1/edge:revolved:{1}|revolved:{2}/piece:0/chord:0", -3.41421356237, -1.41421356237, 5},
    {"E revolve_1/edge:revolved:{1}|revolved:{2}/piece:0/chord:1", -1.41421356237, -3.41421356237, 5},
    {"E revolve_1/edge:revolved:{1}|revolved:{2}/piece:0/chord:2", 1.41421356237, -3.41421356237, 5},
    {"E revolve_1/edge:revolved:{1}|revolved:{2}/piece:0/chord:3", 3.41421356237, -1.41421356237, 5},
    {"E revolve_1/edge:revolved:{1}|revolved:{2}/piece:1/chord:0", 3.41421356237, 1.41421356237, 5},
    {"E revolve_1/edge:revolved:{1}|revolved:{2}/piece:1/chord:1", 1.41421356237, 3.41421356237, 5},
    {"E revolve_1/edge:revolved:{1}|revolved:{2}/piece:1/chord:2", -1.41421356237, 3.41421356237, 5},
    {"E revolve_1/edge:revolved:{1}|revolved:{2}/piece:1/chord:3", -3.41421356237, 1.41421356237, 5},
    {"E revolve_1/edge:revolved:{2}/piece:0|revolved:{2}/piece:1/chord:0", 3.5, 0, 5},
    {"E revolve_1/edge:revolved:{2}/piece:0|revolved:{2}/piece:1/chord:1", 1.5, -3.06161699787e-17, 5},
    {"E revolve_1/edge:revolved:{2}/piece:0|revolved:{2}/piece:1/chord:2", -2.5, 3.06161699787e-16, 5},
    {"E revolve_1/edge:revolved:{2}/piece:0|revolved:{3}/chord:0", 0.853553390593, -0.353553390593, 5},
    {"E revolve_1/edge:revolved:{2}/piece:0|revolved:{3}/chord:1", 0.353553390593, -0.853553390593, 5},
    {"E revolve_1/edge:revolved:{2}/piece:0|revolved:{3}/chord:2", -0.353553390593, -0.853553390593, 5},
    {"E revolve_1/edge:revolved:{2}/piece:0|revolved:{3}/chord:3", -0.853553390593, -0.353553390593, 5},
    {"E revolve_1/edge:revolved:{2}/piece:1|revolved:{3}/chord:0", -0.853553390593, 0.353553390593, 5},
    {"E revolve_1/edge:revolved:{2}/piece:1|revolved:{3}/chord:1", -0.353553390593, 0.853553390593, 5},
    {"E revolve_1/edge:revolved:{2}/piece:1|revolved:{3}/chord:2", 0.353553390593, 0.853553390593, 5},
    {"E revolve_1/edge:revolved:{2}/piece:1|revolved:{3}/chord:3", 0.853553390593, 0.353553390593, 5},
    {"E revolve_1/revolved:{1}/seam:0", 2.82842712475, 2.82842712475, 2.5},
    {"E revolve_1/revolved:{1}/seam:1", 4, 0, 2.5},
    {"E revolve_1/revolved:{1}/seam:2", 2.44929359829e-16, 4, 2.5},
    {"E revolve_1/revolved:{1}/seam:3", -2.82842712475, 2.82842712475, 2.5},
    {"E revolve_1/revolved:{1}/seam:4", -4, 4.89858719659e-16, 2.5},
    {"E revolve_1/revolved:{1}/seam:5", -2.82842712475, -2.82842712475, 2.5},
    {"E revolve_1/revolved:{1}/seam:6", -7.34788079488e-16, -4, 2.5},
    {"E revolve_1/revolved:{1}/seam:7", 2.82842712475, -2.82842712475, 2.5},
    {"E revolve_1/revolved:{3}/seam:0", 0.707106781187, 0.707106781187, 2.5},
    {"E revolve_1/revolved:{3}/seam:1", 1, 0, 2.5},
    {"E revolve_1/revolved:{3}/seam:2", 6.12323399574e-17, 1, 2.5},
    {"E revolve_1/revolved:{3}/seam:3", -0.707106781187, 0.707106781187, 2.5},
    {"E revolve_1/revolved:{3}/seam:4", -1, 1.22464679915e-16, 2.5},
    {"E revolve_1/revolved:{3}/seam:5", -0.707106781187, -0.707106781187, 2.5},
    {"E revolve_1/revolved:{3}/seam:6", -1.83697019872e-16, -1, 2.5},
    {"E revolve_1/revolved:{3}/seam:7", 0.707106781187, -0.707106781187, 2.5},
    {"F hole_1/bottom/facet:0", 2.78451779686, -0.117851130198, 1.8998565635},
    {"F hole_1/bottom/facet:1", 2.6178511302, -0.284517796864, 1.8998565635},
    {"F hole_1/bottom/facet:2", 2.3821488698, -0.284517796864, 1.8998565635},
    {"F hole_1/bottom/facet:3", 2.21548220314, -0.117851130198, 1.8998565635},
    {"F hole_1/bottom/facet:4", 2.21548220314, 0.117851130198, 1.8998565635},
    {"F hole_1/bottom/facet:5", 2.3821488698, 0.284517796864, 1.8998565635},
    {"F hole_1/bottom/facet:6", 2.6178511302, 0.284517796864, 1.8998565635},
    {"F hole_1/bottom/facet:7", 2.78451779686, 0.117851130198, 1.8998565635},
    {"F hole_1/wall/facet:0", 2.9267766953, -0.176776695297, 3.5},
    {"F hole_1/wall/facet:1", 2.6767766953, -0.426776695297, 3.5},
    {"F hole_1/wall/facet:2", 2.3232233047, -0.426776695297, 3.5},
    {"F hole_1/wall/facet:3", 2.0732233047, -0.176776695297, 3.5},
    {"F hole_1/wall/facet:4", 2.0732233047, 0.176776695297, 3.5},
    {"F hole_1/wall/facet:5", 2.3232233047, 0.426776695297, 3.5},
    {"F hole_1/wall/facet:6", 2.6767766953, 0.426776695297, 3.5},
    {"F hole_1/wall/facet:7", 2.9267766953, 0.176776695297, 3.5},
    {"F revolve_1/revolved:{0}/piece:0", 1.68994949366, -2.09345661158e-16, 0},
    {"F revolve_1/revolved:{0}/piece:1", -1.41777219078, -0.178924517536, 0},
    {"F revolve_1/revolved:{1}/facet:0", 3.41421356237, 1.41421356237, 2.5},
    {"F revolve_1/revolved:{1}/facet:1", 1.41421356237, 3.41421356237, 2.5},
    {"F revolve_1/revolved:{1}/facet:2", -1.41421356237, 3.41421356237, 2.5},
    {"F revolve_1/revolved:{1}/facet:3", -3.41421356237, 1.41421356237, 2.5},
    {"F revolve_1/revolved:{1}/facet:4", -3.41421356237, -1.41421356237, 2.5},
    {"F revolve_1/revolved:{1}/facet:5", -1.41421356237, -3.41421356237, 2.5},
    {"F revolve_1/revolved:{1}/facet:6", 1.41421356237, -3.41421356237, 2.5},
    {"F revolve_1/revolved:{1}/facet:7", 3.41421356237, -1.41421356237, 2.5},
    {"F revolve_1/revolved:{2}/piece:0", -0.00799016050296, -1.67084208306, 5},
    {"F revolve_1/revolved:{2}/piece:1", 0.253609326654, 1.38349835322, 5},
    {"F revolve_1/revolved:{3}/facet:0", 0.853553390593, 0.353553390593, 2.5},
    {"F revolve_1/revolved:{3}/facet:1", 0.353553390593, 0.853553390593, 2.5},
    {"F revolve_1/revolved:{3}/facet:2", -0.353553390593, 0.853553390593, 2.5},
    {"F revolve_1/revolved:{3}/facet:3", -0.853553390593, 0.353553390593, 2.5},
    {"F revolve_1/revolved:{3}/facet:4", -0.853553390593, -0.353553390593, 2.5},
    {"F revolve_1/revolved:{3}/facet:5", -0.353553390593, -0.853553390593, 2.5},
    {"F revolve_1/revolved:{3}/facet:6", 0.353553390593, -0.853553390593, 2.5},
    {"F revolve_1/revolved:{3}/facet:7", 0.853553390593, -0.353553390593, 2.5},
};
// hole: 16 faces, 44 edges, 3886.86291501.
const Place kHole[] = {
    {"E edge:hole_1/wall|primitive_1/bottom/piece:0/chord:0", 7.70710678119, 9.29289321881, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:0/chord:1", 6.70710678119, 8.29289321881, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:0/chord:2", 6.70710678119, 11.7071067812, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:0/chord:3", 7.70710678119, 10.7071067812, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:1/chord:0", 5.29289321881, 8.29289321881, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:1/chord:1", 4.29289321881, 9.29289321881, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:1/chord:2", 4.29289321881, 10.7071067812, 0},
    {"E edge:hole_1/wall|primitive_1/bottom/piece:1/chord:3", 5.29289321881, 11.7071067812, 0},
    {"E edge:hole_1/wall|primitive_1/top/piece:0/chord:0", 7.70710678119, 9.29289321881, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:0/chord:1", 6.70710678119, 8.29289321881, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:0/chord:2", 6.70710678119, 11.7071067812, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:0/chord:3", 7.70710678119, 10.7071067812, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:1/chord:0", 5.29289321881, 8.29289321881, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:1/chord:1", 4.29289321881, 9.29289321881, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:1/chord:2", 4.29289321881, 10.7071067812, 10},
    {"E edge:hole_1/wall|primitive_1/top/piece:1/chord:3", 5.29289321881, 11.7071067812, 10},
    {"E hole_1/wall/seam:0", 8, 10, 5},
    {"E hole_1/wall/seam:1", 7.41421356237, 8.58578643763, 5},
    {"E hole_1/wall/seam:2", 6, 8, 5},
    {"E hole_1/wall/seam:3", 4.58578643763, 8.58578643763, 5},
    {"E hole_1/wall/seam:4", 4, 10, 5},
    {"E hole_1/wall/seam:5", 4.58578643763, 11.4142135624, 5},
    {"E hole_1/wall/seam:6", 6, 12, 5},
    {"E hole_1/wall/seam:7", 7.41421356237, 11.4142135624, 5},
    {"E primitive_1/edge:back|bottom/piece:0", 13, 20, 0},
    {"E primitive_1/edge:back|bottom/piece:1", 3, 20, 0},
    {"E primitive_1/edge:back|left", 0, 20, 5},
    {"E primitive_1/edge:back|right", 20, 20, 5},
    {"E primitive_1/edge:back|top/piece:0", 13, 20, 10},
    {"E primitive_1/edge:back|top/piece:1", 3, 20, 10},
    {"E primitive_1/edge:bottom/piece:0|bottom/piece:1/chord:0", 6, 4, 0},
    {"E primitive_1/edge:bottom/piece:0|bottom/piece:1/chord:1", 6, 16, 0},
    {"E primitive_1/edge:bottom/piece:0|front", 13, 0, 0},
    {"E primitive_1/edge:bottom/piece:0|right", 20, 10, 0},
    {"E primitive_1/edge:bottom/piece:1|front", 3, 0, 0},
    {"E primitive_1/edge:bottom/piece:1|left", 0, 10, 0},
    {"E primitive_1/edge:front|left", 0, 0, 5},
    {"E primitive_1/edge:front|right", 20, 0, 5},
    {"E primitive_1/edge:front|top/piece:0", 13, 0, 10},
    {"E primitive_1/edge:front|top/piece:1", 3, 0, 10},
    {"E primitive_1/edge:left|top/piece:1", 0, 10, 10},
    {"E primitive_1/edge:right|top/piece:0", 20, 10, 10},
    {"E primitive_1/edge:top/piece:0|top/piece:1/chord:0", 6, 16, 10},
    {"E primitive_1/edge:top/piece:0|top/piece:1/chord:1", 6, 4, 10},
    {"F hole_1/wall/facet:0", 7.70710678119, 9.29289321881, 5},
    {"F hole_1/wall/facet:1", 6.70710678119, 8.29289321881, 5},
    {"F hole_1/wall/facet:2", 5.29289321881, 8.29289321881, 5},
    {"F hole_1/wall/facet:3", 4.29289321881, 9.29289321881, 5},
    {"F hole_1/wall/facet:4", 4.29289321881, 10.7071067812, 5},
    {"F hole_1/wall/facet:5", 5.29289321881, 11.7071067812, 5},
    {"F hole_1/wall/facet:6", 6.70710678119, 11.7071067812, 5},
    {"F hole_1/wall/facet:7", 7.70710678119, 10.7071067812, 5},
    {"F primitive_1/back", 10, 20, 5},
    {"F primitive_1/bottom/piece:0", 13.0661129381, 10.0679993056, 0},
    {"F primitive_1/bottom/piece:1", 3.33333333333, 9.55632584345, 0},
    {"F primitive_1/front", 10, 0, 5},
    {"F primitive_1/left", 0, 10, 5},
    {"F primitive_1/right", 20, 10, 5},
    {"F primitive_1/top/piece:0", 12.5777777778, 9.79295206028, 10},
    {"F primitive_1/top/piece:1", 3.33333333333, 9.55632584345, 10},
};
// clang-format on

/// The centroid of @p face's outer loop, by area (as BridgeNumbering's tables have it).
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

/// @p tag with the two faces an edge `<source>/edge:<a>|<b>` is named after in sorted order.
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

/// @p recorded with each "{k}" the k-th entity of @p profile's name, "e<id>".
std::string named(std::string recorded, const Profile& profile) {
    for (std::size_t k = 0; k < profile.size(); ++k) {
        const std::string mark = "{" + std::to_string(k) + "}";
        const std::string id = "e" + std::to_string(profile[k]->id());
        for (std::size_t at = recorded.find(mark); at != std::string::npos;
             at = recorded.find(mark)) {
            recorded.replace(at, mark.size(), id);
        }
    }
    return recorded;
}

/// @p solid's faces and edges, each name with its place; a name twice is said.
std::map<std::string, Vec3> placesOf(const hz::topo::Solid& solid) {
    std::map<std::string, Vec3> places;
    for (const auto& face : solid.faces()) {
        if (!face.alive) continue;
        EXPECT_TRUE(places.emplace("F " + face.topoId.tag(), centroidOf(face)).second)
            << face.topoId.tag() << " names two faces";
    }
    for (const auto& edge : solid.edges()) {
        if (!edge.alive) continue;
        const Vec3 mid = (edge.halfEdge->origin->point + edge.halfEdge->twin->origin->point) * 0.5;
        EXPECT_TRUE(places.emplace("E " + canonical(edge.topoId.tag()), mid).second)
            << edge.topoId.tag() << " names two edges";
    }
    return places;
}

/// @p solid names every face and edge as @p masters named it at the same place (within 1e-9 of
/// 100, the parts' reach), and names nothing master did not.
template <std::size_t N>
void expectNamedAsMasters(const hz::topo::Solid& solid, const Place (&masters)[N],
                          const Profile& profile = {}) {
    const auto places = placesOf(solid);
    std::set<std::string> recorded;
    for (const Place& place : masters) {
        const std::string tag = canonical(named(place.tag, profile));
        recorded.insert(tag);
        const auto found = places.find(tag);
        if (found == places.end()) {
            ADD_FAILURE() << tag << " is gone";
            continue;
        }
        EXPECT_LT((found->second - Vec3(place.x, place.y, place.z)).length(), 1e-7) << tag;
    }
    for (const auto& entry : places) {
        EXPECT_EQ(recorded.count(entry.first), 1u) << entry.first << " is no name of master's";
    }
}

std::shared_ptr<hz::draft::DraftEntity> line(Vec2 a, Vec2 b) {
    return std::make_shared<hz::draft::DraftLine>(a, b);
}

std::shared_ptr<hz::draft::DraftEntity> arc(Vec2 c, double r, double a0, double a1) {
    return std::make_shared<hz::draft::DraftArc>(c, r, a0, a1);
}

/// A slot of @p len between its arcs' centres, @p r about, from (@p x0, @p y0).
Profile slot(double x0, double y0, double len, double r) {
    return {line({x0, y0 - r}, {x0 + len, y0 - r}), arc({x0 + len, y0}, r, -kPi / 2, kPi / 2),
            line({x0 + len, y0 + r}, {x0, y0 + r}), arc({x0, y0}, r, kPi / 2, 3 * kPi / 2)};
}

std::unique_ptr<PrimitiveFeature> box(double w, double h, double d, const char* id) {
    auto b = PrimitiveFeature::makeBox(w, h, d);
    b->restoreFeatureID(id);
    return b;
}

/// The whole name of @p solid's flat face facing up through @p point.
std::optional<std::string> topAt(const hz::topo::Solid& solid, const Vec3& point) {
    const double outward = hz::model::outwardSign(solid);
    for (const auto& face : solid.faces()) {
        const auto plane = hz::model::planeOf(face, outward);
        if (plane && plane->normal.dot(Vec3(0, 0, 1)) > 0.999 &&
            std::abs((point - plane->origin).dot(plane->normal)) < 1e-6) {
            return hz::model::wholeFaceName(face.topoId.tag());
        }
    }
    return std::nullopt;
}

/// A hole @p dia across into @p part's face facing up at @p at, through all or @p depth deep,
/// @p segments round, as `hole_1`.
void drill(Document& part, const Vec3& at, double dia, int segments, bool through = true,
           double depth = 6.0) {
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    const auto face = topAt(*part.solid(), at);
    ASSERT_TRUE(face.has_value());
    auto hole = hz::doc::HoleFeature::make(*face, at, dia, depth);
    hole->restoreFeatureID("hole_1");
    ASSERT_TRUE(hole->setParameter("segments", segments));
    if (through) {
        ASSERT_TRUE(hole->setParameter("extent", 1));
    }
    part.featureTree().addFeature(std::move(hole));
}

/// @p part, built: its faceted part.
const hz::topo::Solid& built(Document& part) {
    EXPECT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_NE(part.solid(), nullptr);
    return *part.solid();
}

}  // namespace

// A box joined by a cylinder standing in it: the top's pieces round the cylinder, and the
// cylinder's chords and seams where they meet it, as master numbered them.
TEST(DownstreamNumbering, AJoinNamesAsMastersDid) {
    Document part;
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(box(20, 20, 10, "primitive_1"));
    auto cylinder = PrimitiveFeature::makeCylinder(4, 20);
    cylinder->restoreFeatureID("primitive_2");
    ASSERT_TRUE(cylinder->setParameter("segments", 8));
    ASSERT_TRUE(cylinder->setVector("basePoint", Vec3(10, 10, 0)));
    cylinder->setOperation(hz::doc::BodyOperation::Join);
    part.featureTree().addFeature(std::move(cylinder));
    expectNamedAsMasters(built(part), kJoin);
}

// A box less a slot (lines and arcs) extruded through it.
TEST(DownstreamNumbering, ACutNamesAsMastersDid) {
    Document part;
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(box(30, 20, 10, "primitive_1"));
    const Profile profile = slot(8, 10, 12, 3);
    auto sketch = std::make_shared<hz::doc::Sketch>();
    for (const auto& e : profile) sketch->addEntity(e);
    part.addSketch(sketch);
    auto cut = std::make_unique<hz::doc::ExtrudeFeature>(sketch, Vec3(0, 0, 1), 10.0);
    cut->restoreFeatureID("extrude_1");
    ASSERT_TRUE(cut->setParameter("segments", 8));
    cut->setOperation(hz::doc::BodyOperation::Cut);
    part.featureTree().addFeature(std::move(cut));
    expectNamedAsMasters(built(part), kCut, profile);
}

// A cylinder shelled through its top.
TEST(DownstreamNumbering, AShellNamesAsMastersDid) {
    Document part;
    part.setType(DocumentType::Part);
    auto cylinder = PrimitiveFeature::makeCylinder(5, 10);
    cylinder->restoreFeatureID("primitive_1");
    ASSERT_TRUE(cylinder->setParameter("segments", 8));
    part.featureTree().addFeature(std::move(cylinder));
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    const auto top = topAt(*part.solid(), Vec3(0, 0, 10));
    ASSERT_TRUE(top.has_value());
    auto shell = std::make_unique<hz::doc::ShellFeature>(
        1.0, std::vector<hz::topo::TopologyID>{hz::topo::TopologyID::fromTag(*top)});
    shell->restoreFeatureID("shell_1");
    part.featureTree().addFeature(std::move(shell));
    expectNamedAsMasters(built(part), kShell);
}

// A box with a hole through it, the hole patterned three times along it.
TEST(DownstreamNumbering, APatternNamesAsMastersDid) {
    Document part;
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(box(40, 20, 10, "primitive_1"));
    drill(part, Vec3(8, 10, 10), 4, 8);
    auto row = hz::doc::PatternFeature::makeLinear(Vec3(1, 0, 0), 10.0, 3);
    row->restoreFeatureID("pattern_1");
    row->setTargets({"hole_1"});
    part.featureTree().addFeature(std::move(row));
    expectNamedAsMasters(built(part), kPattern);
}

// A slot (lines and arcs) extruded, with a hole through it off its middle: its caps' pieces.
TEST(DownstreamNumbering, AnExtrudeWithArcsNamesAsMastersDid) {
    Document part;
    part.setType(DocumentType::Part);
    const Profile profile = slot(0, 0, 20, 5);
    auto sketch = std::make_shared<hz::doc::Sketch>();
    for (const auto& e : profile) sketch->addEntity(e);
    part.addSketch(sketch);
    auto extrude = std::make_unique<hz::doc::ExtrudeFeature>(sketch, Vec3(0, 0, 1), 6.0);
    extrude->restoreFeatureID("extrude_1");
    ASSERT_TRUE(extrude->setParameter("segments", 8));
    part.featureTree().addFeature(std::move(extrude));
    drill(part, Vec3(6, 1, 6), 3, 8);
    expectNamedAsMasters(built(part), kExtrude, profile);
}

// A ring turned from a rectangle on the XZ plane about the z axis, a blind hole into its top.
TEST(DownstreamNumbering, ARevolveNamesAsMastersDid) {
    Document part;
    part.setType(DocumentType::Part);
    const Profile profile = {line({1, 0}, {4, 0}), line({4, 0}, {4, 5}), line({4, 5}, {1, 5}),
                             line({1, 5}, {1, 0})};
    auto sketch = std::make_shared<hz::doc::Sketch>(
        hz::draft::SketchPlane(Vec3(0, 0, 0), Vec3(0, -1, 0), Vec3(1, 0, 0)));
    for (const auto& e : profile) sketch->addEntity(e);
    part.addSketch(sketch);
    auto revolve =
        std::make_unique<hz::doc::RevolveFeature>(sketch, Vec3(0, 0, 0), Vec3(0, 0, 1), 2 * kPi);
    revolve->restoreFeatureID("revolve_1");
    ASSERT_TRUE(revolve->setParameter("segments", 8));
    part.featureTree().addFeature(std::move(revolve));
    drill(part, Vec3(2.5, 0, 5), 1, 8, false, 3.0);
    expectNamedAsMasters(built(part), kRevolve, profile);
}

// A box drilled through off its middle: its bottom's and top's two pieces, as master numbered
// them (the bridge cut from the exact box numbered its bottom's the other way round).
TEST(DownstreamNumbering, AHoleNamesAsMastersDid) {
    Document part;
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(box(20, 20, 10, "primitive_1"));
    drill(part, Vec3(6, 10, 10), 4, 8);
    expectNamedAsMasters(built(part), kHole);
}

// The regression set's example (Task 16): a sheet saved with a length on an edge of a piece of a
// drilled box's bottom measures what it measured on master, 14 (and its neighbour 6), where the
// bridge cut from the exact box read 6 for it, saying nothing.
TEST(DownstreamNumbering, ASheetsLengthOnAPiecesEdgeMeasuresAsOnMaster) {
    const auto dir = std::filesystem::temp_directory_path() / "hz_downstream_numbering";
    std::filesystem::create_directories(dir);
    Document part;
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(box(20, 20, 10, "primitive_1"));
    drill(part, Vec3(6, 10, 10), 4, 32);
    const hz::topo::Solid& solid = built(part);
    const std::string partPath = (dir / "drilled.hzpart").string();
    ASSERT_TRUE(hz::io::NativeFormat::save(partPath, part));
    hz::io::DrawingDocumentSpec spec;
    spec.partPath = partPath;
    spec.views = hz::io::DrawingDocumentIO::viewsOf(
        hz::model::DrawingGenerator::sheetLayout(solid, spec.sheet, spec.titleBlock));
    const std::vector<std::pair<std::string, double>> lengths = {
        {"primitive_1/edge:bottom/piece:0|front", 14.0},
        {"primitive_1/edge:bottom/piece:1|front", 6.0},
        {"primitive_1/edge:back|bottom/piece:0", 14.0}};
    ASSERT_FALSE(spec.views.empty());
    for (const auto& length : lengths) {
        spec.views.front().dimensions.push_back(
            {length.first, hz::io::DrawingDimensionSpec::Kind::Length});
    }
    const std::string sheet = (dir / "drilled.hzdwg").string();
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(sheet, spec));
    hz::io::DrawingDocumentSpec read;
    hz::model::Drawing drawing;
    std::string error;
    ASSERT_TRUE(hz::io::DrawingDocumentIO::load(sheet, read, drawing, &error)) << error;
    ASSERT_FALSE(drawing.views.empty());
    std::map<std::string, double> measured;
    for (const auto& dimension : drawing.views.front().dimensions) {
        measured[dimension.edge.tag()] = dimension.value;
    }
    for (const auto& length : lengths) {
        const auto found = measured.find(length.first);
        ASSERT_NE(found, measured.end()) << length.first << " is not drawn";
        EXPECT_NEAR(found->second, length.second, 1e-9) << length.first;
    }
    std::filesystem::remove_all(dir);
}
