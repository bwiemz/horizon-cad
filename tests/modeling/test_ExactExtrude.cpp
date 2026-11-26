// The exact extrude (stage 3): a profile of lines, arcs and circles swept along a direction, built
// from whole faces carrying exact geometry. A line sweeps a plane; an arc or a circle a cylinder
// when it is swept along the sketch's normal, and a NURBS extrusion along any other way. The caps
// are planes whose holes are inner loops, and each region of the profile, an island inside a
// hole among them, is its own shell: nothing is a Boolean. Faces are named with master's Stable
// logical names, edges after them (model::nameEdgesLogically).
//
// Master's logical face names below were recorded from master's faceted extrude
// (Extrude::execute, Stable naming, 32 segments) at 1709c556, before the extrude was exact; "{k}"
// in a recorded name is the k-th entity of the profile ("e<id>" in the tag).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/ExactValidator.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

using hz::draft::DraftArc;
using hz::draft::DraftCircle;
using hz::draft::DraftEntity;
using hz::draft::DraftLine;
using hz::draft::SketchPlane;
using hz::geo::SurfaceKind;
using hz::math::kPi;
using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::Extrude;
using hz::model::NamingScheme;
using Profile = std::vector<std::shared_ptr<DraftEntity>>;

namespace {

std::shared_ptr<DraftEntity> line(Vec2 a, Vec2 b) {
    return std::make_shared<DraftLine>(a, b);
}

std::shared_ptr<DraftEntity> arc(Vec2 c, double r, double a0, double a1) {
    return std::make_shared<DraftArc>(c, r, a0, a1);
}

std::shared_ptr<DraftEntity> circle(Vec2 c, double r) {
    return std::make_shared<DraftCircle>(c, r);
}

/// A 20 x 10 rectangle of four lines.
Profile rectangle() {
    return {line({0, 0}, {20, 0}), line({20, 0}, {20, 10}), line({20, 10}, {0, 10}),
            line({0, 10}, {0, 0})};
}

/// Two lines of 20 and two half circles of radius 5: (20 x 10 + 25 pi) in area.
Profile slot() {
    return {line({0, -5}, {20, -5}), arc({20, 0}, 5, -kPi / 2, kPi / 2), line({20, 5}, {0, 5}),
            arc({0, 0}, 5, kPi / 2, 3 * kPi / 2)};
}
const double kSlotArea = 20.0 * 10.0 + 25.0 * kPi;

/// A 100 x 60 plate with two round holes, of radius 5 and 8.
Profile plate() {
    return {line({0, 0}, {100, 0}), line({100, 0}, {100, 60}), line({100, 60}, {0, 60}),
            line({0, 60}, {0, 0}),  circle({20, 30}, 5),       circle({70, 30}, 8)};
}
const double kPlateArea = 6000.0 - 25.0 * kPi - 64.0 * kPi;

/// A 40 x 40 square with a 20 x 20 square hole, and a disc of radius 5 inside the hole.
Profile island() {
    return {line({0, 0}, {40, 0}),    line({40, 0}, {40, 40}),  line({40, 40}, {0, 40}),
            line({0, 40}, {0, 0}),    line({10, 10}, {30, 10}), line({30, 10}, {30, 30}),
            line({30, 30}, {10, 30}), line({10, 30}, {10, 10}), circle({20, 20}, 5)};
}
const double kIslandArea = 1600.0 - 400.0 + 25.0 * kPi;

/// A half disc: a line and an arc between the same two corners.
Profile dProfile() {
    return {line({-5, 0}, {5, 0}), arc({0, 0}, 5, 0, kPi)};
}

/// A 100 x 70 profile whose corner is rounded by an arc of radius 20 that both its lines meet
/// tangentially, and a notch at the origin of two 0.01 edges.
Profile tangentAndTiny() {
    return {line({0.01, 0}, {100, 0}),    line({100, 0}, {100, 50}), arc({80, 50}, 20, 0, kPi / 2),
            line({80, 70}, {0, 70}),      line({0, 70}, {0, 0.01}),  line({0, 0.01}, {0.01, 0.01}),
            line({0.01, 0.01}, {0.01, 0})};
}
const double kTangentArea = 7000.0 - (400.0 - 100.0 * kPi) - 0.01 * 0.01;

/// A disc of radius 6 about (3, 4).
Profile disc() {
    return {circle({3, 4}, 6)};
}

/// A sketch plane tilted 30 degrees about x, then turned 20 degrees about z, at (3, -2, 5).
SketchPlane inclined() {
    const double a = 30.0 * kPi / 180.0;
    const double b = 20.0 * kPi / 180.0;
    const auto turn = [a, b](Vec3 p) {
        const Vec3 q(p.x, p.y * std::cos(a) - p.z * std::sin(a),
                     p.y * std::sin(a) + p.z * std::cos(a));
        return Vec3(q.x * std::cos(b) - q.y * std::sin(b), q.x * std::sin(b) + q.y * std::cos(b),
                    q.z);
    };
    return SketchPlane(Vec3(3, -2, 5), turn(Vec3(0, 0, 1)), turn(Vec3(1, 0, 0)));
}

/// A recorded name, "{k}" standing for the k-th entity of @p profile, as its tag has it.
std::string named(std::string recorded, const Profile& profile) {
    for (size_t k = 0; k < profile.size(); ++k) {
        const std::string mark = "{" + std::to_string(k) + "}";
        const std::string id = "e" + std::to_string(profile[k]->id());
        for (size_t at = recorded.find(mark); at != std::string::npos; at = recorded.find(mark)) {
            recorded.replace(at, mark.size(), id);
        }
    }
    return recorded;
}

std::set<std::string> named(const std::set<std::string>& recorded, const Profile& profile) {
    std::set<std::string> out;
    for (const auto& name : recorded) out.insert(named(name, profile));
    return out;
}

std::set<std::string> faceNames(const hz::topo::Solid& solid) {
    std::set<std::string> names;
    for (const auto& face : solid.faces()) {
        if (face.alive) names.insert(face.topoId.tag());
    }
    return names;
}

double volumeOf(const hz::topo::Solid& solid) {
    const auto props = hz::model::MassPropertiesCalculator::compute(solid);
    EXPECT_TRUE(props.valid) << props.problem;
    return props.volume;
}

/// @p solid is combinatorially valid, its exact geometry agrees with its topology at its own
/// tolerance, and it measures @p volume to 1e-9.
void expectValidAndExact(const hz::topo::Solid& solid, double volume) {
    EXPECT_TRUE(solid.isValid()) << solid.validationReport();
    const auto issues = hz::topo::validateExact(solid, hz::topo::exactTolerance(solid));
    EXPECT_TRUE(issues.empty()) << issues.size() << " issues, the first: "
                                << (issues.empty() ? "" : issues.front().detail);
    for (const auto& face : solid.faces()) {
        EXPECT_NE(face.exactSurface, nullptr) << face.topoId.tag();
        EXPECT_EQ(face.surface, nullptr) << "a constructed face has no NURBS carrier";
    }
    EXPECT_NEAR(volumeOf(solid), volume, 1e-9 * volume);
}

/// The exact extrusion of @p profile, which must build.
std::unique_ptr<hz::topo::Solid> exactly(const Profile& profile, const SketchPlane& plane,
                                         const Vec3& direction, double distance) {
    std::string why;
    auto solid = Extrude::exact(profile, plane, direction, distance, "x", &why);
    EXPECT_NE(solid, nullptr) << why;
    return solid;
}

/// How many of @p solid's faces lie on each kind of surface.
std::map<SurfaceKind, int> surfaceKinds(const hz::topo::Solid& solid) {
    std::map<SurfaceKind, int> kinds;
    for (const auto& face : solid.faces()) {
        if (face.exactSurface) ++kinds[face.exactSurface->kind()];
    }
    return kinds;
}

const hz::topo::Face* faceNamed(const hz::topo::Solid& solid, const std::string& name) {
    for (const auto& face : solid.faces()) {
        if (face.topoId.tag() == name) return &face;
    }
    ADD_FAILURE() << "no face " << name;
    return nullptr;
}

}  // namespace

// A rectangle is a block: six planes and its exact volume, either way up.
TEST(ExactExtrude, ARectangleIsABlock) {
    const Profile profile = rectangle();
    for (const double distance : {3.0, -3.0}) {
        SCOPED_TRACE(distance);
        const auto solid = exactly(profile, SketchPlane(), Vec3(0, 0, 1), distance);
        ASSERT_NE(solid, nullptr);
        EXPECT_EQ(solid->faceCount(), 6u);
        EXPECT_EQ(solid->edges().size(), 12u);
        EXPECT_EQ(solid->vertices().size(), 8u);
        EXPECT_EQ(solid->shellCount(), 1u);
        EXPECT_EQ(surfaceKinds(*solid), (std::map<SurfaceKind, int>{{SurfaceKind::Plane, 6}}));
        expectValidAndExact(*solid, 600.0);
    }
}

// A slot: two planes, two half cylinders and two caps, (20 x 10 + 25 pi) x 3 to 1e-9.
TEST(ExactExtrude, ASlotIsExact) {
    const Profile profile = slot();
    const auto solid = exactly(profile, SketchPlane(), Vec3(0, 0, 1), 3.0);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_EQ(surfaceKinds(*solid),
              (std::map<SurfaceKind, int>{{SurfaceKind::Plane, 4}, {SurfaceKind::Cylinder, 2}}));
    expectValidAndExact(*solid, kSlotArea * 3.0);
}

// A plate with two holes is one shell whose caps each have the holes as two inner loops, its
// walls reversed cylinders; its volume is exact. Master cut each hole from the plate with a
// Boolean, which split each cap in two (`x/cap_top/piece:0` and `/piece:1`, recorded below); its
// caps are one face each, and the hole walls keep master's logical names,
// `x~hole<k>/side:<circle>`.
TEST(ExactExtrude, APlateWithTwoHolesHasInnerLoops) {
    const Profile profile = plate();
    const auto solid = exactly(profile, SketchPlane(), Vec3(0, 0, 1), 10.0);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_EQ(solid->faceCount(), 8u);
    for (const char* cap : {"x/cap_bottom", "x/cap_top"}) {
        const hz::topo::Face* face = faceNamed(*solid, cap);
        ASSERT_NE(face, nullptr);
        EXPECT_EQ(face->innerLoops.size(), 2u) << cap;
    }
    for (const char* wall : {"x~hole0/side:{4}", "x~hole1/side:{5}"}) {
        const hz::topo::Face* face = faceNamed(*solid, named(wall, profile));
        ASSERT_NE(face, nullptr);
        EXPECT_EQ(face->exactSurface->kind(), SurfaceKind::Cylinder);
        EXPECT_TRUE(face->reversed) << "a hole's wall faces its axis";
    }
    expectValidAndExact(*solid, kPlateArea * 10.0);

    const std::set<std::string> masters{
        "x/cap_bottom/piece:0", "x/cap_bottom/piece:1", "x/cap_top/piece:0", "x/cap_top/piece:1",
        "x/side:{0}",           "x/side:{1}",           "x/side:{2}",        "x/side:{3}",
        "x~hole0/side:{4}",     "x~hole1/side:{5}"};
    std::set<std::string> expected;
    for (const auto& name : named(masters, profile)) {
        expected.insert(name.substr(0, name.find("/piece:")));
    }
    EXPECT_EQ(faceNames(*solid), expected);
}

// Swept along any direction but the sketch's normal, an arc sweeps an elliptic cylinder: a NURBS
// extrusion, exactly. The volume is the area times the distance times the direction's rise.
TEST(ExactExtrude, AnObliqueArcIsANurbsExtrusion) {
    const Vec3 direction = Vec3(0.3, -0.2, 1.0).normalized();
    const double rise = direction.z;
    const Profile profile = slot();
    const auto solid = exactly(profile, SketchPlane(), direction, 3.0);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(surfaceKinds(*solid),
              (std::map<SurfaceKind, int>{{SurfaceKind::Plane, 4}, {SurfaceKind::Nurbs, 2}}));
    expectValidAndExact(*solid, kSlotArea * 3.0 * rise);

    // A whole circle: one NURBS face with its seam.
    const auto rod = exactly(disc(), SketchPlane(), direction, 3.0);
    ASSERT_NE(rod, nullptr);
    EXPECT_EQ(surfaceKinds(*rod),
              (std::map<SurfaceKind, int>{{SurfaceKind::Plane, 2}, {SurfaceKind::Nurbs, 1}}));
    expectValidAndExact(*rod, 36.0 * kPi * 3.0 * rise);
}

// An island inside a hole is a region of its own, and its own shell, beside the square with the
// hole: no Boolean joins them, as master's did (which split the square's caps in two, `/piece:0`
// and `/piece:1`, recorded below).
TEST(ExactExtrude, AnIslandInAHoleIsItsOwnShell) {
    const Profile profile = island();
    const auto solid = exactly(profile, SketchPlane(), Vec3(0, 0, 1), 4.0);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->shellCount(), 2u);
    expectValidAndExact(*solid, kIslandArea * 4.0);
    for (const auto& shell : solid->shells()) {
        std::set<std::string> sources;
        for (const auto* face : shell.faces) {
            const std::string& tag = face->topoId.tag();
            sources.insert(tag.substr(0, tag.find('/')));
        }
        EXPECT_TRUE(sources == std::set<std::string>({"x", "x~hole0"}) ||
                    sources == std::set<std::string>({"x~region1"}))
            << *sources.begin();
    }
    const std::set<std::string> masters{
        "x/cap_bottom/piece:0", "x/cap_bottom/piece:1", "x/cap_top/piece:0", "x/cap_top/piece:1",
        "x/side:{0}",           "x/side:{1}",           "x/side:{2}",        "x/side:{3}",
        "x~hole0/side:{4}",     "x~hole0/side:{5}",     "x~hole0/side:{6}",  "x~hole0/side:{7}",
        "x~region1/cap_bottom", "x~region1/cap_top",    "x~region1/side:{8}"};
    std::set<std::string> expected;
    for (const auto& name : named(masters, profile)) {
        expected.insert(name.substr(0, name.find("/piece:")));
    }
    EXPECT_EQ(faceNames(*solid), expected);
}

// Each exact face is named as master's Stable extrude named its facets, logically (recorded at
// 1709c556), and its edges as nameEdgesLogically names them. A half disc's line and arc meet along
// two vertical lines: two edges between the same two faces, which are `<edge>/chord:0` and
// `/chord:1` as master's two were, one curve to logicalEdge, as master's faceted part had it.
TEST(ExactExtrude, NamesAreMastersLogicalNames) {
    struct Case {
        const char* what;
        Profile profile;
        SketchPlane plane;
        Vec3 direction;
        std::set<std::string> masters;  ///< master's logical faces
    };
    const std::set<std::string> four{"x/cap_bottom", "x/cap_top",  "x/side:{0}",
                                     "x/side:{1}",   "x/side:{2}", "x/side:{3}"};
    std::vector<Case> cases;
    cases.push_back({"a rectangle", rectangle(), SketchPlane(), Vec3(0, 0, 1), four});
    cases.push_back({"a slot", slot(), SketchPlane(), Vec3(0, 0, 1), four});
    cases.push_back({"a half disc",
                     dProfile(),
                     SketchPlane(),
                     Vec3(0, 0, 1),
                     {"x/cap_bottom", "x/cap_top", "x/side:{0}", "x/side:{1}"}});
    cases.push_back({"a disc",
                     disc(),
                     SketchPlane(),
                     Vec3(0, 0, 1),
                     {"x/cap_bottom", "x/cap_top", "x/side:{0}"}});
    cases.push_back({"a tangent arc and a notch",
                     tangentAndTiny(),
                     SketchPlane(),
                     Vec3(0, 0, 1),
                     {"x/cap_bottom", "x/cap_top", "x/side:{0}", "x/side:{1}", "x/side:{2}",
                      "x/side:{3}", "x/side:{4}", "x/side:{5}", "x/side:{6}"}});
    cases.push_back({"a slot on an inclined plane", slot(), inclined(), inclined().normal(), four});
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        const auto solid = exactly(c.profile, c.plane, c.direction, 2.0);
        ASSERT_NE(solid, nullptr);
        EXPECT_EQ(faceNames(*solid), named(c.masters, c.profile));
        for (const auto& edge : solid->edges()) {
            EXPECT_TRUE(edge.topoId.isValid()) << "every edge is named";
        }
    }

    // The half disc's two vertical edges, between its line's face and its arc's.
    const Profile half = dProfile();
    const auto solid = exactly(half, SketchPlane(), Vec3(0, 0, 1), 2.0);
    ASSERT_NE(solid, nullptr);
    std::multiset<std::string> between;
    for (const auto& edge : solid->edges()) {
        const std::string& tag = edge.topoId.tag();
        if (tag.find("cap_") == std::string::npos) between.insert(tag);
    }
    // Named after its two faces in sorted order, as nameEdgesLogically names an edge.
    const std::string a = named("side:{0}", half);
    const std::string b = named("side:{1}", half);
    const std::string edge = "x/edge:" + std::min(a, b) + "|" + std::max(a, b);
    EXPECT_EQ(between, (std::multiset<std::string>{edge + "/chord:0", edge + "/chord:1"}));
}

// Review Focus 1: a sketch tilted 30 degrees about x and turned 20 about z, extruded along its
// normal and at a slant: the volume is exact.
TEST(ExactExtrude, OnAnInclinedPlane) {
    const SketchPlane plane = inclined();
    const Vec3 slant = (plane.normal() + plane.xAxis() * 0.4 - plane.yAxis() * 0.25).normalized();
    for (const Vec3& direction : {plane.normal(), slant}) {
        SCOPED_TRACE(direction.x);
        for (const Profile& profile : {slot(), plate()}) {
            const double area = profile.size() == 4 ? kSlotArea : kPlateArea;
            const auto solid = exactly(profile, plane, direction, 3.0);
            ASSERT_NE(solid, nullptr);
            expectValidAndExact(*solid, area * 3.0 * direction.dot(plane.normal()));
        }
    }
}

// Review Focus 2: a 100 mm profile with a line tangent to an arc at each end of it and a notch of
// two 0.01 mm edges validates with no tolerance grown: every edge's and vertex's is the model's
// resolution.
TEST(ExactExtrude, ATangentJoinAndATinyEdge) {
    const auto solid = exactly(tangentAndTiny(), SketchPlane(), Vec3(0, 0, 1), 30.0);
    ASSERT_NE(solid, nullptr);
    const hz::math::ModelTolerance tolerance = hz::topo::exactTolerance(*solid);
    EXPECT_TRUE(hz::topo::validateExact(*solid, tolerance).empty());
    for (const auto& edge : solid->edges()) {
        EXPECT_LE(tolerance.effective(edge.tolerance), tolerance.linear) << edge.topoId.tag();
    }
    for (const auto& v : solid->vertices()) {
        EXPECT_LE(tolerance.effective(v.tolerance), tolerance.linear);
    }
    expectValidAndExact(*solid, kTangentArea * 30.0);
}

// Drawn either way round, extruded either way up: the solid faces out and measures the same.
TEST(ExactExtrude, EitherWayRoundAndEitherWayUp) {
    // Chained from its top line drawn left to right, the slot runs clockwise.
    const Profile clockwise = {line({0, 5}, {20, 5}), arc({20, 0}, 5, -kPi / 2, kPi / 2),
                               line({20, -5}, {0, -5}), arc({0, 0}, 5, kPi / 2, 3 * kPi / 2)};
    for (const Profile& profile : {slot(), clockwise}) {
        for (const Vec3& direction : {Vec3(0, 0, 1), Vec3(0, 0, -1), Vec3(0.2, 0.1, -1.0)}) {
            for (const double distance : {3.0, -3.0}) {
                SCOPED_TRACE(direction.z * distance);
                const Vec3 unit = direction.normalized();
                const auto solid = exactly(profile, SketchPlane(), unit, distance);
                ASSERT_NE(solid, nullptr);
                expectValidAndExact(*solid, kSlotArea * 3.0 * std::abs(unit.z));
            }
        }
    }
}

// What master refuses, the exact extrude refuses in master's words.
TEST(ExactExtrude, WhatMasterRefusesItRefusesAlike) {
    std::string why;
    EXPECT_EQ(Extrude::exact({line({0, 0}, {1, 0})}, SketchPlane(), Vec3(0, 0, 1), 1.0, "x", &why),
              nullptr);
    std::string masters;
    EXPECT_EQ(Extrude::execute({line({0, 0}, {1, 0})}, SketchPlane(), Vec3(0, 0, 1), 1.0, "x", 32,
                               0.0, &masters, NamingScheme::FromGeometry),
              nullptr);
    EXPECT_EQ(why, masters);
    EXPECT_EQ(Extrude::exact(rectangle(), SketchPlane(), Vec3(1, 0, 0), 1.0, "x", &why), nullptr);
    EXPECT_NE(why.find("lies in the sketch plane"), std::string::npos) << why;
    EXPECT_EQ(Extrude::exact(rectangle(), SketchPlane(), Vec3(0, 0, 1), 0.0, "x", &why), nullptr);
    EXPECT_NE(why.find("non-zero"), std::string::npos) << why;
}

// A sketch whose ends meet within the profile's join tolerance but miss by more than the model's
// resolution: a slot about a millimetre across whose top line misses both its arcs by 0.9 nm.
// Each corner where an arc's own end misses its vertex has the gap as its vertices' tolerance and
// its edge's along the offset (topo::ToleranceGrowth), and the solid validates; nothing else grows.
TEST(ExactExtrude, ASketchsLooseEndsAreItsCornersTolerances) {
    const double gap = 9e-7;
    const Profile loose = {line({0, -0.25}, {1, -0.25}), arc({1, 0}, 0.25, -kPi / 2, kPi / 2),
                           line({1, 0.25 + gap}, {0, 0.25 + gap}),
                           arc({0, 0}, 0.25, kPi / 2, 3 * kPi / 2)};
    const auto solid = exactly(loose, SketchPlane(), Vec3(0, 0, 1), 0.2);
    ASSERT_NE(solid, nullptr);
    const hz::math::ModelTolerance tolerance = hz::topo::exactTolerance(*solid);
    ASSERT_GT(gap, tolerance.linear) << "a gap the resolution does not cover";
    const auto issues = hz::topo::validateExact(*solid, tolerance);
    EXPECT_TRUE(issues.empty()) << issues.size() << " issues, the first: "
                                << (issues.empty() ? "" : issues.front().detail);
    int looseCorners = 0;
    for (const auto& v : solid->vertices()) {
        if (v.tolerance > 0.0) {
            ++looseCorners;
            EXPECT_NEAR(v.tolerance, gap, 1e-12);
        }
    }
    EXPECT_EQ(looseCorners, 2) << "the top line's far end, at the sketch and at the offset";
    int grown = 0;
    for (const auto& e : solid->edges()) {
        if (e.tolerance > 0.0) ++grown;
    }
    EXPECT_EQ(grown, 1) << "the edge along the offset at that corner";
}
