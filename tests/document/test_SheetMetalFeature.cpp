// The base flange (sheet metal): an open sketch of lines folded into a sheet
// of a thickness, each corner a bend, extruded to a width along the sketch's
// normal; a feature like any other in the history.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/geometry/MeshData.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/modeling/FacetResolution.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/SheetMetal.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/ExactValidator.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

using hz::doc::Document;
using hz::doc::SheetMetalFeature;
using hz::doc::Sketch;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

constexpr double kPi = 3.14159265358979323846;

std::shared_ptr<Sketch> polyline(const std::vector<Vec2>& points,
                                 const hz::draft::SketchPlane& plane = {}) {
    auto sketch = std::make_shared<Sketch>(plane);
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        sketch->addEntity(std::make_shared<hz::draft::DraftLine>(points[i], points[i + 1]));
    }
    return sketch;
}

double volumeOf(const Document& doc) {
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

hz::math::BoundingBox boundsOf(const Document& doc) {
    hz::math::BoundingBox box;
    if (!doc.solid()) return box;
    for (const auto& vertex : doc.solid()->vertices()) box.expand(vertex.point);
    return box;
}

/// The area of a folded L's cross-section as the fold facets it: the flats,
/// and each bend's ring between radii r and r + t cut into @p chords chords.
double facetedArea(double flats, double t, double r, double angle, int chords) {
    const double ring = 0.5 * chords * std::sin(angle / chords) * ((r + t) * (r + t) - r * r);
    return flats * t + ring;
}

/// The radii of @p exact's cylinders, in order.
std::vector<double> cylinderRadii(const hz::topo::Solid& exact) {
    std::vector<double> radii;
    for (const auto& face : exact.faces()) {
        if (const auto* c = dynamic_cast<const hz::geo::Cylinder*>(face.exactSurface.get())) {
            radii.push_back(c->radius());
        }
    }
    std::sort(radii.begin(), radii.end());
    return radii;
}

/// How many chords of @p faceted, the bridge copy of @p exact, lie along each of @p exact's arcs
/// (the edges between a cylinder and a cap): the edges model::logicalEdge takes to it.
std::vector<int> chordsPerArc(const hz::topo::Solid& exact, const hz::topo::Solid& faceted) {
    const auto bend = [](const hz::topo::Face* face) {
        return dynamic_cast<const hz::geo::Cylinder*>(face->exactSurface.get()) != nullptr;
    };
    const auto cap = [](const hz::topo::Face* face) {
        const std::string tag = face->topoId.tag();
        return tag.find("/cap_") != std::string::npos;
    };
    std::map<std::string, int> chords;
    for (const auto& edge : exact.edges()) {
        const hz::topo::Face* a = edge.halfEdge->face;
        const hz::topo::Face* b = edge.halfEdge->twin->face;
        if ((bend(a) && cap(b)) || (cap(a) && bend(b))) chords[edge.topoId.tag()] = 0;
    }
    for (const auto& edge : faceted.edges()) {
        const auto found = chords.find(hz::model::logicalEdge(edge.topoId.tag()));
        if (found != chords.end()) ++found->second;
    }
    std::vector<int> counts;
    counts.reserve(chords.size());
    for (const auto& entry : chords) counts.push_back(entry.second);
    return counts;
}

/// @p exact is a valid exact solid: combinatorially, and its geometry agrees with its topology at
/// its own tolerance.
void expectValidExactly(const hz::topo::Solid& exact) {
    EXPECT_TRUE(exact.isValid()) << exact.validationReport();
    const auto issues = hz::topo::validateExact(exact, hz::topo::exactTolerance(exact));
    EXPECT_TRUE(issues.empty()) << issues.size() << " issues, the first: "
                                << (issues.empty() ? "" : issues.front().detail);
}

}  // namespace

// An L sketched on the XY plane, its lines the outside of the sheet: a plate
// 50 by 30 on the outside, 2 thick, bent with an inside radius of 1, 20 wide.
TEST(SheetMetalFeatureTest, AnLFoldsWhereItWasSketched) {
    Document doc;
    doc.featureTree().addFeature(
        std::make_unique<SheetMetalFeature>(polyline({{0, 0}, {50, 0}, {50, 30}}), 2.0, 1.0, 20.0));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    // Flats 47 and 27 (each line less the setback (r + t) tan 45° = 3).
    EXPECT_NEAR(volumeOf(doc), 20.0 * facetedArea(47.0 + 27.0, 2.0, 1.0, kPi / 2, 8), 1e-6);
    const auto box = boundsOf(doc);
    EXPECT_NEAR(box.min().x, 0.0, 1e-9);
    EXPECT_NEAR(box.max().x, 50.0, 1e-9) << "the sketched corner is the outside's";
    EXPECT_NEAR(box.min().y, 0.0, 1e-9);
    EXPECT_NEAR(box.max().y, 30.0, 1e-9);
    EXPECT_NEAR(box.min().z, 0.0, 1e-9);
    EXPECT_NEAR(box.max().z, 20.0, 1e-9) << "the width along the sketch's normal";
}

// Flipped, the same lines are the sheet's inside: it grows outward from them.
TEST(SheetMetalFeatureTest, FlippedTheMaterialIsOnTheOtherSide) {
    Document doc;
    auto flange =
        std::make_unique<SheetMetalFeature>(polyline({{0, 0}, {50, 0}, {50, 30}}), 2.0, 1.0, 20.0);
    ASSERT_TRUE(flange->setParameter("flip", 1.0));
    doc.featureTree().addFeature(std::move(flange));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    EXPECT_NEAR(volumeOf(doc), 20.0 * facetedArea(49.0 + 29.0, 2.0, 1.0, kPi / 2, 8), 1e-6);
    const auto box = boundsOf(doc);
    EXPECT_NEAR(box.min().y, -2.0, 1e-9) << "below the first line";
    EXPECT_NEAR(box.max().x, 52.0, 1e-9) << "beyond the second";
}

// On a plane of its own: the sheet lies where the sketch is, its width
// along that plane's normal.
TEST(SheetMetalFeatureTest, OnASketchPlaneOfItsOwn) {
    // The XZ plane facing -Y... any plane: here the YZ plane at x = 100.
    const hz::draft::SketchPlane plane(Vec3(100, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0));
    Document doc;
    doc.featureTree().addFeature(std::make_unique<SheetMetalFeature>(
        polyline({{0, 0}, {40, 0}, {40, 25}}, plane), 1.5, 2.0, 30.0));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    const auto box = boundsOf(doc);
    EXPECT_NEAR(box.min().x, 100.0, 1e-9);
    EXPECT_NEAR(box.max().x, 130.0, 1e-9);
    EXPECT_NEAR(box.max().y, 40.0, 1e-9);
    EXPECT_NEAR(box.max().z, 25.0, 1e-9);
}

// Its parameters, edited as any feature's: checked, and the K-factor a plain
// number.
TEST(SheetMetalFeatureTest, ItsParameters) {
    SheetMetalFeature flange(polyline({{0, 0}, {50, 0}, {50, 30}}), 2.0, 1.0, 20.0);
    const auto params = flange.parameters();
    EXPECT_EQ(params.at("thickness"), 2.0);
    EXPECT_EQ(params.at("bendRadius"), 1.0);
    EXPECT_EQ(params.at("kFactor"), 0.44);
    EXPECT_EQ(params.at("width"), 20.0);
    EXPECT_EQ(params.at("flip"), 0.0);
    EXPECT_EQ(flange.parameterKind("kFactor"), hz::doc::Feature::ParameterKind::Number);
    EXPECT_EQ(flange.parameterKind("thickness"), hz::doc::Feature::ParameterKind::Length);
    EXPECT_EQ(flange.parameterKind("bendSegments"), hz::doc::Feature::ParameterKind::Count);
    EXPECT_EQ(flange.parameterKind("flip"), hz::doc::Feature::ParameterKind::Choice);
    EXPECT_EQ(flange.parameterChoices("flip"), (std::vector<std::string>{"Left", "Right"}));
    EXPECT_TRUE(flange.setParameter("kFactor", 0.33));
    EXPECT_FALSE(flange.setParameter("kFactor", 1.5));
    EXPECT_FALSE(flange.setParameter("thickness", 0.0));
    EXPECT_FALSE(flange.setParameter("bendRadius", -1.0));
    EXPECT_FALSE(flange.setParameter("width", std::nan("")));
    EXPECT_FALSE(flange.setParameter("flip", 0.5));
    EXPECT_FALSE(flange.setParameter("nonsense", 1.0));
    EXPECT_EQ(flange.params().kFactor, 0.33);
    // Its flat pattern: the developed length of what it folds.
    const auto strip = flange.strip();
    ASSERT_TRUE(strip.has_value());
    EXPECT_NEAR(hz::model::developedLength(*strip, flange.params()),
                50.0 + 30.0 - hz::model::bendDeduction(kPi / 2, flange.params()), 1e-9);
}

// A sketch that is not one open run of lines fails, and says why.
TEST(SheetMetalFeatureTest, WhatItCannotBeMadeFrom) {
    const auto fails = [](std::shared_ptr<Sketch> sketch, const char* why) {
        Document doc;
        doc.featureTree().addFeature(
            std::make_unique<SheetMetalFeature>(std::move(sketch), 2.0, 1.0, 20.0));
        EXPECT_FALSE(doc.rebuildModel());
        EXPECT_NE(doc.lastBuildMessage().find(why), std::string::npos) << doc.lastBuildMessage();
    };
    fails(polyline({{0, 0}, {40, 0}, {40, 30}, {0, 30}, {0, 0}}), "closed");
    auto circle = polyline({{0, 0}, {40, 0}});
    circle->addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(10, 10), 5.0));
    fails(circle, "lines only");
    auto apart = polyline({{0, 0}, {40, 0}});
    apart->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(60, 0), Vec2(60, 20)));
    fails(apart, "join end to end");
    fails(polyline({{0, 0}, {40, 0}, {40, 4}, {0, 4}}), "too short");
    fails(std::make_shared<Sketch>(), "no lines");
}

// Built on like any feature: a hole through the base leaves less sheet.
TEST(SheetMetalFeatureTest, LaterFeaturesBuildOnIt) {
    Document doc;
    doc.featureTree().addFeature(
        std::make_unique<SheetMetalFeature>(polyline({{0, 0}, {50, 0}, {50, 30}}), 2.0, 1.0, 20.0));
    ASSERT_TRUE(doc.rebuildModel());
    const double before = volumeOf(doc);
    auto cutter = hz::doc::PrimitiveFeature::makeBox(10, 10, 10);
    cutter->setOperation(hz::doc::BodyOperation::Cut);
    doc.featureTree().addFeature(std::move(cutter));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    EXPECT_LT(volumeOf(doc), before);
    EXPECT_GT(volumeOf(doc), before - 10.0 * 10.0 * 2.0 - 1.0);
}

// Drawn with the polyline tool, an open polyline is its lines.
TEST(SheetMetalFeatureTest, AnOpenPolylineIsItsLines) {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftPolyline>(
        std::vector<Vec2>{{0, 0}, {50, 0}, {50, 30}}, false));
    Document doc;
    doc.featureTree().addFeature(std::make_unique<SheetMetalFeature>(sketch, 2.0, 1.0, 20.0));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    EXPECT_NEAR(volumeOf(doc), 20.0 * facetedArea(47.0 + 27.0, 2.0, 1.0, kPi / 2, 8), 1e-6);
}

// Stage 3: a bend is exact, two cylinders about one axis, its inside of the bend radius and its
// outside of the radius and the thickness, and the flange's volume is the closed form. Its faceted
// copy, which every operation not yet exact reads, cuts a 90-degree bend into master's 8 chords
// (facetResolution(): four times the chords a bend, a turn).
TEST(SheetMetalFeature, ABendIsTwoCylinders) {
    // An L 30 by 20 on the outside, 1 thick, an inside radius of 2, 10 wide.
    auto flange =
        std::make_unique<SheetMetalFeature>(polyline({{0, 0}, {30, 0}, {30, 20}}), 1.0, 2.0, 10.0);
    const auto strip = flange->strip();
    ASSERT_TRUE(strip.has_value());
    double flats = 0.0;
    for (const double length : strip->segments) flats += length;
    EXPECT_NEAR(flats, 27.0 + 17.0, 1e-12) << "each line less the setback r + t = 3";
    Document doc;
    doc.featureTree().addFeature(std::move(flange));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    const hz::topo::Solid* exact = doc.exactSolid();
    ASSERT_NE(exact, nullptr) << doc.exactNote();
    expectValidExactly(*exact);
    const auto radii = cylinderRadii(*exact);
    ASSERT_EQ(radii.size(), 2u);
    EXPECT_NEAR(radii[0], 2.0, 1e-12);
    EXPECT_NEAR(radii[1], 3.0, 1e-12);
    const double volume = (flats * 1.0 + (kPi / 4.0) * (3.0 * 3.0 - 2.0 * 2.0)) * 10.0;
    const auto mass = hz::model::MassPropertiesCalculator::compute(*exact);
    ASSERT_TRUE(mass.valid) << mass.problem;
    EXPECT_NEAR(mass.volume, volume, 1e-9 * volume);

    ASSERT_NE(doc.solid(), nullptr);
    EXPECT_EQ(chordsPerArc(*exact, *doc.solid()), (std::vector<int>{8, 8, 8, 8}))
        << "each arc, at each cap";
    // Master's faceted flange, face for face: 4 flats, 2 ends, 2 caps and 16 chords' facets.
    EXPECT_EQ(doc.solid()->faceCount(), 24u);
    EXPECT_NEAR(volumeOf(doc), 10.0 * facetedArea(flats, 1.0, 2.0, kPi / 2, 8), 1e-9);
}

// A bend of 60 degrees: exact as any, and its faceted copy cut by its arcs' own sweep, as every
// exact arc's bridge is (FacetResolution::stepsFor, at 32 chords a turn): 6 chords, where master
// cut every bend, whatever its angle, into 8 (re-baselined: master's faceted 60-degree flange had
// 8 chords to each arc).
TEST(SheetMetalFeature, ABendOtherThanNinetyDegrees) {
    const double angle = kPi / 3.0;
    auto flange = std::make_unique<SheetMetalFeature>(
        polyline({{0, 0}, {30, 0}, {30 + 20 * std::cos(angle), 20 * std::sin(angle)}}), 1.0, 2.0,
        10.0);
    const auto strip = flange->strip();
    ASSERT_TRUE(strip.has_value());
    ASSERT_EQ(strip->bendAngles.size(), 1u);
    EXPECT_NEAR(strip->bendAngles[0], angle, 1e-12);
    double flats = 0.0;
    for (const double length : strip->segments) flats += length;
    const hz::model::FacetResolution resolution = flange->facetResolution();
    EXPECT_EQ(resolution.segmentsPerTurn, 32);
    EXPECT_EQ(resolution.chordTolerance, 0.0);
    const int chords = resolution.stepsFor(2.0, angle);
    EXPECT_EQ(chords, 6);
    EXPECT_EQ(resolution.stepsFor(3.0, angle), chords);
    Document doc;
    doc.featureTree().addFeature(std::move(flange));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    const hz::topo::Solid* exact = doc.exactSolid();
    ASSERT_NE(exact, nullptr) << doc.exactNote();
    expectValidExactly(*exact);
    const auto radii = cylinderRadii(*exact);
    ASSERT_EQ(radii.size(), 2u);
    EXPECT_NEAR(radii[0], 2.0, 1e-12);
    EXPECT_NEAR(radii[1], 3.0, 1e-12);
    const double volume = (flats * 1.0 + (angle / 2.0) * (3.0 * 3.0 - 2.0 * 2.0)) * 10.0;
    const auto mass = hz::model::MassPropertiesCalculator::compute(*exact);
    ASSERT_TRUE(mass.valid) << mass.problem;
    EXPECT_NEAR(mass.volume, volume, 1e-9 * volume);

    ASSERT_NE(doc.solid(), nullptr);
    EXPECT_EQ(chordsPerArc(*exact, *doc.solid()),
              (std::vector<int>{chords, chords, chords, chords}));
    EXPECT_EQ(doc.solid()->faceCount(), 8u + 2u * static_cast<std::size_t>(chords));
    EXPECT_NEAR(volumeOf(doc), 10.0 * facetedArea(flats, 1.0, 2.0, angle, chords), 1e-9);
}

// Named by geometry or by position (a part saved before Stable naming), a flange builds as
// master did: its cross-section's polygon, every bend in its bendSegments chords whatever its
// angle, and no exact part.
TEST(SheetMetalFeature, NamedTheOlderWaysItBuildsAsMasterDid) {
    const double angle = kPi / 3.0;
    for (const auto naming :
         {hz::model::NamingScheme::FromGeometry, hz::model::NamingScheme::Positional}) {
        auto flange = std::make_unique<SheetMetalFeature>(
            polyline({{0, 0}, {30, 0}, {30 + 20 * std::cos(angle), 20 * std::sin(angle)}}), 1.0,
            2.0, 10.0);
        flange->setNaming(naming);
        const auto strip = flange->strip();
        ASSERT_TRUE(strip.has_value());
        double flats = 0.0;
        for (const double length : strip->segments) flats += length;
        Document doc;
        doc.featureTree().addFeature(std::move(flange));
        ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
        EXPECT_EQ(doc.exactSolid(), nullptr);
        ASSERT_NE(doc.solid(), nullptr);
        EXPECT_EQ(doc.solid()->faceCount(), 8u + 2u * 8u);
        EXPECT_NEAR(volumeOf(doc), 10.0 * facetedArea(flats, 1.0, 2.0, angle, 8), 1e-9);
    }
}

// Every way a flange folds is exact, and its faceted copy is named for its exact faces: bent up or
// down, flipped, on a plane of its own, and several bends; its volume the closed form, the flats
// times the thickness and each bend's ring between its radii, times the width.
TEST(SheetMetalFeature, EveryWayItFoldsIsExact) {
    struct Case {
        const char* what;
        std::vector<Vec2> points;
        hz::draft::SketchPlane plane;
        bool flipped;
    };
    const double c60 = std::cos(kPi / 3.0);
    const double s60 = std::sin(kPi / 3.0);
    const hz::draft::SketchPlane yz(Vec3(100, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0));
    const std::vector<Case> cases = {
        {"an L", {{0, 0}, {50, 0}, {50, 30}}, {}, false},
        {"an L flipped", {{0, 0}, {50, 0}, {50, 30}}, {}, true},
        {"a Z", {{0, 0}, {40, 0}, {40, 30}, {80, 30}}, {}, false},
        {"a Z flipped", {{0, 0}, {40, 0}, {40, 30}, {80, 30}}, {}, true},
        {"a hat", {{0, 0}, {20, 0}, {20, 20}, {50, 20}, {50, 0}, {70, 0}}, {}, false},
        {"bent 60 degrees down", {{0, 0}, {40, 0}, {40 + 30 * c60, -30 * s60}}, {}, false},
        {"on the YZ plane, flipped", {{10, 5}, {40, 5}, {40, 25}}, yz, true},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        auto flange =
            std::make_unique<SheetMetalFeature>(polyline(c.points, c.plane), 1.5, 2.0, 12.0);
        ASSERT_TRUE(flange->setParameter("flip", c.flipped ? 1.0 : 0.0));
        const auto strip = flange->strip();
        ASSERT_TRUE(strip.has_value());
        double area = 0.0;
        for (const double length : strip->segments) area += length * 1.5;
        for (const double angle : strip->bendAngles) {
            area += (std::abs(angle) / 2.0) * (3.5 * 3.5 - 2.0 * 2.0);
        }
        Document doc;
        doc.featureTree().addFeature(std::move(flange));
        ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
        const hz::topo::Solid* exact = doc.exactSolid();
        ASSERT_NE(exact, nullptr) << doc.exactNote();
        expectValidExactly(*exact);
        EXPECT_EQ(cylinderRadii(*exact).size(), 2 * strip->bendAngles.size());
        // Shown from its exact form's mesh, every face of it.
        ASSERT_NE(doc.exactMesh(), nullptr);
        EXPECT_EQ(std::set<std::string>(doc.exactMesh()->faceTags.begin(),
                                        doc.exactMesh()->faceTags.end())
                      .size(),
                  exact->faceCount());
        const auto mass = hz::model::MassPropertiesCalculator::compute(*exact);
        ASSERT_TRUE(mass.valid) << mass.problem;
        EXPECT_NEAR(mass.volume, area * 12.0, 1e-9 * area * 12.0);
        // Its faceted copy: a facet of each exact face, or several of a bend's, each named for it.
        ASSERT_NE(doc.solid(), nullptr);
        std::set<std::string> faces;
        std::set<std::string> facets;
        for (const auto& face : exact->faces()) faces.insert(face.topoId.tag());
        for (const auto& face : doc.solid()->faces()) {
            facets.insert(hz::model::logicalFace(face.topoId.tag()));
        }
        EXPECT_EQ(facets, faces);
        // And it stands where master's faceted flange stood: the same box.
        hz::math::BoundingBox exactBox;
        for (const auto& v : exact->vertices()) exactBox.expand(v.point);
        const auto facetedBox = boundsOf(doc);
        EXPECT_LT((exactBox.min() - facetedBox.min()).length(), 1e-9);
        EXPECT_LT((exactBox.max() - facetedBox.max()).length(), 1e-9);
    }
}

// Ruling 28: a flange's faces are named after its own sketch, the same in every build and every
// session, so a hole drilled into a flange by the face's stored name builds when the part is built
// again, after other entities were made, and when it is saved and opened again. (Master named
// them after lines it made anew in each build, from the count every entity takes its id from:
// such a hole was lost at the next build.)
TEST(SheetMetalFeature, AHoleOnAFlangeSurvivesARebuild) {
    auto sketch = polyline({{0, 0}, {50, 0}, {50, 30}});
    const std::string firstLine = "e" + std::to_string(sketch->entities().front()->id());
    Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    doc.addSketch(sketch);
    doc.featureTree().addFeature(std::make_unique<SheetMetalFeature>(sketch, 2.0, 1.0, 20.0));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    const double flange = volumeOf(doc);
    // The first flat's face on the sketch's side: y = 0, from x = 0 to 47, 20 wide.
    std::string face;
    for (const auto& f : doc.solid()->faces()) {
        hz::math::BoundingBox box;
        const hz::topo::HalfEdge* he = f.outerLoop->halfEdge;
        do {
            box.expand(he->origin->point);
            he = he->next;
        } while (he != f.outerLoop->halfEdge);
        if (box.max().y < 1e-9 && box.min().y > -1e-9 && box.max().x > 40.0) {
            face = f.topoId.tag();
        }
    }
    ASSERT_FALSE(face.empty());
    EXPECT_EQ(face, doc.featureTree().feature(0)->featureID() + "/flat:" + firstLine);
    doc.featureTree().addFeature(hz::doc::HoleFeature::make(face, Vec3(20, 0, 10), 4.0, 1.0));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    const double drilled = volumeOf(doc);
    EXPECT_LT(drilled, flange - 10.0) << "a hole 4 across and 1 deep";

    // Built again, twice, after other entities took ids from the count.
    for (int i = 0; i < 7; ++i) (void)hz::draft::DraftEntity::newId();
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    EXPECT_NEAR(volumeOf(doc), drilled, 1e-9 * drilled);

    // Saved and opened again in this session.
    Document opened;
    std::string error;
    ASSERT_TRUE(hz::io::NativeFormat::documentFromJson(
        hz::io::NativeFormat::documentToJson(doc, false), opened, &error))
        << error;
    ASSERT_EQ(opened.featureTree().featureCount(), 2u);
    ASSERT_TRUE(opened.rebuildModel()) << opened.lastBuildMessage();
    EXPECT_NEAR(volumeOf(opened), drilled, 1e-9 * drilled);
}

// Ruling 28: the names are the sketch's, not the build's. Two builds in one session, other
// entities made between them, name every face and edge alike. The scheme, for the sketch's
// segments S0, S1, ... in the order they run (a line `e<id>`, a polyline's segment k `e<id>.<k>`):
// each flat's face on the sketch's side `flat:<S>`, and a thickness away `flat:<S>:back`; each
// bend's `bend:<S>-<S'>` and `bend:<S>-<S'>:back` (the corner from S to S'); the near end
// `start:<S0>`, the far end `end:<Sn>`; the caps `cap_bottom` and `cap_top`, as master named them.
TEST(SheetMetalFeature, ItsNamesAreTheSameInEveryBuild) {
    const auto namesOf = [](const hz::topo::Solid& solid) {
        std::set<std::string> names;
        for (const auto& f : solid.faces()) names.insert("F " + f.topoId.tag());
        for (const auto& e : solid.edges()) names.insert("E " + e.topoId.tag());
        return names;
    };
    const auto faceNamesOf = [](const hz::topo::Solid& solid) {
        std::set<std::string> names;
        for (const auto& f : solid.faces()) names.insert(f.topoId.tag());
        return names;
    };
    {
        SCOPED_TRACE("a Z of lines, flipped");
        auto sketch = polyline({{0, 0}, {40, 0}, {40, 30}, {80, 30}});
        std::vector<std::string> s;
        for (const auto& e : sketch->entities()) s.push_back("e" + std::to_string(e->id()));
        SheetMetalFeature flange(sketch, 2.0, 1.0, 20.0);
        flange.restoreFeatureID("sheetmetal_7");
        ASSERT_TRUE(flange.setParameter("flip", 1.0));
        std::string why;
        const auto once = flange.executeWithExact({}, nullptr, &why);
        ASSERT_NE(once.exact, nullptr) << why;
        for (int i = 0; i < 11; ++i) (void)hz::draft::DraftEntity::newId();
        const auto again = flange.executeWithExact({}, nullptr, &why);
        ASSERT_NE(again.exact, nullptr) << why;
        EXPECT_EQ(namesOf(*again.exact), namesOf(*once.exact));
        EXPECT_EQ(namesOf(*again.faceted), namesOf(*once.faceted));
        const std::string f = "sheetmetal_7/";
        EXPECT_EQ(
            faceNamesOf(*once.exact),
            (std::set<std::string>{
                f + "cap_bottom", f + "cap_top", f + "flat:" + s[0], f + "flat:" + s[0] + ":back",
                f + "flat:" + s[1], f + "flat:" + s[1] + ":back", f + "flat:" + s[2],
                f + "flat:" + s[2] + ":back", f + "bend:" + s[0] + "-" + s[1],
                f + "bend:" + s[0] + "-" + s[1] + ":back", f + "bend:" + s[1] + "-" + s[2],
                f + "bend:" + s[1] + "-" + s[2] + ":back", f + "start:" + s[0],
                f + "end:" + s[2]}));
    }
    {
        SCOPED_TRACE("an L drawn as one polyline");
        auto sketch = std::make_shared<Sketch>();
        sketch->addEntity(std::make_shared<hz::draft::DraftPolyline>(
            std::vector<Vec2>{{0, 0}, {50, 0}, {50, 30}}, false));
        const std::string e = "e" + std::to_string(sketch->entities().front()->id());
        SheetMetalFeature flange(sketch, 2.0, 1.0, 20.0);
        flange.restoreFeatureID("sheetmetal_8");
        std::string why;
        const auto out = flange.executeWithExact({}, nullptr, &why);
        ASSERT_NE(out.exact, nullptr) << why;
        const std::string f = "sheetmetal_8/";
        EXPECT_EQ(
            faceNamesOf(*out.exact),
            (std::set<std::string>{f + "cap_bottom", f + "cap_top", f + "flat:" + e + ".0",
                                   f + "flat:" + e + ".0:back", f + "flat:" + e + ".1",
                                   f + "flat:" + e + ".1:back", f + "bend:" + e + ".0-" + e + ".1",
                                   f + "bend:" + e + ".0-" + e + ".1:back", f + "start:" + e + ".0",
                                   f + "end:" + e + ".1"}));
    }
}
