// The base flange (sheet metal): an open sketch of lines folded into a sheet
// of a thickness, each corner a bend, extruded to a width along the sketch's
// normal; a feature like any other in the history.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/SheetMetal.h"

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
