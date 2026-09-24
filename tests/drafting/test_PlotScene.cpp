// What a drawing plots (Phase 127): styled strokes and text in world
// coordinates, with ByLayer and ByBlock resolved and blocks expanded, and
// how it sits on the paper.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DimensionStyle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/drafting/PlotScene.h"
#include "horizon/math/Constants.h"

using hz::draft::DimensionStyle;
using hz::draft::DraftDocument;
using hz::draft::DraftLine;
using hz::draft::LayerManager;
using hz::draft::PlotScene;
using hz::math::Vec2;

namespace {

bool near(const Vec2& a, const Vec2& b, double tol = 1e-9) {
    return (a - b).length() <= tol;
}

}  // namespace

// ByLayer comes from the layer; an invisible layer does not plot, a locked
// one does; a circle is a closed stroke a degree or less per step.
TEST(PlotSceneTest, EntitiesPlotWithTheirLayersStyle) {
    DraftDocument d;
    LayerManager layers;
    hz::draft::LayerProperties red;
    red.name = "Red";
    red.color = 0xFFFF0000;
    red.lineWidth = 0.5;
    red.lineType = 2;
    layers.addLayer(red);
    hz::draft::LayerProperties hidden;
    hidden.name = "Hidden";
    hidden.visible = false;
    layers.addLayer(hidden);
    hz::draft::LayerProperties locked;
    locked.name = "Locked";
    locked.locked = true;
    layers.addLayer(locked);

    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 0));
    line->setLayer("Red");
    d.addEntity(line);
    auto gone = std::make_shared<DraftLine>(Vec2(0, 5), Vec2(10, 5));
    gone->setLayer("Hidden");
    d.addEntity(gone);
    auto kept = std::make_shared<hz::draft::DraftCircle>(Vec2(20, 0), 2.0);
    kept->setLayer("Locked");
    d.addEntity(kept);

    const PlotScene scene = hz::draft::buildPlotScene(d, layers, DimensionStyle{});
    ASSERT_EQ(scene.strokes.size(), 2u) << "the hidden layer's line is left out";
    EXPECT_EQ(scene.strokes[0].color, 0xFFFF0000u);
    EXPECT_DOUBLE_EQ(scene.strokes[0].width, 0.5);
    EXPECT_EQ(scene.strokes[0].lineType, 2);
    EXPECT_TRUE(scene.strokes[1].closed);
    EXPECT_GE(scene.strokes[1].points.size(), 360u);
    for (const auto& p : scene.strokes[1].points)
        EXPECT_NEAR((p - Vec2(20, 0)).length(), 2.0, 1e-9);
    EXPECT_NEAR(scene.bounds.min().x, 0.0, 1e-9);
    EXPECT_NEAR(scene.bounds.max().x, 22.0, 1e-9);
}

// A block reference plots its block placed: turned, scaled and moved, blocks
// inside it too, text included (the viewport drew only lines and curves from
// a block). What the block leaves unset is ByBlock: the reference's.
TEST(PlotSceneTest, BlocksAreExpandedWithTheirReferencesStyle) {
    DraftDocument d;
    LayerManager layers;
    auto inner = std::make_shared<hz::draft::BlockDefinition>();
    inner->name = "Inner";
    inner->entities.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(1, 0)));
    auto outer = std::make_shared<hz::draft::BlockDefinition>();
    outer->name = "Outer";
    outer->entities.push_back(std::make_shared<hz::draft::DraftBlockRef>(inner, Vec2(0, 1)));
    outer->entities.push_back(std::make_shared<hz::draft::DraftText>(Vec2(0, 0), "TAG", 1.0));

    auto ref =
        std::make_shared<hz::draft::DraftBlockRef>(outer, Vec2(10, 0), hz::math::kPi / 2, 2.0);
    ref->setColor(0xFF00FF00);
    d.addEntity(ref);

    const PlotScene scene = hz::draft::buildPlotScene(d, layers, DimensionStyle{});
    ASSERT_EQ(scene.strokes.size(), 1u);
    // Inner's line (0,0)-(1,0), at (0,1) in Outer: (0,1)-(1,1); Outer turned a
    // quarter and doubled about its base (0,0), then moved to (10,0).
    EXPECT_TRUE(near(scene.strokes[0].points[0], Vec2(8, 0), 1e-9));
    EXPECT_TRUE(near(scene.strokes[0].points[1], Vec2(8, 2), 1e-9));
    EXPECT_EQ(scene.strokes[0].color, 0xFF00FF00u) << "ByBlock";
    ASSERT_EQ(scene.texts.size(), 1u);
    EXPECT_EQ(scene.texts[0].text, "TAG");
    EXPECT_NEAR(scene.texts[0].height, 2.0, 1e-9) << "scaled with the block";
    EXPECT_NEAR(scene.texts[0].rotation, hz::math::kPi / 2, 1e-9);
}

namespace {

/// A door: its leaf from the hinge (the base point) up to (0, 10), its swing
/// a quarter round from (10, 0) to the leaf, and a tag right of the hinge.
std::shared_ptr<hz::draft::BlockDefinition> door() {
    auto def = std::make_shared<hz::draft::BlockDefinition>();
    def->name = "Door";
    def->entities.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(0, 10)));
    def->entities.push_back(
        std::make_shared<hz::draft::DraftArc>(Vec2(0, 0), 10.0, 0.0, hz::math::kPi / 2));
    auto tag = std::make_shared<hz::draft::DraftText>(Vec2(2, 1), "D", 1.0);
    tag->setAlignment(hz::draft::TextAlignment::Left);
    def->entities.push_back(tag);
    return def;
}

/// Every point of the scene's strokes, which is only the door's here.
std::vector<Vec2> allPoints(const PlotScene& scene) {
    std::vector<Vec2> out;
    for (const auto& stroke : scene.strokes) {
        out.insert(out.end(), stroke.points.begin(), stroke.points.end());
    }
    return out;
}

}  // namespace

// A door mirrored to swing the other way plots swinging the other way, its tag
// readable, from its other end. It was drawn as it was, since a mirror was
// stored as a negative scale, which the arc and the text ignored.
TEST(PlotSceneTest, AMirroredBlockPlotsMirrored) {
    DraftDocument d;
    LayerManager layers;
    auto ref = std::make_shared<hz::draft::DraftBlockRef>(door(), Vec2(0, 0));
    ref->mirror(Vec2(0, 0), Vec2(0, 1));
    d.addEntity(ref);
    const PlotScene scene = hz::draft::buildPlotScene(d, layers, DimensionStyle{});
    ASSERT_EQ(scene.strokes.size(), 2u);
    for (const Vec2& p : allPoints(scene)) {
        EXPECT_LE(p.x, 1e-9) << "all of it left of the hinge";
        EXPECT_GE(p.y, -1e-9);
    }
    EXPECT_TRUE(near(scene.strokes[1].points.front(), Vec2(-10, 0), 1e-9)) << "the swing's start";
    ASSERT_EQ(scene.texts.size(), 1u);
    EXPECT_TRUE(near(scene.texts[0].position, Vec2(-2, 1), 1e-9));
    EXPECT_NEAR(std::cos(scene.texts[0].rotation), 1.0, 1e-9) << "reads left to right";
    EXPECT_EQ(scene.texts[0].alignment, hz::draft::TextAlignment::Right) << "ends where it began";
    EXPECT_LE(scene.bounds.max().x, 1e-9) << "the tag's bounds too";
}

// A negative scale turns the block half round: the swing goes below and
// left, the tag upside down.
TEST(PlotSceneTest, ABlockScaledNegativeIsTurnedHalfRound) {
    DraftDocument d;
    LayerManager layers;
    d.addEntity(std::make_shared<hz::draft::DraftBlockRef>(door(), Vec2(0, 0), 0.0, -1.0));
    const PlotScene scene = hz::draft::buildPlotScene(d, layers, DimensionStyle{});
    ASSERT_EQ(scene.strokes.size(), 2u);
    for (const Vec2& p : allPoints(scene)) {
        EXPECT_LE(p.x, 1e-9);
        EXPECT_LE(p.y, 1e-9);
    }
    ASSERT_EQ(scene.texts.size(), 1u);
    EXPECT_TRUE(near(scene.texts[0].position, Vec2(-2, -1), 1e-9));
    EXPECT_NEAR(std::cos(scene.texts[0].rotation), -1.0, 1e-9);
    EXPECT_EQ(scene.texts[0].alignment, hz::draft::TextAlignment::Left);
}

TEST(PlotSceneTest, ADimensionPlotsItsLinesAndValue) {
    DraftDocument d;
    LayerManager layers;
    d.addEntity(std::make_shared<hz::draft::DraftLinearDimension>(
        Vec2(0, 0), Vec2(40, 0), Vec2(20, 10),
        hz::draft::DraftLinearDimension::Orientation::Horizontal));
    const PlotScene scene = hz::draft::buildPlotScene(d, layers, DimensionStyle{});
    EXPECT_GE(scene.strokes.size(), 5u) << "extension lines, the dimension line, arrowheads";
    ASSERT_EQ(scene.texts.size(), 1u);
    EXPECT_NE(scene.texts[0].text.find("40"), std::string::npos) << scene.texts[0].text;
}

TEST(PlotSceneTest, WeightsDashesAndColoursOnPaper) {
    EXPECT_DOUBLE_EQ(hz::draft::plotWeightMm(1.0), 0.25) << "the default width";
    EXPECT_DOUBLE_EQ(hz::draft::plotWeightMm(0.0), 0.25);
    EXPECT_DOUBLE_EQ(hz::draft::plotWeightMm(0.5), 0.5);
    EXPECT_DOUBLE_EQ(hz::draft::plotWeightMm(9.0), 2.11);
    EXPECT_TRUE(hz::draft::plotDashMm(1).empty());
    EXPECT_EQ(hz::draft::plotDashMm(2).size(), 2u);
    EXPECT_EQ(hz::draft::plotColor(0xFFFFFFFF, false), 0xFF000000u) << "white on white paper";
    EXPECT_EQ(hz::draft::plotColor(0xFF3366CC, false), 0xFF3366CCu);
    EXPECT_EQ(hz::draft::plotColor(0xFF3366CC, true), 0xFF000000u);
}

// Fitted, the drawing fills the printable area, centred, y up the page; at a
// scale it may not fit, and that is said.
TEST(PlotSceneTest, TheDrawingSitsOnThePaper) {
    PlotScene scene;
    scene.strokes.push_back({{Vec2(0, 0), Vec2(100, 50)}, false, 0xFF000000, 1.0, 1});
    scene.bounds.expand(hz::math::Vec3(0, 0, 0));
    scene.bounds.expand(hz::math::Vec3(100, 50, 0));

    hz::draft::PlotLayout layout;  // A4 landscape, 10 mm margins
    bool fits = false;
    auto t = hz::draft::plotTransform(scene, layout, &fits);
    EXPECT_TRUE(fits);
    EXPECT_NEAR(t.scale, 2.77, 1e-12) << "277 mm across for 100";
    EXPECT_TRUE(near(t.toPaper(Vec2(50, 25)), Vec2(148.5, 105))) << "centred";
    EXPECT_GT(t.toPaper(Vec2(0, 0)).y, t.toPaper(Vec2(0, 50)).y) << "the page's y runs down";

    layout.scale = 5.0;  // 5:1
    t = hz::draft::plotTransform(scene, layout, &fits);
    EXPECT_FALSE(fits);
    EXPECT_DOUBLE_EQ(t.scale, 5.0);
}
