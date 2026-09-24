// A failing feature says why, in the user's terms — not "Feature 'X' failed
// to execute".

#include <gtest/gtest.h>

#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/topology/TopologyID.h"

using hz::doc::Document;
using hz::doc::Sketch;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

std::shared_ptr<Sketch> polyline(std::initializer_list<Vec2> points, bool close) {
    auto sketch = std::make_shared<Sketch>();
    std::vector<Vec2> pts(points);
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        sketch->addEntity(std::make_shared<hz::draft::DraftLine>(pts[i], pts[i + 1]));
    }
    if (close) sketch->addEntity(std::make_shared<hz::draft::DraftLine>(pts.back(), pts.front()));
    return sketch;
}

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

/// Rebuild; require failure at `index` and return the message.
std::string failureAt(Document& doc, int index) {
    EXPECT_FALSE(doc.rebuildModel());
    EXPECT_EQ(doc.failedFeatureIndex(), index);
    return doc.lastBuildMessage();
}

}  // namespace

TEST(FeatureReasonsTest, AnOpenProfileSaysWhereItIsOpen) {
    // Three sides of a square: the ends at (0, 0) and (0, 10) do not meet.
    auto open = polyline({{0, 0}, {10, 0}, {10, 10}, {0, 10}}, false);
    std::string why;
    EXPECT_EQ(hz::model::Extrude::execute(open->entities(), open->plane(), Vec3(0, 0, 1), 5.0, "x",
                                          32, 0.0, &why),
              nullptr);
    EXPECT_TRUE(contains(why, "open")) << why;
    EXPECT_TRUE(contains(why, "(0, 0)")) << why;
    EXPECT_TRUE(contains(why, "(0, 10)")) << why;

    Document doc;
    doc.featureTree().addFeature(
        std::make_unique<hz::doc::ExtrudeFeature>(open, Vec3(0, 0, 1), 5.0));
    const std::string message = failureAt(doc, 0);
    EXPECT_TRUE(contains(message, "Extrude")) << message;
    EXPECT_TRUE(contains(message, "open")) << message;
}

TEST(FeatureReasonsTest, AnExtrusionAlongItsOwnPlaneSweepsNoVolume) {
    auto square = polyline({{0, 0}, {1, 0}, {1, 1}, {0, 1}}, true);
    std::string why;
    EXPECT_EQ(hz::model::Extrude::execute(square->entities(), square->plane(), Vec3(1, 0, 0), 5.0,
                                          "x", 32, 0.0, &why),
              nullptr);
    EXPECT_TRUE(contains(why, "sketch plane")) << why;
    EXPECT_EQ(hz::model::Extrude::execute(square->entities(), square->plane(), Vec3(0, 0, 1), 0.0,
                                          "x", 32, 0.0, &why),
              nullptr);
    EXPECT_TRUE(contains(why, "distance")) << why;
}

TEST(FeatureReasonsTest, ARevolveAcrossItsAxisSaysSo) {
    // A rectangle straddling the Y axis: revolving it would self-intersect.
    auto straddle = polyline({{-2, 0}, {2, 0}, {2, 5}, {-2, 5}}, true);
    std::string why;
    EXPECT_EQ(hz::model::Revolve::execute(straddle->entities(), straddle->plane(), Vec3(0, 0, 0),
                                          Vec3(0, 1, 0), hz::math::kPi, "x", 32, 0.0, &why),
              nullptr);
    EXPECT_TRUE(contains(why, "one side of the axis")) << why;
}

TEST(FeatureReasonsTest, AFilletOnAMissingEdgeNamesIt) {
    Document doc;
    doc.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    doc.featureTree().addFeature(std::make_unique<hz::doc::FilletFeature>(
        std::vector<hz::topo::TopologyID>{hz::topo::TopologyID::make("box", "no_such_edge")}, 1.0));
    const std::string message = failureAt(doc, 1);
    EXPECT_TRUE(contains(message, "no_such_edge"))
        << "FilletOp's own message is not dropped: " << message;
}

TEST(FeatureReasonsTest, ImpossiblePrimitiveDimensionsSaySo) {
    Document doc;
    doc.featureTree().addFeature(hz::doc::PrimitiveFeature::makeSphere(0.0));
    const std::string message = failureAt(doc, 0);
    EXPECT_TRUE(contains(message, "dimensions")) << message;
}

TEST(FeatureReasonsTest, ASweepWithoutAPathSaysSo) {
    Document doc;
    doc.featureTree().addFeature(
        std::make_unique<hz::doc::SweepFeature>(polyline({{0, 0}, {1, 0}, {1, 1}}, true), nullptr));
    const std::string message = failureAt(doc, 0);
    EXPECT_TRUE(contains(message, "missing")) << message;
}

TEST(FeatureReasonsTest, BooleanReasonsDistinguishEmptyFromFailed) {
    auto a = hz::model::PrimitiveFactory::makeBox(10, 10, 10);
    auto b = hz::model::PrimitiveFactory::makeBox(10, 10, 10);
    std::string why;
    EXPECT_EQ(hz::model::BooleanOp::execute(*a, *b, hz::model::BooleanType::Subtract, &why),
              nullptr);
    EXPECT_TRUE(contains(why, "removes the whole body")) << why;

    auto far = hz::model::Pattern::transformed(*b, hz::math::Mat4::translation(Vec3(100, 0, 0)));
    EXPECT_EQ(hz::model::BooleanOp::execute(*a, *far, hz::model::BooleanType::Intersect, &why),
              nullptr);
    EXPECT_TRUE(contains(why, "do not overlap")) << why;
}
