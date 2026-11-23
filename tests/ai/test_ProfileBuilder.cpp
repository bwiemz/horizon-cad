#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "horizon/ai/ProfileBuilder.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftText.h"

using hz::ai::buildEntities;
using hz::ai::entitySchema;
using namespace hz::draft;
using nlohmann::json;

namespace {

constexpr double kPi = 3.14159265358979323846;

template <typename T>
const T* as(const std::shared_ptr<DraftEntity>& entity) {
    return dynamic_cast<const T*>(entity.get());
}

std::string errorOf(const json& entities, bool allowText = false) {
    try {
        buildEntities(entities, allowText);
    } catch (const std::invalid_argument& e) {
        return e.what();
    }
    return {};
}

}  // namespace

TEST(ProfileBuilder, Rectangle) {
    const auto out = buildEntities(
        json::array({{{"type", "rectangle"}, {"corner", {1, 2}}, {"width", 10}, {"height", 4}}}),
        false);
    ASSERT_EQ(out.size(), 1u);
    const auto* rect = as<DraftRectangle>(out[0]);
    ASSERT_NE(rect, nullptr);
    EXPECT_DOUBLE_EQ(rect->corner1().x, 1.0);
    EXPECT_DOUBLE_EQ(rect->corner2().x, 11.0);
    EXPECT_DOUBLE_EQ(rect->corner2().y, 6.0);
}

TEST(ProfileBuilder, CentredAndRotatedRectangle) {
    const auto centred = buildEntities(
        json::array({{{"type", "rectangle"}, {"center", {0, 0}}, {"width", 10}, {"height", 4}}}),
        false);
    const auto* rect = as<DraftRectangle>(centred[0]);
    ASSERT_NE(rect, nullptr);
    EXPECT_DOUBLE_EQ(rect->corner1().x, -5.0);
    EXPECT_DOUBLE_EQ(rect->corner1().y, -2.0);

    const auto turned = buildEntities(json::array({{{"type", "rectangle"},
                                                    {"center", {0, 0}},
                                                    {"width", 10},
                                                    {"height", 4},
                                                    {"angle", 90}}}),
                                      false);
    const auto* poly = as<DraftPolyline>(turned[0]);
    ASSERT_NE(poly, nullptr);
    EXPECT_TRUE(poly->closed());
    ASSERT_EQ(poly->points().size(), 4u);
    // Turned a quarter: the 10 mm side now runs along y.
    EXPECT_NEAR(poly->points()[0].x, 2.0, 1e-9);
    EXPECT_NEAR(poly->points()[0].y, -5.0, 1e-9);
}

TEST(ProfileBuilder, Circle) {
    const auto out = buildEntities(
        json::array({{{"type", "circle"}, {"center", {3, 4}}, {"radius", 2}},
                     {{"type", "circle"}, {"center", {{"x", 0}, {"y", 0}}}, {"diameter", 10}}}),
        false);
    ASSERT_EQ(out.size(), 2u);
    EXPECT_DOUBLE_EQ(as<DraftCircle>(out[0])->radius(), 2.0);
    EXPECT_DOUBLE_EQ(as<DraftCircle>(out[1])->radius(), 5.0);
}

TEST(ProfileBuilder, ArcInDegrees) {
    const auto out = buildEntities(json::array({{{"type", "arc"},
                                                 {"center", {0, 0}},
                                                 {"radius", 5},
                                                 {"start_angle", 0},
                                                 {"end_angle", 90}}}),
                                   false);
    const auto* arc = as<DraftArc>(out[0]);
    ASSERT_NE(arc, nullptr);
    EXPECT_NEAR(arc->startAngle(), 0.0, 1e-12);
    EXPECT_NEAR(arc->endAngle(), kPi / 2.0, 1e-12);
}

TEST(ProfileBuilder, ClosedPolyline) {
    const auto out = buildEntities(json::array({{{"type", "polyline"},
                                                 {"points", {{0, 0}, {10, 0}, {10, 5}, {0, 0}}},
                                                 {"closed", true}}}),
                                   false);
    const auto* poly = as<DraftPolyline>(out[0]);
    ASSERT_NE(poly, nullptr);
    EXPECT_TRUE(poly->closed());
    EXPECT_EQ(poly->points().size(), 3u);  // the repeated first point dropped
}

TEST(ProfileBuilder, Slot) {
    const auto out = buildEntities(
        json::array({{{"type", "slot"}, {"center", {0, 0}}, {"length", 20}, {"width", 6}}}), false);
    ASSERT_EQ(out.size(), 4u);
    const auto* top = as<DraftLine>(out[0]);
    ASSERT_NE(top, nullptr);
    EXPECT_NEAR(top->start().x, -10.0, 1e-9);
    EXPECT_NEAR(top->start().y, 3.0, 1e-9);
    EXPECT_NEAR(top->end().x, 10.0, 1e-9);
    const auto* right = as<DraftArc>(out[1]);
    ASSERT_NE(right, nullptr);
    EXPECT_NEAR(right->center().x, 10.0, 1e-9);
    EXPECT_NEAR(right->radius(), 3.0, 1e-9);
    ASSERT_NE(as<DraftArc>(out[3]), nullptr);
    // A slot of no length is a circle.
    const auto round = buildEntities(
        json::array({{{"type", "slot"}, {"center", {0, 0}}, {"length", 0}, {"width", 6}}}), false);
    ASSERT_EQ(round.size(), 1u);
    EXPECT_NE(as<DraftCircle>(round[0]), nullptr);
}

TEST(ProfileBuilder, RegularPolygon) {
    const auto out = buildEntities(
        json::array({{{"type", "polygon"}, {"center", {0, 0}}, {"sides", 6}, {"radius", 10}}}),
        false);
    const auto* poly = as<DraftPolyline>(out[0]);
    ASSERT_NE(poly, nullptr);
    ASSERT_EQ(poly->points().size(), 6u);
    EXPECT_NEAR(poly->points()[0].x, 0.0, 1e-9);  // a vertex at the top
    EXPECT_NEAR(poly->points()[0].y, 10.0, 1e-9);
}

TEST(ProfileBuilder, ConstructionAndText) {
    const auto out = buildEntities(
        json::array({{{"type", "line"}, {"from", {0, 0}}, {"to", {1, 0}}, {"construction", true}},
                     {{"type", "text"}, {"position", {0, 0}}, {"text", "A"}, {"height", 5}}}),
        true);
    ASSERT_EQ(out.size(), 2u);
    EXPECT_TRUE(out[0]->construction());
    EXPECT_DOUBLE_EQ(as<DraftText>(out[1])->textHeight(), 5.0);
    // Text is not part of a sketch.
    EXPECT_NE(errorOf(json::array({{{"type", "text"}, {"position", {0, 0}}, {"text", "A"}}})), "");
}

TEST(ProfileBuilder, UnknownTypeIsAnError) {
    EXPECT_NE(errorOf(json::array({{{"type", "spline"}}})).find("unknown type"), std::string::npos);
    EXPECT_NE(errorOf(json::array({{{"radius", 3}}})).find("\"type\""), std::string::npos);
    EXPECT_NE(errorOf(json::object()).find("list"), std::string::npos);
    EXPECT_NE(errorOf(json::array()).find("empty"), std::string::npos);
}

TEST(ProfileBuilder, BadValuesAreErrors) {
    const std::string negative =
        errorOf(json::array({{{"type", "circle"}, {"center", {0, 0}}, {"radius", -1}}}));
    EXPECT_NE(negative.find("entity 0 (circle)"), std::string::npos);
    EXPECT_NE(negative.find("radius"), std::string::npos);
    EXPECT_NE(errorOf(json::array({{{"type", "circle"}, {"center", {0}}, {"radius", 1}}})), "");
    EXPECT_NE(errorOf(json::array({{{"type", "circle"}, {"center", "here"}, {"radius", 1}}})), "");
    EXPECT_NE(errorOf(json::array({{{"type", "line"}, {"from", {1, 1}}, {"to", {1, 1}}}})), "");
    EXPECT_NE(errorOf(json::array(
                  {{{"type", "polygon"}, {"center", {0, 0}}, {"sides", 2}, {"radius", 1}}})),
              "");
    EXPECT_NE(errorOf(json::array(
                  {{{"type", "polygon"}, {"center", {0, 0}}, {"sides", 4.5}, {"radius", 1}}})),
              "");
    EXPECT_NE(errorOf(json::array(
                  {{{"type", "polyline"}, {"points", {{0, 0}, {1, 1}}}, {"closed", true}}})),
              "");
    EXPECT_NE(errorOf(json::array({{{"type", "arc"},
                                    {"center", {0, 0}},
                                    {"radius", 1},
                                    {"start_angle", 10},
                                    {"end_angle", 10}}})),
              "");
}

TEST(ProfileBuilder, SchemaListsTheTypes) {
    const json schema = entitySchema(false);
    EXPECT_EQ(schema["type"], "object");
    EXPECT_EQ(schema["properties"]["type"]["enum"].size(), 7u);
    EXPECT_EQ(entitySchema(true)["properties"]["type"]["enum"].size(), 8u);
}
