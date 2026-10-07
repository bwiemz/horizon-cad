#include <gtest/gtest.h>

#include <stdexcept>

#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/constraint/ParameterTable.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/math/Vec2.h"

using namespace hz;

TEST(ParameterTable, RegisterLine) {
    cstr::ParameterTable params;

    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    params.registerEntity(*line);

    // A line has 4 parameters: startX, startY, endX, endY.
    EXPECT_EQ(params.parameterCount(), 4);
}

TEST(ParameterTable, RegisterCircle) {
    cstr::ParameterTable params;

    auto circle = std::make_shared<draft::DraftCircle>(math::Vec2{5, 5}, 3.0);
    params.registerEntity(*circle);

    // A circle has 3 parameters: centerX, centerY, radius.
    EXPECT_EQ(params.parameterCount(), 3);
}

TEST(ParameterTable, MultipleEntities) {
    cstr::ParameterTable params;

    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    auto circle = std::make_shared<draft::DraftCircle>(math::Vec2{5, 5}, 3.0);
    params.registerEntity(*line);
    params.registerEntity(*circle);

    // 4 (line) + 3 (circle) = 7
    EXPECT_EQ(params.parameterCount(), 7);
}

TEST(ParameterTable, PointPosition) {
    cstr::ParameterTable params;

    auto line = std::make_shared<draft::DraftLine>(math::Vec2{1.0, 2.0}, math::Vec2{3.0, 4.0});
    params.registerEntity(*line);

    // Point(0) = start, Point(1) = end.
    cstr::GeometryRef startRef{line->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef endRef{line->id(), cstr::FeatureType::Point, 1};

    math::Vec2 start = params.pointPosition(startRef);
    math::Vec2 end = params.pointPosition(endRef);

    EXPECT_NEAR(start.x, 1.0, 1e-10);
    EXPECT_NEAR(start.y, 2.0, 1e-10);
    EXPECT_NEAR(end.x, 3.0, 1e-10);
    EXPECT_NEAR(end.y, 4.0, 1e-10);
}

TEST(ParameterTable, ApplyToEntities) {
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});

    cstr::ParameterTable params;
    params.registerEntity(*line);

    // Modify parameters directly.
    cstr::GeometryRef startRef{line->id(), cstr::FeatureType::Point, 0};
    int idx = params.parameterIndex(startRef);
    params.values()(idx + 0) = 1.0;   // startX
    params.values()(idx + 1) = 2.0;   // startY
    params.values()(idx + 2) = 11.0;  // endX
    params.values()(idx + 3) = 2.0;   // endY

    std::vector<std::shared_ptr<draft::DraftEntity>> entities;
    entities.push_back(line);
    params.applyToEntities(entities);

    // Verify the entity was updated.
    EXPECT_NEAR(line->start().x, 1.0, 1e-10);
    EXPECT_NEAR(line->start().y, 2.0, 1e-10);
    EXPECT_NEAR(line->end().x, 11.0, 1e-10);
    EXPECT_NEAR(line->end().y, 2.0, 1e-10);
}

TEST(ParameterTable, BuildFromEntities) {
    auto line1 = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    auto line2 = std::make_shared<draft::DraftLine>(math::Vec2{10, 0}, math::Vec2{20, 0});
    auto circle = std::make_shared<draft::DraftCircle>(math::Vec2{5, 5}, 3.0);

    std::vector<std::shared_ptr<draft::DraftEntity>> entities{line1, line2, circle};

    // Only register entities that are in constraints.
    cstr::ConstraintSystem sys;
    cstr::GeometryRef refA{line1->id(), cstr::FeatureType::Point, 1};
    cstr::GeometryRef refB{line2->id(), cstr::FeatureType::Point, 0};
    sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(refA, refB));

    auto params = cstr::ParameterTable::buildFromEntities(entities, sys);

    // Only line1 and line2 should be registered (not circle).
    EXPECT_TRUE(params.hasEntity(line1->id()));
    EXPECT_TRUE(params.hasEntity(line2->id()));
    EXPECT_FALSE(params.hasEntity(circle->id()));
    EXPECT_EQ(params.parameterCount(), 8);  // 4 + 4
}

TEST(ParameterTable, HasEntity) {
    cstr::ParameterTable params;

    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    EXPECT_FALSE(params.hasEntity(line->id()));

    params.registerEntity(*line);
    EXPECT_TRUE(params.hasEntity(line->id()));
}

// A ref its entity does not have is refused with an exception, as a circle
// read as a line already was: a third end of a line, a vertex past a
// polyline's last, an open polyline's closing segment. Each was read from
// whatever parameters came next, another entity's or none at all.
TEST(ParameterTable, ARefItsEntityDoesNotHaveThrows) {
    draft::DraftLine line(math::Vec2{0, 0}, math::Vec2{10, 0});
    draft::DraftPolyline open({math::Vec2{0, 0}, math::Vec2{5, 0}, math::Vec2{5, 5}});
    draft::DraftCircle circle(math::Vec2{5, 5}, 3.0);
    draft::DraftArc arc(math::Vec2{0, 0}, 4.0, 0.0, 2.0);
    draft::DraftRectangle rect(math::Vec2{0, 0}, math::Vec2{4, 3});
    cstr::ParameterTable params;
    // In this order a ref past one entity's end reads the next one's
    // parameters rather than past the table's.
    params.registerEntity(line);
    params.registerEntity(open);
    params.registerEntity(circle);
    params.registerEntity(arc);
    params.registerEntity(rect);
    using cstr::FeatureType;
    const auto point = [](const draft::DraftEntity& e, int i) {
        return cstr::GeometryRef{e.id(), FeatureType::Point, i};
    };
    const auto edge = [](const draft::DraftEntity& e, int i) {
        return cstr::GeometryRef{e.id(), FeatureType::Line, i};
    };
    EXPECT_THROW(params.pointPosition(point(line, 2)), std::runtime_error);
    EXPECT_THROW(params.parameterIndex(point(line, 2)), std::runtime_error);
    EXPECT_THROW(params.pointPosition(point(open, 3)), std::runtime_error);
    EXPECT_THROW(params.pointPosition(point(open, -1)), std::runtime_error);
    EXPECT_THROW(params.pointPosition(point(circle, 1)), std::runtime_error);
    EXPECT_THROW(params.pointPosition(point(arc, 3)), std::runtime_error);
    EXPECT_THROW(params.pointPosition(point(rect, 4)), std::runtime_error);
    EXPECT_THROW(params.lineEndpoints(edge(line, 1)), std::runtime_error);
    EXPECT_THROW(params.lineEndpoints(edge(open, 2)), std::runtime_error)
        << "open: no closing segment";
    EXPECT_THROW(params.lineEndpoints(edge(rect, 4)), std::runtime_error);
    EXPECT_THROW(params.lineEndpoints(edge(circle, 0)), std::runtime_error);
    EXPECT_THROW(params.circleData({line.id(), FeatureType::Circle, 0}), std::runtime_error);

    // What each has, it gives.
    EXPECT_NO_THROW(params.pointPosition(point(line, 1)));
    EXPECT_NO_THROW(params.pointPosition(point(open, 2)));
    EXPECT_NO_THROW(params.pointPosition(point(arc, 2)));
    EXPECT_NO_THROW(params.pointPosition(point(rect, 3)));
    EXPECT_NO_THROW(params.lineEndpoints(edge(open, 1)));
    EXPECT_NO_THROW(params.lineEndpoints(edge(rect, 3)));
    EXPECT_NO_THROW(params.circleData({arc.id(), FeatureType::Circle, 0}));
}
