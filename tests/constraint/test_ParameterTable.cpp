#include <gtest/gtest.h>
#include "horizon/constraint/ParameterTable.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
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
