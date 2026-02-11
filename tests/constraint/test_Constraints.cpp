#include <gtest/gtest.h>
#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/constraint/ParameterTable.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/math/Vec2.h"
#include <Eigen/Dense>

using namespace hz;

// --- ConstraintSystem tests ---

TEST(ConstraintSystem, AddAndRemove) {
    cstr::ConstraintSystem sys;
    EXPECT_TRUE(sys.empty());

    cstr::GeometryRef refA{1, cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{2, cstr::FeatureType::Point, 0};
    auto c = std::make_shared<cstr::CoincidentConstraint>(refA, refB);

    uint64_t id = sys.addConstraint(c);
    EXPECT_FALSE(sys.empty());
    EXPECT_NE(sys.getConstraint(id), nullptr);

    auto removed = sys.removeConstraint(id);
    EXPECT_TRUE(sys.empty());
    EXPECT_NE(removed, nullptr);
}

TEST(ConstraintSystem, ConstraintsForEntity) {
    cstr::ConstraintSystem sys;

    cstr::GeometryRef refA{1, cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{2, cstr::FeatureType::Point, 0};
    cstr::GeometryRef refC{3, cstr::FeatureType::Point, 0};

    sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(refA, refB));
    sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(refB, refC));

    auto forEntity1 = sys.constraintsForEntity(1);
    auto forEntity2 = sys.constraintsForEntity(2);
    auto forEntity3 = sys.constraintsForEntity(3);

    EXPECT_EQ(forEntity1.size(), 1u);
    EXPECT_EQ(forEntity2.size(), 2u);  // Entity 2 is in both constraints.
    EXPECT_EQ(forEntity3.size(), 1u);
}

TEST(ConstraintSystem, RemoveConstraintsForEntity) {
    cstr::ConstraintSystem sys;

    cstr::GeometryRef refA{1, cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{2, cstr::FeatureType::Point, 0};
    cstr::GeometryRef refC{3, cstr::FeatureType::Point, 0};

    sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(refA, refB));
    sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(refB, refC));

    auto removed = sys.removeConstraintsForEntity(2);
    EXPECT_EQ(removed.size(), 2u);
    EXPECT_TRUE(sys.empty());
}

// --- Residual tests ---

TEST(Constraints, CoincidentResidual) {
    auto line1 = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    auto line2 = std::make_shared<draft::DraftLine>(math::Vec2{10.5, 0.5}, math::Vec2{20, 0});

    cstr::ParameterTable params;
    params.registerEntity(*line1);
    params.registerEntity(*line2);

    // Coincident: line1.end == line2.start.
    cstr::GeometryRef refA{line1->id(), cstr::FeatureType::Point, 1};
    cstr::GeometryRef refB{line2->id(), cstr::FeatureType::Point, 0};
    cstr::CoincidentConstraint cc(refA, refB);

    EXPECT_EQ(cc.equationCount(), 2);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(2);
    cc.evaluate(params, F, 0);

    // Residual should be non-zero (10.0 - 10.5 = -0.5, 0.0 - 0.5 = -0.5).
    EXPECT_NEAR(F(0), -0.5, 1e-10);
    EXPECT_NEAR(F(1), -0.5, 1e-10);
}

TEST(Constraints, HorizontalResidual) {
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 1}, math::Vec2{10, 2});

    cstr::ParameterTable params;
    params.registerEntity(*line);

    cstr::GeometryRef refA{line->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{line->id(), cstr::FeatureType::Point, 1};
    cstr::HorizontalConstraint hc(refA, refB);

    EXPECT_EQ(hc.equationCount(), 1);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(1);
    hc.evaluate(params, F, 0);

    // Residual: pA.y - pB.y = 1 - 2 = -1.
    EXPECT_NEAR(F(0), -1.0, 1e-10);
}

TEST(Constraints, VerticalResidual) {
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{1, 0}, math::Vec2{2, 10});

    cstr::ParameterTable params;
    params.registerEntity(*line);

    cstr::GeometryRef refA{line->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{line->id(), cstr::FeatureType::Point, 1};
    cstr::VerticalConstraint vc(refA, refB);

    EXPECT_EQ(vc.equationCount(), 1);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(1);
    vc.evaluate(params, F, 0);

    // Residual: pA.x - pB.x = 1 - 2 = -1.
    EXPECT_NEAR(F(0), -1.0, 1e-10);
}

TEST(Constraints, FixedResidual) {
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0.1, 0.2}, math::Vec2{10, 0});

    cstr::ParameterTable params;
    params.registerEntity(*line);

    cstr::GeometryRef ref{line->id(), cstr::FeatureType::Point, 0};
    cstr::FixedConstraint fc(ref, math::Vec2{0.0, 0.0});

    EXPECT_EQ(fc.equationCount(), 2);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(2);
    fc.evaluate(params, F, 0);

    EXPECT_NEAR(F(0), 0.1, 1e-10);
    EXPECT_NEAR(F(1), 0.2, 1e-10);
}

TEST(Constraints, DistanceResidual) {
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{3, 4});

    cstr::ParameterTable params;
    params.registerEntity(*line);

    // Distance between start and end = 5.0.
    cstr::GeometryRef refA{line->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{line->id(), cstr::FeatureType::Point, 1};
    cstr::DistanceConstraint dc(refA, refB, 5.0);

    EXPECT_EQ(dc.equationCount(), 1);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(1);
    dc.evaluate(params, F, 0);

    // dist^2 - value^2 = 25 - 25 = 0.
    EXPECT_NEAR(F(0), 0.0, 1e-10);
}

TEST(Constraints, DistanceResidualNonZero) {
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{3, 4});

    cstr::ParameterTable params;
    params.registerEntity(*line);

    // Constraint wants distance = 10.
    cstr::GeometryRef refA{line->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{line->id(), cstr::FeatureType::Point, 1};
    cstr::DistanceConstraint dc(refA, refB, 10.0);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(1);
    dc.evaluate(params, F, 0);

    // dist^2 - value^2 = 25 - 100 = -75.
    EXPECT_NEAR(F(0), -75.0, 1e-10);
}

TEST(Constraints, ClonePreservesType) {
    cstr::GeometryRef refA{1, cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{2, cstr::FeatureType::Point, 0};
    cstr::CoincidentConstraint cc(refA, refB);

    auto cloned = cc.clone();
    EXPECT_EQ(cloned->type(), cstr::ConstraintType::Coincident);
    EXPECT_EQ(cloned->equationCount(), 2);
}

TEST(Constraints, DimensionalValueAccessors) {
    cstr::GeometryRef refA{1, cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{2, cstr::FeatureType::Point, 0};

    cstr::DistanceConstraint dc(refA, refB, 10.0);
    EXPECT_TRUE(dc.hasDimensionalValue());
    EXPECT_NEAR(dc.dimensionalValue(), 10.0, 1e-10);

    dc.setDimensionalValue(20.0);
    EXPECT_NEAR(dc.dimensionalValue(), 20.0, 1e-10);

    cstr::CoincidentConstraint cc(refA, refB);
    EXPECT_FALSE(cc.hasDimensionalValue());
}
