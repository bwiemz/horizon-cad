#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/constraint/ParameterTable.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/math/Vec2.h"

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

    // dist - value = 5 - 5 = 0.
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

    // dist - value = 5 - 10 = -5: a length, as the solver's tolerance is.
    EXPECT_NEAR(F(0), -5.0, 1e-10);
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

// A ref that does not fit its entity gives nothing, where extractPoint() and
// extractLine() throw; the constraint tool's preview caught and dropped that.
TEST(GeometryRef, ARefThatDoesNotFitItsEntityGivesNothing) {
    draft::DraftLine line(math::Vec2(0, 0), math::Vec2(3, 4));
    const cstr::GeometryRef end{line.id(), cstr::FeatureType::Point, 1};
    const auto p = cstr::pointOf(end, line);
    ASSERT_TRUE(p.has_value());
    EXPECT_DOUBLE_EQ(p->x, 3.0);
    EXPECT_DOUBLE_EQ(p->y, 4.0);

    const cstr::GeometryRef past{line.id(), cstr::FeatureType::Point, 2};
    EXPECT_FALSE(cstr::pointOf(past, line).has_value());
    EXPECT_THROW(cstr::extractPoint(past, line), std::runtime_error);

    const cstr::GeometryRef edge{line.id(), cstr::FeatureType::Line, 0};
    EXPECT_TRUE(cstr::lineOf(edge, line).has_value());
    EXPECT_FALSE(cstr::lineOf({line.id(), cstr::FeatureType::Line, 1}, line).has_value());
    draft::DraftCircle circle(math::Vec2(0, 0), 1.0);
    EXPECT_FALSE(cstr::lineOf(edge, circle).has_value()) << "a circle has no line";
    EXPECT_THROW(cstr::extractLine(edge, circle), std::runtime_error);
}

// A tangent is between a line and a circle or arc, picked in either order:
// the line is put first. It was kept as picked, and a circle picked first was
// read as a line, which threw at every solve.
TEST(Constraints, ATangentTakesItsLineAndCircleInEitherOrder) {
    draft::DraftLine line(math::Vec2(-10, 0), math::Vec2(10, 0));
    draft::DraftCircle circle(math::Vec2(0, 8), 5.0);
    const cstr::GeometryRef edge{line.id(), cstr::FeatureType::Line, 0};
    const cstr::GeometryRef rim{circle.id(), cstr::FeatureType::Circle, 0};
    const cstr::TangentConstraint forward(edge, rim);
    const cstr::TangentConstraint reversed(rim, edge);
    EXPECT_EQ(reversed.lineRef(), edge);
    EXPECT_EQ(reversed.circleRef(), rim);

    cstr::ParameterTable params;
    params.registerEntity(line);
    params.registerEntity(circle);
    Eigen::VectorXd f = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(1);
    forward.evaluate(params, f, 0);
    ASSERT_NO_THROW(reversed.evaluate(params, g, 0));
    EXPECT_EQ(f(0), g(0));
}

// Two lines, or two circles, have no tangent here; equal is two lines or two
// circles (an arc is one). A pair neither can hold is refused when it is
// made, so none is in a system to throw at its every solve.
TEST(Constraints, TangentAndEqualRefuseAPairTheyCannotHold) {
    const cstr::GeometryRef lineA{1, cstr::FeatureType::Line, 0};
    const cstr::GeometryRef lineB{2, cstr::FeatureType::Line, 0};
    const cstr::GeometryRef circleA{3, cstr::FeatureType::Circle, 0};
    const cstr::GeometryRef circleB{4, cstr::FeatureType::Circle, 0};
    const cstr::GeometryRef end{1, cstr::FeatureType::Point, 1};
    using Tangent = cstr::TangentConstraint;
    using Equal = cstr::EqualConstraint;
    EXPECT_THROW((void)std::make_shared<Tangent>(lineA, lineB), std::invalid_argument);
    EXPECT_THROW((void)std::make_shared<Tangent>(circleA, circleB), std::invalid_argument);
    EXPECT_THROW((void)std::make_shared<Tangent>(end, circleA), std::invalid_argument);
    EXPECT_THROW((void)std::make_shared<Equal>(lineA, circleA), std::invalid_argument);
    EXPECT_THROW((void)std::make_shared<Equal>(circleA, lineA), std::invalid_argument);
    EXPECT_THROW((void)std::make_shared<Equal>(end, end), std::invalid_argument);
    EXPECT_NO_THROW((void)std::make_shared<Tangent>(circleA, lineA));
    EXPECT_NO_THROW((void)std::make_shared<Equal>(lineA, lineB));
    EXPECT_NO_THROW((void)std::make_shared<Equal>(circleA, circleB));
}

// Each residual is a length or an angle, so one tolerance suits them all.
// Distance, equal lengths, perpendicular and parallel were lengths squared,
// and a tangent a length to the fourth: met, at ordinary sizes, their
// rounding alone was more than the tolerance.
TEST(Constraints, EachResidualIsALengthOrAnAngle) {
    draft::DraftLine across(math::Vec2(-10, 0), math::Vec2(10, 0));               // 20 long
    draft::DraftLine slope(math::Vec2(0, 0), math::Vec2(5, 5 * std::sqrt(3.0)));  // 10, at 60 deg
    draft::DraftCircle circle(math::Vec2(0, 8), 5.0);
    cstr::ParameterTable params;
    params.registerEntity(across);
    params.registerEntity(slope);
    params.registerEntity(circle);
    const cstr::GeometryRef a{across.id(), cstr::FeatureType::Line, 0};
    const cstr::GeometryRef b{slope.id(), cstr::FeatureType::Line, 0};
    const cstr::GeometryRef c{circle.id(), cstr::FeatureType::Circle, 0};
    const auto residual = [&](const cstr::Constraint& k) {
        Eigen::VectorXd f = Eigen::VectorXd::Zero(1);
        k.evaluate(params, f, 0);
        return f(0);
    };
    EXPECT_NEAR(residual(cstr::TangentConstraint(a, c)), 8.0 - 5.0, 1e-12)
        << "centre to line, less r";
    EXPECT_NEAR(residual(cstr::EqualConstraint(a, b)), 20.0 - 10.0, 1e-12);
    EXPECT_NEAR(residual(cstr::PerpendicularConstraint(a, b)), 0.5, 1e-12) << "cos 60 deg";
    EXPECT_NEAR(residual(cstr::ParallelConstraint(a, b)), std::sqrt(3.0) / 2, 1e-12)
        << "sin 60 deg";
    const cstr::GeometryRef start{across.id(), cstr::FeatureType::Point, 0};
    const cstr::GeometryRef end{across.id(), cstr::FeatureType::Point, 1};
    EXPECT_NEAR(residual(cstr::DistanceConstraint(start, end, 25.0)), 20.0 - 25.0, 1e-12);
    // A distance read from a variable that came out negative is its size, as
    // the squared form took it.
    EXPECT_NEAR(residual(cstr::DistanceConstraint(start, end, -20.0)), 0.0, 1e-12);
}
