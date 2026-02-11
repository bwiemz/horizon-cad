#define _USE_MATH_DEFINES
#include <cmath>
#include <gtest/gtest.h>
#include "horizon/constraint/SketchSolver.h"
#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/ParameterTable.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/math/Vec2.h"

using namespace hz;

TEST(SketchSolver, NoConstraintsReturnsNoConstraints) {
    cstr::ConstraintSystem sys;
    cstr::ParameterTable params;
    cstr::SketchSolver solver;

    auto result = solver.solve(params, sys);
    EXPECT_EQ(result.status, cstr::SolveStatus::NoConstraints);
}

TEST(SketchSolver, CoincidentSolve) {
    draft::DraftDocument doc;
    // Two lines: line1 end at (10, 0), line2 start at (10.5, 0.3).
    auto line1 = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    auto line2 = std::make_shared<draft::DraftLine>(math::Vec2{10.5, 0.3}, math::Vec2{20, 0});
    doc.addEntity(line1);
    doc.addEntity(line2);

    cstr::ConstraintSystem sys;
    cstr::GeometryRef refA{line1->id(), cstr::FeatureType::Point, 1};   // line1 end
    cstr::GeometryRef refB{line2->id(), cstr::FeatureType::Point, 0};   // line2 start
    sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(refA, refB));

    // Fix line1 start and end, and line2 end so the solver only moves line2 start.
    cstr::GeometryRef fixLine1Start{line1->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef fixLine1End{line1->id(), cstr::FeatureType::Point, 1};
    cstr::GeometryRef fixLine2End{line2->id(), cstr::FeatureType::Point, 1};
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(fixLine1Start, math::Vec2{0.0, 0.0}));
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(fixLine1End, math::Vec2{10.0, 0.0}));
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(fixLine2End, math::Vec2{20.0, 0.0}));

    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);

    cstr::SketchSolver solver;
    auto result = solver.solve(params, sys);

    EXPECT_EQ(result.status, cstr::SolveStatus::Success);
    EXPECT_LT(result.residualNorm, 1e-8);

    params.applyToEntities(doc.entities());

    // line2's start should now be at (10, 0).
    auto* updatedLine2 = dynamic_cast<draft::DraftLine*>(doc.entities()[1].get());
    ASSERT_NE(updatedLine2, nullptr);
    EXPECT_NEAR(updatedLine2->start().x, 10.0, 1e-6);
    EXPECT_NEAR(updatedLine2->start().y, 0.0, 1e-6);
}

TEST(SketchSolver, HorizontalConstraint) {
    draft::DraftDocument doc;
    // Line from (0, 0) to (10, 2) — not horizontal.
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 2});
    doc.addEntity(line);

    cstr::ConstraintSystem sys;
    cstr::GeometryRef refA{line->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{line->id(), cstr::FeatureType::Point, 1};
    sys.addConstraint(std::make_shared<cstr::HorizontalConstraint>(refA, refB));

    // Fix the start point.
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(refA, math::Vec2{0.0, 0.0}));

    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);

    cstr::SketchSolver solver;
    auto result = solver.solve(params, sys);

    EXPECT_TRUE(result.status == cstr::SolveStatus::Success ||
                result.status == cstr::SolveStatus::UnderConstrained);

    params.applyToEntities(doc.entities());

    auto* updated = dynamic_cast<draft::DraftLine*>(doc.entities().front().get());
    ASSERT_NE(updated, nullptr);
    // The Y coordinates should be equal (horizontal).
    EXPECT_NEAR(updated->start().y, updated->end().y, 1e-6);
}

TEST(SketchSolver, DistanceConstraint) {
    draft::DraftDocument doc;
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{8, 0});
    doc.addEntity(line);

    cstr::ConstraintSystem sys;
    cstr::GeometryRef refA{line->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{line->id(), cstr::FeatureType::Point, 1};

    // Fix start, constrain distance to 10.
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(refA, math::Vec2{0.0, 0.0}));
    sys.addConstraint(std::make_shared<cstr::DistanceConstraint>(refA, refB, 10.0));

    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);

    cstr::SketchSolver solver;
    auto result = solver.solve(params, sys);

    EXPECT_TRUE(result.status == cstr::SolveStatus::Success ||
                result.status == cstr::SolveStatus::UnderConstrained);

    params.applyToEntities(doc.entities());

    auto* updated = dynamic_cast<draft::DraftLine*>(doc.entities().front().get());
    ASSERT_NE(updated, nullptr);

    double dx = updated->end().x - updated->start().x;
    double dy = updated->end().y - updated->start().y;
    double dist = std::sqrt(dx * dx + dy * dy);
    EXPECT_NEAR(dist, 10.0, 1e-6);
}

TEST(SketchSolver, FixedConstraint) {
    draft::DraftDocument doc;
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0.1, 0.2}, math::Vec2{10, 0});
    doc.addEntity(line);

    cstr::ConstraintSystem sys;
    cstr::GeometryRef ref{line->id(), cstr::FeatureType::Point, 0};
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(ref, math::Vec2{0.0, 0.0}));

    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);

    cstr::SketchSolver solver;
    auto result = solver.solve(params, sys);

    EXPECT_TRUE(result.status == cstr::SolveStatus::Success ||
                result.status == cstr::SolveStatus::UnderConstrained);

    params.applyToEntities(doc.entities());

    auto* updated = dynamic_cast<draft::DraftLine*>(doc.entities().front().get());
    ASSERT_NE(updated, nullptr);
    EXPECT_NEAR(updated->start().x, 0.0, 1e-6);
    EXPECT_NEAR(updated->start().y, 0.0, 1e-6);
}

TEST(SketchSolver, OverConstrainedDetection) {
    draft::DraftDocument doc;
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    doc.addEntity(line);

    cstr::ConstraintSystem sys;
    cstr::GeometryRef refA{line->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{line->id(), cstr::FeatureType::Point, 1};

    // Fix both endpoints AND add a distance constraint — over-constrained if distance doesn't match.
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(refA, math::Vec2{0.0, 0.0}));
    sys.addConstraint(std::make_shared<cstr::FixedConstraint>(refB, math::Vec2{10.0, 0.0}));
    sys.addConstraint(std::make_shared<cstr::DistanceConstraint>(refA, refB, 5.0));  // Contradicts fixed.

    auto params = cstr::ParameterTable::buildFromEntities(doc.entities(), sys);

    cstr::SketchSolver solver;
    auto result = solver.solve(params, sys);

    // Should detect inconsistency or over-constraint.
    EXPECT_TRUE(result.status == cstr::SolveStatus::OverConstrained ||
                result.status == cstr::SolveStatus::Inconsistent ||
                result.status == cstr::SolveStatus::FailedToConverge);
}
