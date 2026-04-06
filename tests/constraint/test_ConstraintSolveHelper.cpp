#define _USE_MATH_DEFINES
#include <cmath>
#include <gtest/gtest.h>

#include "horizon/document/ConstraintSolveHelper.h"
#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/GeometryRef.h"
#include "horizon/document/Document.h"
#include "horizon/document/ParameterRegistry.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/math/Vec2.h"

using namespace hz;

// ---------------------------------------------------------------------------
// Test 1: Coincident constraint moves line2.start to line1.end
// ---------------------------------------------------------------------------
TEST(ConstraintSolveHelper, SolveMovesEntitiesToSatisfyCoincident) {
    draft::DraftDocument doc;
    auto line1 = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    auto line2 = std::make_shared<draft::DraftLine>(math::Vec2{10.5, 0.3}, math::Vec2{20, 0});
    doc.addEntity(line1);
    doc.addEntity(line2);

    cstr::ConstraintSystem sys;

    // Fix line1 completely and line2 end.
    cstr::GeometryRef fixL1Start{line1->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef fixL1End{line1->id(), cstr::FeatureType::Point, 1};
    cstr::GeometryRef fixL2End{line2->id(), cstr::FeatureType::Point, 1};
    sys.addConstraint(
        std::make_shared<cstr::FixedConstraint>(fixL1Start, math::Vec2{0.0, 0.0}));
    sys.addConstraint(
        std::make_shared<cstr::FixedConstraint>(fixL1End, math::Vec2{10.0, 0.0}));
    sys.addConstraint(
        std::make_shared<cstr::FixedConstraint>(fixL2End, math::Vec2{20.0, 0.0}));

    // Coincident: line1.end == line2.start
    cstr::GeometryRef refA{line1->id(), cstr::FeatureType::Point, 1};
    cstr::GeometryRef refB{line2->id(), cstr::FeatureType::Point, 0};
    sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(refA, refB));

    auto result = doc::ConstraintSolveHelper::solveAndApply(doc, sys);

    EXPECT_TRUE(result.success);

    // line2's start should now be at (10, 0).
    auto* updatedLine2 = dynamic_cast<draft::DraftLine*>(doc.entities()[1].get());
    ASSERT_NE(updatedLine2, nullptr);
    EXPECT_NEAR(updatedLine2->start().x, 10.0, 1e-6);
    EXPECT_NEAR(updatedLine2->start().y, 0.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Test 2: Snapshots are produced for undo/redo
// ---------------------------------------------------------------------------
TEST(ConstraintSolveHelper, SolveReturnsSnapshotsForUndoCommand) {
    draft::DraftDocument doc;
    auto line1 = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    auto line2 = std::make_shared<draft::DraftLine>(math::Vec2{10.5, 0.3}, math::Vec2{20, 0});
    doc.addEntity(line1);
    doc.addEntity(line2);

    cstr::ConstraintSystem sys;

    cstr::GeometryRef fixL1Start{line1->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef fixL1End{line1->id(), cstr::FeatureType::Point, 1};
    cstr::GeometryRef fixL2End{line2->id(), cstr::FeatureType::Point, 1};
    sys.addConstraint(
        std::make_shared<cstr::FixedConstraint>(fixL1Start, math::Vec2{0.0, 0.0}));
    sys.addConstraint(
        std::make_shared<cstr::FixedConstraint>(fixL1End, math::Vec2{10.0, 0.0}));
    sys.addConstraint(
        std::make_shared<cstr::FixedConstraint>(fixL2End, math::Vec2{20.0, 0.0}));

    cstr::GeometryRef refA{line1->id(), cstr::FeatureType::Point, 1};
    cstr::GeometryRef refB{line2->id(), cstr::FeatureType::Point, 0};
    sys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(refA, refB));

    auto result = doc::ConstraintSolveHelper::solveAndApply(doc, sys);

    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.snapshots.empty());

    // Each snapshot should have before and after states.
    for (const auto& snap : result.snapshots) {
        EXPECT_NE(snap.beforeState, nullptr);
        EXPECT_NE(snap.afterState, nullptr);
        EXPECT_NE(snap.entityId, 0u);
    }

    // Verify the convenience method also works.
    // Reset line2 start to the original offset to create a fresh scenario.
    auto* l2 = dynamic_cast<draft::DraftLine*>(doc.entities()[1].get());
    ASSERT_NE(l2, nullptr);
    l2->setStart(math::Vec2{10.5, 0.3});

    auto cmd = doc::ConstraintSolveHelper::solveAndCreateCommand(doc, sys);
    EXPECT_NE(cmd, nullptr);
}

// ---------------------------------------------------------------------------
// Test 3: Distance constraint reshapes line on value edit
// ---------------------------------------------------------------------------
TEST(ConstraintSolveHelper, DistanceConstraintReshapesOnValueEdit) {
    draft::DraftDocument doc;
    // A single horizontal line from (0,0) to (10,0).
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 0});
    doc.addEntity(line);

    cstr::ConstraintSystem sys;

    cstr::GeometryRef ptStart{line->id(), cstr::FeatureType::Point, 0};
    cstr::GeometryRef ptEnd{line->id(), cstr::FeatureType::Point, 1};

    // Fix start at origin.
    sys.addConstraint(
        std::make_shared<cstr::FixedConstraint>(ptStart, math::Vec2{0.0, 0.0}));

    // Horizontal constraint.
    sys.addConstraint(std::make_shared<cstr::HorizontalConstraint>(ptStart, ptEnd));

    // Distance = 10.
    auto distConstraint =
        std::make_shared<cstr::DistanceConstraint>(ptStart, ptEnd, 10.0);
    sys.addConstraint(distConstraint);

    // Solve with distance = 10.
    auto result = doc::ConstraintSolveHelper::solveAndApply(doc, sys);
    EXPECT_TRUE(result.success);

    auto* updated = dynamic_cast<draft::DraftLine*>(doc.entities().front().get());
    ASSERT_NE(updated, nullptr);
    EXPECT_NEAR(updated->start().y, updated->end().y, 1e-6);  // horizontal
    double dx = updated->end().x - updated->start().x;
    double dy = updated->end().y - updated->start().y;
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 10.0, 1e-6);

    // Change distance to 20 and re-solve.
    distConstraint->setDimensionalValue(20.0);
    auto result2 = doc::ConstraintSolveHelper::solveAndApply(doc, sys);
    EXPECT_TRUE(result2.success);

    updated = dynamic_cast<draft::DraftLine*>(doc.entities().front().get());
    ASSERT_NE(updated, nullptr);
    EXPECT_NEAR(updated->start().y, updated->end().y, 1e-6);  // still horizontal
    dx = updated->end().x - updated->start().x;
    dy = updated->end().y - updated->start().y;
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), 20.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Test 4: No constraints is a no-op
// ---------------------------------------------------------------------------
TEST(ConstraintSolveHelper, SolveWithNoConstraintsIsNoOp) {
    draft::DraftDocument doc;
    auto line = std::make_shared<draft::DraftLine>(math::Vec2{0, 0}, math::Vec2{10, 5});
    doc.addEntity(line);

    cstr::ConstraintSystem sys;  // empty

    auto result = doc::ConstraintSolveHelper::solveAndApply(doc, sys);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.snapshots.empty());

    // The line should be unchanged.
    auto* updated = dynamic_cast<draft::DraftLine*>(doc.entities().front().get());
    ASSERT_NE(updated, nullptr);
    EXPECT_NEAR(updated->start().x, 0.0, 1e-9);
    EXPECT_NEAR(updated->start().y, 0.0, 1e-9);
    EXPECT_NEAR(updated->end().x, 10.0, 1e-9);
    EXPECT_NEAR(updated->end().y, 5.0, 1e-9);

    // Convenience method should return nullptr.
    auto cmd = doc::ConstraintSolveHelper::solveAndCreateCommand(doc, sys);
    EXPECT_EQ(cmd, nullptr);
}

// ---------------------------------------------------------------------------
// Test 5: Distance constraint references a variable from ParameterRegistry
// ---------------------------------------------------------------------------
TEST(ConstraintSolveHelper, DistanceConstraintReferencesVariable) {
    doc::Document document;
    auto& draftDoc = document.draftDocument();
    auto& csys = document.constraintSystem();
    auto& params = document.parameterRegistry();

    params.set("gap", 15.0);

    auto line1 = std::make_shared<draft::DraftLine>(math::Vec2(0, 0), math::Vec2(10, 0));
    auto line2 = std::make_shared<draft::DraftLine>(math::Vec2(10.5, 0.3), math::Vec2(20, 0));
    draftDoc.addEntity(line1);
    draftDoc.addEntity(line2);

    cstr::GeometryRef fixRef{line1->id(), cstr::FeatureType::Point, 0};
    csys.addConstraint(std::make_shared<cstr::FixedConstraint>(fixRef, math::Vec2(0, 0)));

    cstr::GeometryRef ref1{line1->id(), cstr::FeatureType::Point, 1};
    cstr::GeometryRef ref2{line2->id(), cstr::FeatureType::Point, 0};

    auto distConstraint = std::make_shared<cstr::DistanceConstraint>(ref1, ref2, 0.0);
    distConstraint->setVariableReference("gap");
    csys.addConstraint(distConstraint);

    // Create resolver from ParameterRegistry.
    auto resolver = [&params](const std::string& name) { return params.get(name); };

    auto result = doc::ConstraintSolveHelper::solveAndApply(draftDoc, csys, resolver);
    EXPECT_TRUE(result.success);

    double actualDist = line1->end().distanceTo(line2->start());
    EXPECT_NEAR(actualDist, 15.0, 1e-4);

    // Change variable and re-solve.
    params.set("gap", 5.0);
    auto result2 = doc::ConstraintSolveHelper::solveAndApply(draftDoc, csys, resolver);
    EXPECT_TRUE(result2.success);

    double newDist = line1->end().distanceTo(line2->start());
    EXPECT_NEAR(newDist, 5.0, 1e-4);
}

// ---------------------------------------------------------------------------
// Test 6: clone() preserves variable reference
// ---------------------------------------------------------------------------
TEST(ConstraintSolveHelper, ClonePreservesVariableReference) {
    cstr::GeometryRef refA{1, cstr::FeatureType::Point, 0};
    cstr::GeometryRef refB{2, cstr::FeatureType::Point, 0};

    auto dist = std::make_shared<cstr::DistanceConstraint>(refA, refB, 42.0);
    dist->setVariableReference("myVar");

    auto cloned = dist->clone();
    EXPECT_TRUE(cloned->hasVariableReference());
    EXPECT_EQ(cloned->variableReference(), "myVar");
    EXPECT_NEAR(cloned->dimensionalValue(), 42.0, 1e-9);

    auto angle = std::make_shared<cstr::AngleConstraint>(refA, refB, 1.57);
    angle->setVariableReference("theta");

    auto clonedAngle = angle->clone();
    EXPECT_TRUE(clonedAngle->hasVariableReference());
    EXPECT_EQ(clonedAngle->variableReference(), "theta");
    EXPECT_NEAR(clonedAngle->dimensionalValue(), 1.57, 1e-9);
}
