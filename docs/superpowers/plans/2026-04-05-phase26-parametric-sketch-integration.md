# Phase 26: Parametric Sketch Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire the constraint solver into the editing workflow so constraints are functional — entities auto-solve when moved, DOF is visualized, and constraint dimensions are editable.

**Architecture:** The existing ConstraintSystem, SketchSolver, ParameterTable, and ApplyConstraintSolveCommand are already functional. This phase adds: (1) a `ConstraintSolveHelper` utility that encapsulates the snapshot→solve→apply→command pattern, (2) post-move auto-solve in entity-mutating commands, (3) DOF visualization via entity color overrides in the viewport, (4) constraint annotation rendering via QPainter overlay, (5) double-click editing of dimensional constraints, and (6) a ParameterRegistry for design variables.

**Tech Stack:** C++20, Qt6, Eigen, Google Test

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 3.2 (Phase 26)

---

## File Map

| Action | File | Responsibility |
|--------|------|---------------|
| Create | `src/document/include/horizon/document/ConstraintSolveHelper.h` | Encapsulates snapshot→solve→apply→command pattern |
| Create | `src/document/src/ConstraintSolveHelper.cpp` | Implementation |
| Modify | `src/document/CMakeLists.txt` | Add ConstraintSolveHelper.cpp |
| Modify | `src/document/src/Commands.cpp` | Post-move/rotate/scale auto-solve via helper |
| Create | `src/document/include/horizon/document/ParameterRegistry.h` | Design variables (name→double map) |
| Create | `src/document/src/ParameterRegistry.cpp` | Implementation |
| Modify | `src/document/include/horizon/document/Document.h` | Add ParameterRegistry member |
| Modify | `src/document/src/Document.cpp` | Wire ParameterRegistry |
| Modify | `src/ui/src/ViewportWidget.cpp` | DOF color override + constraint annotation overlay |
| Modify | `src/ui/src/SelectTool.cpp` | Double-click to edit constraint dimension |
| Create | `tests/constraint/test_ConstraintSolveHelper.cpp` | Helper integration tests |
| Modify | `tests/constraint/CMakeLists.txt` | Add new test file |
| Create | `tests/document/test_ParameterRegistry.cpp` | ParameterRegistry tests |
| Create | `tests/document/CMakeLists.txt` | Document test target |
| Modify | `tests/CMakeLists.txt` | Add document subdirectory |
| Modify | `src/fileio/src/NativeFormat.cpp` | Serialize/deserialize ParameterRegistry |

---

## Task 1: ConstraintSolveHelper — Encapsulate the Solve Pattern

The current `ConstraintTool::commitConstraint()` contains 90+ lines of snapshot→solve→restore→push logic that must be reusable for post-move auto-solve. Extract it into a utility.

**Files:**
- Create: `src/document/include/horizon/document/ConstraintSolveHelper.h`
- Create: `src/document/src/ConstraintSolveHelper.cpp`
- Modify: `src/document/CMakeLists.txt`
- Create: `tests/constraint/test_ConstraintSolveHelper.cpp`

- [ ] **Step 1: Write failing test for ConstraintSolveHelper**

Create `tests/constraint/test_ConstraintSolveHelper.cpp`:
```cpp
#include <gtest/gtest.h>
#include "horizon/document/ConstraintSolveHelper.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/GeometryRef.h"

using namespace hz;

TEST(ConstraintSolveHelperTest, SolveMovesEntitiesToSatisfyCoincident) {
    doc::Document document;
    auto& draftDoc = document.draftDocument();
    auto& csys = document.constraintSystem();

    // Two lines with endpoints that don't quite meet.
    auto line1 = std::make_shared<draft::DraftLine>(math::Vec2(0, 0), math::Vec2(10, 0));
    auto line2 = std::make_shared<draft::DraftLine>(math::Vec2(10.5, 0.3), math::Vec2(20, 0));
    draftDoc.addEntity(line1);
    draftDoc.addEntity(line2);

    // Fix line1 start so solver doesn't move everything.
    cstr::GeometryRef fixRef{line1->id(), cstr::FeatureType::Point, 0};
    csys.addConstraint(std::make_shared<cstr::FixedConstraint>(fixRef, math::Vec2(0, 0)));

    // Coincident: line1 end == line2 start.
    cstr::GeometryRef ref1{line1->id(), cstr::FeatureType::Point, 1};
    cstr::GeometryRef ref2{line2->id(), cstr::FeatureType::Point, 0};
    csys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(ref1, ref2));

    // Solve and apply.
    auto result = doc::ConstraintSolveHelper::solveAndApply(draftDoc, csys);
    EXPECT_TRUE(result.success);

    // line2 start should now be at (10, 0) — coincident with line1 end.
    EXPECT_NEAR(line2->start().x, 10.0, 1e-6);
    EXPECT_NEAR(line2->start().y, 0.0, 1e-6);
}

TEST(ConstraintSolveHelperTest, SolveReturnsSnapshotsForUndoCommand) {
    doc::Document document;
    auto& draftDoc = document.draftDocument();
    auto& csys = document.constraintSystem();

    auto line1 = std::make_shared<draft::DraftLine>(math::Vec2(0, 0), math::Vec2(10, 0));
    auto line2 = std::make_shared<draft::DraftLine>(math::Vec2(10.5, 0.3), math::Vec2(20, 0));
    draftDoc.addEntity(line1);
    draftDoc.addEntity(line2);

    cstr::GeometryRef fixRef{line1->id(), cstr::FeatureType::Point, 0};
    csys.addConstraint(std::make_shared<cstr::FixedConstraint>(fixRef, math::Vec2(0, 0)));

    cstr::GeometryRef ref1{line1->id(), cstr::FeatureType::Point, 1};
    cstr::GeometryRef ref2{line2->id(), cstr::FeatureType::Point, 0};
    csys.addConstraint(std::make_shared<cstr::CoincidentConstraint>(ref1, ref2));

    auto result = doc::ConstraintSolveHelper::solveAndApply(draftDoc, csys);
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.snapshots.empty());

    // Snapshots have before and after states.
    for (const auto& snap : result.snapshots) {
        EXPECT_NE(snap.beforeState, nullptr);
        EXPECT_NE(snap.afterState, nullptr);
    }
}

TEST(ConstraintSolveHelperTest, SolveWithNoConstraintsIsNoOp) {
    doc::Document document;
    auto& draftDoc = document.draftDocument();
    auto& csys = document.constraintSystem();

    auto line = std::make_shared<draft::DraftLine>(math::Vec2(0, 0), math::Vec2(10, 0));
    draftDoc.addEntity(line);

    auto result = doc::ConstraintSolveHelper::solveAndApply(draftDoc, csys);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.snapshots.empty());
}
```

- [ ] **Step 2: Add test to constraint CMakeLists**

Modify `tests/constraint/CMakeLists.txt` — add `test_ConstraintSolveHelper.cpp` to the test executable. Also add `Horizon::Document` to the link libraries since ConstraintSolveHelper lives in the document module.

- [ ] **Step 3: Run tests to verify they fail**

Run: `cmake --build build/debug --config Debug --target hz_constraint_tests`
Expected: Compilation fails — `ConstraintSolveHelper.h` not found.

- [ ] **Step 4: Implement ConstraintSolveHelper header**

Create `src/document/include/horizon/document/ConstraintSolveHelper.h`:
```cpp
#pragma once

#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/SketchSolver.h"
#include "horizon/document/ConstraintCommands.h"
#include "horizon/drafting/DraftDocument.h"
#include <memory>
#include <vector>

namespace hz::doc {

/// Encapsulates the constraint-solve workflow:
/// snapshot before-states → build parameter table → solve → apply → snapshot after-states.
/// Returns snapshots suitable for creating an ApplyConstraintSolveCommand.
class ConstraintSolveHelper {
public:
    struct SolveAndApplyResult {
        bool success = false;
        cstr::SolveResult solveResult;
        std::vector<ApplyConstraintSolveCommand::EntitySnapshot> snapshots;
    };

    /// Solve all constraints in the system against current entity positions.
    /// On success, entity positions are ALREADY updated in draftDoc.
    /// Caller should create an ApplyConstraintSolveCommand from the returned snapshots
    /// if undo support is needed.
    static SolveAndApplyResult solveAndApply(
        draft::DraftDocument& draftDoc,
        const cstr::ConstraintSystem& csys);

    /// Convenience: solve + create an ApplyConstraintSolveCommand (or nullptr if nothing changed).
    static std::unique_ptr<ApplyConstraintSolveCommand> solveAndCreateCommand(
        draft::DraftDocument& draftDoc,
        const cstr::ConstraintSystem& csys);
};

}  // namespace hz::doc
```

- [ ] **Step 5: Implement ConstraintSolveHelper**

Create `src/document/src/ConstraintSolveHelper.cpp`:
```cpp
#include "horizon/document/ConstraintSolveHelper.h"
#include "horizon/constraint/ParameterTable.h"

namespace hz::doc {

ConstraintSolveHelper::SolveAndApplyResult ConstraintSolveHelper::solveAndApply(
    draft::DraftDocument& draftDoc,
    const cstr::ConstraintSystem& csys) {

    SolveAndApplyResult result;

    if (csys.empty()) {
        result.success = true;
        return result;
    }

    auto& entities = draftDoc.entities();
    if (entities.empty()) {
        result.success = true;
        return result;
    }

    // Build parameter table from entities referenced by constraints.
    auto paramTable = cstr::ParameterTable::buildFromEntities(entities, csys);
    if (paramTable.parameterCount() == 0) {
        result.success = true;
        return result;
    }

    // Snapshot before-states for all entities in the parameter table.
    std::vector<ApplyConstraintSolveCommand::EntitySnapshot> snapshots;
    for (const auto& entity : entities) {
        if (!paramTable.hasEntity(entity->id())) continue;
        ApplyConstraintSolveCommand::EntitySnapshot snap;
        snap.entityId = entity->id();
        snap.beforeState = entity->clone();
        snapshots.push_back(std::move(snap));
    }

    // Solve.
    cstr::SketchSolver solver;
    result.solveResult = solver.solve(paramTable, csys);

    if (result.solveResult.status == cstr::SolveStatus::Success ||
        result.solveResult.status == cstr::SolveStatus::Converged ||
        result.solveResult.status == cstr::SolveStatus::UnderConstrained) {

        // Apply solved parameters to entities.
        paramTable.applyToEntities(entities);

        // Snapshot after-states.
        for (auto& snap : snapshots) {
            for (const auto& entity : entities) {
                if (entity->id() == snap.entityId) {
                    snap.afterState = entity->clone();
                    break;
                }
            }
        }

        // Filter out snapshots where nothing changed (before == after).
        std::vector<ApplyConstraintSolveCommand::EntitySnapshot> changed;
        for (auto& snap : snapshots) {
            if (snap.afterState) {
                changed.push_back(std::move(snap));
            }
        }

        result.snapshots = std::move(changed);
        result.success = true;
    } else {
        // Solve failed — restore before-states.
        for (const auto& snap : snapshots) {
            if (!snap.beforeState) continue;
            for (auto& entity : entities) {
                if (entity->id() == snap.entityId) {
                    // Use clone of beforeState to restore.
                    auto restored = snap.beforeState->clone();
                    // Copy geometry only (not ID/layer/color).
                    // We reuse the entity pointer, just overwrite geometry.
                    // For simplicity, swap the shared_ptr.
                    entity = snap.beforeState->clone();
                    entity->setId(snap.entityId);
                    break;
                }
            }
        }
        result.success = false;
    }

    return result;
}

std::unique_ptr<ApplyConstraintSolveCommand> ConstraintSolveHelper::solveAndCreateCommand(
    draft::DraftDocument& draftDoc,
    const cstr::ConstraintSystem& csys) {

    auto result = solveAndApply(draftDoc, csys);
    if (!result.success || result.snapshots.empty()) {
        return nullptr;
    }
    return std::make_unique<ApplyConstraintSolveCommand>(draftDoc, std::move(result.snapshots));
}

}  // namespace hz::doc
```

- [ ] **Step 6: Add to document CMakeLists**

Add `src/ConstraintSolveHelper.cpp` to `src/document/CMakeLists.txt` source list.

- [ ] **Step 7: Build and run tests**

Run: `cmake --build build/debug --config Debug && ctest --test-dir build/debug -C Debug --output-on-failure`
Expected: All tests pass including the 3 new ConstraintSolveHelper tests.

- [ ] **Step 8: Commit**

```bash
git add src/document/include/horizon/document/ConstraintSolveHelper.h \
        src/document/src/ConstraintSolveHelper.cpp \
        src/document/CMakeLists.txt \
        tests/constraint/test_ConstraintSolveHelper.cpp \
        tests/constraint/CMakeLists.txt
git commit -m "feat(document): add ConstraintSolveHelper for reusable solve workflow

Encapsulates snapshot→solve→apply→command pattern. Returns snapshots
suitable for ApplyConstraintSolveCommand. Used by constraint tool and
upcoming post-move auto-solve."
```

---

## Task 2: Refactor ConstraintTool to Use ConstraintSolveHelper

The current `commitConstraint()` in ConstraintTool has 90+ lines of manual snapshot/solve/restore logic. Replace it with `ConstraintSolveHelper`.

**Files:**
- Modify: `src/ui/src/ConstraintTool.cpp`

- [ ] **Step 1: Read current ConstraintTool::commitConstraint()**

Read `src/ui/src/ConstraintTool.cpp` lines 290-440 to understand the full commit flow.

- [ ] **Step 2: Refactor commitConstraint to use ConstraintSolveHelper**

Replace the manual snapshot/solve/restore block with:
```cpp
void ConstraintTool::commitConstraint() {
    // ... (existing feature validation and constraint creation code stays) ...

    auto& doc = *m_viewport->document();
    auto& draftDoc = doc.draftDocument();
    auto& csys = doc.constraintSystem();

    auto composite = std::make_unique<doc::CompositeCommand>(
        "Add " + constraint->typeName() + " Constraint");

    // Add the constraint command.
    composite->addCommand(
        std::make_unique<doc::AddConstraintCommand>(csys, constraint));

    // Temporarily add constraint for the solver to use.
    csys.addConstraint(constraint);

    // Use helper to solve and create apply command.
    auto solveCmd = doc::ConstraintSolveHelper::solveAndCreateCommand(draftDoc, csys);
    if (solveCmd) {
        // Undo the solve (command will re-apply on push).
        solveCmd->undo();
        composite->addCommand(std::move(solveCmd));
    }

    // Remove the temporary constraint (push will re-add via AddConstraintCommand).
    csys.removeConstraint(constraint->id());

    doc.undoStack().push(std::move(composite));
    doc.setDirty(true);
}
```

The key insight: we temporarily add the constraint so the solver can see it, solve, create the undo command, then undo the solve and remove the constraint. When `push()` calls `execute()`, AddConstraintCommand re-adds it and ApplyConstraintSolveCommand re-applies the solve.

- [ ] **Step 3: Build and run all tests**

Run: `cmake --build build/debug --config Debug && ctest --test-dir build/debug -C Debug --output-on-failure`
Expected: All tests pass. No regressions.

- [ ] **Step 4: Commit**

```bash
git add src/ui/src/ConstraintTool.cpp
git commit -m "refactor(ui): simplify ConstraintTool using ConstraintSolveHelper

Replaces 90+ lines of manual snapshot/solve/restore logic with
a clean ConstraintSolveHelper call. Same behavior, much less code."
```

---

## Task 3: Post-Move Auto-Solve

When entities with active constraints are moved (via MoveTool, GripMove, etc.), constraints should automatically re-solve.

**Files:**
- Modify: `src/document/src/Commands.cpp`
- Modify: `src/document/include/horizon/document/Commands.h` (if needed for Document& reference)

- [ ] **Step 1: Identify entity-mutating commands that need post-solve**

Read `src/document/src/Commands.cpp` and `src/document/include/horizon/document/Commands.h`. The commands that modify entity geometry in-place and should trigger a post-solve are:
- `MoveEntityCommand`
- `GripMoveCommand`

NOTE: `ApplyConstraintSolveCommand` should NOT trigger a re-solve (it IS the solve result). Rotation/scale/mirror commands are handled by add/remove patterns already.

- [ ] **Step 2: Add constraint system reference to MoveEntityCommand**

The MoveEntityCommand needs access to the ConstraintSystem to trigger a post-move solve. Add a `Document&` or `cstr::ConstraintSystem&` reference to its constructor.

Read the current MoveEntityCommand constructor to understand what references it already holds, then add the minimum needed for post-solve.

- [ ] **Step 3: Implement post-move auto-solve**

In `MoveEntityCommand::execute()`, after moving the entities:
```cpp
void MoveEntityCommand::execute() {
    // ... existing move logic ...

    // Post-move: re-solve constraints if any affected entities are constrained.
    if (!m_constraintSystem.empty()) {
        auto solveCmd = doc::ConstraintSolveHelper::solveAndCreateCommand(m_doc, m_constraintSystem);
        // Store the solve command for undo.
        m_solveCmd = std::move(solveCmd);
        m_doc.rebuildSpatialIndex();
    }
}

void MoveEntityCommand::undo() {
    // Undo the solve first (if it happened).
    if (m_solveCmd) {
        m_solveCmd->undo();
        m_solveCmd.reset();
    }
    // ... existing undo logic (reverse move) ...
}
```

Apply the same pattern to `GripMoveCommand`.

- [ ] **Step 4: Update callers to pass ConstraintSystem to MoveEntityCommand**

Search for all places that create `MoveEntityCommand` (MoveTool, DuplicateTool, etc.) and update the constructor call to pass the constraint system reference.

- [ ] **Step 5: Build and run all tests**

Run: `cmake --build build/debug --config Debug && ctest --test-dir build/debug -C Debug --output-on-failure`
Expected: All tests pass.

- [ ] **Step 6: Manually test post-move solve**

1. Launch horizon.exe
2. Draw two lines with endpoints near each other
3. Add a Coincident constraint between the endpoints
4. Move one of the lines — the constrained endpoint should stay coincident
5. Undo — both the move and the solve should revert

- [ ] **Step 7: Commit**

```bash
git add src/document/src/Commands.cpp src/document/include/horizon/document/Commands.h \
        src/ui/src/MoveTool.cpp src/ui/src/SelectTool.cpp
git commit -m "feat(document): auto-solve constraints after entity move/grip-edit

MoveEntityCommand and GripMoveCommand now trigger ConstraintSolveHelper
after modifying entity geometry. Undo reverts both move and solve."
```

---

## Task 4: DOF Visualization

Color entities by their constraint status: green = free DOF, black/white = fully constrained, red = over-constrained.

**Files:**
- Modify: `src/ui/src/ViewportWidget.cpp`
- Modify: `src/constraint/include/horizon/constraint/SketchSolver.h` (add DOF-per-entity query)
- Modify: `src/constraint/src/SketchSolver.cpp`

- [ ] **Step 1: Add DOF analysis to SketchSolver**

Add a method to compute per-entity DOF status:
```cpp
enum class EntityDOFStatus { Free, FullyConstrained, OverConstrained };

struct DOFAnalysis {
    std::unordered_map<uint64_t, EntityDOFStatus> entityStatus;
    int totalDOF = 0;
};

DOFAnalysis analyzeDOF(const ParameterTable& params, const ConstraintSystem& constraints) const;
```

The analysis works by:
1. Building the Jacobian matrix
2. Computing its rank via SVD or QR factorization
3. If rank == total equations: fully constrained if rank == params, under-constrained if rank < params
4. For per-entity status: check which parameter columns have nonzero Jacobian entries

- [ ] **Step 2: Implement DOF analysis**

In `SketchSolver.cpp`:
```cpp
DOFAnalysis SketchSolver::analyzeDOF(const ParameterTable& params,
                                      const ConstraintSystem& constraints) const {
    DOFAnalysis result;
    if (constraints.empty()) {
        // All entities are free.
        return result;
    }

    int nParams = params.parameterCount();
    int nEquations = constraints.totalEquations();

    Eigen::MatrixXd J = buildJacobian(params, constraints);
    
    // Compute rank via SVD.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    double threshold = 1e-8 * std::max(J.rows(), J.cols()) * svd.singularValues()(0);
    int rank = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > threshold) ++rank;
    }

    result.totalDOF = nParams - rank;

    // Per-entity: check if all parameters for an entity are "covered" by constraints.
    // Simple heuristic: if an entity's parameter columns all have nonzero Jacobian entries,
    // it's participating in constraints. If rank >= nParams, it's fully constrained.
    // If nEquations > rank, there are redundant constraints (over-constrained).
    bool overConstrained = nEquations > rank;

    // For simplicity in this phase: mark all constrained entities based on global status.
    // Entity is "constrained" if it appears in any constraint.
    for (const auto& c : constraints.constraints()) {
        for (uint64_t eid : c->referencedEntityIds()) {
            if (overConstrained) {
                result.entityStatus[eid] = EntityDOFStatus::OverConstrained;
            } else if (result.totalDOF == 0) {
                result.entityStatus[eid] = EntityDOFStatus::FullyConstrained;
            } else {
                result.entityStatus[eid] = EntityDOFStatus::Free;
            }
        }
    }

    return result;
}
```

- [ ] **Step 3: Add DOF color override to ViewportWidget rendering**

In `ViewportWidget::paintGL()`, after computing entity colors but before rendering, override the color for constrained entities based on DOF analysis:

```cpp
// Compute DOF status (cached, re-computed when constraints change).
if (m_dofDirty) {
    auto& csys = document()->constraintSystem();
    if (!csys.empty()) {
        auto paramTable = cstr::ParameterTable::buildFromEntities(
            document()->draftDocument().entities(), csys);
        cstr::SketchSolver solver;
        m_dofAnalysis = solver.analyzeDOF(paramTable, csys);
    } else {
        m_dofAnalysis = {};
    }
    m_dofDirty = false;
}

// In the entity rendering loop, override color:
auto dofIt = m_dofAnalysis.entityStatus.find(entity->id());
if (dofIt != m_dofAnalysis.entityStatus.end()) {
    switch (dofIt->second) {
        case cstr::EntityDOFStatus::Free:
            entityColor = math::Vec3(0.0, 0.8, 0.0);  // Green
            break;
        case cstr::EntityDOFStatus::FullyConstrained:
            // Use normal entity color (black/white depending on theme).
            break;
        case cstr::EntityDOFStatus::OverConstrained:
            entityColor = math::Vec3(1.0, 0.0, 0.0);  // Red
            break;
    }
}
```

Add `m_dofDirty = true` flag that's set whenever constraints change (in the constraint tool, on undo/redo, on file load).

- [ ] **Step 4: Add DOFAnalysis member to ViewportWidget**

Add to ViewportWidget.h:
```cpp
cstr::DOFAnalysis m_dofAnalysis;
bool m_dofDirty = true;
```

Mark dirty on: constraint add/remove, entity move (post-solve), undo/redo, file load.

- [ ] **Step 5: Build and manually test DOF visualization**

1. Draw two lines
2. No constraints → both render in normal color
3. Add coincident constraint → constrained entities turn green (under-constrained, DOF > 0)
4. Add enough constraints to fully constrain → entities render in normal color
5. Add one more constraint → entities turn red (over-constrained)

- [ ] **Step 6: Commit**

```bash
git add src/constraint/include/horizon/constraint/SketchSolver.h \
        src/constraint/src/SketchSolver.cpp \
        src/ui/src/ViewportWidget.cpp \
        src/ui/include/horizon/ui/ViewportWidget.h
git commit -m "feat(ui): add DOF visualization for constrained entities

Green = under-constrained, normal = fully constrained, red = over-constrained.
DOF analysis computed via Jacobian SVD rank. Cached and re-computed on change."
```

---

## Task 5: Constraint Annotation Rendering

Render small visual indicators for active constraints on the canvas (coincident dots, perpendicularity squares, parallel arrows, dimensional values).

**Files:**
- Modify: `src/ui/src/ViewportWidget.cpp`

- [ ] **Step 1: Add constraint icon rendering to QPainter overlay**

In `ViewportWidget::paintGL()`, after the entity rendering and text overlay, add a constraint annotation pass. For each constraint in the system, render a small icon at the constraint's reference point:

```cpp
void ViewportWidget::renderConstraintAnnotations(QPainter& painter) {
    if (!document()) return;
    const auto& csys = document()->constraintSystem();
    const auto& entities = document()->draftDocument().entities();

    for (const auto& c : csys.constraints()) {
        auto refIds = c->referencedEntityIds();
        if (refIds.empty()) continue;

        // Find the midpoint of the constraint for annotation placement.
        // Use the first referenced entity's relevant feature point.
        math::Vec2 annotationPos = getConstraintAnnotationPos(*c, entities);
        QPointF screenPos = worldToScreen(annotationPos);

        painter.setPen(QPen(QColor(255, 200, 0), 1));  // Yellow annotation color
        painter.setFont(QFont("Arial", 9));

        switch (c->type()) {
            case cstr::ConstraintType::Coincident:
                // Small filled circle
                painter.setBrush(QColor(255, 200, 0));
                painter.drawEllipse(screenPos, 4, 4);
                break;
            case cstr::ConstraintType::Horizontal:
                painter.drawText(screenPos.x() + 6, screenPos.y() - 2, "H");
                break;
            case cstr::ConstraintType::Vertical:
                painter.drawText(screenPos.x() + 6, screenPos.y() - 2, "V");
                break;
            case cstr::ConstraintType::Perpendicular:
                painter.drawText(screenPos.x() + 6, screenPos.y() - 2, "\xE2\x9F\x82");  // ⟂
                break;
            case cstr::ConstraintType::Parallel:
                painter.drawText(screenPos.x() + 6, screenPos.y() - 2, "//");
                break;
            case cstr::ConstraintType::Distance:
                if (c->hasDimensionalValue()) {
                    QString text = QString::number(c->dimensionalValue(), 'f', 2);
                    painter.drawText(screenPos.x() + 6, screenPos.y() - 2, text);
                }
                break;
            case cstr::ConstraintType::Angle:
                if (c->hasDimensionalValue()) {
                    double deg = c->dimensionalValue() * 180.0 / 3.14159265358979;
                    QString text = QString::number(deg, 'f', 1) + "\xC2\xB0";
                    painter.drawText(screenPos.x() + 6, screenPos.y() - 2, text);
                }
                break;
            default:
                // Fixed, Tangent, Equal — small text label
                painter.drawText(screenPos.x() + 6, screenPos.y() - 2,
                                 QString::fromStdString(c->typeName().substr(0, 3)));
                break;
        }
    }
}
```

- [ ] **Step 2: Add helper to compute annotation position**

```cpp
math::Vec2 ViewportWidget::getConstraintAnnotationPos(
    const cstr::Constraint& c,
    const std::vector<std::shared_ptr<draft::DraftEntity>>& entities) {
    // Use first feature's position as base, offset slightly.
    auto refs = c.referencedEntityIds();
    for (uint64_t eid : refs) {
        for (const auto& e : entities) {
            if (e->id() == eid) {
                auto bbox = e->boundingBox();
                if (bbox.isValid()) return math::Vec2(bbox.center().x, bbox.center().y);
            }
        }
    }
    return math::Vec2(0, 0);
}
```

- [ ] **Step 3: Call renderConstraintAnnotations from paintGL**

Add the call after the dimension text rendering in the QPainter overlay section.

- [ ] **Step 4: Build and manually test**

1. Add various constraints and verify icons appear
2. Distance constraints show their value
3. Angle constraints show degrees with ° symbol

- [ ] **Step 5: Commit**

```bash
git add src/ui/src/ViewportWidget.cpp src/ui/include/horizon/ui/ViewportWidget.h
git commit -m "feat(ui): render constraint annotations on canvas

Shows visual indicators for each constraint type: coincident dots,
H/V labels, perpendicularity/parallel symbols, dimensional values."
```

---

## Task 6: Constraint Dimension Editing (Double-Click)

Double-clicking on an entity with a dimensional constraint (Distance/Angle) opens an input dialog to change the value and triggers a re-solve.

**Files:**
- Modify: `src/ui/src/SelectTool.cpp`

- [ ] **Step 1: Add double-click handling to SelectTool**

Override `mouseDoubleClickEvent` (or detect double-click timing in `mousePressEvent`):

```cpp
bool SelectTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    // ... existing code ...

    // Check for double-click on dimensional constraint.
    // Qt sends both press and double-click, so check event type.
    if (event->type() == QEvent::MouseButtonDblClick && event->button() == Qt::LeftButton) {
        return handleConstraintDoubleClick(worldPos);
    }

    // ... rest of existing code ...
}
```

- [ ] **Step 2: Implement handleConstraintDoubleClick**

```cpp
bool SelectTool::handleConstraintDoubleClick(const math::Vec2& worldPos) {
    auto& doc = *m_viewport->document();
    auto& csys = doc.constraintSystem();
    auto& draftDoc = doc.draftDocument();

    double pixelScale = m_viewport->pixelToWorldScale();
    double tolerance = std::max(15.0 * pixelScale, 0.3);

    // Find the closest dimensional constraint near the click.
    for (const auto& c : csys.constraints()) {
        if (!c->hasDimensionalValue()) continue;

        // Check if click is near any entity referenced by this constraint.
        for (uint64_t eid : c->referencedEntityIds()) {
            for (const auto& entity : draftDoc.entities()) {
                if (entity->id() != eid) continue;
                if (entity->hitTest(worldPos, tolerance)) {
                    // Found a dimensional constraint on this entity.
                    return editConstraintDimension(c->id(), c->dimensionalValue(),
                                                   c->type() == cstr::ConstraintType::Angle);
                }
            }
        }
    }
    return false;
}

bool SelectTool::editConstraintDimension(uint64_t constraintId, double currentValue, bool isAngle) {
    auto& doc = *m_viewport->document();
    auto& csys = doc.constraintSystem();
    auto& draftDoc = doc.draftDocument();

    double displayValue = isAngle ? (currentValue * 180.0 / 3.14159265358979) : currentValue;
    QString label = isAngle ? "Angle (degrees):" : "Distance:";

    bool ok;
    double newDisplay = QInputDialog::getDouble(
        m_viewport, "Edit Constraint", label, displayValue, 0.0, 1e9, 4, &ok);
    if (!ok || newDisplay == displayValue) return true;

    double newValue = isAngle ? (newDisplay * 3.14159265358979 / 180.0) : newDisplay;

    // Create composite: modify value + re-solve.
    auto composite = std::make_unique<doc::CompositeCommand>("Edit Constraint Value");
    composite->addCommand(
        std::make_unique<doc::ModifyConstraintValueCommand>(csys, constraintId, newValue));

    // Temporarily set the value for solving.
    auto* c = csys.getConstraint(constraintId);
    double oldVal = c->dimensionalValue();
    c->setDimensionalValue(newValue);

    auto solveCmd = doc::ConstraintSolveHelper::solveAndCreateCommand(draftDoc, csys);
    if (solveCmd) {
        solveCmd->undo();
        composite->addCommand(std::move(solveCmd));
    }

    // Restore old value (push will re-set via ModifyConstraintValueCommand).
    c->setDimensionalValue(oldVal);

    doc.undoStack().push(std::move(composite));
    doc.setDirty(true);
    m_viewport->update();
    return true;
}
```

- [ ] **Step 3: Add QInputDialog include**

Add `#include <QInputDialog>` to SelectTool.cpp.

- [ ] **Step 4: Build and manually test**

1. Draw two lines, add a Distance constraint
2. Double-click on one of the constrained entities
3. Dialog appears with current distance value
4. Change the value → entities move to satisfy the new distance
5. Undo → reverts both the value change and the entity positions

- [ ] **Step 5: Commit**

```bash
git add src/ui/src/SelectTool.cpp src/ui/include/horizon/ui/SelectTool.h
git commit -m "feat(ui): double-click to edit constraint dimensional values

Double-clicking an entity with a Distance or Angle constraint opens
an input dialog. Changing the value re-solves constraints and is fully undoable."
```

---

## Task 7: ParameterRegistry (Design Variables)

A named-variable registry in Document that maps string names to double values for future expression support.

**Files:**
- Create: `src/document/include/horizon/document/ParameterRegistry.h`
- Create: `src/document/src/ParameterRegistry.cpp`
- Modify: `src/document/include/horizon/document/Document.h`
- Modify: `src/document/src/Document.cpp`
- Modify: `src/document/CMakeLists.txt`
- Create: `tests/document/test_ParameterRegistry.cpp`
- Create: `tests/document/CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write failing tests**

Create `tests/document/test_ParameterRegistry.cpp`:
```cpp
#include <gtest/gtest.h>
#include "horizon/document/ParameterRegistry.h"

using namespace hz::doc;

TEST(ParameterRegistryTest, SetAndGet) {
    ParameterRegistry reg;
    reg.set("width", 50.0);
    EXPECT_DOUBLE_EQ(reg.get("width"), 50.0);
}

TEST(ParameterRegistryTest, GetNonExistentReturnsZero) {
    ParameterRegistry reg;
    EXPECT_DOUBLE_EQ(reg.get("missing"), 0.0);
}

TEST(ParameterRegistryTest, HasVariable) {
    ParameterRegistry reg;
    EXPECT_FALSE(reg.has("width"));
    reg.set("width", 10.0);
    EXPECT_TRUE(reg.has("width"));
}

TEST(ParameterRegistryTest, RemoveVariable) {
    ParameterRegistry reg;
    reg.set("width", 10.0);
    reg.remove("width");
    EXPECT_FALSE(reg.has("width"));
}

TEST(ParameterRegistryTest, AllVariables) {
    ParameterRegistry reg;
    reg.set("width", 10.0);
    reg.set("height", 20.0);
    auto vars = reg.all();
    EXPECT_EQ(vars.size(), 2u);
    EXPECT_DOUBLE_EQ(vars.at("width"), 10.0);
    EXPECT_DOUBLE_EQ(vars.at("height"), 20.0);
}

TEST(ParameterRegistryTest, Clear) {
    ParameterRegistry reg;
    reg.set("a", 1.0);
    reg.set("b", 2.0);
    reg.clear();
    EXPECT_EQ(reg.all().size(), 0u);
}
```

- [ ] **Step 2: Create document test infrastructure**

Create `tests/document/CMakeLists.txt`:
```cmake
add_executable(hz_document_tests
    test_ParameterRegistry.cpp
)

target_link_libraries(hz_document_tests
    PRIVATE
        Horizon::Document
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_document_tests)
```

Add `add_subdirectory(document)` to `tests/CMakeLists.txt`.

- [ ] **Step 3: Implement ParameterRegistry**

Create `src/document/include/horizon/document/ParameterRegistry.h`:
```cpp
#pragma once

#include <map>
#include <string>

namespace hz::doc {

/// Registry of named design variables (name → double value).
/// Used by the expression engine (Phase 27) and constraint system.
class ParameterRegistry {
public:
    ParameterRegistry() = default;

    void set(const std::string& name, double value);
    [[nodiscard]] double get(const std::string& name) const;
    [[nodiscard]] bool has(const std::string& name) const;
    void remove(const std::string& name);
    [[nodiscard]] const std::map<std::string, double>& all() const;
    void clear();

private:
    std::map<std::string, double> m_variables;
};

}  // namespace hz::doc
```

Create `src/document/src/ParameterRegistry.cpp`:
```cpp
#include "horizon/document/ParameterRegistry.h"

namespace hz::doc {

void ParameterRegistry::set(const std::string& name, double value) {
    m_variables[name] = value;
}

double ParameterRegistry::get(const std::string& name) const {
    auto it = m_variables.find(name);
    return (it != m_variables.end()) ? it->second : 0.0;
}

bool ParameterRegistry::has(const std::string& name) const {
    return m_variables.count(name) > 0;
}

void ParameterRegistry::remove(const std::string& name) {
    m_variables.erase(name);
}

const std::map<std::string, double>& ParameterRegistry::all() const {
    return m_variables;
}

void ParameterRegistry::clear() {
    m_variables.clear();
}

}  // namespace hz::doc
```

- [ ] **Step 4: Add ParameterRegistry to Document**

Add to `Document.h`:
```cpp
#include "ParameterRegistry.h"
// ...
ParameterRegistry& parameterRegistry();
const ParameterRegistry& parameterRegistry() const;
// ...
ParameterRegistry m_parameterRegistry;
```

Add to `Document.cpp`:
```cpp
ParameterRegistry& Document::parameterRegistry() { return m_parameterRegistry; }
const ParameterRegistry& Document::parameterRegistry() const { return m_parameterRegistry; }
```

Also clear it in `Document::clear()`.

- [ ] **Step 5: Add ParameterRegistry.cpp to CMakeLists**

Add to `src/document/CMakeLists.txt`.

- [ ] **Step 6: Build and run all tests**

Expected: All tests pass including 6 new ParameterRegistry tests.

- [ ] **Step 7: Commit**

```bash
git add src/document/include/horizon/document/ParameterRegistry.h \
        src/document/src/ParameterRegistry.cpp \
        src/document/include/horizon/document/Document.h \
        src/document/src/Document.cpp \
        src/document/CMakeLists.txt \
        tests/document/test_ParameterRegistry.cpp \
        tests/document/CMakeLists.txt \
        tests/CMakeLists.txt
git commit -m "feat(document): add ParameterRegistry for design variables

Named variable registry (name → double) in Document. Foundation for
Phase 27 expression engine. Supports set/get/has/remove/clear."
```

---

## Task 8: Serialize ParameterRegistry + Constraints in .hcad

Design variables and constraints must persist across save/load.

**Files:**
- Modify: `src/fileio/src/NativeFormat.cpp`

- [ ] **Step 1: Read current NativeFormat save/load**

Read `src/fileio/src/NativeFormat.cpp` to understand the JSON structure and where to add parameter registry and verify constraints are already serialized.

- [ ] **Step 2: Add ParameterRegistry serialization**

In the save function, after the existing sections, add:
```json
"designVariables": {
    "width": 50.0,
    "height": 20.0
}
```

In the load function, parse the `designVariables` key (if present — backward compatible).

- [ ] **Step 3: Verify constraint serialization exists**

Check if constraints are already serialized in NativeFormat. If not, add serialization for the ConstraintSystem (this was already done in a previous phase — verify and add if missing).

- [ ] **Step 4: Update format version to v13**

Increment the native format version to v13 to indicate design variables support.

- [ ] **Step 5: Build and test round-trip**

1. Create a drawing with design variables set
2. Save as .hcad
3. Load the file
4. Verify design variables are preserved

- [ ] **Step 6: Run all tests**

Expected: All tests pass.

- [ ] **Step 7: Commit**

```bash
git add src/fileio/src/NativeFormat.cpp
git commit -m "feat(fileio): serialize ParameterRegistry in .hcad v13

Design variables now persist across save/load. Backward compatible —
older files without designVariables key load with empty registry."
```

---

## Task 9: Final Phase Commit and Regression Testing

- [ ] **Step 1: Run the complete test suite**

Run: `cmake --build build/debug --config Debug && ctest --test-dir build/debug -C Debug --output-on-failure`
Report exact test count.

- [ ] **Step 2: Manual smoke test**

1. Draw lines and circles
2. Add coincident, horizontal, distance constraints
3. Verify DOF colors update (green → black as constraints added)
4. Move a constrained entity → verify auto-solve
5. Double-click to edit a distance → verify re-solve
6. Undo/redo entire workflow — verify all states restore correctly
7. Save and reload file — verify constraints and design variables persist

- [ ] **Step 3: Final commit**

```bash
git add -A
git commit -m "Phase 26: Parametric sketch integration with auto-solve and DOF visualization

- ConstraintSolveHelper encapsulates the snapshot→solve→apply workflow
- Post-move auto-solve: MoveEntityCommand and GripMoveCommand trigger re-solve
- DOF visualization: green = free, normal = constrained, red = over-constrained
- Constraint annotations: icons and dimensional values on canvas
- Double-click editing of Distance/Angle constraint values
- ParameterRegistry for named design variables (Phase 27 foundation)
- .hcad v13 with design variable serialization"
```
