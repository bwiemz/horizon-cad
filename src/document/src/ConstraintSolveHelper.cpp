#include "horizon/document/ConstraintSolveHelper.h"

#include "horizon/constraint/ParameterTable.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"

namespace hz::doc {

/// Copy geometric properties from src to dst (same pattern as ConstraintCommands.cpp).
static void copyEntityGeometry(const draft::DraftEntity& src, draft::DraftEntity& dst) {
    if (auto* sl = dynamic_cast<const draft::DraftLine*>(&src)) {
        if (auto* dl = dynamic_cast<draft::DraftLine*>(&dst)) {
            dl->setStart(sl->start());
            dl->setEnd(sl->end());
        }
    } else if (auto* sc = dynamic_cast<const draft::DraftCircle*>(&src)) {
        if (auto* dc = dynamic_cast<draft::DraftCircle*>(&dst)) {
            dc->setCenter(sc->center());
            dc->setRadius(sc->radius());
        }
    } else if (auto* sa = dynamic_cast<const draft::DraftArc*>(&src)) {
        if (auto* da = dynamic_cast<draft::DraftArc*>(&dst)) {
            da->setCenter(sa->center());
            da->setRadius(sa->radius());
            da->setStartAngle(sa->startAngle());
            da->setEndAngle(sa->endAngle());
        }
    } else if (auto* sr = dynamic_cast<const draft::DraftRectangle*>(&src)) {
        if (auto* dr = dynamic_cast<draft::DraftRectangle*>(&dst)) {
            dr->setCorner1(sr->corner1());
            dr->setCorner2(sr->corner2());
        }
    } else if (auto* sp = dynamic_cast<const draft::DraftPolyline*>(&src)) {
        if (auto* dp = dynamic_cast<draft::DraftPolyline*>(&dst)) {
            dp->setPoints(sp->points());
        }
    }
}

static bool isSolveSuccess(cstr::SolveStatus status) {
    return status == cstr::SolveStatus::Success ||
           status == cstr::SolveStatus::Converged ||
           status == cstr::SolveStatus::UnderConstrained;
}

ConstraintSolveHelper::SolveAndApplyResult ConstraintSolveHelper::solveAndApply(
    draft::DraftDocument& draftDoc, const cstr::ConstraintSystem& csys) {
    SolveAndApplyResult result;

    // If no constraints, nothing to do — return success with empty snapshots.
    if (csys.empty()) {
        result.success = true;
        result.solveResult.status = cstr::SolveStatus::NoConstraints;
        return result;
    }

    auto& entities = draftDoc.entities();
    if (entities.empty()) {
        result.success = true;
        result.solveResult.status = cstr::SolveStatus::NoConstraints;
        return result;
    }

    // Build parameter table from entities referenced by constraints.
    auto paramTable = cstr::ParameterTable::buildFromEntities(entities, csys);

    if (paramTable.parameterCount() == 0) {
        result.success = true;
        result.solveResult.status = cstr::SolveStatus::NoConstraints;
        return result;
    }

    // Snapshot before-states for all entities in the parameter table.
    std::vector<ApplyConstraintSolveCommand::EntitySnapshot> snapshots;
    for (const auto& entity : entities) {
        if (paramTable.hasEntity(entity->id())) {
            ApplyConstraintSolveCommand::EntitySnapshot snap;
            snap.entityId = entity->id();
            snap.beforeState = entity->clone();
            snapshots.push_back(std::move(snap));
        }
    }

    // Run the solver.
    cstr::SketchSolver solver;
    result.solveResult = solver.solve(paramTable, csys);

    if (isSolveSuccess(result.solveResult.status)) {
        // Apply solved parameters back to entities.
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

        result.success = true;
        result.snapshots = std::move(snapshots);
    } else {
        // Solve failed — restore entities to before-states.
        for (const auto& snap : snapshots) {
            if (!snap.beforeState) continue;
            for (auto& entity : entities) {
                if (entity->id() == snap.entityId) {
                    copyEntityGeometry(*snap.beforeState, *entity);
                    break;
                }
            }
        }

        result.success = false;
    }

    return result;
}

std::unique_ptr<ApplyConstraintSolveCommand> ConstraintSolveHelper::solveAndCreateCommand(
    draft::DraftDocument& draftDoc, const cstr::ConstraintSystem& csys) {
    auto result = solveAndApply(draftDoc, csys);

    if (!result.success || result.snapshots.empty()) {
        return nullptr;
    }

    return std::make_unique<ApplyConstraintSolveCommand>(draftDoc, std::move(result.snapshots));
}

}  // namespace hz::doc
