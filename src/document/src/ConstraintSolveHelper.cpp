#include "horizon/document/ConstraintSolveHelper.h"

#include <algorithm>
#include <cmath>
#include <exception>

namespace hz::doc {

/// Whether a solve's result stands: it converged, with or without freedom left.
static bool isSolveSuccess(cstr::SolveStatus status) {
    return status == cstr::SolveStatus::Success || status == cstr::SolveStatus::Converged ||
           status == cstr::SolveStatus::UnderConstrained;
}

ConstraintSolveHelper::SolveAndApplyResult ConstraintSolveHelper::solveAndApply(
    draft::DraftDocument& draftDoc, cstr::ConstraintSystem& csys,
    std::function<double(const std::string&)> variableResolver) {
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

    // Resolve any variable-referenced dimensional values before solving.
    if (variableResolver) {
        csys.resolveVariables(variableResolver);
    }

    // Build parameter table from entities referenced by constraints.
    auto paramTable = cstr::ParameterTable::buildFromEntities(entities, csys);
    const cstr::ConstraintSystem constraints = solvable(paramTable, csys);

    if (paramTable.parameterCount() == 0 || constraints.empty()) {
        result.success = true;
        result.solveResult.status = cstr::SolveStatus::NoConstraints;
        return result;
    }

    // Run the solver. It works on the table alone: the entities are
    // untouched until it has succeeded, so a failure has nothing to undo.
    // A constraint that does not fit its entity (a line's on a circle, from
    // a damaged file) fails it: thrown, it left the command that solved
    // half done.
    const Eigen::VectorXd before = paramTable.values();
    cstr::SketchSolver solver;
    try {
        result.solveResult = solver.solve(paramTable, constraints);
    } catch (const std::exception& e) {
        result.solveResult = {};
        result.solveResult.status = cstr::SolveStatus::Inconsistent;
        result.solveResult.message = e.what();
    }
    if (!isSolveSuccess(result.solveResult.status)) {
        result.success = false;
        return result;
    }

    // Only the entities the solve moved are changed, and kept for undo: every
    // constrained entity was cloned twice, before and after, into every move.
    std::vector<ApplyConstraintSolveCommand::EntitySnapshot> snapshots;
    for (const auto& entity : entities) {
        const auto [first, count] = paramTable.parameterRange(entity->id());
        bool moved = false;
        for (int p = first; p < first + count && !moved; ++p) {
            // Beyond rounding: a solve's step can touch what it need not.
            moved = std::abs(paramTable.values()(p) - before(p)) >
                    1e-12 * std::max(1.0, std::abs(before(p)));
        }
        if (!moved) continue;
        ApplyConstraintSolveCommand::EntitySnapshot snap;
        snap.entityId = entity->id();
        snap.beforeState = entity->clone();
        paramTable.applyToEntity(*entity);
        draftDoc.updateEntityBounds(entity->id());  // picked and drawn where it is now
        snap.afterState = entity->clone();
        snapshots.push_back(std::move(snap));
    }
    result.success = true;
    result.snapshots = std::move(snapshots);
    return result;
}

cstr::ConstraintSystem ConstraintSolveHelper::solvable(const cstr::ParameterTable& table,
                                                       const cstr::ConstraintSystem& csys) {
    cstr::ConstraintSystem kept;
    for (const auto& constraint : csys.constraints()) {
        const auto ids = constraint->referencedEntityIds();
        const bool whole = std::all_of(ids.begin(), ids.end(),
                                       [&table](uint64_t id) { return table.hasEntity(id); });
        if (whole) kept.addConstraint(constraint);
    }
    return kept;
}

std::unique_ptr<ApplyConstraintSolveCommand> ConstraintSolveHelper::solveAndCreateCommand(
    draft::DraftDocument& draftDoc, cstr::ConstraintSystem& csys,
    std::function<double(const std::string&)> variableResolver) {
    auto result = solveAndApply(draftDoc, csys, std::move(variableResolver));

    if (!result.success || result.snapshots.empty()) {
        return nullptr;
    }

    return std::make_unique<ApplyConstraintSolveCommand>(draftDoc, std::move(result.snapshots));
}

}  // namespace hz::doc
