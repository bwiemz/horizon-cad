#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/ParameterTable.h"
#include "horizon/constraint/SketchSolver.h"
#include "horizon/document/ConstraintCommands.h"
#include "horizon/drafting/DraftDocument.h"

namespace hz::doc {

class ConstraintSolveHelper {
public:
    struct SolveAndApplyResult {
        bool success = false;
        cstr::SolveResult solveResult;
        std::vector<ApplyConstraintSolveCommand::EntitySnapshot> snapshots;
    };

    /// Solve all constraints against current entity positions.
    /// On success, entity positions ARE updated in draftDoc.
    /// Returns snapshots for creating ApplyConstraintSolveCommand.
    /// If variableResolver is provided, variable-referenced constraints are
    /// resolved before solving. A constraint on an entity that is not there
    /// is left out (see solvable()), and one that does not fit its entity
    /// fails the solve rather than throwing.
    static SolveAndApplyResult solveAndApply(
        draft::DraftDocument& draftDoc, cstr::ConstraintSystem& csys,
        std::function<double(const std::string&)> variableResolver = nullptr);

    /// The constraints of @p csys whose entities are all in @p table: what a
    /// solve, or a count of what is left free, can take. One that names an
    /// entity gone from the drawing (an older file's, or one read in part)
    /// is left out: the solver stopped at it.
    static cstr::ConstraintSystem solvable(const cstr::ParameterTable& table,
                                           const cstr::ConstraintSystem& csys);

    /// Convenience: solve + create command (nullptr if nothing changed).
    static std::unique_ptr<ApplyConstraintSolveCommand> solveAndCreateCommand(
        draft::DraftDocument& draftDoc, cstr::ConstraintSystem& csys,
        std::function<double(const std::string&)> variableResolver = nullptr);
};

}  // namespace hz::doc
