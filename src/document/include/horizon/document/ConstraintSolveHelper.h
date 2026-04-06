#pragma once

#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/SketchSolver.h"
#include "horizon/document/ConstraintCommands.h"
#include "horizon/drafting/DraftDocument.h"
#include <functional>
#include <memory>
#include <string>
#include <vector>

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
    /// resolved before solving.
    static SolveAndApplyResult solveAndApply(
        draft::DraftDocument& draftDoc, cstr::ConstraintSystem& csys,
        std::function<double(const std::string&)> variableResolver = nullptr);

    /// Convenience: solve + create command (nullptr if nothing changed).
    static std::unique_ptr<ApplyConstraintSolveCommand> solveAndCreateCommand(
        draft::DraftDocument& draftDoc, cstr::ConstraintSystem& csys,
        std::function<double(const std::string&)> variableResolver = nullptr);
};

}  // namespace hz::doc
