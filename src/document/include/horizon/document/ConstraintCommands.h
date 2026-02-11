#pragma once

#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftEntity.h"
#include <memory>
#include <vector>

namespace hz::doc {

/// Command to add a constraint.
class AddConstraintCommand : public Command {
public:
    AddConstraintCommand(cstr::ConstraintSystem& system,
                         std::shared_ptr<cstr::Constraint> constraint);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    cstr::ConstraintSystem& m_system;
    std::shared_ptr<cstr::Constraint> m_constraint;
    uint64_t m_constraintId = 0;
};

/// Command to remove a constraint.
class RemoveConstraintCommand : public Command {
public:
    RemoveConstraintCommand(cstr::ConstraintSystem& system, uint64_t constraintId);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    cstr::ConstraintSystem& m_system;
    uint64_t m_constraintId;
    std::shared_ptr<cstr::Constraint> m_constraint;
};

/// Command to modify the dimensional value of a Distance or Angle constraint.
class ModifyConstraintValueCommand : public Command {
public:
    ModifyConstraintValueCommand(cstr::ConstraintSystem& system,
                                  uint64_t constraintId, double newValue);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    cstr::ConstraintSystem& m_system;
    uint64_t m_constraintId;
    double m_newValue;
    double m_oldValue = 0.0;
};

/// Command that records before/after entity states from a constraint solve.
class ApplyConstraintSolveCommand : public Command {
public:
    struct EntitySnapshot {
        uint64_t entityId;
        std::shared_ptr<draft::DraftEntity> beforeState;
        std::shared_ptr<draft::DraftEntity> afterState;
    };

    ApplyConstraintSolveCommand(draft::DraftDocument& doc,
                                 std::vector<EntitySnapshot> snapshots);
    void execute() override;
    void undo() override;
    std::string description() const override;

private:
    void applyStates(bool useAfter);

    draft::DraftDocument& m_doc;
    std::vector<EntitySnapshot> m_snapshots;
};

}  // namespace hz::doc
