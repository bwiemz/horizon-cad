#include "horizon/document/ConstraintCommands.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"

namespace hz::doc {

// ---------------------------------------------------------------------------
// AddConstraintCommand
// ---------------------------------------------------------------------------

AddConstraintCommand::AddConstraintCommand(cstr::ConstraintSystem& system,
                                           std::shared_ptr<cstr::Constraint> constraint)
    : m_system(system), m_constraint(std::move(constraint)) {
    m_constraintId = m_constraint->id();
}

void AddConstraintCommand::execute() {
    m_system.addConstraint(m_constraint);
}

void AddConstraintCommand::undo() {
    m_system.removeConstraint(m_constraintId);
}

std::string AddConstraintCommand::description() const {
    return "Add " + m_constraint->typeName() + " Constraint";
}

// ---------------------------------------------------------------------------
// RemoveConstraintCommand
// ---------------------------------------------------------------------------

RemoveConstraintCommand::RemoveConstraintCommand(cstr::ConstraintSystem& system,
                                                 uint64_t constraintId)
    : m_system(system), m_constraintId(constraintId) {}

void RemoveConstraintCommand::execute() {
    m_constraint = m_system.removeConstraint(m_constraintId);
}

void RemoveConstraintCommand::undo() {
    if (m_constraint) {
        m_system.addConstraint(m_constraint);
    }
}

std::string RemoveConstraintCommand::description() const {
    return "Remove Constraint";
}

// ---------------------------------------------------------------------------
// ModifyConstraintValueCommand
// ---------------------------------------------------------------------------

ModifyConstraintValueCommand::ModifyConstraintValueCommand(cstr::ConstraintSystem& system,
                                                            uint64_t constraintId,
                                                            double newValue)
    : m_system(system), m_constraintId(constraintId), m_newValue(newValue) {}

void ModifyConstraintValueCommand::execute() {
    auto* c = m_system.getConstraint(m_constraintId);
    if (c && c->hasDimensionalValue()) {
        m_oldValue = c->dimensionalValue();
        c->setDimensionalValue(m_newValue);
    }
}

void ModifyConstraintValueCommand::undo() {
    auto* c = m_system.getConstraint(m_constraintId);
    if (c && c->hasDimensionalValue()) {
        c->setDimensionalValue(m_oldValue);
    }
}

std::string ModifyConstraintValueCommand::description() const {
    return "Modify Constraint Value";
}

// ---------------------------------------------------------------------------
// ApplyConstraintSolveCommand
// ---------------------------------------------------------------------------

static void copyEntityGeometry(const draft::DraftEntity& src, draft::DraftEntity& dst) {
    if (auto* sl = dynamic_cast<const draft::DraftLine*>(&src)) {
        auto* dl = dynamic_cast<draft::DraftLine*>(&dst);
        if (dl) {
            dl->setStart(sl->start());
            dl->setEnd(sl->end());
        }
    } else if (auto* sc = dynamic_cast<const draft::DraftCircle*>(&src)) {
        auto* dc = dynamic_cast<draft::DraftCircle*>(&dst);
        if (dc) {
            dc->setCenter(sc->center());
            dc->setRadius(sc->radius());
        }
    } else if (auto* sa = dynamic_cast<const draft::DraftArc*>(&src)) {
        auto* da = dynamic_cast<draft::DraftArc*>(&dst);
        if (da) {
            da->setCenter(sa->center());
            da->setRadius(sa->radius());
            da->setStartAngle(sa->startAngle());
            da->setEndAngle(sa->endAngle());
        }
    } else if (auto* sr = dynamic_cast<const draft::DraftRectangle*>(&src)) {
        auto* dr = dynamic_cast<draft::DraftRectangle*>(&dst);
        if (dr) {
            dr->setCorner1(sr->corner1());
            dr->setCorner2(sr->corner2());
        }
    } else if (auto* sp = dynamic_cast<const draft::DraftPolyline*>(&src)) {
        auto* dp = dynamic_cast<draft::DraftPolyline*>(&dst);
        if (dp) {
            dp->setPoints(sp->points());
        }
    }
}

ApplyConstraintSolveCommand::ApplyConstraintSolveCommand(
    draft::DraftDocument& doc, std::vector<EntitySnapshot> snapshots)
    : m_doc(doc), m_snapshots(std::move(snapshots)) {}

void ApplyConstraintSolveCommand::execute() {
    applyStates(true);  // Apply afterState
}

void ApplyConstraintSolveCommand::undo() {
    applyStates(false);  // Apply beforeState
}

void ApplyConstraintSolveCommand::applyStates(bool useAfter) {
    for (const auto& snap : m_snapshots) {
        const auto& src = useAfter ? snap.afterState : snap.beforeState;
        if (!src) continue;
        for (auto& entity : m_doc.entities()) {
            if (entity->id() == snap.entityId) {
                copyEntityGeometry(*src, *entity);
                break;
            }
        }
    }
}

std::string ApplyConstraintSolveCommand::description() const {
    return "Apply Constraint Solve";
}

}  // namespace hz::doc
