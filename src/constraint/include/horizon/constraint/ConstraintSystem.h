#pragma once

#include "horizon/constraint/Constraint.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace hz::cstr {

/// Stores and manages all geometric constraints for a document.
class ConstraintSystem {
public:
    ConstraintSystem() = default;

    uint64_t addConstraint(std::shared_ptr<Constraint> constraint);
    std::shared_ptr<Constraint> removeConstraint(uint64_t constraintId);
    const Constraint* getConstraint(uint64_t constraintId) const;
    Constraint* getConstraint(uint64_t constraintId);

    const std::vector<std::shared_ptr<Constraint>>& constraints() const { return m_constraints; }

    /// All constraints referencing a given entity.
    std::vector<const Constraint*> constraintsForEntity(uint64_t entityId) const;

    /// Remove all constraints that reference the given entity. Returns removed constraints.
    std::vector<std::shared_ptr<Constraint>> removeConstraintsForEntity(uint64_t entityId);

    int totalEquations() const;
    bool empty() const { return m_constraints.empty(); }
    void clear();

private:
    std::vector<std::shared_ptr<Constraint>> m_constraints;
};

}  // namespace hz::cstr
