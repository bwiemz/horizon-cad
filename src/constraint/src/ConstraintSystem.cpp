#include "horizon/constraint/ConstraintSystem.h"
#include <algorithm>

namespace hz::cstr {

uint64_t ConstraintSystem::addConstraint(std::shared_ptr<Constraint> constraint) {
    uint64_t cid = constraint->id();
    m_constraints.push_back(std::move(constraint));
    return cid;
}

std::shared_ptr<Constraint> ConstraintSystem::removeConstraint(uint64_t constraintId) {
    for (auto it = m_constraints.begin(); it != m_constraints.end(); ++it) {
        if ((*it)->id() == constraintId) {
            auto removed = std::move(*it);
            m_constraints.erase(it);
            return removed;
        }
    }
    return nullptr;
}

const Constraint* ConstraintSystem::getConstraint(uint64_t constraintId) const {
    for (const auto& c : m_constraints) {
        if (c->id() == constraintId) return c.get();
    }
    return nullptr;
}

Constraint* ConstraintSystem::getConstraint(uint64_t constraintId) {
    for (auto& c : m_constraints) {
        if (c->id() == constraintId) return c.get();
    }
    return nullptr;
}

std::vector<const Constraint*> ConstraintSystem::constraintsForEntity(
    uint64_t entityId) const {
    std::vector<const Constraint*> result;
    for (const auto& c : m_constraints) {
        auto ids = c->referencedEntityIds();
        for (uint64_t id : ids) {
            if (id == entityId) {
                result.push_back(c.get());
                break;
            }
        }
    }
    return result;
}

std::vector<std::shared_ptr<Constraint>> ConstraintSystem::removeConstraintsForEntity(
    uint64_t entityId) {
    std::vector<std::shared_ptr<Constraint>> removed;

    // Use stable_partition to avoid O(n^2) repeated mid-vector erases.
    auto partition = std::stable_partition(
        m_constraints.begin(), m_constraints.end(),
        [entityId, &removed](const auto& c) {
            auto ids = c->referencedEntityIds();
            for (uint64_t id : ids) {
                if (id == entityId) {
                    removed.push_back(c);
                    return false;  // Move to "remove" partition
                }
            }
            return true;  // Keep
        });

    m_constraints.erase(partition, m_constraints.end());
    return removed;
}

void ConstraintSystem::resolveVariables(
    const std::function<double(const std::string&)>& resolver) {
    if (!resolver) return;
    for (auto& c : m_constraints) {
        if (c->hasVariableReference() && c->hasDimensionalValue()) {
            double value = resolver(c->variableReference());
            c->setDimensionalValue(value);
        }
    }
}

int ConstraintSystem::totalEquations() const {
    int total = 0;
    for (const auto& c : m_constraints) {
        total += c->equationCount();
    }
    return total;
}

void ConstraintSystem::clear() {
    m_constraints.clear();
}

}  // namespace hz::cstr
