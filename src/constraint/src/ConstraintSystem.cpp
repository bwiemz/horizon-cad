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
    auto it = m_constraints.begin();
    while (it != m_constraints.end()) {
        auto ids = (*it)->referencedEntityIds();
        bool references = false;
        for (uint64_t id : ids) {
            if (id == entityId) {
                references = true;
                break;
            }
        }
        if (references) {
            removed.push_back(std::move(*it));
            it = m_constraints.erase(it);
        } else {
            ++it;
        }
    }
    return removed;
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
