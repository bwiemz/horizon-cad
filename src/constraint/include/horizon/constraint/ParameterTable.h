#pragma once

#include "horizon/constraint/GeometryRef.h"
#include "horizon/math/Vec2.h"
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hz::draft {
class DraftEntity;
}

namespace hz::cstr {

class ConstraintSystem;

/// Maps DraftEntity geometry to a flat parameter vector for the solver.
class ParameterTable {
public:
    ParameterTable() = default;

    /// Register an entity's parameters. Returns the starting index.
    int registerEntity(const draft::DraftEntity& entity);

    int parameterCount() const { return static_cast<int>(m_values.size()); }

    const Eigen::VectorXd& values() const { return m_values; }
    Eigen::VectorXd& values() { return m_values; }

    /// Get the parameter index for the start of a geometry feature's parameters.
    /// Point: index of [x, y]. Line: index of [startX, startY, endX, endY].
    /// Circle: index of [centerX, centerY, radius].
    int parameterIndex(const GeometryRef& ref) const;

    /// Extract a point position from current parameter values.
    math::Vec2 pointPosition(const GeometryRef& ref) const;

    /// Extract line endpoints from current parameter values.
    std::pair<math::Vec2, math::Vec2> lineEndpoints(const GeometryRef& ref) const;

    /// Extract circle center and radius from current parameter values.
    std::pair<math::Vec2, double> circleData(const GeometryRef& ref) const;

    /// Write solved parameters back to entities.
    void applyToEntities(std::vector<std::shared_ptr<draft::DraftEntity>>& entities) const;

    /// Build parameter table from entities involved in constraints.
    static ParameterTable buildFromEntities(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& entities,
        const ConstraintSystem& constraints);

    /// Check if an entity is registered.
    bool hasEntity(uint64_t entityId) const;

private:
    struct EntityParams {
        uint64_t entityId = 0;
        int startIndex = 0;
        int paramCount = 0;
        std::string entityType;
    };

    Eigen::VectorXd m_values;
    std::vector<EntityParams> m_entityParams;

    const EntityParams* findEntityParams(uint64_t entityId) const;
};

}  // namespace hz::cstr
