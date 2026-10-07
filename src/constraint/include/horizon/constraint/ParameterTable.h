#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "horizon/constraint/GeometryRef.h"
#include "horizon/math/Vec2.h"

namespace hz::draft {
class DraftEntity;
}

namespace hz::cstr {

class ConstraintSystem;

/// How a point of an entity moves with the solver's parameters: its partial
/// derivatives, by column. A line's end is two parameters of the table; an
/// arc's end is made of four (its centre's two, its radius and its angle),
/// and a constraint on it must be differentiated by all four.
struct PointJacobian {
    struct Term {
        int column = 0;
        math::Vec2 d;  ///< d(point) / d(parameter `column`)
    };
    std::array<Term, 4> terms{};
    int count = 0;

    void add(int column, const math::Vec2& d) { terms[static_cast<size_t>(count++)] = {column, d}; }

    /// Add to row @p row of @p jac the derivative of an equation that depends
    /// on the point as @p weight (its d/d(point.x) and d/d(point.y)).
    void addTo(Eigen::MatrixXd& jac, int row, const math::Vec2& weight) const {
        for (int k = 0; k < count; ++k) {
            const Term& t = terms[static_cast<size_t>(k)];
            jac(row, t.column) += weight.x * t.d.x + weight.y * t.d.y;
        }
    }
};

/// Maps DraftEntity geometry to a flat parameter vector for the solver.
///
/// A ref its entity does not have (a third end of a line, a vertex past a
/// polyline's last, a circle read as a line) throws std::runtime_error from
/// every accessor: a hand-edited file can hold one.
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
    /// Circle: index of [centerX, centerY, radius]. A feature that is not a
    /// run of the table's parameters (an arc's end, a rectangle's corner or
    /// edge, a closed polyline's last segment) throws std::runtime_error:
    /// pointJacobian() and lineJacobian() say how those move.
    int parameterIndex(const GeometryRef& ref) const;

    /// Extract a point position from current parameter values.
    math::Vec2 pointPosition(const GeometryRef& ref) const;

    /// How the point pointPosition() gives moves with the parameters.
    PointJacobian pointJacobian(const GeometryRef& ref) const;

    /// Extract line endpoints from current parameter values. A closed
    /// polyline's last segment runs from its last point to its first.
    std::pair<math::Vec2, math::Vec2> lineEndpoints(const GeometryRef& ref) const;

    /// How the two ends lineEndpoints() gives move with the parameters.
    std::pair<PointJacobian, PointJacobian> lineJacobian(const GeometryRef& ref) const;

    /// Extract circle center and radius from current parameter values.
    std::pair<math::Vec2, double> circleData(const GeometryRef& ref) const;

    /// Write solved parameters back to entities.
    void applyToEntities(std::vector<std::shared_ptr<draft::DraftEntity>>& entities) const;
    /// Write @p entity's parameters back to it; nothing for one not
    /// registered.
    void applyToEntity(draft::DraftEntity& entity) const;

    /// Build parameter table from entities involved in constraints.
    static ParameterTable buildFromEntities(
        const std::vector<std::shared_ptr<draft::DraftEntity>>& entities,
        const ConstraintSystem& constraints);

    /// Check if an entity is registered.
    bool hasEntity(uint64_t entityId) const;

    /// Whether parameter @p index is held where it is: one of an edge of the
    /// part projected into the sketch (Phase 157), which the part places.
    bool isFixed(int index) const {
        return index >= 0 && static_cast<size_t>(index) < m_fixed.size() &&
               m_fixed[static_cast<size_t>(index)];
    }
    /// How many parameters are held.
    int fixedCount() const {
        return static_cast<int>(std::count(m_fixed.begin(), m_fixed.end(), true));
    }

    /// Entity @p entityId's parameters: the first one's index and how many
    /// there are; {0, 0} when it is not registered.
    std::pair<int, int> parameterRange(uint64_t entityId) const;

private:
    struct EntityParams {
        uint64_t entityId = 0;
        int startIndex = 0;
        int paramCount = 0;
        std::string entityType;
        bool closed = false;  ///< a polyline's last point joined to its first
    };

    Eigen::VectorXd m_values;
    std::vector<bool> m_fixed;  ///< by parameter: held (isFixed())
    std::vector<EntityParams> m_entityParams;
    /// Entity id -> its entry in m_entityParams (the first, if registered
    /// twice): lookups were a search of them all, for every constraint.
    std::unordered_map<uint64_t, std::size_t> m_byId;

    const EntityParams* findEntityParams(uint64_t entityId) const;

    /// @p ref's entity, or a throw when it is not registered.
    const EntityParams& entityOf(const GeometryRef& ref) const;
    /// Point @p index of @p ep, and how it moves if @p jac is given; throws
    /// when @p ep has no such point.
    math::Vec2 point(const EntityParams& ep, int index, PointJacobian* jac) const;
    /// Segment @p index of @p ep, as point() its ends.
    std::pair<math::Vec2, math::Vec2> segment(const EntityParams& ep, int index, PointJacobian* js,
                                              PointJacobian* je) const;
};

}  // namespace hz::cstr
