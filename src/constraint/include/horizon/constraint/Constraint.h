#pragma once

#include "horizon/constraint/GeometryRef.h"
#include "horizon/math/Vec2.h"
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hz::cstr {

class ParameterTable;

enum class ConstraintType {
    Coincident,
    Horizontal,
    Vertical,
    Perpendicular,
    Parallel,
    Tangent,
    Equal,
    Fixed,
    Distance,
    Angle,
};

/// Abstract base for all geometric constraints.
class Constraint {
public:
    Constraint();
    virtual ~Constraint() = default;

    uint64_t id() const { return m_id; }

    /// Override the auto-generated ID (used when loading from file).
    void setId(uint64_t newId) { m_id = newId; }

    /// Ensure the next auto-generated ID is greater than the given value.
    static void advanceIdCounter(uint64_t minId) {
        if (s_nextId <= minId) s_nextId = minId + 1;
    }

    virtual ConstraintType type() const = 0;
    virtual std::string typeName() const = 0;

    /// Number of scalar equations this constraint contributes.
    virtual int equationCount() const = 0;

    /// The entity IDs referenced by this constraint.
    virtual std::vector<uint64_t> referencedEntityIds() const = 0;

    /// Evaluate residual values for this constraint.
    virtual void evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                          int offset) const = 0;

    /// Fill Jacobian rows for this constraint.
    virtual void jacobian(const ParameterTable& params, Eigen::MatrixXd& jacobian,
                          int offset) const = 0;

    /// Whether this constraint has an editable dimensional value.
    virtual bool hasDimensionalValue() const { return false; }
    virtual double dimensionalValue() const { return 0.0; }
    virtual void setDimensionalValue(double /*v*/) {}

    /// Clone the constraint (preserving the same ID).
    virtual std::shared_ptr<Constraint> clone() const = 0;

private:
    uint64_t m_id;
    static uint64_t s_nextId;
};

// ---------------------------------------------------------------------------
// Concrete constraint types
// ---------------------------------------------------------------------------

/// Two points coincide: pA == pB (2 equations).
class CoincidentConstraint : public Constraint {
public:
    CoincidentConstraint(const GeometryRef& pointA, const GeometryRef& pointB);
    ConstraintType type() const override { return ConstraintType::Coincident; }
    std::string typeName() const override { return "Coincident"; }
    int equationCount() const override { return 2; }
    std::vector<uint64_t> referencedEntityIds() const override;
    void evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                  int offset) const override;
    void jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                  int offset) const override;
    std::shared_ptr<Constraint> clone() const override;

    const GeometryRef& pointA() const { return m_pointA; }
    const GeometryRef& pointB() const { return m_pointB; }

private:
    GeometryRef m_pointA, m_pointB;
};

/// Two points at the same Y: pA.y == pB.y (1 equation).
/// Also works on a single line (refA=start, refB=end).
class HorizontalConstraint : public Constraint {
public:
    HorizontalConstraint(const GeometryRef& refA, const GeometryRef& refB);
    ConstraintType type() const override { return ConstraintType::Horizontal; }
    std::string typeName() const override { return "Horizontal"; }
    int equationCount() const override { return 1; }
    std::vector<uint64_t> referencedEntityIds() const override;
    void evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                  int offset) const override;
    void jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                  int offset) const override;
    std::shared_ptr<Constraint> clone() const override;

    const GeometryRef& refA() const { return m_refA; }
    const GeometryRef& refB() const { return m_refB; }

private:
    GeometryRef m_refA, m_refB;
};

/// Two points at the same X: pA.x == pB.x (1 equation).
class VerticalConstraint : public Constraint {
public:
    VerticalConstraint(const GeometryRef& refA, const GeometryRef& refB);
    ConstraintType type() const override { return ConstraintType::Vertical; }
    std::string typeName() const override { return "Vertical"; }
    int equationCount() const override { return 1; }
    std::vector<uint64_t> referencedEntityIds() const override;
    void evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                  int offset) const override;
    void jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                  int offset) const override;
    std::shared_ptr<Constraint> clone() const override;

    const GeometryRef& refA() const { return m_refA; }
    const GeometryRef& refB() const { return m_refB; }

private:
    GeometryRef m_refA, m_refB;
};

/// Two lines are perpendicular: d1.dot(d2) == 0 (1 equation).
class PerpendicularConstraint : public Constraint {
public:
    PerpendicularConstraint(const GeometryRef& lineA, const GeometryRef& lineB);
    ConstraintType type() const override { return ConstraintType::Perpendicular; }
    std::string typeName() const override { return "Perpendicular"; }
    int equationCount() const override { return 1; }
    std::vector<uint64_t> referencedEntityIds() const override;
    void evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                  int offset) const override;
    void jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                  int offset) const override;
    std::shared_ptr<Constraint> clone() const override;

    const GeometryRef& lineA() const { return m_lineA; }
    const GeometryRef& lineB() const { return m_lineB; }

private:
    GeometryRef m_lineA, m_lineB;
};

/// Two lines are parallel: d1.cross(d2) == 0 (1 equation).
class ParallelConstraint : public Constraint {
public:
    ParallelConstraint(const GeometryRef& lineA, const GeometryRef& lineB);
    ConstraintType type() const override { return ConstraintType::Parallel; }
    std::string typeName() const override { return "Parallel"; }
    int equationCount() const override { return 1; }
    std::vector<uint64_t> referencedEntityIds() const override;
    void evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                  int offset) const override;
    void jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                  int offset) const override;
    std::shared_ptr<Constraint> clone() const override;

    const GeometryRef& lineA() const { return m_lineA; }
    const GeometryRef& lineB() const { return m_lineB; }

private:
    GeometryRef m_lineA, m_lineB;
};

/// Line is tangent to circle: |signed_dist(line, center)| == radius (1 equation).
class TangentConstraint : public Constraint {
public:
    TangentConstraint(const GeometryRef& lineRef, const GeometryRef& circleRef);
    ConstraintType type() const override { return ConstraintType::Tangent; }
    std::string typeName() const override { return "Tangent"; }
    int equationCount() const override { return 1; }
    std::vector<uint64_t> referencedEntityIds() const override;
    void evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                  int offset) const override;
    void jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                  int offset) const override;
    std::shared_ptr<Constraint> clone() const override;

    const GeometryRef& lineRef() const { return m_lineRef; }
    const GeometryRef& circleRef() const { return m_circleRef; }

private:
    GeometryRef m_lineRef, m_circleRef;
};

/// Two features have equal measure: same length (lines) or same radius (circles).
class EqualConstraint : public Constraint {
public:
    EqualConstraint(const GeometryRef& refA, const GeometryRef& refB);
    ConstraintType type() const override { return ConstraintType::Equal; }
    std::string typeName() const override { return "Equal"; }
    int equationCount() const override { return 1; }
    std::vector<uint64_t> referencedEntityIds() const override;
    void evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                  int offset) const override;
    void jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                  int offset) const override;
    std::shared_ptr<Constraint> clone() const override;

    const GeometryRef& refA() const { return m_refA; }
    const GeometryRef& refB() const { return m_refB; }

private:
    GeometryRef m_refA, m_refB;
};

/// Lock a point at a fixed position (2 equations).
class FixedConstraint : public Constraint {
public:
    FixedConstraint(const GeometryRef& pointRef, const math::Vec2& position);
    ConstraintType type() const override { return ConstraintType::Fixed; }
    std::string typeName() const override { return "Fixed"; }
    int equationCount() const override { return 2; }
    std::vector<uint64_t> referencedEntityIds() const override;
    void evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                  int offset) const override;
    void jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                  int offset) const override;
    std::shared_ptr<Constraint> clone() const override;

    bool hasDimensionalValue() const override { return false; }

    const GeometryRef& pointRef() const { return m_pointRef; }
    const math::Vec2& position() const { return m_position; }

private:
    GeometryRef m_pointRef;
    math::Vec2 m_position;
};

/// Distance between two features equals a value (1 equation).
class DistanceConstraint : public Constraint {
public:
    DistanceConstraint(const GeometryRef& refA, const GeometryRef& refB, double distance);
    ConstraintType type() const override { return ConstraintType::Distance; }
    std::string typeName() const override { return "Distance"; }
    int equationCount() const override { return 1; }
    std::vector<uint64_t> referencedEntityIds() const override;
    void evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                  int offset) const override;
    void jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                  int offset) const override;
    std::shared_ptr<Constraint> clone() const override;

    bool hasDimensionalValue() const override { return true; }
    double dimensionalValue() const override { return m_distance; }
    void setDimensionalValue(double v) override { m_distance = v; }

    const GeometryRef& refA() const { return m_refA; }
    const GeometryRef& refB() const { return m_refB; }

private:
    GeometryRef m_refA, m_refB;
    double m_distance;
};

/// Angle between two lines equals a value in radians (1 equation).
class AngleConstraint : public Constraint {
public:
    AngleConstraint(const GeometryRef& lineA, const GeometryRef& lineB, double angleRad);
    ConstraintType type() const override { return ConstraintType::Angle; }
    std::string typeName() const override { return "Angle"; }
    int equationCount() const override { return 1; }
    std::vector<uint64_t> referencedEntityIds() const override;
    void evaluate(const ParameterTable& params, Eigen::VectorXd& residuals,
                  int offset) const override;
    void jacobian(const ParameterTable& params, Eigen::MatrixXd& jac,
                  int offset) const override;
    std::shared_ptr<Constraint> clone() const override;

    bool hasDimensionalValue() const override { return true; }
    double dimensionalValue() const override { return m_angle; }
    void setDimensionalValue(double v) override { m_angle = v; }

    const GeometryRef& lineA() const { return m_lineA; }
    const GeometryRef& lineB() const { return m_lineB; }

private:
    GeometryRef m_lineA, m_lineB;
    double m_angle;
};

}  // namespace hz::cstr
