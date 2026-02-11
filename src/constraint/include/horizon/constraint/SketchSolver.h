#pragma once

#include <Eigen/Dense>
#include <string>

namespace hz::cstr {

class ParameterTable;
class ConstraintSystem;

enum class SolveStatus {
    Success,
    Converged,
    OverConstrained,
    UnderConstrained,
    FailedToConverge,
    Inconsistent,
    NoConstraints,
};

struct SolveResult {
    SolveStatus status = SolveStatus::NoConstraints;
    int iterations = 0;
    double residualNorm = 0.0;
    int degreesOfFreedom = 0;
    std::string message;
};

/// Newton-Raphson constraint solver with Levenberg-Marquardt damping.
class SketchSolver {
public:
    SketchSolver();

    SolveResult solve(ParameterTable& params, const ConstraintSystem& constraints);

    void setMaxIterations(int n) { m_maxIterations = n; }
    void setTolerance(double tol) { m_tolerance = tol; }
    void setDampingFactor(double d) { m_damping = d; }

    int maxIterations() const { return m_maxIterations; }
    double tolerance() const { return m_tolerance; }

private:
    Eigen::VectorXd buildResiduals(const ParameterTable& params,
                                    const ConstraintSystem& constraints) const;
    Eigen::MatrixXd buildJacobian(const ParameterTable& params,
                                   const ConstraintSystem& constraints) const;

    int m_maxIterations = 100;
    double m_tolerance = 1e-10;
    double m_damping = 1.0;
};

}  // namespace hz::cstr
