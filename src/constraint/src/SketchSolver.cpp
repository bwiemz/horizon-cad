#include "horizon/constraint/SketchSolver.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/ParameterTable.h"
#include <Eigen/SVD>
#include <cmath>
#include <set>

namespace hz::cstr {

SketchSolver::SketchSolver() = default;

Eigen::VectorXd SketchSolver::buildResiduals(const ParameterTable& params,
                                              const ConstraintSystem& constraints) const {
    int m = constraints.totalEquations();
    Eigen::VectorXd F = Eigen::VectorXd::Zero(m);
    int offset = 0;
    for (const auto& c : constraints.constraints()) {
        c->evaluate(params, F, offset);
        offset += c->equationCount();
    }
    return F;
}

Eigen::MatrixXd SketchSolver::buildJacobian(const ParameterTable& params,
                                              const ConstraintSystem& constraints) const {
    int m = constraints.totalEquations();
    int n = params.parameterCount();
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(m, n);
    int offset = 0;
    for (const auto& c : constraints.constraints()) {
        c->jacobian(params, J, offset);
        offset += c->equationCount();
    }
    return J;
}

SolveResult SketchSolver::solve(ParameterTable& params,
                                 const ConstraintSystem& constraints) {
    SolveResult result;

    int m = constraints.totalEquations();
    int n = params.parameterCount();

    if (m == 0 || constraints.empty()) {
        result.status = SolveStatus::NoConstraints;
        result.message = "No constraints to solve";
        return result;
    }

    if (n == 0) {
        result.status = SolveStatus::OverConstrained;
        result.message = "No parameters but constraints exist";
        return result;
    }

    for (int iter = 0; iter < m_maxIterations; ++iter) {
        Eigen::VectorXd F = buildResiduals(params, constraints);
        double norm = F.norm();
        result.residualNorm = norm;
        result.iterations = iter + 1;

        if (norm < m_tolerance) {
            // Check degrees of freedom
            Eigen::MatrixXd J = buildJacobian(params, constraints);
            Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(J);
            int rank = qr.rank();
            result.degreesOfFreedom = n - rank;

            if (result.degreesOfFreedom > 0) {
                result.status = SolveStatus::UnderConstrained;
                result.message = "Solved but " + std::to_string(result.degreesOfFreedom) +
                                 " degrees of freedom remain";
            } else {
                result.status = SolveStatus::Success;
                result.message = "All constraints satisfied";
            }
            return result;
        }

        Eigen::MatrixXd J = buildJacobian(params, constraints);

        // Gauss-Newton with Levenberg-Marquardt damping:
        // (J^T J + lambda * I) * dx = -J^T F
        Eigen::MatrixXd JtJ = J.transpose() * J;
        Eigen::VectorXd JtF = J.transpose() * F;

        if (m_damping > 0.0) {
            JtJ.diagonal().array() += m_damping;
        }

        Eigen::VectorXd dx = JtJ.colPivHouseholderQr().solve(-JtF);

        // Update parameters
        params.values() += dx;
    }

    // Did not converge — check if over-constrained
    Eigen::MatrixXd J = buildJacobian(params, constraints);
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(J);
    int rank = qr.rank();
    result.degreesOfFreedom = n - rank;

    if (m > rank) {
        result.status = SolveStatus::OverConstrained;
        result.message =
            "Over-constrained: " + std::to_string(m) + " equations, rank " +
            std::to_string(rank);
    } else if (result.residualNorm > m_tolerance * 100.0) {
        result.status = SolveStatus::Inconsistent;
        result.message =
            "Inconsistent constraints (residual = " + std::to_string(result.residualNorm) + ")";
    } else {
        result.status = SolveStatus::FailedToConverge;
        result.message = "Failed to converge after " + std::to_string(m_maxIterations) +
                         " iterations (residual = " + std::to_string(result.residualNorm) + ")";
    }
    return result;
}

DOFAnalysis SketchSolver::analyzeDOF(const ParameterTable& params,
                                     const ConstraintSystem& constraints) const {
    DOFAnalysis result;

    int m = constraints.totalEquations();
    int n = params.parameterCount();

    if (m == 0 || constraints.empty() || n == 0) {
        // No constraints — all constrained entities are free (none exist).
        return result;
    }

    // Build Jacobian and compute rank via SVD.
    Eigen::MatrixXd J = buildJacobian(params, constraints);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);

    // Count singular values above threshold for rank determination.
    double threshold = 1e-8 * std::max(m, n) * svd.singularValues()(0);
    int rank = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > threshold) ++rank;
    }

    result.totalDOF = n - rank;
    bool overConstrained = (m > rank);

    // Collect all entity IDs referenced by any constraint.
    std::set<uint64_t> constrainedIds;
    for (const auto& c : constraints.constraints()) {
        for (uint64_t eid : c->referencedEntityIds()) {
            constrainedIds.insert(eid);
        }
    }

    // Assign status per entity based on global analysis.
    for (uint64_t eid : constrainedIds) {
        if (overConstrained) {
            result.entityStatus[eid] = EntityDOFStatus::OverConstrained;
        } else if (result.totalDOF == 0) {
            result.entityStatus[eid] = EntityDOFStatus::FullyConstrained;
        } else {
            result.entityStatus[eid] = EntityDOFStatus::Free;
        }
    }

    return result;
}

}  // namespace hz::cstr
