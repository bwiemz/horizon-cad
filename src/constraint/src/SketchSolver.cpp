#include "horizon/constraint/SketchSolver.h"

#include <Eigen/SVD>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <set>
#include <utility>
#include <vector>

#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/ParameterTable.h"

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

SolveResult SketchSolver::solve(ParameterTable& params, const ConstraintSystem& constraints) {
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

    // Use a local copy of damping so each solve() starts from the configured value.
    double damping = m_damping;

    for (int iter = 0; iter < m_maxIterations; ++iter) {
        Eigen::VectorXd F = buildResiduals(params, constraints);
        double currentNorm = F.norm();
        result.residualNorm = currentNorm;
        result.iterations = iter + 1;

        if (currentNorm < m_tolerance) {
            // Check degrees of freedom
            Eigen::MatrixXd J = buildJacobian(params, constraints);
            Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(J);
            const int rank = static_cast<int>(qr.rank());
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

        if (damping > 0.0) {
            JtJ.diagonal().array() += damping;
        }

        Eigen::VectorXd dx = JtJ.colPivHouseholderQr().solve(-JtF);

        // Save current state and try the step
        Eigen::VectorXd savedParams = params.values();
        params.values() = savedParams + dx;

        // Evaluate new residuals to check step quality
        Eigen::VectorXd F_new = buildResiduals(params, constraints);
        double newNorm = F_new.norm();

        if (newNorm < currentNorm) {
            // Good step — accept and decrease damping
            damping = std::max(damping / 10.0, 1e-12);
        } else {
            // Bad step — reject, increase damping, retry without advancing iteration
            params.values() = savedParams;
            damping *= 10.0;
            if (damping > 1e10) {
                // Damping too high — solver is stuck
                break;
            }
            --iter;  // Don't count rejected steps
            continue;
        }
    }

    // Did not converge — check if over-constrained
    Eigen::MatrixXd J = buildJacobian(params, constraints);
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(J);
    const int rank = static_cast<int>(qr.rank());
    result.degreesOfFreedom = n - rank;

    if (m > rank) {
        result.status = SolveStatus::OverConstrained;
        result.message =
            "Over-constrained: " + std::to_string(m) + " equations, rank " + std::to_string(rank);
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

namespace {

/// Union-find over parameter indices.
class Clusters {
public:
    explicit Clusters(int n) : m_parent(static_cast<size_t>(n)) {
        std::iota(m_parent.begin(), m_parent.end(), 0);
    }
    int find(int i) {
        while (m_parent[static_cast<size_t>(i)] != i) {
            auto& p = m_parent[static_cast<size_t>(i)];
            p = m_parent[static_cast<size_t>(p)];  // halve the path
            i = p;
        }
        return i;
    }
    void join(int a, int b) { m_parent[static_cast<size_t>(find(a))] = find(b); }

private:
    std::vector<int> m_parent;
};

/// Up to this many parameters, a cluster's rank comes from a dense SVD;
/// above it, from a sparse QR.
constexpr int kDenseClusterParameters = 96;

/// The rank of the @p rows x @p cols matrix of @p entries (local indices).
int rankOf(int rows, int cols, const std::vector<Eigen::Triplet<double>>& entries) {
    if (rows == 0 || cols == 0 || entries.empty()) return 0;
    if (cols <= kDenseClusterParameters) {
        Eigen::MatrixXd block = Eigen::MatrixXd::Zero(rows, cols);
        for (const auto& e : entries) block(e.row(), e.col()) += e.value();
        const Eigen::JacobiSVD<Eigen::MatrixXd> svd(block);
        const auto& sigma = svd.singularValues();
        if (sigma.size() == 0 || sigma(0) <= 0.0) return 0;
        const double threshold = 1e-8 * std::max(rows, cols) * sigma(0);
        int rank = 0;
        for (int i = 0; i < sigma.size(); ++i) {
            if (sigma(i) > threshold) ++rank;
        }
        return rank;
    }
    Eigen::SparseMatrix<double> block(rows, cols);
    block.setFromTriplets(entries.begin(), entries.end());
    block.makeCompressed();
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr;
    qr.compute(block);
    if (qr.info() != Eigen::Success) return 0;
    return static_cast<int>(qr.rank());
}

}  // namespace

DOFAnalysis SketchSolver::analyzeDOF(const ParameterTable& params,
                                     const ConstraintSystem& constraints) const {
    DOFAnalysis result;
    const int n = params.parameterCount();
    if (constraints.empty() || n == 0 || constraints.totalEquations() == 0) return result;

    // The Jacobian is sparse: an equation touches a few parameters, of the
    // entities its constraint names. It is taken a constraint at a time, in
    // those columns only, never as the whole m x n matrix, which for a large
    // sketch was hundreds of megabytes and a dense SVD of it seconds.
    const auto& all = constraints.constraints();
    int widest = 1;
    for (const auto& c : all) widest = std::max(widest, c->equationCount());
    Eigen::MatrixXd scratch = Eigen::MatrixXd::Zero(widest, n);

    /// An equation: its non-zeros (global columns), and the constraint it is
    /// of. One with none cannot be met.
    struct Equation {
        size_t constraint = 0;
        std::vector<std::pair<int, double>> entries;
    };
    std::vector<Equation> equations;
    equations.reserve(static_cast<size_t>(constraints.totalEquations()));
    std::vector<std::vector<uint64_t>> entitiesOf(all.size());
    Clusters clusters(n);
    for (size_t k = 0; k < all.size(); ++k) {
        const auto& c = *all[k];
        const int rows = c.equationCount();
        if (rows <= 0) continue;
        // Each entity once: one named twice (a line's two ends) would have
        // its columns read twice.
        auto& entities = entitiesOf[k];
        for (const uint64_t id : c.referencedEntityIds()) {
            if (std::find(entities.begin(), entities.end(), id) == entities.end()) {
                entities.push_back(id);
            }
        }
        std::vector<std::pair<int, int>> ranges;
        bool usable = true;
        for (const uint64_t id : entities) {
            const auto range = params.parameterRange(id);
            usable = usable && range.second > 0;
            ranges.push_back(range);
        }
        const size_t firstRow = equations.size();
        for (int r = 0; r < rows; ++r) equations.push_back(Equation{k, {}});
        // A constraint on something not in the table cannot be met: its
        // equations stay empty.
        if (!usable) continue;
        for (const auto& [first, count] : ranges) {
            scratch.block(0, first, rows, count).setZero();
        }
        c.jacobian(params, scratch, 0);
        for (const auto& [first, count] : ranges) {
            for (int col = first; col < first + count; ++col) {
                for (int r = 0; r < rows; ++r) {
                    const double v = scratch(r, col);
                    if (v != 0.0)
                        equations[firstRow + static_cast<size_t>(r)].entries.emplace_back(col, v);
                }
            }
            scratch.block(0, first, rows, count).setZero();
        }
    }
    // Parameters an equation ties are in one cluster; the clusters are
    // analysed apart. By equation, not by constraint: a coincidence's x and
    // y are two clusters, and a chain of lines falls into many small ones.
    std::vector<bool> tied(static_cast<size_t>(n), false);
    for (const Equation& e : equations) {
        for (const auto& [col, v] : e.entries) {
            tied[static_cast<size_t>(col)] = true;
            clusters.join(e.entries.front().first, col);
        }
    }

    // Each cluster alone: its rank, whether it has more equations than that
    // (over-constrained), and the freedom left in it.
    struct Cluster {
        int rows = 0;
        std::map<int, int> local;  ///< global column -> its own
        std::vector<Eigen::Triplet<double>> entries;
        bool over = false;
        int freedom = 0;
    };
    std::map<int, Cluster> byRoot;
    std::vector<int> rootOf(equations.size(), -1);  // -1: nothing to meet it with
    for (size_t i = 0; i < equations.size(); ++i) {
        const Equation& e = equations[i];
        if (e.entries.empty()) continue;
        const int root = clusters.find(e.entries.front().first);
        rootOf[i] = root;
        Cluster& cluster = byRoot[root];
        for (const auto& [col, v] : e.entries) {
            const auto added = cluster.local.emplace(col, static_cast<int>(cluster.local.size()));
            cluster.entries.emplace_back(cluster.rows, added.first->second, v);
        }
        ++cluster.rows;
    }
    int rankTotal = 0;
    for (auto& [root, cluster] : byRoot) {
        const int cols = static_cast<int>(cluster.local.size());
        const int rank = rankOf(cluster.rows, cols, cluster.entries);
        rankTotal += rank;
        cluster.over = cluster.rows > rank;
        cluster.freedom = cols - rank;
    }
    result.totalDOF = n - rankTotal;

    // An entity is over-constrained when an equation of a constraint on it
    // is in an over-constrained cluster, or cannot be met; free when a
    // parameter of it is tied by nothing, or is in a cluster with freedom
    // left; fully constrained otherwise.
    std::vector<bool> overConstraint(all.size(), false);
    for (size_t i = 0; i < equations.size(); ++i) {
        if (rootOf[i] < 0 || byRoot[rootOf[i]].over) overConstraint[equations[i].constraint] = true;
    }
    std::map<uint64_t, EntityDOFStatus> status;
    for (size_t k = 0; k < all.size(); ++k) {
        for (const uint64_t id : entitiesOf[k]) {
            auto it = status.emplace(id, EntityDOFStatus::FullyConstrained).first;
            if (overConstraint[k]) it->second = EntityDOFStatus::OverConstrained;
        }
    }
    for (auto& [id, s] : status) {
        if (s == EntityDOFStatus::OverConstrained) continue;
        const auto [first, count] = params.parameterRange(id);
        for (int p = first; p < first + count; ++p) {
            if (!tied[static_cast<size_t>(p)] || byRoot[clusters.find(p)].freedom > 0) {
                s = EntityDOFStatus::Free;
                break;
            }
        }
    }
    for (const auto& [id, s] : status) result.entityStatus[id] = s;
    return result;
}

}  // namespace hz::cstr
