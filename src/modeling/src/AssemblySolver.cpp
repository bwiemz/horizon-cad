#include "horizon/modeling/AssemblySolver.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <unordered_map>

#include "horizon/math/Quaternion.h"

namespace hz::model {

using hz::math::Mat4;
using hz::math::Quaternion;
using hz::math::Vec3;

namespace {

// Placement from the 6 unknowns of one component: an incremental rigid
// motion (rotation about the component's pivot, then translation) applied
// on top of the base transform. Rotating about a pivot near the part's
// mated geometry decouples rotation from translation, which conditions
// the Newton iteration far better than rotating about the world origin.
Mat4 placement(const Mat4& base, const Vec3& pivot, const double* x) {
    const Vec3 t(x[0], x[1], x[2]);
    const Vec3 w(x[3], x[4], x[5]);
    const double angle = w.length();
    Mat4 rot = angle < 1e-14 ? Mat4::identity()
                             : Mat4::rotation(Quaternion::fromAxisAngle(w * (1.0 / angle), angle));
    return Mat4::translation(t + pivot) * rot * Mat4::translation(pivot * -1.0) * base;
}

struct ResidualBlock {
    size_t mateIndex = 0;
    int rows = 0;          ///< Residual rows emitted.
    int expectedRank = 0;  ///< Constrained DOF this mate should contribute.
};

// Emit residual rows for one mate, given the placed (world) frames.
void mateResiduals(const SolverMate& mate, const MateFrame& a, const MateFrame& b,
                   std::vector<double>& out) {
    const Vec3 d = b.origin - a.origin;
    switch (mate.type) {
        case MateType::Coincident: {
            // Normals parallel up to sign (extracted surface normals carry
            // no guaranteed orientation) + point on plane. The sign-agnostic
            // cross form also avoids the saddle at a 180° misalignment.
            const Vec3 cross = a.direction.cross(b.direction);
            out.push_back(cross.x);
            out.push_back(cross.y);
            out.push_back(cross.z);
            out.push_back(d.dot(a.direction));
            break;
        }
        case MateType::Distance: {
            const Vec3 cross = a.direction.cross(b.direction);
            out.push_back(cross.x);
            out.push_back(cross.y);
            out.push_back(cross.z);
            // Offset measured along A's normal.
            out.push_back(d.dot(a.direction) - mate.value);
            break;
        }
        case MateType::Concentric: {
            const Vec3 cross = a.direction.cross(b.direction);
            out.push_back(cross.x);
            out.push_back(cross.y);
            out.push_back(cross.z);
            const Vec3 perp = d - a.direction * d.dot(a.direction);
            out.push_back(perp.x);
            out.push_back(perp.y);
            out.push_back(perp.z);
            break;
        }
        case MateType::Angle:
            out.push_back(a.direction.dot(b.direction) - std::cos(mate.value));
            break;
        case MateType::Parallel: {
            const Vec3 cross = a.direction.cross(b.direction);
            out.push_back(cross.x);
            out.push_back(cross.y);
            out.push_back(cross.z);
            break;
        }
        case MateType::Perpendicular:
            out.push_back(a.direction.dot(b.direction));
            break;
        case MateType::Tangent:
            // Plane (a) tangent to cylinder (b): axis parallel to the plane,
            // axis at distance radius from the plane.
            out.push_back(a.direction.dot(b.direction));
            out.push_back(d.dot(a.direction) - b.radius);
            break;
        case MateType::Fixed:
            break;  // Handled by grounding, no equations.
    }
}

int expectedRankFor(MateType type) {
    switch (type) {
        case MateType::Coincident:
            return 3;
        case MateType::Distance:
            return 3;
        case MateType::Concentric:
            return 4;
        case MateType::Angle:
            return 1;
        case MateType::Parallel:
            return 2;
        case MateType::Perpendicular:
            return 1;
        case MateType::Tangent:
            return 2;
        case MateType::Fixed:
            return 0;
    }
    return 0;
}

}  // namespace

AssemblySolveResult AssemblySolver::solve(const std::vector<SolverComponent>& components,
                                          const std::vector<SolverMate>& mates) const {
    AssemblySolveResult result;

    // Component lookup + initial transforms.
    std::unordered_map<uint64_t, size_t> componentIndex;
    for (size_t i = 0; i < components.size(); ++i) {
        componentIndex[components[i].id] = i;
        result.transforms[components[i].id] = components[i].transform;
    }

    // --- Kinematic pre-analysis -------------------------------------------

    std::vector<bool> grounded(components.size(), false);
    for (size_t i = 0; i < components.size(); ++i) {
        grounded[i] = components[i].grounded;
    }
    for (const auto& mate : mates) {
        if (mate.type != MateType::Fixed) continue;
        auto it = componentIndex.find(mate.componentA);
        if (it == componentIndex.end()) {
            result.status = AssemblySolveStatus::InvalidReference;
            result.message = "Fixed mate references unknown component";
            return result;
        }
        grounded[it->second] = true;
    }
    if (!components.empty() &&
        std::none_of(grounded.begin(), grounded.end(), [](bool g) { return g; })) {
        grounded[0] = true;
        result.message = "no Fixed mate: grounding first component by convention";
    }

    // Validate references and build the mate graph.
    std::vector<std::vector<size_t>> adjacency(components.size());
    std::vector<SolverMate> activeMates;
    for (const auto& mate : mates) {
        if (mate.type == MateType::Fixed) continue;
        auto ia = componentIndex.find(mate.componentA);
        auto ib = componentIndex.find(mate.componentB);
        if (ia == componentIndex.end() || ib == componentIndex.end()) {
            result.status = AssemblySolveStatus::InvalidReference;
            result.message = "mate references unknown component";
            return result;
        }
        adjacency[ia->second].push_back(ib->second);
        adjacency[ib->second].push_back(ia->second);
        activeMates.push_back(mate);
    }

    // Reachability from grounded components.
    std::vector<bool> reachable(components.size(), false);
    std::queue<size_t> frontier;
    for (size_t i = 0; i < components.size(); ++i) {
        if (grounded[i]) {
            reachable[i] = true;
            frontier.push(i);
        }
    }
    while (!frontier.empty()) {
        size_t i = frontier.front();
        frontier.pop();
        for (size_t j : adjacency[i]) {
            if (!reachable[j]) {
                reachable[j] = true;
                frontier.push(j);
            }
        }
    }
    for (size_t i = 0; i < components.size(); ++i) {
        if (!reachable[i]) {
            result.ungroundedComponents.push_back(components[i].id);
        }
    }

    if (activeMates.empty()) {
        result.status = AssemblySolveStatus::NoMates;
        for (size_t i = 0; i < components.size(); ++i) {
            result.componentDOF[components[i].id] = grounded[i] ? 0 : 6;
            if (!grounded[i]) result.remainingDOF += 6;
        }
        return result;
    }

    // --- Unknown layout: 6 per free component ------------------------------

    std::vector<size_t> freeComponents;  // indices into `components`
    std::unordered_map<uint64_t, size_t> unknownOffset;
    for (size_t i = 0; i < components.size(); ++i) {
        if (!grounded[i]) {
            unknownOffset[components[i].id] = freeComponents.size() * 6;
            freeComponents.push_back(i);
        }
    }
    const auto numUnknowns = static_cast<Eigen::Index>(freeComponents.size() * 6);

    // Rotation pivot per component: the centroid of its mate-frame origins
    // at the initial placement (falls back to the base translation).
    std::unordered_map<uint64_t, Vec3> pivots;
    std::unordered_map<uint64_t, int> pivotCounts;
    for (const auto& mate : activeMates) {
        const size_t ia = componentIndex.at(mate.componentA);
        const size_t ib = componentIndex.at(mate.componentB);
        Vec3 originA = mate.frameA.transformed(components[ia].transform).origin;
        Vec3 originB = mate.frameB.transformed(components[ib].transform).origin;
        pivots[mate.componentA] = pivots[mate.componentA] + originA;
        pivotCounts[mate.componentA] += 1;
        pivots[mate.componentB] = pivots[mate.componentB] + originB;
        pivotCounts[mate.componentB] += 1;
    }
    for (const auto& comp : components) {
        auto it = pivotCounts.find(comp.id);
        if (it != pivotCounts.end() && it->second > 0) {
            pivots[comp.id] = pivots[comp.id] * (1.0 / it->second);
        } else {
            pivots[comp.id] =
                Vec3(comp.transform.at(0, 3), comp.transform.at(1, 3), comp.transform.at(2, 3));
        }
    }

    // Residual evaluation for the full system at unknown vector x.
    std::vector<ResidualBlock> blocks;
    auto evaluate = [&](const Eigen::VectorXd& x, Eigen::VectorXd& residuals, bool recordBlocks) {
        std::vector<double> values;
        if (recordBlocks) blocks.clear();
        for (size_t mi = 0; mi < activeMates.size(); ++mi) {
            const auto& mate = activeMates[mi];
            const size_t ia = componentIndex.at(mate.componentA);
            const size_t ib = componentIndex.at(mate.componentB);

            auto placed = [&](size_t ci) {
                if (grounded[ci]) return components[ci].transform;
                const size_t off = unknownOffset.at(components[ci].id);
                return placement(components[ci].transform, pivots.at(components[ci].id),
                                 x.data() + off);
            };

            MateFrame a = mate.frameA.transformed(placed(ia));
            MateFrame b = mate.frameB.transformed(placed(ib));

            const size_t before = values.size();
            mateResiduals(mate, a, b, values);
            if (recordBlocks) {
                blocks.push_back(
                    {mi, static_cast<int>(values.size() - before), expectedRankFor(mate.type)});
            }
        }
        residuals =
            Eigen::Map<Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
    };

    Eigen::VectorXd x = Eigen::VectorXd::Zero(numUnknowns);
    Eigen::VectorXd residuals;
    evaluate(x, residuals, /*recordBlocks=*/true);

    // --- Newton-Raphson with LM damping ------------------------------------

    const auto numResiduals = static_cast<Eigen::Index>(residuals.size());
    double lambda = 1e-4;
    double residualNorm = residuals.norm();

    Eigen::MatrixXd jacobian(numResiduals, numUnknowns);
    int iteration = 0;
    for (; iteration < m_maxIterations; ++iteration) {
        if (residuals.lpNorm<Eigen::Infinity>() < m_tolerance) break;
        if (numUnknowns == 0) break;

        // Numerical Jacobian (forward differences).
        constexpr double kStep = 1e-7;
        Eigen::VectorXd perturbed;
        for (Eigen::Index c = 0; c < numUnknowns; ++c) {
            Eigen::VectorXd xp = x;
            xp(c) += kStep;
            evaluate(xp, perturbed, false);
            jacobian.col(c) = (perturbed - residuals) / kStep;
        }

        // LM step: (JᵀJ + λ diag) dx = -JᵀF
        Eigen::MatrixXd jtj = jacobian.transpose() * jacobian;
        Eigen::VectorXd jtf = jacobian.transpose() * residuals;
        for (int attempt = 0; attempt < 8; ++attempt) {
            Eigen::MatrixXd damped = jtj;
            damped.diagonal().array() += lambda;
            Eigen::VectorXd dx = damped.ldlt().solve(-jtf);

            Eigen::VectorXd xNew = x + dx;
            Eigen::VectorXd rNew;
            evaluate(xNew, rNew, false);
            if (rNew.norm() < residualNorm) {
                x = xNew;
                residuals = rNew;
                residualNorm = rNew.norm();
                lambda = std::max(lambda * 0.3, 1e-12);
                break;
            }
            lambda *= 10.0;
        }
    }

    // --- Rank analysis (redundancy + DOF) -----------------------------------

    if (numUnknowns > 0) {
        constexpr double kStep = 1e-7;
        Eigen::VectorXd perturbed;
        for (Eigen::Index c = 0; c < numUnknowns; ++c) {
            Eigen::VectorXd xp = x;
            xp(c) += kStep;
            evaluate(xp, perturbed, false);
            jacobian.col(c) = (perturbed - residuals) / kStep;
        }
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(jacobian);
        qr.setThreshold(1e-8);
        const auto rank = static_cast<int>(qr.rank());

        int expectedConstrained = 0;
        for (const auto& block : blocks) expectedConstrained += block.expectedRank;
        result.redundantCount = std::max(0, expectedConstrained - rank);
        result.remainingDOF = static_cast<int>(numUnknowns) - rank;

        // Per-component DOF: rank of the Jacobian restricted to that
        // component's 6 columns.
        for (size_t fi = 0; fi < freeComponents.size(); ++fi) {
            Eigen::MatrixXd sub = jacobian.middleCols(static_cast<Eigen::Index>(fi * 6), 6);
            Eigen::ColPivHouseholderQR<Eigen::MatrixXd> subQr(sub);
            subQr.setThreshold(1e-8);
            result.componentDOF[components[freeComponents[fi]].id] =
                6 - static_cast<int>(subQr.rank());
        }
    }
    for (size_t i = 0; i < components.size(); ++i) {
        if (grounded[i]) result.componentDOF[components[i].id] = 0;
    }

    // --- Write back placements ----------------------------------------------

    for (size_t fi = 0; fi < freeComponents.size(); ++fi) {
        const auto& comp = components[freeComponents[fi]];
        result.transforms[comp.id] =
            placement(comp.transform, pivots.at(comp.id), x.data() + fi * 6);
    }

    result.iterations = iteration;
    result.residualNorm = residualNorm;
    result.status = residuals.size() == 0 || residuals.lpNorm<Eigen::Infinity>() < m_tolerance * 10
                        ? AssemblySolveStatus::Success
                        : AssemblySolveStatus::NotConverged;
    return result;
}

}  // namespace hz::model
