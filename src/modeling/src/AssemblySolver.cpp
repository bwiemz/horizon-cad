#include "horizon/modeling/AssemblySolver.h"

#include <Eigen/Dense>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>

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

/// What a frame is to a mate (Phase 160): a plane; an axis (a cylinder's,
/// a cone's, a straight edge or datum axis); a circle (its plane and its
/// centre); a point (a datum point, a sphere's centre).
enum class Role { Plane, Axis, Circle, Point };

Role roleOf(MateFrameKind kind) {
    switch (kind) {
        case MateFrameKind::Planar:
            return Role::Plane;
        case MateFrameKind::Cylindrical:
        case MateFrameKind::Line:
        case MateFrameKind::Conical:
            return Role::Axis;
        case MateFrameKind::Circle:
            return Role::Circle;
        case MateFrameKind::Point:
        case MateFrameKind::Spherical:
            return Role::Point;
    }
    return Role::Point;
}

/// The equations one mate makes of its two placed frames (Phase 160): the
/// rows it always holds at zero, and, for a Distance or an Angle mate, the
/// value it measures, which is held at its value, or kept between its
/// limits. Nullopt for kinds the mate does not relate (a Concentric of two
/// planes).
struct Equations {
    std::vector<double> rows;
    int rank = 0;                   ///< the degrees of freedom the rows take
    std::optional<double> measure;  ///< Distance and Angle
};

void push(std::vector<double>& rows, const Vec3& v) {
    rows.push_back(v.x);
    rows.push_back(v.y);
    rows.push_back(v.z);
}

std::optional<Equations> equationsFor(const SolverMate& mate, const MateFrame& first,
                                      const MateFrame& second) {
    Role ra = roleOf(first.kind);
    Role rb = roleOf(second.kind);
    // One order for each pair: plane, circle, axis, point. The frames are
    // swapped to it; d runs from the first to the second as given.
    const MateFrame* a = &first;
    const MateFrame* b = &second;
    const auto order = [](Role r) {
        switch (r) {
            case Role::Plane:
                return 0;
            case Role::Circle:
                return 1;
            case Role::Axis:
                return 2;
            case Role::Point:
                return 3;
        }
        return 3;
    };
    if (order(rb) < order(ra)) {
        std::swap(a, b);
        std::swap(ra, rb);
    }
    const Vec3 d = b->origin - a->origin;
    const Vec3 cross = a->direction.cross(b->direction);
    const Vec3 perp = d - a->direction * d.dot(a->direction);  // off a's axis
    const bool directed = rb != Role::Point;                   // both have a direction
    Equations eq;
    const auto is = [&](Role x, Role y) { return ra == x && rb == y; };
    const bool planeLike = is(Role::Plane, Role::Plane) || is(Role::Plane, Role::Circle);
    const bool axial =
        (ra == Role::Axis || ra == Role::Circle) && (rb == Role::Axis || rb == Role::Circle);

    switch (mate.type) {
        case MateType::Coincident:
            if (planeLike) {
                // Normals parallel up to sign (extracted surface normals carry
                // no guaranteed orientation) + point on plane. The sign-agnostic
                // cross form also avoids the saddle at a 180° misalignment.
                push(eq.rows, cross);
                eq.rows.push_back(d.dot(a->direction));
                eq.rank = 3;
            } else if (is(Role::Plane, Role::Axis)) {  // the axis in the plane
                eq.rows.push_back(a->direction.dot(b->direction));
                eq.rows.push_back(d.dot(a->direction));
                eq.rank = 2;
            } else if (is(Role::Plane, Role::Point)) {  // the point on the plane
                eq.rows.push_back(d.dot(a->direction));
                eq.rank = 1;
            } else if (is(Role::Circle, Role::Circle)) {  // one circle on the other
                push(eq.rows, cross);
                push(eq.rows, d);
                eq.rank = 5;
            } else if (axial) {  // collinear
                push(eq.rows, cross);
                push(eq.rows, perp);
                eq.rank = 4;
            } else if (is(Role::Circle, Role::Point) || is(Role::Point, Role::Point)) {
                push(eq.rows, d);  // at its centre; on it
                eq.rank = 3;
            } else if (is(Role::Axis, Role::Point)) {  // the point on the axis
                push(eq.rows, perp);
                eq.rank = 2;
            } else {
                return std::nullopt;
            }
            return eq;
        case MateType::Concentric:
            if (axial) {
                push(eq.rows, cross);
                push(eq.rows, perp);
                eq.rank = 4;
            } else if (is(Role::Axis, Role::Point) || is(Role::Circle, Role::Point)) {
                push(eq.rows, perp);  // a sphere's centre on the axis
                eq.rank = 2;
            } else if (is(Role::Point, Role::Point)) {
                push(eq.rows, d);
                eq.rank = 3;
            } else {
                return std::nullopt;
            }
            return eq;
        case MateType::Distance:
            if (planeLike) {
                push(eq.rows, cross);
                eq.rank = 2;
                eq.measure = d.dot(a->direction);      // offset along the first's normal
            } else if (is(Role::Plane, Role::Axis)) {  // the axis parallel to the plane
                eq.rows.push_back(a->direction.dot(b->direction));
                eq.rank = 1;
                eq.measure = d.dot(a->direction);
            } else if (is(Role::Plane, Role::Point)) {
                eq.measure = d.dot(a->direction);
            } else if (axial) {  // parallel axes, apart
                push(eq.rows, cross);
                eq.rank = 2;
                eq.measure = perp.length();
            } else if (is(Role::Axis, Role::Point) || is(Role::Circle, Role::Point)) {
                eq.measure = perp.length();
            } else if (is(Role::Point, Role::Point)) {
                eq.measure = d.length();
            } else {
                return std::nullopt;
            }
            return eq;
        case MateType::Angle:
            if (!directed) return std::nullopt;
            // Measured with atan2: steady at 0 and 180 degrees, where the
            // cosine's slope is none.
            eq.measure = std::atan2(cross.length(), a->direction.dot(b->direction));
            return eq;
        case MateType::Parallel:
            if (!directed) return std::nullopt;
            push(eq.rows, cross);
            eq.rank = 2;
            return eq;
        case MateType::Perpendicular:
            if (!directed) return std::nullopt;
            eq.rows.push_back(a->direction.dot(b->direction));
            eq.rank = 1;
            return eq;
        case MateType::Tangent:
            // Plane (a) tangent to cylinder (b): axis parallel to the plane,
            // axis at distance radius from the plane.
            if (is(Role::Plane, Role::Axis) && b->kind == MateFrameKind::Cylindrical) {
                eq.rows.push_back(a->direction.dot(b->direction));
                eq.rows.push_back(d.dot(a->direction) - b->radius);
                eq.rank = 2;
            } else if (is(Role::Plane, Role::Point) && b->kind == MateFrameKind::Spherical) {
                eq.rows.push_back(std::abs(d.dot(a->direction)) - b->radius);  // Phase 160
                eq.rank = 1;
            } else {
                return std::nullopt;
            }
            return eq;
        case MateType::Fixed:
            return eq;  // Handled by grounding, no equations.
    }
    return std::nullopt;
}

/// Mate @p mate's residual rows at its placed frames, with its measure held
/// at @p target when it has one (a limit mate between its limits holds none).
void mateResiduals(const SolverMate& mate, const MateFrame& a, const MateFrame& b,
                   const std::optional<double>& target, std::vector<double>& out, int* rank) {
    const auto eq = equationsFor(mate, a, b);
    if (!eq) return;  // refused before the solve
    out.insert(out.end(), eq->rows.begin(), eq->rows.end());
    int taken = eq->rank;
    if (eq->measure && target) {
        // An angle held at 0 or 180 degrees is the directions parallel: two
        // freedoms, not one, and the angle's slope there is none.
        const bool parallel =
            mate.type == MateType::Angle &&
            (std::abs(*target) < 1e-9 || std::abs(*target - std::numbers::pi) < 1e-9);
        if (parallel) {
            const Vec3 cross = a.direction.cross(b.direction);
            out.insert(out.end(), {cross.x, cross.y, cross.z});
            taken += 2;
        } else {
            out.push_back(*eq->measure - *target);
            ++taken;
        }
    }
    if (rank != nullptr) *rank = taken;
}

std::string kindName(MateFrameKind kind) {
    switch (kind) {
        case MateFrameKind::Planar:
            return "a plane";
        case MateFrameKind::Cylindrical:
            return "a cylinder";
        case MateFrameKind::Line:
            return "a line";
        case MateFrameKind::Point:
            return "a point";
        case MateFrameKind::Circle:
            return "a circle";
        case MateFrameKind::Spherical:
            return "a sphere";
        case MateFrameKind::Conical:
            return "a cone";
    }
    return "something";
}

std::string typeName(MateType type) {
    switch (type) {
        case MateType::Coincident:
            return "Coincident";
        case MateType::Concentric:
            return "Concentric";
        case MateType::Distance:
            return "Distance";
        case MateType::Angle:
            return "Angle";
        case MateType::Parallel:
            return "Parallel";
        case MateType::Perpendicular:
            return "Perpendicular";
        case MateType::Tangent:
            return "Tangent";
        case MateType::Fixed:
            return "Fixed";
    }
    return "A";
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

    // What each mate relates (Phase 160): a pair of kinds its type makes no
    // equations of is refused, and so are limits on a mate that measures
    // nothing.
    for (const auto& mate : activeMates) {
        if (!equationsFor(mate, mate.frameA, mate.frameB)) {
            result.status = AssemblySolveStatus::InvalidReference;
            result.message = typeName(mate.type) + " between " + kindName(mate.frameA.kind) +
                             " and " + kindName(mate.frameB.kind) + " is not a mate";
            return result;
        }
        if (mate.limited() && mate.type != MateType::Distance && mate.type != MateType::Angle) {
            result.status = AssemblySolveStatus::InvalidReference;
            result.message = "only a Distance or an Angle mate has limits";
            return result;
        }
    }
    // The value each measuring mate holds: its own, or, for one with limits,
    // none until the solve finds it past one (below).
    std::vector<std::optional<double>> targets(activeMates.size());
    for (size_t mi = 0; mi < activeMates.size(); ++mi) {
        const auto& mate = activeMates[mi];
        if ((mate.type == MateType::Distance || mate.type == MateType::Angle) && !mate.limited()) {
            targets[mi] = mate.value;
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
            int rank = 0;
            mateResiduals(mate, a, b, targets[mi], values, &rank);
            if (recordBlocks) {
                blocks.push_back({mi, static_cast<int>(values.size() - before), rank});
            }
        }
        residuals =
            Eigen::Map<Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
    };

    Eigen::VectorXd x = Eigen::VectorXd::Zero(numUnknowns);
    Eigen::VectorXd residuals;
    Eigen::Index numResiduals = 0;
    constexpr double kStep = 1e-7;

    // Row offset of each mate's residual block, for assembling the sparse
    // Jacobian by mate. Worked out again each round: a limit pinned adds a
    // row.
    std::vector<int> mateRowStart(activeMates.size(), 0);
    const auto prepare = [&] {
        evaluate(x, residuals, /*recordBlocks=*/true);
        numResiduals = static_cast<Eigen::Index>(residuals.size());
        for (size_t mi = 1; mi < activeMates.size(); ++mi) {
            mateRowStart[mi] = mateRowStart[mi - 1] + blocks[mi - 1].rows;
        }
    };
    prepare();

    // Placement of one component, and one mate's residual rows, at a given
    // unknown vector — the per-mate primitives the sparse Jacobian needs.
    auto placedAt = [&](size_t ci, const Eigen::VectorXd& xv) -> Mat4 {
        if (grounded[ci]) return components[ci].transform;
        const size_t off = unknownOffset.at(components[ci].id);
        return placement(components[ci].transform, pivots.at(components[ci].id), xv.data() + off);
    };
    auto evalMate = [&](size_t mi, const Eigen::VectorXd& xv, std::vector<double>& out) {
        out.clear();
        const auto& mate = activeMates[mi];
        MateFrame a = mate.frameA.transformed(placedAt(componentIndex.at(mate.componentA), xv));
        MateFrame b = mate.frameB.transformed(placedAt(componentIndex.at(mate.componentB), xv));
        mateResiduals(mate, a, b, targets[mi], out, nullptr);
    };

    // --- Newton-Raphson with LM damping (block-sparse) ---------------------
    //
    // A mate's residual rows depend only on the 12 columns of its two
    // components, so the Jacobian is assembled per mate (perturbing in place to
    // avoid per-column vector copies) and the normal equations are solved with
    // a sparse Cholesky. The solve is then near-linear in the component count
    // rather than the dense O(unknowns^3).

    Eigen::VectorXd xw = x;  // scratch for in-place differencing
    std::vector<double> baseM, pertM;
    auto buildJacobian = [&](const Eigen::VectorXd& xEval) {
        std::vector<Eigen::Triplet<double>> trips;
        trips.reserve(static_cast<size_t>(numResiduals) * 12);
        xw = xEval;
        for (size_t mi = 0; mi < activeMates.size(); ++mi) {
            evalMate(mi, xEval, baseM);
            const int rows = static_cast<int>(baseM.size());
            const int rowStart = mateRowStart[mi];
            const auto& mate = activeMates[mi];
            for (uint64_t cid : {mate.componentA, mate.componentB}) {
                const size_t ci = componentIndex.at(cid);
                if (grounded[ci]) continue;
                const auto off = static_cast<Eigen::Index>(unknownOffset.at(cid));
                for (int k = 0; k < 6; ++k) {
                    const Eigen::Index col = off + k;
                    const double saved = xw(col);
                    xw(col) = saved + kStep;
                    evalMate(mi, xw, pertM);
                    xw(col) = saved;
                    for (int r = 0; r < rows; ++r) {
                        const double dv =
                            (pertM[static_cast<size_t>(r)] - baseM[static_cast<size_t>(r)]) / kStep;
                        if (dv != 0.0) trips.emplace_back(rowStart + r, static_cast<int>(col), dv);
                    }
                }
            }
        }
        Eigen::SparseMatrix<double> J(numResiduals, numUnknowns);
        J.setFromTriplets(trips.begin(), trips.end());
        return J;
    };

    Eigen::SparseMatrix<double> identity(numUnknowns, numUnknowns);
    identity.setIdentity();

    double lambda = 1e-4;
    double residualNorm = residuals.norm();
    int iteration = 0;
    // Rounds (Phase 160): solved with every limit mate free between its
    // limits; each that is then past one is held at it, and the mates solved
    // again from there, until none is. A limit held stays held.
    constexpr int kLimitRounds = 4;
    bool limitsHeld = false;
    for (int round = 0; round <= kLimitRounds; ++round) {
        if (round > 0) {
            prepare();
            residualNorm = residuals.norm();
            lambda = 1e-4;
        }
        for (int step = 0; step < m_maxIterations; ++step, ++iteration) {
            if (residuals.lpNorm<Eigen::Infinity>() < m_tolerance) break;
            if (numUnknowns == 0) break;

            const Eigen::SparseMatrix<double> jac = buildJacobian(x);
            const Eigen::SparseMatrix<double> jt = jac.transpose();
            const Eigen::SparseMatrix<double> jtj = jt * jac;
            const Eigen::VectorXd jtf = jt * residuals;

            for (int attempt = 0; attempt < 8; ++attempt) {
                Eigen::SparseMatrix<double> damped = jtj + lambda * identity;
                Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(damped);
                if (solver.info() != Eigen::Success) {
                    lambda *= 10.0;
                    continue;
                }
                const Eigen::VectorXd dx = solver.solve(-jtf);
                if (solver.info() != Eigen::Success) {
                    lambda *= 10.0;
                    continue;
                }

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
        // Each limit mate past a limit now held at it.
        bool pinned = false;
        for (size_t mi = 0; mi < activeMates.size(); ++mi) {
            const auto& mate = activeMates[mi];
            if (!mate.limited() || targets[mi]) continue;
            const MateFrame a =
                mate.frameA.transformed(placedAt(componentIndex.at(mate.componentA), x));
            const MateFrame b =
                mate.frameB.transformed(placedAt(componentIndex.at(mate.componentB), x));
            const auto eq = equationsFor(mate, a, b);
            if (!eq || !eq->measure) continue;
            const double at = *eq->measure;
            const auto past = [](double value, double bound) {
                return 1e-9 * (1.0 + std::abs(bound)) < value - bound;
            };
            if (mate.minimum && past(*mate.minimum, at)) {
                targets[mi] = *mate.minimum;
                pinned = true;
            } else if (mate.maximum && past(at, *mate.maximum)) {
                targets[mi] = *mate.maximum;
                pinned = true;
            }
        }
        limitsHeld = limitsHeld || pinned;
        if (!pinned) break;
    }
    // The rank analysis reads the rows of the last round.
    prepare();
    residualNorm = residuals.norm();

    // --- Rank analysis (redundancy + DOF), optional -------------------------
    //
    // A dense QR (O(unknowns^3)) — kept dense because its rank threshold must
    // match the established DOF results. Skipped when diagnostics are off (the
    // path large assemblies take to stay within the sparse solve budget).

    if (m_computeDiagnostics && numUnknowns > 0) {
        Eigen::MatrixXd denseJac(numResiduals, numUnknowns);
        Eigen::VectorXd perturbed;
        for (Eigen::Index c = 0; c < numUnknowns; ++c) {
            Eigen::VectorXd xp = x;
            xp(c) += kStep;
            evaluate(xp, perturbed, false);
            denseJac.col(c) = (perturbed - residuals) / kStep;
        }
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(denseJac);
        qr.setThreshold(1e-8);
        const auto rank = static_cast<int>(qr.rank());

        int expectedConstrained = 0;
        for (const auto& block : blocks) expectedConstrained += block.expectedRank;
        result.redundantCount = std::max(0, expectedConstrained - rank);
        result.remainingDOF = static_cast<int>(numUnknowns) - rank;

        // Per-component DOF: rank of the Jacobian restricted to that
        // component's 6 columns.
        for (size_t fi = 0; fi < freeComponents.size(); ++fi) {
            Eigen::MatrixXd sub = denseJac.middleCols(static_cast<Eigen::Index>(fi * 6), 6);
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
    if (result.status == AssemblySolveStatus::NotConverged && limitsHeld &&
        result.message.empty()) {
        result.message = "the mates cannot be met within their limits";
    }
    return result;
}

}  // namespace hz::model
