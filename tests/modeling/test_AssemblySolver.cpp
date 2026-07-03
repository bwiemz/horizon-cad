#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <numbers>

#include "horizon/modeling/AssemblySolver.h"

using namespace hz::model;
using hz::math::Mat4;
using hz::math::Vec3;

namespace {

MateFrame plane(const Vec3& origin, const Vec3& normal) {
    MateFrame f;
    f.kind = MateFrameKind::Planar;
    f.origin = origin;
    f.direction = normal.normalized();
    return f;
}

MateFrame axis(const Vec3& origin, const Vec3& dir, double radius) {
    MateFrame f;
    f.kind = MateFrameKind::Cylindrical;
    f.origin = origin;
    f.direction = dir.normalized();
    f.radius = radius;
    return f;
}

// Two unit-ish boxes: A grounded at origin, B starts displaced.
// A's top face: plane z=10 with +Z normal (local).
// B's bottom face: plane z=0 with -Z normal (local).
std::vector<SolverComponent> twoComponents(const Mat4& bStart) {
    SolverComponent a;
    a.id = 1;
    a.grounded = true;
    SolverComponent b;
    b.id = 2;
    b.transform = bStart;
    return {a, b};
}

}  // namespace

// ---------------------------------------------------------------------------
// CoincidentPlanesSnapTogether
// ---------------------------------------------------------------------------

TEST(AssemblySolverTest, CoincidentPlanesSnapTogether) {
    auto components = twoComponents(Mat4::translation(Vec3(3, -2, 7)));

    SolverMate mate;
    mate.type = MateType::Coincident;
    mate.componentA = 1;
    mate.componentB = 2;
    mate.frameA = plane(Vec3(5, 5, 10), Vec3(0, 0, 1));  // A top
    mate.frameB = plane(Vec3(5, 5, 0), Vec3(0, 0, -1));  // B bottom

    AssemblySolver solver;
    auto result = solver.solve(components, {mate});

    ASSERT_EQ(result.status, AssemblySolveStatus::Success);
    EXPECT_LT(result.residualNorm, 1e-7);

    // B's bottom plane must land on z = 10 with normal -Z.
    const Mat4& tb = result.transforms.at(2);
    MateFrame placedB = mate.frameB.transformed(tb);
    EXPECT_NEAR(placedB.origin.z, 10.0, 1e-6);
    EXPECT_NEAR(placedB.direction.z, -1.0, 1e-6);

    // Coincident constrains 3 DOF; B keeps 3 (2 in-plane translations + spin).
    EXPECT_EQ(result.componentDOF.at(2), 3);
    EXPECT_EQ(result.componentDOF.at(1), 0);
    EXPECT_EQ(result.redundantCount, 0);
}

// ---------------------------------------------------------------------------
// DistanceMateHoldsOffset
// ---------------------------------------------------------------------------

TEST(AssemblySolverTest, DistanceMateHoldsOffset) {
    auto components = twoComponents(Mat4::translation(Vec3(0, 0, 25)));

    SolverMate mate;
    mate.type = MateType::Distance;
    mate.componentA = 1;
    mate.componentB = 2;
    mate.frameA = plane(Vec3(0, 0, 10), Vec3(0, 0, 1));
    mate.frameB = plane(Vec3(0, 0, 0), Vec3(0, 0, -1));
    mate.value = 5.0;

    AssemblySolver solver;
    auto result = solver.solve(components, {mate});

    ASSERT_EQ(result.status, AssemblySolveStatus::Success);
    MateFrame placedB = mate.frameB.transformed(result.transforms.at(2));
    EXPECT_NEAR(placedB.origin.z, 15.0, 1e-6);
}

// ---------------------------------------------------------------------------
// ConcentricAlignsAxes
// ---------------------------------------------------------------------------

TEST(AssemblySolverTest, ConcentricAlignsAxes) {
    auto components = twoComponents(Mat4::translation(Vec3(4, 6, 0)) * Mat4::rotationX(0.3));

    SolverMate mate;
    mate.type = MateType::Concentric;
    mate.componentA = 1;
    mate.componentB = 2;
    mate.frameA = axis(Vec3(0, 0, 0), Vec3(0, 0, 1), 5.0);
    mate.frameB = axis(Vec3(0, 0, 0), Vec3(0, 0, 1), 3.0);

    AssemblySolver solver;
    auto result = solver.solve(components, {mate});

    ASSERT_EQ(result.status, AssemblySolveStatus::Success);
    MateFrame placedB = mate.frameB.transformed(result.transforms.at(2));
    // Axis direction parallel to Z and passing through the Z axis.
    EXPECT_NEAR(std::abs(placedB.direction.z), 1.0, 1e-6);
    Vec3 perp = placedB.origin - Vec3(0, 0, 1) * placedB.origin.dot(Vec3(0, 0, 1));
    EXPECT_NEAR(perp.length(), 0.0, 1e-6);

    // Concentric constrains 4 DOF; axial slide + spin remain.
    EXPECT_EQ(result.componentDOF.at(2), 4 == 4 ? 2 : -1);
}

// ---------------------------------------------------------------------------
// ParallelPerpendicularAngle
// ---------------------------------------------------------------------------

TEST(AssemblySolverTest, ParallelPerpendicularAngle) {
    AssemblySolver solver;

    // Parallel: B rotated 0.4 rad about X must align its +Z normal to A's.
    {
        auto components = twoComponents(Mat4::rotationX(0.4));
        SolverMate mate;
        mate.type = MateType::Parallel;
        mate.componentA = 1;
        mate.componentB = 2;
        mate.frameA = plane(Vec3(0, 0, 0), Vec3(0, 0, 1));
        mate.frameB = plane(Vec3(0, 0, 0), Vec3(0, 0, 1));
        auto result = solver.solve(components, {mate});
        ASSERT_EQ(result.status, AssemblySolveStatus::Success);
        MateFrame placedB = mate.frameB.transformed(result.transforms.at(2));
        EXPECT_NEAR(std::abs(placedB.direction.dot(Vec3(0, 0, 1))), 1.0, 1e-6);
    }

    // Perpendicular.
    {
        auto components = twoComponents(Mat4::identity());
        SolverMate mate;
        mate.type = MateType::Perpendicular;
        mate.componentA = 1;
        mate.componentB = 2;
        mate.frameA = plane(Vec3(0, 0, 0), Vec3(0, 0, 1));
        mate.frameB = plane(Vec3(0, 0, 0), Vec3(0, 0.2, 0.98).normalized());
        auto result = solver.solve(components, {mate});
        ASSERT_EQ(result.status, AssemblySolveStatus::Success);
        MateFrame placedB = mate.frameB.transformed(result.transforms.at(2));
        EXPECT_NEAR(placedB.direction.dot(Vec3(0, 0, 1)), 0.0, 1e-6);
    }

    // Angle: 60 degrees between normals.
    {
        auto components = twoComponents(Mat4::identity());
        SolverMate mate;
        mate.type = MateType::Angle;
        mate.componentA = 1;
        mate.componentB = 2;
        mate.frameA = plane(Vec3(0, 0, 0), Vec3(0, 0, 1));
        mate.frameB = plane(Vec3(0, 0, 0), Vec3(0, 0.1, 1.0).normalized());
        mate.value = std::numbers::pi / 3.0;
        auto result = solver.solve(components, {mate});
        ASSERT_EQ(result.status, AssemblySolveStatus::Success);
        MateFrame placedB = mate.frameB.transformed(result.transforms.at(2));
        EXPECT_NEAR(placedB.direction.dot(Vec3(0, 0, 1)), std::cos(std::numbers::pi / 3.0), 1e-6);
    }
}

// ---------------------------------------------------------------------------
// TangentPlaneCylinder
// ---------------------------------------------------------------------------

TEST(AssemblySolverTest, TangentPlaneCylinder) {
    auto components = twoComponents(Mat4::translation(Vec3(0, 0, 20)));

    SolverMate mate;
    mate.type = MateType::Tangent;
    mate.componentA = 1;
    mate.componentB = 2;
    mate.frameA = plane(Vec3(0, 0, 10), Vec3(0, 0, 1));     // table at z=10
    mate.frameB = axis(Vec3(0, 0, 0), Vec3(1, 0, 0), 3.0);  // cylinder axis
    // (axis along X in B's local frame)

    AssemblySolver solver;
    auto result = solver.solve(components, {mate});

    ASSERT_EQ(result.status, AssemblySolveStatus::Success);
    MateFrame placedB = mate.frameB.transformed(result.transforms.at(2));
    // Axis parallel to the plane at height radius above it.
    EXPECT_NEAR(placedB.direction.dot(Vec3(0, 0, 1)), 0.0, 1e-6);
    EXPECT_NEAR(placedB.origin.z, 13.0, 1e-6);
}

// ---------------------------------------------------------------------------
// RedundantMateDetected
// ---------------------------------------------------------------------------

TEST(AssemblySolverTest, RedundantMateDetected) {
    auto components = twoComponents(Mat4::translation(Vec3(0, 0, 3)));

    SolverMate mate;
    mate.type = MateType::Coincident;
    mate.componentA = 1;
    mate.componentB = 2;
    mate.frameA = plane(Vec3(0, 0, 10), Vec3(0, 0, 1));
    mate.frameB = plane(Vec3(0, 0, 0), Vec3(0, 0, -1));

    // The same mate twice: the second contributes nothing new.
    AssemblySolver solver;
    auto result = solver.solve(components, {mate, mate});

    ASSERT_EQ(result.status, AssemblySolveStatus::Success);
    EXPECT_EQ(result.redundantCount, 3);
}

// ---------------------------------------------------------------------------
// UngroundedIslandReported
// ---------------------------------------------------------------------------

TEST(AssemblySolverTest, UngroundedIslandReported) {
    SolverComponent a;
    a.id = 1;
    a.grounded = true;
    SolverComponent b;
    b.id = 2;
    SolverComponent c;  // no mates touch c
    c.id = 3;

    SolverMate mate;
    mate.type = MateType::Coincident;
    mate.componentA = 1;
    mate.componentB = 2;
    mate.frameA = plane(Vec3(0, 0, 10), Vec3(0, 0, 1));
    mate.frameB = plane(Vec3(0, 0, 0), Vec3(0, 0, -1));

    AssemblySolver solver;
    auto result = solver.solve({a, b, c}, {mate});

    ASSERT_EQ(result.status, AssemblySolveStatus::Success);
    ASSERT_EQ(result.ungroundedComponents.size(), 1u);
    EXPECT_EQ(result.ungroundedComponents[0], 3u);
    EXPECT_EQ(result.componentDOF.at(3), 6);
}

// ---------------------------------------------------------------------------
// FixedMateGroundsComponent
// ---------------------------------------------------------------------------

TEST(AssemblySolverTest, FixedMateGroundsComponent) {
    SolverComponent a;
    a.id = 1;
    SolverComponent b;
    b.id = 2;
    b.transform = Mat4::translation(Vec3(0, 0, 4));

    SolverMate fixedMate;
    fixedMate.type = MateType::Fixed;
    fixedMate.componentA = 1;

    SolverMate coincident;
    coincident.type = MateType::Coincident;
    coincident.componentA = 1;
    coincident.componentB = 2;
    coincident.frameA = plane(Vec3(0, 0, 10), Vec3(0, 0, 1));
    coincident.frameB = plane(Vec3(0, 0, 0), Vec3(0, 0, -1));

    AssemblySolver solver;
    auto result = solver.solve({a, b}, {fixedMate, coincident});

    ASSERT_EQ(result.status, AssemblySolveStatus::Success);
    // A must not move (it was grounded by the Fixed mate, not by convention).
    EXPECT_TRUE(result.message.empty());
    const Mat4& ta = result.transforms.at(1);
    EXPECT_NEAR(ta.at(0, 3), 0.0, 1e-12);
    EXPECT_NEAR(ta.at(2, 3), 0.0, 1e-12);
    EXPECT_EQ(result.componentDOF.at(1), 0);
}

// ---------------------------------------------------------------------------
// ThreePartChainSolvesQuickly
// ---------------------------------------------------------------------------

TEST(AssemblySolverTest, ThreePartChainSolvesQuickly) {
    SolverComponent a;
    a.id = 1;
    a.grounded = true;
    SolverComponent b;
    b.id = 2;
    b.transform = Mat4::translation(Vec3(1, 2, 30)) * Mat4::rotationY(0.2);
    SolverComponent c;
    c.id = 3;
    c.transform = Mat4::translation(Vec3(-4, 1, 60)) * Mat4::rotationX(-0.3);

    auto stackMate = [](uint64_t lower, uint64_t upper, double lowerTopZ) {
        SolverMate m;
        m.type = MateType::Coincident;
        m.componentA = lower;
        m.componentB = upper;
        m.frameA = plane(Vec3(0, 0, lowerTopZ), Vec3(0, 0, 1));
        m.frameB = plane(Vec3(0, 0, 0), Vec3(0, 0, -1));
        return m;
    };

    AssemblySolver solver;
    auto start = std::chrono::steady_clock::now();
    auto result = solver.solve({a, b, c}, {stackMate(1, 2, 10), stackMate(2, 3, 10)});
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - start)
                       .count();

    ASSERT_EQ(result.status, AssemblySolveStatus::Success);
    EXPECT_LT(elapsed, 1000) << "assembly solve must complete in < 1s";

    // B's bottom sits on z=10; C's bottom sits on B's top (z=20).
    MateFrame placedB = plane(Vec3(0, 0, 0), Vec3(0, 0, -1)).transformed(result.transforms.at(2));
    EXPECT_NEAR(placedB.origin.z, 10.0, 1e-6);
    MateFrame placedC = plane(Vec3(0, 0, 0), Vec3(0, 0, -1)).transformed(result.transforms.at(3));
    EXPECT_NEAR(placedC.origin.z, 20.0, 1e-6);
}

// ---------------------------------------------------------------------------
// NoMatesReportsFullDOF
// ---------------------------------------------------------------------------

TEST(AssemblySolverTest, NoMatesReportsFullDOF) {
    SolverComponent a;
    a.id = 1;
    a.grounded = true;
    SolverComponent b;
    b.id = 2;

    AssemblySolver solver;
    auto result = solver.solve({a, b}, {});
    EXPECT_EQ(result.status, AssemblySolveStatus::NoMates);
    EXPECT_EQ(result.componentDOF.at(2), 6);
    EXPECT_EQ(result.remainingDOF, 6);
}

// ---------------------------------------------------------------------------
// LargeAssemblySolvesQuickly — 100 parts, ~200 mates, sparse-solve scaling
// ---------------------------------------------------------------------------

TEST(AssemblySolverTest, LargeAssemblySolvesQuickly) {
    // A 100-component stacked chain with two coincident mates per adjacent pair
    // (198 mates). Diagnostics are disabled — the rank analysis is a separate
    // dense O(n^3) step; this measures the sparse Newton solve, which must scale
    // to the roadmap's 100-part / <1 s target.
    constexpr int kN = 100;
    std::vector<SolverComponent> comps;
    comps.reserve(kN);
    for (int i = 0; i < kN; ++i) {
        SolverComponent c;
        c.id = static_cast<uint64_t>(i + 1);
        c.grounded = (i == 0);
        // Start each roughly stacked but perturbed, so the solve has real work.
        c.transform = Mat4::translation(Vec3(0.3 * i, -0.2 * i, 10.0 * i + 0.7 * ((i * 37) % 5)));
        comps.push_back(c);
    }

    std::vector<SolverMate> mates;
    for (int i = 1; i < kN; ++i) {
        SolverMate top;  // top of i coincident with bottom of i+1
        top.type = MateType::Coincident;
        top.componentA = static_cast<uint64_t>(i);
        top.componentB = static_cast<uint64_t>(i + 1);
        top.frameA = plane(Vec3(0, 0, 5), Vec3(0, 0, 1));
        top.frameB = plane(Vec3(0, 0, -5), Vec3(0, 0, -1));
        mates.push_back(top);
        SolverMate side;  // a side face pair to constrain in-plane sliding
        side.type = MateType::Coincident;
        side.componentA = static_cast<uint64_t>(i);
        side.componentB = static_cast<uint64_t>(i + 1);
        side.frameA = plane(Vec3(5, 0, 0), Vec3(1, 0, 0));
        side.frameB = plane(Vec3(5, 0, 0), Vec3(1, 0, 0));
        mates.push_back(side);
    }

    AssemblySolver solver;
    solver.setComputeDiagnostics(false);
    const auto start = std::chrono::steady_clock::now();
    auto result = solver.solve(comps, mates);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

    EXPECT_EQ(result.status, AssemblySolveStatus::Success);
    EXPECT_LT(result.residualNorm, 1e-6);
#ifdef NDEBUG
    EXPECT_LT(ms, 1000.0) << "100-part assembly must solve in < 1 s (release)";
#else
    EXPECT_LT(ms, 6000.0) << "debug/unoptimized Eigen; generous bound for slow CI";
#endif
}
