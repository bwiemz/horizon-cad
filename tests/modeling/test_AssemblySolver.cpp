#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <numbers>

#include "../TimeLimits.h"
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
#if HZ_TIME_LIMITS
    EXPECT_LT(elapsed, 1000) << "assembly solve must complete in < 1s";
#else
    (void)elapsed;
#endif

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
        c.id = static_cast<uint64_t>(i) + 1U;
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
        top.componentB = static_cast<uint64_t>(i) + 1U;
        top.frameA = plane(Vec3(0, 0, 5), Vec3(0, 0, 1));
        top.frameB = plane(Vec3(0, 0, -5), Vec3(0, 0, -1));
        mates.push_back(top);
        SolverMate side;  // a side face pair to constrain in-plane sliding
        side.type = MateType::Coincident;
        side.componentA = static_cast<uint64_t>(i);
        side.componentB = static_cast<uint64_t>(i) + 1U;
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
#if HZ_TIME_LIMITS
#ifdef NDEBUG
    EXPECT_LT(ms, 1000.0) << "100-part assembly must solve in < 1 s (release)";
#else
    EXPECT_LT(ms, 6000.0) << "debug/unoptimized Eigen; generous bound for slow CI";
#endif
#else
    (void)ms;
#endif
}

// ---------------------------------------------------------------------------
// Phase 160: more to mate — points, lines, spheres, limits
// ---------------------------------------------------------------------------

namespace {

MateFrame pointAt(const Vec3& at) {
    MateFrame f;
    f.kind = MateFrameKind::Point;
    f.origin = at;
    return f;
}

MateFrame line(const Vec3& origin, const Vec3& dir) {
    MateFrame f;
    f.kind = MateFrameKind::Line;
    f.origin = origin;
    f.direction = dir.normalized();
    return f;
}

MateFrame sphere(const Vec3& centre, double radius) {
    MateFrame f;
    f.kind = MateFrameKind::Spherical;
    f.origin = centre;
    f.radius = radius;
    return f;
}

SolverMate mateOf(MateType type, const MateFrame& a, const MateFrame& b, double value = 0.0) {
    SolverMate mate;
    mate.type = type;
    mate.componentA = 1;
    mate.componentB = 2;
    mate.frameA = a;
    mate.frameB = b;
    mate.value = value;
    return mate;
}

}  // namespace

TEST(AssemblySolverTest, APointGoesOntoAPlaneAndOntoALine) {
    auto components = twoComponents(Mat4::translation(Vec3(3, -2, 7)));
    AssemblySolver solver;
    // B's point (1, 2, 3) onto A's plane z = 10.
    auto result = solver.solve(
        components,
        {mateOf(MateType::Coincident, plane(Vec3(0, 0, 10), Vec3::UnitZ), pointAt(Vec3(1, 2, 3)))});
    ASSERT_EQ(result.status, AssemblySolveStatus::Success) << result.message;
    EXPECT_NEAR(result.transforms.at(2).transformPoint(Vec3(1, 2, 3)).z, 10.0, 1e-6);
    EXPECT_EQ(result.componentDOF.at(2), 5) << "a point on a plane takes one freedom";

    // The same point onto A's line, the x axis at y = 4, z = 10.
    result = solver.solve(
        components,
        {mateOf(MateType::Coincident, line(Vec3(0, 4, 10), Vec3::UnitX), pointAt(Vec3(1, 2, 3)))});
    ASSERT_EQ(result.status, AssemblySolveStatus::Success) << result.message;
    const Vec3 on = result.transforms.at(2).transformPoint(Vec3(1, 2, 3));
    EXPECT_NEAR(on.y, 4.0, 1e-6);
    EXPECT_NEAR(on.z, 10.0, 1e-6);
}

TEST(AssemblySolverTest, LinesAreMadeCollinearAndSpheresConcentric) {
    auto components = twoComponents(Mat4::translation(Vec3(3, -2, 7)) * Mat4::rotationZ(0.4));
    AssemblySolver solver;
    auto result =
        solver.solve(components, {mateOf(MateType::Coincident, line(Vec3(0, 0, 0), Vec3::UnitZ),
                                         line(Vec3(1, 1, 0), Vec3::UnitX))});
    ASSERT_EQ(result.status, AssemblySolveStatus::Success) << result.message;
    const Mat4& placed = result.transforms.at(2);
    const Vec3 dir = placed.transformDirection(Vec3::UnitX);
    EXPECT_NEAR(std::abs(dir.z), 1.0, 1e-6) << "along A's line";
    const Vec3 at = placed.transformPoint(Vec3(1, 1, 0));
    EXPECT_NEAR(at.x, 0.0, 1e-6);
    EXPECT_NEAR(at.y, 0.0, 1e-6);

    result = solver.solve(components, {mateOf(MateType::Concentric, sphere(Vec3(5, 5, 5), 2.0),
                                              sphere(Vec3(0, 0, 0), 1.0))});
    ASSERT_EQ(result.status, AssemblySolveStatus::Success) << result.message;
    const Vec3 centre = result.transforms.at(2).transformPoint(Vec3());
    EXPECT_NEAR((centre - Vec3(5, 5, 5)).length(), 0.0, 1e-6);
    EXPECT_EQ(result.componentDOF.at(2), 3) << "free to turn about the centre";
}

TEST(AssemblySolverTest, ADistanceBetweenPointsHoldsAndAnAngleHoldsAtZero) {
    auto components = twoComponents(Mat4::translation(Vec3(3, -2, 7)));
    AssemblySolver solver;
    auto result = solver.solve(components, {mateOf(MateType::Distance, pointAt(Vec3(0, 0, 0)),
                                                   pointAt(Vec3(0, 0, 0)), 5.0)});
    ASSERT_EQ(result.status, AssemblySolveStatus::Success) << result.message;
    EXPECT_NEAR(result.transforms.at(2).transformPoint(Vec3()).length(), 5.0, 1e-6);

    // An angle of 0: the cosine is flat there; the angle held all the same.
    components = twoComponents(Mat4::rotationX(0.3));
    result = solver.solve(components, {mateOf(MateType::Angle, plane(Vec3(), Vec3::UnitZ),
                                              plane(Vec3(), Vec3::UnitZ), 0.0)});
    ASSERT_EQ(result.status, AssemblySolveStatus::Success) << result.message;
    EXPECT_NEAR(result.transforms.at(2).transformDirection(Vec3::UnitZ).z, 1.0, 1e-6);

    // 180 degrees is the other way, not parallel either way: from the same
    // start, it turns over.
    components = twoComponents(Mat4::rotationX(0.3));
    result = solver.solve(components, {mateOf(MateType::Angle, plane(Vec3(), Vec3::UnitZ),
                                              plane(Vec3(), Vec3::UnitZ), std::numbers::pi)});
    ASSERT_EQ(result.status, AssemblySolveStatus::Success) << result.message;
    EXPECT_NEAR(result.transforms.at(2).transformDirection(Vec3::UnitZ).z, -1.0, 1e-6);
    // And 0 from nearly opposite comes all the way round.
    components = twoComponents(Mat4::rotationX(std::numbers::pi - 0.3));
    result = solver.solve(components, {mateOf(MateType::Angle, plane(Vec3(), Vec3::UnitZ),
                                              plane(Vec3(), Vec3::UnitZ), 0.0)});
    ASSERT_EQ(result.status, AssemblySolveStatus::Success) << result.message;
    EXPECT_NEAR(result.transforms.at(2).transformDirection(Vec3::UnitZ).z, 1.0, 1e-6);
}

// A Distance mate with limits keeps the distance between them, and leaves
// it alone within them.
TEST(AssemblySolverTest, ADistanceBetweenItsLimitsIsLeftAlone) {
    const auto gapAfter = [](double startZ) {
        auto components = twoComponents(Mat4::translation(Vec3(0, 0, startZ)));
        SolverMate mate = mateOf(MateType::Distance, plane(Vec3(0, 0, 0), Vec3::UnitZ),
                                 plane(Vec3(0, 0, 0), Vec3::UnitZ), 0.0);
        mate.minimum = 5.0;
        mate.maximum = 10.0;
        AssemblySolver solver;
        const auto result = solver.solve(components, {mate});
        EXPECT_EQ(result.status, AssemblySolveStatus::Success) << result.message;
        return result.transforms.at(2).transformPoint(Vec3()).z;
    };
    EXPECT_NEAR(gapAfter(7.0), 7.0, 1e-9) << "between them: as it is";
    EXPECT_NEAR(gapAfter(20.0), 10.0, 1e-6) << "held at the maximum";
    EXPECT_NEAR(gapAfter(2.0), 5.0, 1e-6) << "held at the minimum";
}

TEST(AssemblySolverTest, AMateBetweenKindsItDoesNotRelateIsRefused) {
    auto components = twoComponents(Mat4::identity());
    AssemblySolver solver;
    auto result = solver.solve(components, {mateOf(MateType::Concentric, plane(Vec3(), Vec3::UnitZ),
                                                   plane(Vec3(), Vec3::UnitZ))});
    EXPECT_EQ(result.status, AssemblySolveStatus::InvalidReference);
    EXPECT_EQ(result.message, "Concentric between a plane and a plane is not a mate");

    SolverMate limited =
        mateOf(MateType::Coincident, plane(Vec3(), Vec3::UnitZ), plane(Vec3(), Vec3::UnitZ));
    limited.maximum = 3.0;
    result = solver.solve(components, {limited});
    EXPECT_EQ(result.status, AssemblySolveStatus::InvalidReference);
}
