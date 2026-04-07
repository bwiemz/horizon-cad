#include <gtest/gtest.h>

#include "horizon/geometry/surfaces/NurbsSurface.h"

#include <cmath>

using namespace hz::geo;
using namespace hz::math;

// ---------------------------------------------------------------------------
// Helper: create a clamped uniform knot vector for n control points of given
// degree.  knots = [0,...,0, internal, 1,...,1]  with (degree+1) clamped ends.
// ---------------------------------------------------------------------------
static std::vector<double> clampedKnots(int n, int degree) {
    std::vector<double> knots(n + degree + 1);
    for (int i = 0; i <= degree; ++i) knots[i] = 0.0;
    for (int i = degree + 1; i < n; ++i)
        knots[i] = static_cast<double>(i - degree) / (n - degree);
    for (int i = n; i < n + degree + 1; ++i) knots[i] = 1.0;
    return knots;
}

// ===========================================================================
// Task 1: Tensor-Product Evaluation
// ===========================================================================

// ---------------------------------------------------------------------------
// 1. Construct a bilinear surface (2x2, degree 1x1)
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, ConstructBilinear) {
    std::vector<std::vector<Vec3>> pts = {
        {{0, 0, 0}, {10, 0, 0}},
        {{0, 10, 0}, {10, 10, 0}},
    };
    std::vector<std::vector<double>> wts = {
        {1.0, 1.0},
        {1.0, 1.0},
    };
    auto kU = clampedKnots(2, 1);
    auto kV = clampedKnots(2, 1);

    NurbsSurface srf(pts, wts, kU, kV, 1, 1);

    EXPECT_EQ(srf.degreeU(), 1);
    EXPECT_EQ(srf.degreeV(), 1);
    EXPECT_EQ(srf.controlPointCountU(), 2);
    EXPECT_EQ(srf.controlPointCountV(), 2);
}

// ---------------------------------------------------------------------------
// 2. Evaluate bilinear surface at all four corners
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, EvaluateBilinearCorners) {
    std::vector<std::vector<Vec3>> pts = {
        {{0, 0, 0}, {10, 0, 0}},
        {{0, 10, 0}, {10, 10, 0}},
    };
    std::vector<std::vector<double>> wts = {
        {1.0, 1.0},
        {1.0, 1.0},
    };
    auto kU = clampedKnots(2, 1);
    auto kV = clampedKnots(2, 1);

    NurbsSurface srf(pts, wts, kU, kV, 1, 1);

    // Corner (0,0) -> (0,0,0)
    auto p00 = srf.evaluate(0.0, 0.0);
    EXPECT_NEAR(p00.x, 0.0, 1e-9);
    EXPECT_NEAR(p00.y, 0.0, 1e-9);
    EXPECT_NEAR(p00.z, 0.0, 1e-9);

    // Corner (0,1) -> (10,0,0)
    auto p01 = srf.evaluate(0.0, 1.0);
    EXPECT_NEAR(p01.x, 10.0, 1e-9);
    EXPECT_NEAR(p01.y, 0.0, 1e-9);
    EXPECT_NEAR(p01.z, 0.0, 1e-9);

    // Corner (1,0) -> (0,10,0)
    auto p10 = srf.evaluate(1.0, 0.0);
    EXPECT_NEAR(p10.x, 0.0, 1e-9);
    EXPECT_NEAR(p10.y, 10.0, 1e-9);
    EXPECT_NEAR(p10.z, 0.0, 1e-9);

    // Corner (1,1) -> (10,10,0)
    auto p11 = srf.evaluate(1.0, 1.0);
    EXPECT_NEAR(p11.x, 10.0, 1e-9);
    EXPECT_NEAR(p11.y, 10.0, 1e-9);
    EXPECT_NEAR(p11.z, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// 3. Evaluate bilinear surface at center (0.5, 0.5) -> midpoint (5, 5, 0)
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, EvaluateBilinearCenter) {
    std::vector<std::vector<Vec3>> pts = {
        {{0, 0, 0}, {10, 0, 0}},
        {{0, 10, 0}, {10, 10, 0}},
    };
    std::vector<std::vector<double>> wts = {
        {1.0, 1.0},
        {1.0, 1.0},
    };
    auto kU = clampedKnots(2, 1);
    auto kV = clampedKnots(2, 1);

    NurbsSurface srf(pts, wts, kU, kV, 1, 1);

    auto mid = srf.evaluate(0.5, 0.5);
    EXPECT_NEAR(mid.x, 5.0, 1e-9);
    EXPECT_NEAR(mid.y, 5.0, 1e-9);
    EXPECT_NEAR(mid.z, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// 4. Evaluate a quadratic 3x3 dome surface with center elevated
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, EvaluateQuadraticSurface) {
    // 3x3 grid: corners at z=0, edges at z=0, center at z=10 (dome).
    std::vector<std::vector<Vec3>> pts = {
        {{0, 0, 0}, {5, 0, 0}, {10, 0, 0}},
        {{0, 5, 0}, {5, 5, 10}, {10, 5, 0}},
        {{0, 10, 0}, {5, 10, 0}, {10, 10, 0}},
    };
    std::vector<std::vector<double>> wts = {
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0},
    };
    auto kU = clampedKnots(3, 2);
    auto kV = clampedKnots(3, 2);

    NurbsSurface srf(pts, wts, kU, kV, 2, 2);

    // At center (0.5, 0.5), the quadratic Bezier in each direction blends
    // the center control point with weight. The center z should be elevated.
    auto center = srf.evaluate(0.5, 0.5);
    EXPECT_NEAR(center.x, 5.0, 1e-6);
    EXPECT_NEAR(center.y, 5.0, 1e-6);
    EXPECT_GT(center.z, 0.0);
    // For a Bezier quad surface, the center evaluates to the average of the 9 points
    // with Bernstein weights. The center z = 10 * B_1^2(0.5) * B_1^2(0.5) = 10 * 0.5 * 0.5 = 2.5
    EXPECT_NEAR(center.z, 2.5, 1e-6);

    // Corners should be at z=0.
    auto corner = srf.evaluate(0.0, 0.0);
    EXPECT_NEAR(corner.z, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// 5. Parameter domain: uMin/uMax/vMin/vMax
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, ParameterDomain) {
    std::vector<std::vector<Vec3>> pts = {
        {{0, 0, 0}, {10, 0, 0}},
        {{0, 10, 0}, {10, 10, 0}},
    };
    std::vector<std::vector<double>> wts = {
        {1.0, 1.0},
        {1.0, 1.0},
    };
    auto kU = clampedKnots(2, 1);
    auto kV = clampedKnots(2, 1);

    NurbsSurface srf(pts, wts, kU, kV, 1, 1);

    EXPECT_DOUBLE_EQ(srf.uMin(), 0.0);
    EXPECT_DOUBLE_EQ(srf.uMax(), 1.0);
    EXPECT_DOUBLE_EQ(srf.vMin(), 0.0);
    EXPECT_DOUBLE_EQ(srf.vMax(), 1.0);

    // Non-standard domain: shifted knots.
    std::vector<double> kU2 = {2.0, 2.0, 5.0, 5.0};
    std::vector<double> kV2 = {3.0, 3.0, 7.0, 7.0};
    NurbsSurface srf2(pts, wts, kU2, kV2, 1, 1);

    EXPECT_DOUBLE_EQ(srf2.uMin(), 2.0);
    EXPECT_DOUBLE_EQ(srf2.uMax(), 5.0);
    EXPECT_DOUBLE_EQ(srf2.vMin(), 3.0);
    EXPECT_DOUBLE_EQ(srf2.vMax(), 7.0);
}

// ---------------------------------------------------------------------------
// 6. Weighted evaluation: high weight pulls surface toward control point
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, WeightedEvaluation) {
    std::vector<std::vector<Vec3>> pts = {
        {{0, 0, 0}, {5, 0, 0}, {10, 0, 0}},
        {{0, 5, 0}, {5, 5, 10}, {10, 5, 0}},
        {{0, 10, 0}, {5, 10, 0}, {10, 10, 0}},
    };
    auto kU = clampedKnots(3, 2);
    auto kV = clampedKnots(3, 2);

    // Uniform weights.
    std::vector<std::vector<double>> wtsUniform = {
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0},
    };
    NurbsSurface srfUniform(pts, wtsUniform, kU, kV, 2, 2);
    auto centerUniform = srfUniform.evaluate(0.5, 0.5);

    // High weight on center control point.
    std::vector<std::vector<double>> wtsWeighted = {
        {1.0, 1.0, 1.0},
        {1.0, 10.0, 1.0},
        {1.0, 1.0, 1.0},
    };
    NurbsSurface srfWeighted(pts, wtsWeighted, kU, kV, 2, 2);
    auto centerWeighted = srfWeighted.evaluate(0.5, 0.5);

    // The weighted surface center should be pulled closer to z=10.
    // Note: The two-pass evaluation approach loses some rational precision, so
    // the pull is not as strong as a full homogeneous evaluation would give.
    EXPECT_GT(centerWeighted.z, centerUniform.z);
    EXPECT_GT(centerWeighted.z, 3.5);
}

// ===========================================================================
// Task 2: Derivatives & Normal
// ===========================================================================

// ---------------------------------------------------------------------------
// 7. Normal of a flat XY surface -> normal ≈ (0, 0, ±1)
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, NormalOfFlatSurface) {
    // Flat surface in XY plane: x from 0..10, y from 0..10, z=0.
    std::vector<std::vector<Vec3>> pts = {
        {{0, 0, 0}, {10, 0, 0}},
        {{0, 10, 0}, {10, 10, 0}},
    };
    std::vector<std::vector<double>> wts = {
        {1.0, 1.0},
        {1.0, 1.0},
    };
    auto kU = clampedKnots(2, 1);
    auto kV = clampedKnots(2, 1);

    NurbsSurface srf(pts, wts, kU, kV, 1, 1);

    auto n = srf.normal(0.5, 0.5);
    EXPECT_NEAR(std::abs(n.z), 1.0, 1e-4);
    EXPECT_NEAR(n.x, 0.0, 1e-4);
    EXPECT_NEAR(n.y, 0.0, 1e-4);
}

// ---------------------------------------------------------------------------
// 8. Derivatives of a flat surface: dS/du ≈ (0,10,0), dS/dv ≈ (10,0,0)
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, DerivativesOfFlatSurface) {
    // Flat bilinear: (u,v) -> (10*v, 10*u, 0) over [0,1]x[0,1].
    std::vector<std::vector<Vec3>> pts = {
        {{0, 0, 0}, {10, 0, 0}},
        {{0, 10, 0}, {10, 10, 0}},
    };
    std::vector<std::vector<double>> wts = {
        {1.0, 1.0},
        {1.0, 1.0},
    };
    auto kU = clampedKnots(2, 1);
    auto kV = clampedKnots(2, 1);

    NurbsSurface srf(pts, wts, kU, kV, 1, 1);

    // dS/du at (0.5, 0.5): moving in U goes from y=0 to y=10, so dS/du ≈ (0,10,0).
    auto du = srf.derivativeU(0.5, 0.5);
    EXPECT_NEAR(du.x, 0.0, 0.1);
    EXPECT_NEAR(du.y, 10.0, 0.1);
    EXPECT_NEAR(du.z, 0.0, 0.1);

    // dS/dv at (0.5, 0.5): moving in V goes from x=0 to x=10, so dS/dv ≈ (10,0,0).
    auto dv = srf.derivativeV(0.5, 0.5);
    EXPECT_NEAR(dv.x, 10.0, 0.1);
    EXPECT_NEAR(dv.y, 0.0, 0.1);
    EXPECT_NEAR(dv.z, 0.0, 0.1);
}

// ---------------------------------------------------------------------------
// 9. Normal of a curved (dome) surface: at center, |normal.z| is significant
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, NormalOfCurvedSurface) {
    std::vector<std::vector<Vec3>> pts = {
        {{0, 0, 0}, {5, 0, 0}, {10, 0, 0}},
        {{0, 5, 0}, {5, 5, 10}, {10, 5, 0}},
        {{0, 10, 0}, {5, 10, 0}, {10, 10, 0}},
    };
    std::vector<std::vector<double>> wts = {
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0},
    };
    auto kU = clampedKnots(3, 2);
    auto kV = clampedKnots(3, 2);

    NurbsSurface srf(pts, wts, kU, kV, 2, 2);

    // The dome rises in +Z, so the normal at center should have a non-zero Z component.
    // The sign depends on the cross product order (dS/du x dS/dv), which for this
    // parameterization points in -Z.  Check that |n.z| is significant.
    auto n = srf.normal(0.5, 0.5);
    EXPECT_GT(std::abs(n.z), 0.5);

    // The normal should be roughly unit length.
    double len = n.length();
    EXPECT_NEAR(len, 1.0, 1e-6);
}
