#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"

using namespace hz::geo;
using namespace hz::math;

// ---------------------------------------------------------------------------
// Helper: create a clamped uniform knot vector for n control points of given
// degree.  knots = [0,...,0, internal, 1,...,1]  with (degree+1) clamped ends.
// ---------------------------------------------------------------------------
static std::vector<double> clampedKnots(int n, int degree) {
    std::vector<double> knots(n + degree + 1);
    for (int i = 0; i <= degree; ++i) knots[i] = 0.0;
    for (int i = degree + 1; i < n; ++i) knots[i] = static_cast<double>(i - degree) / (n - degree);
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
    EXPECT_GT(centerWeighted.z, centerUniform.z);

    // And exactly as far as the homogeneous tensor product says.  At u = v =
    // 0.5 every quadratic basis function of a single-span clamped knot vector
    // is (1/4, 1/2, 1/4), and only the centre control point has z != 0:
    //     z = (1/2 * 1/2 * 10 * 10) / sum_ij B_i B_j w_ij
    const double B[3] = {0.25, 0.5, 0.25};
    double denom = 0.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) denom += B[i] * B[j] * wtsWeighted[i][j];
    }
    EXPECT_NEAR(centerWeighted.z, 0.25 * 10.0 * 10.0 / denom, 1e-12);
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

// ===========================================================================
// Task 3: Closest-Point & Iso-Curves
// ===========================================================================

// ---------------------------------------------------------------------------
// 10. closestPoint on a flat bilinear surface
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, ClosestPointOnFlatSurface) {
    // Bilinear surface: (u, v) in [0,1]^2 → (10*v, 10*u, 0).
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

    // Point above (5, 5, 0) → should find (u≈0.5, v≈0.5).
    auto [u, v] = srf.closestPoint({5.0, 5.0, 10.0});
    EXPECT_NEAR(u, 0.5, 1e-3);
    EXPECT_NEAR(v, 0.5, 1e-3);

    // Point above corner (0, 0) → (u≈0, v≈0).
    auto [u2, v2] = srf.closestPoint({0.0, 0.0, 5.0});
    EXPECT_NEAR(u2, 0.0, 1e-3);
    EXPECT_NEAR(v2, 0.0, 1e-3);

    // Point above (10, 10) → (u≈1, v≈1).
    auto [u3, v3] = srf.closestPoint({10.0, 10.0, 5.0});
    EXPECT_NEAR(u3, 1.0, 1e-3);
    EXPECT_NEAR(v3, 1.0, 1e-3);
}

// ---------------------------------------------------------------------------
// 11. closestPoint on a dome surface
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, ClosestPointOnDome) {
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

    // Point far above center → closest point near center.
    auto [u, v] = srf.closestPoint({5.0, 5.0, 100.0});
    EXPECT_NEAR(u, 0.5, 0.1);
    EXPECT_NEAR(v, 0.5, 0.1);

    // Verify the found point is actually close to the surface.
    Vec3 surfPt = srf.evaluate(u, v);
    Vec3 query{5.0, 5.0, 100.0};
    double dist = surfPt.distanceTo(query);
    // The center of the dome is at z ≈ 2.5, so distance should be about 97.5.
    EXPECT_LT(dist, 98.0);
}

// ---------------------------------------------------------------------------
// 12. isoCurveU at midpoint
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, IsoCurveU) {
    // Bilinear surface: x = 10*v, y = 10*u, z = 0.
    // isoCurveU(0.5) fixes u=0.5 → curve from (0, 5, 0) to (10, 5, 0).
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

    NurbsCurve iso = srf.isoCurveU(0.5);

    // Start point should be at (0, 5, 0).
    Vec3 pStart = iso.evaluate(iso.tMin());
    EXPECT_NEAR(pStart.x, 0.0, 1e-6);
    EXPECT_NEAR(pStart.y, 5.0, 1e-6);
    EXPECT_NEAR(pStart.z, 0.0, 1e-6);

    // End point should be at (10, 5, 0).
    Vec3 pEnd = iso.evaluate(iso.tMax());
    EXPECT_NEAR(pEnd.x, 10.0, 1e-6);
    EXPECT_NEAR(pEnd.y, 5.0, 1e-6);
    EXPECT_NEAR(pEnd.z, 0.0, 1e-6);

    // Mid-point should be at (5, 5, 0).
    double tMid = (iso.tMin() + iso.tMax()) * 0.5;
    Vec3 pMid = iso.evaluate(tMid);
    EXPECT_NEAR(pMid.x, 5.0, 0.5);
    EXPECT_NEAR(pMid.y, 5.0, 0.5);
}

// ---------------------------------------------------------------------------
// 13. isoCurveV at midpoint
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, IsoCurveV) {
    // Bilinear surface: isoCurveV(0.5) fixes v=0.5 → curve from (5, 0, 0) to (5, 10, 0).
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

    NurbsCurve iso = srf.isoCurveV(0.5);

    // Start point should be at (5, 0, 0).
    Vec3 pStart = iso.evaluate(iso.tMin());
    EXPECT_NEAR(pStart.x, 5.0, 1e-6);
    EXPECT_NEAR(pStart.y, 0.0, 1e-6);

    // End point should be at (5, 10, 0).
    Vec3 pEnd = iso.evaluate(iso.tMax());
    EXPECT_NEAR(pEnd.x, 5.0, 1e-6);
    EXPECT_NEAR(pEnd.y, 10.0, 1e-6);
}

// ===========================================================================
// Task 4: Tessellation
// ===========================================================================

// ---------------------------------------------------------------------------
// 14. Tessellate flat surface — at least 4 vertices, valid indices
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, TessellateFlatSurface) {
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

    auto result = srf.tessellate(1.0);

    // At least 4 vertices (corners).
    int numVerts = static_cast<int>(result.positions.size()) / 3;
    EXPECT_GE(numVerts, 4);

    // Normals array same size as positions.
    EXPECT_EQ(result.normals.size(), result.positions.size());

    // At least 2 triangles (6 indices).
    EXPECT_GE(result.indices.size(), 6u);

    // All indices in range.
    for (uint32_t idx : result.indices) {
        EXPECT_LT(idx, static_cast<uint32_t>(numVerts));
    }

    // Index count is a multiple of 3 (triangles).
    EXPECT_EQ(result.indices.size() % 3, 0u);
}

// ---------------------------------------------------------------------------
// 15. Finer tolerance → more triangles
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, TessellateCurvedHasMoreTriangles) {
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

    auto coarse = srf.tessellate(1.0);  // resolution ~10
    auto fine = srf.tessellate(0.1);    // resolution ~100

    // Finer tolerance → more triangles.
    EXPECT_GT(fine.indices.size(), coarse.indices.size());
}

// ---------------------------------------------------------------------------
// 16. Tessellation is watertight — all indices in range
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, TessellationIsWatertight) {
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

    auto result = srf.tessellate(0.5);
    int numVerts = static_cast<int>(result.positions.size()) / 3;

    // All indices in valid range.
    for (uint32_t idx : result.indices) {
        EXPECT_LT(idx, static_cast<uint32_t>(numVerts));
    }

    // Every vertex has a corresponding normal.
    EXPECT_EQ(result.normals.size(), result.positions.size());
}

// ===========================================================================
// Task 5: Factory Surfaces
// ===========================================================================

// ---------------------------------------------------------------------------
// 17. makePlane — center point at expected position
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, PlanarSurface) {
    auto srf = NurbsSurface::makePlane({0, 0, 0}, {1, 0, 0}, {0, 1, 0}, 10.0, 10.0);

    // Center (u=0.5, v=0.5) should be at (5, 5, 0).
    Vec3 center = srf.evaluate(0.5, 0.5);
    EXPECT_NEAR(center.x, 5.0, 1e-6);
    EXPECT_NEAR(center.y, 5.0, 1e-6);
    EXPECT_NEAR(center.z, 0.0, 1e-6);

    // Corner (0,0) = origin.
    Vec3 p00 = srf.evaluate(0.0, 0.0);
    EXPECT_NEAR(p00.x, 0.0, 1e-6);
    EXPECT_NEAR(p00.y, 0.0, 1e-6);

    // Corner (1,1) = (10, 10, 0).
    Vec3 p11 = srf.evaluate(1.0, 1.0);
    EXPECT_NEAR(p11.x, 10.0, 1e-6);
    EXPECT_NEAR(p11.y, 10.0, 1e-6);
}

// ---------------------------------------------------------------------------
// 18. makeCylinder — all points at correct radius from axis
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, CylindricalSurface) {
    const double radius = 5.0;
    const double height = 10.0;
    auto srf = NurbsSurface::makeCylinder({0, 0, 0}, {0, 0, 1}, radius, height);

    // Sample many points and check distance from Z axis.
    for (int i = 0; i <= 10; ++i) {
        double u = static_cast<double>(i) / 10.0;
        // Clamp to valid domain.
        u = std::min(u, srf.uMax());
        u = std::max(u, srf.uMin());
        for (int j = 0; j <= 10; ++j) {
            double v = static_cast<double>(j) / 10.0;
            v = std::min(v, srf.vMax());
            v = std::max(v, srf.vMin());

            Vec3 pt = srf.evaluate(u, v);
            // Distance from Z axis = sqrt(x^2 + y^2).
            double distFromAxis = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            EXPECT_NEAR(distFromAxis, radius, 1e-12)
                << "u=" << u << " v=" << v << " pt=(" << pt.x << "," << pt.y << "," << pt.z << ")";

            // Height should be in [0, height].
            EXPECT_GE(pt.z, -0.1);
            EXPECT_LE(pt.z, height + 0.1);
        }
    }
}

// ---------------------------------------------------------------------------
// 19. makeSphere — all points at correct distance from center
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, SphericalSurface) {
    const double radius = 3.0;
    Vec3 center{1.0, 2.0, 3.0};
    auto srf = NurbsSurface::makeSphere(center, radius);

    // Sample many points and check distance from center.  The two-pass
    // evaluation used to drop the U-direction weights, putting points up to
    // 6% of the radius off the sphere; it is now exact to rounding.
    for (int i = 0; i <= 10; ++i) {
        double u = srf.uMin() + (srf.uMax() - srf.uMin()) * static_cast<double>(i) / 10.0;
        for (int j = 0; j <= 10; ++j) {
            double v = srf.vMin() + (srf.vMax() - srf.vMin()) * static_cast<double>(j) / 10.0;

            Vec3 pt = srf.evaluate(u, v);
            double dist = pt.distanceTo(center);
            EXPECT_NEAR(dist, radius, 1e-12) << "u=" << u << " v=" << v << " pt=(" << pt.x << ","
                                             << pt.y << "," << pt.z << ") dist=" << dist;
        }
    }
}

// ---------------------------------------------------------------------------
// 20. makeTorus — outer point at major+minor distance
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, ToroidalSurface) {
    const double majorR = 5.0;
    const double minorR = 2.0;
    auto srf = NurbsSurface::makeTorus({0, 0, 0}, {0, 0, 1}, majorR, minorR);

    // At u=0 (angle 0 in revolution), v=0 (angle 0 in minor circle):
    // The point should be at distance majorR + minorR from the Z axis, z=0.
    Vec3 outerPt = srf.evaluate(srf.uMin(), srf.vMin());
    double distFromAxis = std::sqrt(outerPt.x * outerPt.x + outerPt.y * outerPt.y);
    EXPECT_NEAR(distFromAxis, majorR + minorR, 1e-12);
    EXPECT_NEAR(outerPt.z, 0.0, 1e-12);

    // Sample several points: each should be at distance minorR from the
    // circle of radius majorR in the XY plane.
    for (int i = 0; i <= 8; ++i) {
        double u = srf.uMin() + (srf.uMax() - srf.uMin()) * static_cast<double>(i) / 8.0;
        for (int j = 0; j <= 8; ++j) {
            double v = srf.vMin() + (srf.vMax() - srf.vMin()) * static_cast<double>(j) / 8.0;

            Vec3 pt = srf.evaluate(u, v);

            // Every point is minorR from the circle of radius majorR in XY.
            double axialDist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            EXPECT_NEAR(std::hypot(axialDist - majorR, pt.z), minorR, 1e-12);
        }
    }
}

// ---------------------------------------------------------------------------
// 21. makeCone — radius varies linearly from 0 at apex to base
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, ConicalSurface) {
    const double halfAngle = kPi / 6.0;  // 30 degrees
    const double height = 10.0;
    const double baseRadius = height * std::tan(halfAngle);

    auto srf = NurbsSurface::makeCone({0, 0, 0}, {0, 0, 1}, halfAngle, height);

    // At v=0 (apex): all points collapse to apex (0,0,0).
    Vec3 apexPt = srf.evaluate(srf.uMin(), srf.vMin());
    EXPECT_NEAR(apexPt.distanceTo({0, 0, 0}), 0.0, 1e-12);

    // At v=1 (base): points should be at radius baseRadius from the Z axis at z=height.
    for (int i = 0; i <= 8; ++i) {
        double u = srf.uMin() + (srf.uMax() - srf.uMin()) * static_cast<double>(i) / 8.0;
        Vec3 pt = srf.evaluate(u, srf.vMax());
        double distFromAxis = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        EXPECT_NEAR(distFromAxis, baseRadius, 1e-12)
            << "u=" << u << " pt=(" << pt.x << "," << pt.y << "," << pt.z << ")";
        EXPECT_NEAR(pt.z, height, 1e-12);
    }
}

// ---------------------------------------------------------------------------
// Rational surfaces are exact between knots, not only at them.  The two-pass
// evaluation used to pass unit weights to its second pass, so off-knot points
// of a cylinder, sphere, torus or cone sat up to ~6% of the radius off the
// surface (0.30 on a radius-5 cylinder) while every knot value was exact.
// ---------------------------------------------------------------------------
TEST(NurbsSurfaceTest, RationalSurfacesAreExactBetweenKnots) {
    const auto cyl = NurbsSurface::makeCylinder({0, 0, 0}, {0, 0, 1}, 5.0, 10.0);
    const auto sph = NurbsSurface::makeSphere({0, 0, 0}, 3.0);
    const auto tor = NurbsSurface::makeTorus({0, 0, 0}, {0, 0, 1}, 5.0, 2.0);
    const auto con = NurbsSurface::makeCone({0, 0, 0}, {0, 0, 1}, kPi / 6.0, 10.0);
    const int n = 97;  // prime, so samples land between knots
    for (int i = 0; i <= n; ++i) {
        for (int j = 0; j <= n; ++j) {
            const double s = static_cast<double>(i) / n;
            const double t = static_cast<double>(j) / n;
            auto at = [&](const NurbsSurface& f) {
                return f.evaluate(f.uMin() + (f.uMax() - f.uMin()) * s,
                                  f.vMin() + (f.vMax() - f.vMin()) * t);
            };
            const Vec3 c = at(cyl);
            EXPECT_NEAR(std::hypot(c.x, c.y), 5.0, 1e-12);
            EXPECT_NEAR(at(sph).length(), 3.0, 1e-12);
            const Vec3 q = at(tor);
            EXPECT_NEAR(std::hypot(std::hypot(q.x, q.y) - 5.0, q.z), 2.0, 1e-12);
            const Vec3 k = at(con);
            EXPECT_NEAR(std::hypot(k.x, k.y), k.z * std::tan(kPi / 6.0), 1e-12);
        }
    }
}

// A sphere octant: exact, bounded by the three quarter circles between its
// axes, and facing out when the axes are right-handed.
TEST(NurbsSurfaceTest, SphereOctantIsOneEighthOfTheSphere) {
    const hz::math::Vec3 c(1.0, 2.0, 3.0);
    const double r = 2.5;
    const hz::math::Vec3 e1(0, -1, 0), e2(-1, 0, 0), e3(0, 0, -1);  // right-handed, pointing back
    ASSERT_GT(e1.dot(e2.cross(e3)), 0.0);
    const auto octant = hz::geo::NurbsSurface::makeSphereOctant(c, r, e1, e2, e3);

    EXPECT_NEAR(octant.evaluate(0, 0).distanceTo(c + e1 * r), 0.0, 1e-12);
    EXPECT_NEAR(octant.evaluate(1, 0).distanceTo(c + e2 * r), 0.0, 1e-12);
    EXPECT_NEAR(octant.evaluate(0.3, 1).distanceTo(c + e3 * r), 0.0, 1e-12);
    for (double u = 0.0; u <= 1.0; u += 0.125) {
        for (double v = 0.0; v <= 1.0; v += 0.125) {
            const hz::math::Vec3 p = octant.evaluate(u, v);
            EXPECT_NEAR((p - c).length(), r, 1e-12) << u << ", " << v;
            // Inside the octant: no component against an axis.
            EXPECT_GE((p - c).dot(e1), -1e-12);
            EXPECT_GE((p - c).dot(e2), -1e-12);
            EXPECT_GE((p - c).dot(e3), -1e-12);
        }
    }
    const hz::math::Vec3 mid = octant.evaluate(0.5, 0.5);
    EXPECT_GT(octant.normal(0.5, 0.5).dot(mid - c), 0.0) << "faces away from the center";
}

// ===========================================================================
// Exact derivatives and projection (Phase 141)
// ===========================================================================

namespace {

/// Every kind of surface the kernel makes, and a non-uniform rational cubic.
std::vector<NurbsSurface> sampleSurfaces() {
    std::vector<NurbsSurface> out;
    out.push_back(NurbsSurface::makeCylinder(Vec3(1, 2, 3), Vec3(0.2, 0.1, 1), 2.5, 4.0));
    out.push_back(NurbsSurface::makeSphere(Vec3(-1, 0, 2), 3.0));
    out.push_back(NurbsSurface::makeTorus(Vec3(0, 1, 0), Vec3(0, 0, 1), 5.0, 1.5));
    out.push_back(NurbsSurface::makeCone(Vec3(0, 0, 0), Vec3(1, 0, 1), 0.4, 3.0));
    out.push_back(NurbsSurface::makeSphereOctant(Vec3(1, 1, 1), 2.0, Vec3(1, 0, 0), Vec3(0, 1, 0),
                                                 Vec3(0, 0, 1)));
    std::vector<std::vector<Vec3>> pts(5, std::vector<Vec3>(4));
    std::vector<std::vector<double>> wts(5, std::vector<double>(4));
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 4; ++j) {
            pts[i][j] = Vec3(i, j, std::sin(i * 1.3) * std::cos(j * 0.7) + 0.1 * i * j);
            wts[i][j] = 1.0 + 0.3 * ((i + 2 * j) % 3);
        }
    }
    out.emplace_back(pts, wts, std::vector<double>{0, 0, 0, 0, 0.3, 1, 1, 1, 1},
                     std::vector<double>{0, 0, 0, 0.6, 1, 1, 1}, 3, 2);
    return out;
}

}  // namespace

TEST(NurbsSurfaceTest, ExactDerivativesAgreeWithTheSurface) {
    for (const auto& s : sampleSurfaces()) {
        for (int i = 0; i <= 10; ++i) {
            for (int j = 0; j <= 10; ++j) {
                const double u = s.uMin() + (s.uMax() - s.uMin()) * (0.013 + 0.97 * i / 10.0);
                const double v = s.vMin() + (s.vMax() - s.vMin()) * (0.017 + 0.96 * j / 10.0);
                const SurfacePoint p = s.evaluateWithDerivatives(u, v);
                EXPECT_LT((p.point - s.evaluate(u, v)).length(), 1e-12);
                // Central differences, accurate to h^2.
                const double h = 1e-6;
                const Vec3 du = (s.evaluate(u + h, v) - s.evaluate(u - h, v)) * (0.5 / h);
                const Vec3 dv = (s.evaluate(u, v + h) - s.evaluate(u, v - h)) * (0.5 / h);
                EXPECT_LT((p.du - du).length(), 1e-6 * std::max(1.0, du.length()));
                EXPECT_LT((p.dv - dv).length(), 1e-6 * std::max(1.0, dv.length()));
            }
        }
    }
}

// Where the surface comes to a point (a sphere's poles, a cone's apex) its
// tangents vanish or are parallel, and its normal is the one it comes to:
// the normal just beside the point, whichever way that faces. Not a fixed +Z,
// which is wrong at a south pole, and at every pole of a turned sphere.
TEST(NurbsSurfaceTest, TheNormalAtAPoleIsTheOneBesideIt) {
    const auto sphere = NurbsSurface::makeSphere(Vec3(1, 2, 3), 2.0);
    const auto cone = NurbsSurface::makeCone(Vec3(0, 0, 0), Vec3(1, 0, 0), 0.5, 4.0);
    for (const NurbsSurface* s : {&sphere, &cone}) {
        const double du = 1e-4 * (s->uMax() - s->uMin());
        const double dv = 1e-4 * (s->vMax() - s->vMin());
        for (const double u : {s->uMin() + 0.3 * (s->uMax() - s->uMin()),
                               s->uMin() + 0.8 * (s->uMax() - s->uMin())}) {
            for (const double v : {s->vMin(), s->vMax()}) {
                const auto at = s->evaluateWithDerivatives(u, v);
                if (at.du.cross(at.dv).length() > 1e-9) continue;  // no pole here
                const double inside = v == s->vMin() ? v + dv : v - dv;
                const Vec3 beside = s->normal(u + du, inside);
                EXPECT_GT(s->normal(u, v).dot(beside), 0.99) << "at v = " << v;
            }
        }
    }
}

TEST(NurbsSurfaceTest, AClosedSurfaceKnowsItsSeam) {
    const auto cylinder = NurbsSurface::makeCylinder(Vec3(), Vec3(0, 0, 1), 2.0, 3.0);
    EXPECT_TRUE(cylinder.closedU());
    EXPECT_FALSE(cylinder.closedV());
    const auto torus = NurbsSurface::makeTorus(Vec3(), Vec3(0, 0, 1), 5.0, 1.0);
    EXPECT_TRUE(torus.closedU());
    EXPECT_TRUE(torus.closedV());
    const auto plane = NurbsSurface::makePlane(Vec3(), Vec3(1, 0, 0), Vec3(0, 1, 0), 2.0, 2.0);
    EXPECT_FALSE(plane.closedU());
    EXPECT_FALSE(plane.closedV());
}

// A closed surface cut to close elsewhere: the same surface, its
// parameters starting where it now closes, a period round; where it closed,
// now inside it. Not closed that way: nothing to cut.
TEST(NurbsSurfaceTest, AClosedSurfaceIsCutToCloseWhereAsked) {
    const auto same = [](const NurbsSurface& cut, const NurbsSurface& s, bool inU, double at) {
        const double period = inU ? s.uMax() - s.uMin() : s.vMax() - s.vMin();
        const double start = inU ? s.uMin() : s.vMin();
        for (int i = 0; i <= 16; ++i) {
            for (int j = 0; j <= 16; ++j) {
                const double along = at + period * i / 16.0;  // from where it now closes
                double wrapped = along;
                if (wrapped > start + period) wrapped -= period;
                const double across = (inU ? s.vMin() + (s.vMax() - s.vMin()) * j / 16.0
                                           : s.uMin() + (s.uMax() - s.uMin()) * j / 16.0);
                const Vec3 a = inU ? cut.evaluate(along, across) : cut.evaluate(across, along);
                const Vec3 b = inU ? s.evaluate(wrapped, across) : s.evaluate(across, wrapped);
                EXPECT_LT((a - b).length(), 1e-9) << i << " " << j;
            }
        }
    };
    const auto cylinder = NurbsSurface::makeCylinder(Vec3(1, 2, 3), Vec3(0, 0, 1), 2.0, 3.0);
    const double u = cylinder.uMin() + 0.3 * (cylinder.uMax() - cylinder.uMin());
    const auto cut = cylinder.startingAtU(u);
    ASSERT_TRUE(cut.has_value());
    EXPECT_NEAR(cut->uMin(), u, 1e-12);
    EXPECT_NEAR(cut->uMax() - cut->uMin(), cylinder.uMax() - cylinder.uMin(), 1e-12);
    EXPECT_TRUE(cut->closedU());
    same(*cut, cylinder, true, u);
    // On a knot, too (a quarter round), where no knot is put in.
    const double quarter = cylinder.uMin() + 0.25 * (cylinder.uMax() - cylinder.uMin());
    const auto onKnot = cylinder.startingAtU(quarter);
    ASSERT_TRUE(onKnot.has_value());
    EXPECT_EQ(onKnot->controlPointCountU(), cylinder.controlPointCountU());
    same(*onKnot, cylinder, true, quarter);

    const auto torus = NurbsSurface::makeTorus(Vec3(), Vec3(0, 0, 1), 5.0, 1.0);
    const double v = torus.vMin() + 0.7 * (torus.vMax() - torus.vMin());
    const auto turned = torus.startingAtV(v);
    ASSERT_TRUE(turned.has_value());
    EXPECT_TRUE(turned->closedU());
    EXPECT_TRUE(turned->closedV());
    same(*turned, torus, false, v);

    // Where it closes already: itself.
    const auto asIs = cylinder.startingAtU(cylinder.uMin());
    ASSERT_TRUE(asIs.has_value());
    EXPECT_EQ(asIs->knotsU(), cylinder.knotsU());
    // Not closed that way: nothing.
    EXPECT_FALSE(cylinder.startingAtV(1.0).has_value());
    const auto plane = NurbsSurface::makePlane(Vec3(), Vec3(1, 0, 0), Vec3(0, 1, 0), 2.0, 2.0);
    EXPECT_FALSE(plane.startingAtU(0.5).has_value());
}

// A point off the surface projects to its foot, the start being near: on the
// far side of a seam, and beside a pole, too.
TEST(NurbsSurfaceTest, APointProjectsToItsFoot) {
    for (const auto& s : sampleSurfaces()) {
        for (int i = 0; i <= 8; ++i) {
            for (int j = 0; j <= 8; ++j) {
                const double u = s.uMin() + (s.uMax() - s.uMin()) * (0.05 + 0.9 * i / 8.0);
                const double v = s.vMin() + (s.vMax() - s.vMin()) * (0.05 + 0.9 * j / 8.0);
                const Vec3 foot = s.evaluate(u, v);
                const Vec3 off = foot + s.normal(u, v) * 0.01;
                const double startU = u + 0.02 * (s.uMax() - s.uMin());
                const double startV = v - 0.02 * (s.vMax() - s.vMin());
                const auto [pu, pv] = s.project(off, startU, startV);
                EXPECT_LT((s.evaluate(pu, pv) - foot).length(), 1e-9) << u << " " << v;
            }
        }
    }
    // Round the cylinder's seam: started just before it, the foot just after.
    const auto cylinder = NurbsSurface::makeCylinder(Vec3(), Vec3(0, 0, 1), 2.0, 3.0);
    const double period = cylinder.uMax() - cylinder.uMin();
    const Vec3 after = cylinder.evaluate(cylinder.uMin() + 0.01 * period, 1.0);
    const auto [au, av] = cylinder.project(after * 1.001, cylinder.uMax() - 0.01 * period, 1.2);
    EXPECT_LT((cylinder.evaluate(au, av) - after).length(), 1e-9);
    // On the seam itself it settles, not stepping round and back: its steps
    // land a period apart, which it took for moving, and it searched on.
    const Vec3 onSeam = cylinder.evaluate(cylinder.uMin(), 1.5);
    int settled = 0;
    for (const double start : {cylinder.uMin() + 1e-3, cylinder.uMax() - 1e-3}) {
        const auto [su, sv] = cylinder.project(onSeam, start, 1.2);
        EXPECT_LT((cylinder.evaluate(su, sv) - onSeam).length(), 1e-12);
        settled += 1;
    }
    EXPECT_EQ(settled, 2);
    // Beside the sphere's pole, where dS/du vanishes.
    const auto sphere = NurbsSurface::makeSphere(Vec3(), 1.0);
    // From well away, towards a point just short of either pole: a full
    // step overshoots the pole, and clamped there the search stuck, u no
    // longer moving the point off it.
    for (const double z : {-1.0, 1.0}) {
        for (int k = 0; k < 16; ++k) {
            const double angle = 2.0 * kPi * k / 16.0;
            const Vec3 target = Vec3(0.2 * std::cos(angle), 0.2 * std::sin(angle), z).normalized();
            const double startV = z < 0 ? sphere.vMin() + 0.3 * (sphere.vMax() - sphere.vMin())
                                        : sphere.vMax() - 0.3 * (sphere.vMax() - sphere.vMin());
            const auto [pu, pv] = sphere.project(target * 0.99, 0.37, startV);
            EXPECT_LT((sphere.evaluate(pu, pv) - target).length(), 1e-9) << z << " " << k;
        }
    }
    const Vec3 nearPole = Vec3(0.001, 0.002, 1.0).normalized();
    const auto [su, sv] = sphere.project(nearPole * 1.01, 0.3, sphere.vMax() - 0.01);
    EXPECT_LT((sphere.evaluate(su, sv) - nearPole).length(), 1e-9);
}

// ===========================================================================
// Exact normals; refused inputs; a tolerance too fine for an int
// ===========================================================================

// A cylinder's normal is radial at every point, edges and seam included, to
// rounding: derivativeU and derivativeV differenced evaluate() 1e-7 apart,
// so a normal was good to about 1e-7 in direction (and taken from one side
// at an edge), the size of what decides whether a face is flat.
TEST(NurbsSurfaceTest, ACylindersNormalIsRadialEverywhere) {
    const auto cyl = NurbsSurface::makeCylinder({0, 0, 0}, {0, 0, 1}, 5.0, 10.0);
    for (int i = 0; i <= 8; ++i) {
        for (int j = 0; j <= 4; ++j) {
            const double u = cyl.uMin() + (cyl.uMax() - cyl.uMin()) * i / 8.0;
            const double v = cyl.vMin() + (cyl.vMax() - cyl.vMin()) * j / 4.0;
            const Vec3 p = cyl.evaluate(u, v);
            const Vec3 radial = Vec3(p.x, p.y, 0.0).normalized();
            const Vec3 n = cyl.normal(u, v);
            EXPECT_NEAR(std::abs(n.dot(radial)), 1.0, 1e-13) << u << ", " << v;
            const SurfacePoint sp = cyl.evaluateWithDerivatives(u, v);
            EXPECT_NEAR((cyl.derivativeU(u, v) - sp.du).length(), 0.0, 1e-12);
            EXPECT_NEAR((cyl.derivativeV(u, v) - sp.dv).length(), 0.0, 1e-12);
        }
    }
}

TEST(NurbsSurfaceTest, WeightsThatDoNotFitAreRefused) {
    const std::vector<std::vector<Vec3>> pts = {
        {{0, 0, 0}, {0, 1, 0}}, {{1, 0, 0}, {1, 1, 0}}, {{2, 0, 0}, {2, 1, 0}}};
    const auto knotsU = clampedKnots(3, 1);
    const auto knotsV = clampedKnots(2, 1);
    // Two rows of weights for three rows of points read past the end.
    EXPECT_THROW(NurbsSurface(pts, {{1, 1}, {1, 1}}, knotsU, knotsV, 1, 1), std::invalid_argument);
    EXPECT_THROW(NurbsSurface(pts, {{1, 1}, {1, 0}, {1, 1}}, knotsU, knotsV, 1, 1),
                 std::invalid_argument)
        << "a zero weight";
    EXPECT_THROW(NurbsSurface(pts, {{1, 1}, {1, -2}, {1, 1}}, knotsU, knotsV, 1, 1),
                 std::invalid_argument)
        << "a negative weight";
    EXPECT_THROW(NurbsSurface(pts, {{1, 1}, {1, 1}, {1, 1}}, {0, 0, 1, 0.5, 1}, knotsV, 1, 1),
                 std::invalid_argument)
        << "knots that go back";
    EXPECT_NO_THROW(NurbsSurface(pts, {{1, 1}, {1, 1}, {1, 1}}, knotsU, knotsV, 1, 1));
}

// 10 / tolerance for a tolerance of 0, or under about 5e-9, is past an int;
// converting it was undefined, and on x86 the finest tolerance gave the
// coarsest mesh. The finest asked for is the finest there is.
TEST(NurbsSurfaceTest, AVeryFineToleranceGivesTheFinestMesh) {
    const auto dome =
        NurbsSurface::makeSphereOctant({0, 0, 0}, 1.0, {1, 0, 0}, {0, 1, 0}, {0, 0, 1});
    const size_t finest = dome.tessellate(0.05).positions.size();
    EXPECT_EQ(finest, size_t{201 * 201 * 3});
    for (const double tol : {1e-9, 0.0, -1.0, 1e-300}) {
        EXPECT_EQ(dome.tessellate(tol).positions.size(), finest) << tol;
    }
}

// ===========================================================================
// NURBS surfaces as typed surfaces: second derivatives, invert, transformed,
// knot insertion and splitting
// ===========================================================================

namespace {

void expectVecNear(const Vec3& a, const Vec3& b, double tol) {
    EXPECT_NEAR(a.x, b.x, tol);
    EXPECT_NEAR(a.y, b.y, tol);
    EXPECT_NEAR(a.z, b.z, tol);
}

/// Parameter fractions away from seams, poles and the interior knots used here.
const std::vector<double> kFractions = {0.13, 0.37, 0.61, 0.83};

/// A degree-(3,2) non-rational patch with interior knots, off any axis.
NurbsSurface makeBicubicQuadraticPatch() {
    const int nu = 6;
    const int nv = 5;
    std::vector<std::vector<Vec3>> pts(nu, std::vector<Vec3>(nv));
    std::vector<std::vector<double>> wts(nu, std::vector<double>(nv, 1.0));
    for (int i = 0; i < nu; ++i) {
        for (int j = 0; j < nv; ++j) {
            pts[i][j] = {i * 1.0 + 0.1 * j * j, j * 1.3 - 0.2 * i, std::sin(0.9 * i + 1.7 * j)};
        }
    }
    return NurbsSurface(pts, wts, clampedKnots(nu, 3), clampedKnots(nv, 2), 3, 2);
}

/// A rational patch with uneven weights and a repeated interior knot.
NurbsSurface makeWeightedPatch() {
    const int nu = 5;
    const int nv = 4;
    std::vector<std::vector<Vec3>> pts(nu, std::vector<Vec3>(nv));
    std::vector<std::vector<double>> wts(nu, std::vector<double>(nv));
    for (int i = 0; i < nu; ++i) {
        for (int j = 0; j < nv; ++j) {
            pts[i][j] = {i * 1.1, j * 0.9 + 0.1 * i, std::cos(1.1 * i - 0.7 * j)};
            wts[i][j] = 1.0 + 0.3 * ((i * 3 + j * 5) % 4);
        }
    }
    return NurbsSurface(pts, wts, {0, 0, 0, 0.4, 0.4, 1, 1, 1}, clampedKnots(nv, 2), 2, 2);
}

std::vector<std::pair<std::string, NurbsSurface>> typedSamples() {
    return {{"cylinder", NurbsSurface::makeCylinder({0.5, -1, 2}, {0.2, 0.3, 1}, 1.5, 3.0)},
            {"sphere", NurbsSurface::makeSphere({1, 2, 3}, 2.0)},
            {"torus", NurbsSurface::makeTorus({0, 1, 0}, {0.1, 1, 0.3}, 3.0, 0.8)},
            {"patch32", makeBicubicQuadraticPatch()},
            {"weighted", makeWeightedPatch()}};
}

double at(double lo, double hi, double f) {
    return lo + (hi - lo) * f;
}

}  // namespace

TEST(NurbsSurfaceTyped, IsASurfaceOfKindNurbsWithItsRangesAndPeriods) {
    const NurbsSurface torus = NurbsSurface::makeTorus({0, 0, 0}, {0, 0, 1}, 3.0, 1.0);
    const Surface& s = torus;
    EXPECT_EQ(s.kind(), SurfaceKind::Nurbs);
    EXPECT_EQ(s.uRange().min, torus.uMin());
    EXPECT_EQ(s.uRange().max, torus.uMax());
    EXPECT_EQ(s.vRange().min, torus.vMin());
    EXPECT_EQ(s.vRange().max, torus.vMax());
    EXPECT_TRUE(s.periodicU());
    EXPECT_TRUE(s.periodicV());
    const NurbsSurface patch = makeBicubicQuadraticPatch();
    EXPECT_FALSE(static_cast<const Surface&>(patch).periodicU());
    EXPECT_FALSE(static_cast<const Surface&>(patch).periodicV());
    // The interface's normal is the class's own.
    expectVecNear(s.normal(0.3, 0.6), torus.normal(0.3, 0.6), 0.0);
}

TEST(NurbsSurfaceTyped, SecondDerivativesAgreeWithDifferencesOfFirstDerivatives) {
    const double h = 1e-6;
    for (const auto& [name, s] : typedSamples()) {
        for (double fu : kFractions) {
            for (double fv : kFractions) {
                const double u = at(s.uMin(), s.uMax(), fu);
                const double v = at(s.vMin(), s.vMax(), fv);
                const SurfaceDerivatives d = s.derivatives(u, v);
                const SurfacePoint p = s.evaluateWithDerivatives(u, v);
                expectVecNear(d.point, s.evaluate(u, v), 1e-12);
                expectVecNear(d.du, p.du, 1e-12);
                expectVecNear(d.dv, p.dv, 1e-12);
                const auto first = [&](double a, double b) {
                    return s.evaluateWithDerivatives(a, b);
                };
                const Vec3 duu = (first(u + h, v).du - first(u - h, v).du) / (2 * h);
                const Vec3 duv = (first(u, v + h).du - first(u, v - h).du) / (2 * h);
                const Vec3 dvv = (first(u, v + h).dv - first(u, v - h).dv) / (2 * h);
                const Vec3 dvu = (first(u + h, v).dv - first(u - h, v).dv) / (2 * h);
                EXPECT_NEAR((d.duu - duu).length(), 0.0, 1e-5 * std::max(1.0, d.duu.length()))
                    << name;
                EXPECT_NEAR((d.duv - duv).length(), 0.0, 1e-5 * std::max(1.0, d.duv.length()))
                    << name;
                EXPECT_NEAR((d.duv - dvu).length(), 0.0, 1e-5 * std::max(1.0, d.duv.length()))
                    << name;
                EXPECT_NEAR((d.dvv - dvv).length(), 0.0, 1e-5 * std::max(1.0, d.dvv.length()))
                    << name;
            }
        }
    }
}

TEST(NurbsSurfaceTyped, InvertRoundTripsInteriorParameters) {
    for (const auto& [name, s] : typedSamples()) {
        for (double fu : kFractions) {
            for (double fv : kFractions) {
                const double u = at(s.uMin(), s.uMax(), fu);
                const double v = at(s.vMin(), s.vMax(), fv);
                const Vec2 uv = static_cast<const Surface&>(s).invert(s.evaluate(u, v));
                expectVecNear(s.evaluate(uv.x, uv.y), s.evaluate(u, v), 1e-9);
                EXPECT_NEAR(uv.x, u, 1e-7) << name;
                EXPECT_NEAR(uv.y, v, 1e-7) << name;
            }
        }
    }
}

TEST(NurbsSurfaceTyped, TransformedCommutesWithRigidScaleAndReflection) {
    const Mat4 rigid =
        Mat4::translation({3.0, -1.0, 2.0}) * Mat4::rotationZ(0.7) * Mat4::rotationX(-0.4);
    for (const Mat4& m : {rigid, Mat4::scale(2.5), Mat4::scale({-1.0, 1.0, 1.0})}) {
        for (const auto& [name, s] : typedSamples()) {
            const std::unique_ptr<Surface> t = static_cast<const Surface&>(s).transformed(m);
            ASSERT_NE(t, nullptr);
            EXPECT_EQ(t->kind(), SurfaceKind::Nurbs);
            for (double fu : kFractions) {
                for (double fv : kFractions) {
                    const double u = at(s.uMin(), s.uMax(), fu);
                    const double v = at(s.vMin(), s.vMax(), fv);
                    expectVecNear(t->evaluate(u, v), m.transformPoint(s.evaluate(u, v)), 1e-11);
                    // The copy is a good surface in its own right.
                    const SurfaceDerivatives d = t->derivatives(u, v);
                    const double h = 1e-6;
                    const Vec3 du = (t->evaluate(u + h, v) - t->evaluate(u - h, v)) / (2 * h);
                    expectVecNear(d.du, du, 1e-5 * std::max(1.0, d.du.length()));
                    const Vec2 uv = t->invert(t->evaluate(u, v));
                    EXPECT_NEAR(uv.x, u, 1e-7) << name;
                    EXPECT_NEAR(uv.y, v, 1e-7) << name;
                }
            }
        }
    }
    // A reflection reverses the normal of the mirrored image; weights and knots are kept.
    const NurbsSurface sphere = NurbsSurface::makeSphere({0, 0, 0}, 1.0);
    const auto mirrored = static_cast<const Surface&>(sphere).transformed(Mat4::scale({-1, 1, 1}));
    const Vec3 n0 = sphere.normal(0.3, 0.4);
    expectVecNear(mirrored->normal(0.3, 0.4), Vec3(n0.x, -n0.y, -n0.z), 1e-12);
    const auto* n = dynamic_cast<const NurbsSurface*>(mirrored.get());
    ASSERT_NE(n, nullptr);
    EXPECT_EQ(n->weights(), sphere.weights());
    EXPECT_EQ(n->knotsU(), sphere.knotsU());
    EXPECT_EQ(n->knotsV(), sphere.knotsV());
}

TEST(NurbsSurfaceTyped, InsertKnotLeavesTheSurfaceUnchanged) {
    for (const auto& [name, s] : typedSamples()) {
        const double u = at(s.uMin(), s.uMax(), 0.37);
        const double v = at(s.vMin(), s.vMax(), 0.61);
        // One fresh knot; one at an existing interior knot, where present.
        const NurbsSurface iu = s.insertKnotU(u);
        const NurbsSurface iv = s.insertKnotV(v);
        EXPECT_EQ(iu.knotsU().size(), s.knotsU().size() + 1) << name;
        EXPECT_EQ(iu.controlPointCountU(), s.controlPointCountU() + 1) << name;
        EXPECT_EQ(iu.knotsV(), s.knotsV()) << name;
        EXPECT_EQ(iv.knotsV().size(), s.knotsV().size() + 1) << name;
        EXPECT_EQ(iv.controlPointCountV(), s.controlPointCountV() + 1) << name;
        EXPECT_EQ(iv.knotsU(), s.knotsU()) << name;
        const NurbsSurface both = iu.insertKnotV(v).insertKnotU(u);  // u twice
        for (int i = 0; i <= 20; ++i) {
            for (int j = 0; j <= 20; ++j) {
                const double a = at(s.uMin(), s.uMax(), i / 20.0);
                const double b = at(s.vMin(), s.vMax(), j / 20.0);
                expectVecNear(iu.evaluate(a, b), s.evaluate(a, b), 1e-12);
                expectVecNear(iv.evaluate(a, b), s.evaluate(a, b), 1e-12);
                expectVecNear(both.evaluate(a, b), s.evaluate(a, b), 1e-12);
            }
        }
    }
}

TEST(NurbsSurfaceTyped, InsertKnotAtAnExistingInteriorKnotRaisesItsMultiplicity) {
    const NurbsSurface s = makeWeightedPatch();  // knotsU has 0.4 twice
    const NurbsSurface t = s.insertKnotU(0.4);
    EXPECT_EQ(t.knotsU().size(), s.knotsU().size() + 1);
    for (int i = 0; i <= 20; ++i) {
        for (int j = 0; j <= 20; ++j) {
            const double a = at(s.uMin(), s.uMax(), i / 20.0);
            const double b = at(s.vMin(), s.vMax(), j / 20.0);
            expectVecNear(t.evaluate(a, b), s.evaluate(a, b), 1e-12);
        }
    }
}

TEST(NurbsSurfaceTyped, SplitHalvesEvaluateAsTheOriginalOverTheirRanges) {
    for (const auto& [name, s] : typedSamples()) {
        for (double f : {0.37, 0.61}) {
            const double u = at(s.uMin(), s.uMax(), f);
            const double v = at(s.vMin(), s.vMax(), f);
            const auto [ul, ur] = s.splitU(u);
            const auto [vl, vr] = s.splitV(v);
            EXPECT_EQ(ul.uMin(), s.uMin()) << name;
            EXPECT_EQ(ul.uMax(), u) << name;
            EXPECT_EQ(ur.uMin(), u) << name;
            EXPECT_EQ(ur.uMax(), s.uMax()) << name;
            EXPECT_EQ(vl.vMin(), s.vMin()) << name;
            EXPECT_EQ(vl.vMax(), v) << name;
            EXPECT_EQ(vr.vMin(), v) << name;
            EXPECT_EQ(vr.vMax(), s.vMax()) << name;
            EXPECT_EQ(ul.vMax(), s.vMax()) << name;
            for (int i = 0; i <= 20; ++i) {
                for (int j = 0; j <= 20; ++j) {
                    const double b = at(s.vMin(), s.vMax(), j / 20.0);
                    const double a = at(s.uMin(), s.uMax(), i / 20.0);
                    const double lu = at(ul.uMin(), ul.uMax(), i / 20.0);
                    const double ru = at(ur.uMin(), ur.uMax(), i / 20.0);
                    expectVecNear(ul.evaluate(lu, b), s.evaluate(lu, b), 1e-12);
                    expectVecNear(ur.evaluate(ru, b), s.evaluate(ru, b), 1e-12);
                    const double lv = at(vl.vMin(), vl.vMax(), j / 20.0);
                    const double rv = at(vr.vMin(), vr.vMax(), j / 20.0);
                    expectVecNear(vl.evaluate(a, lv), s.evaluate(a, lv), 1e-12);
                    expectVecNear(vr.evaluate(a, rv), s.evaluate(a, rv), 1e-12);
                }
            }
        }
    }
}

TEST(NurbsSurfaceTyped, SplitAtAnExistingKnotWorks) {
    const NurbsSurface s = makeWeightedPatch();
    const auto [l, r] = s.splitU(0.4);  // already of multiplicity 2 = degree
    EXPECT_EQ(l.uMax(), 0.4);
    EXPECT_EQ(r.uMin(), 0.4);
    expectVecNear(l.evaluate(0.2, 0.5), s.evaluate(0.2, 0.5), 1e-12);
    expectVecNear(r.evaluate(0.7, 0.5), s.evaluate(0.7, 0.5), 1e-12);
    const auto [b, t] = makeBicubicQuadraticPatch().splitV(0.5);
    EXPECT_EQ(b.vMax(), 0.5);
    EXPECT_EQ(t.vMin(), 0.5);
}

TEST(NurbsSurfaceTyped, InsertingOrSplittingAtOrBeyondTheEndThrows) {
    const NurbsSurface s = makeBicubicQuadraticPatch();
    for (const double u : {s.uMin(), s.uMax(), s.uMin() - 1.0, s.uMax() + 1.0}) {
        EXPECT_THROW((void)s.insertKnotU(u), std::invalid_argument) << u;
        EXPECT_THROW((void)s.splitU(u), std::invalid_argument) << u;
    }
    for (const double v : {s.vMin(), s.vMax(), s.vMin() - 1.0, s.vMax() + 1.0}) {
        EXPECT_THROW((void)s.insertKnotV(v), std::invalid_argument) << v;
        EXPECT_THROW((void)s.splitV(v), std::invalid_argument) << v;
    }
    EXPECT_THROW((void)s.splitU(std::nan("")), std::invalid_argument);
}

TEST(NurbsSurfaceTyped, InvertingASeamPointStaysInTheHalfOpenRange) {
    const NurbsSurface cyl = NurbsSurface::makeCylinder({0.5, -1, 2}, {0.2, 0.3, 1}, 1.5, 3.0);
    const NurbsSurface torus = NurbsSurface::makeTorus({0, 1, 0}, {0.1, 1, 0.3}, 3.0, 0.8);
    for (const NurbsSurface* s : {&cyl, &torus}) {
        const Surface& surf = *s;
        for (const double f : {0.0, 0.1, 0.27, 0.5, 0.73, 0.9}) {
            const double v = at(s->vMin(), s->vMax(), f);
            const double u = at(s->uMin(), s->uMax(), f);
            std::vector<std::pair<double, double>> probes = {{s->uMin(), v}, {s->uMax(), v}};
            if (s == &torus) {
                probes.push_back({u, s->vMin()});
                probes.push_back({u, s->vMax()});
            }
            for (const auto& [pu, pv] : probes) {
                const Vec2 uv = surf.invert(s->evaluate(pu, pv));
                EXPECT_GE(uv.x, s->uMin());
                EXPECT_LT(uv.x, s->uMax());
                if (s->closedV()) {
                    EXPECT_GE(uv.y, s->vMin());
                    EXPECT_LT(uv.y, s->vMax());
                }
                expectVecNear(s->evaluate(uv.x, uv.y), s->evaluate(pu, pv), 1e-9);
            }
        }
    }
}

// A surface that closes is periodic, as Surface has it: a parameter past either end of its range
// is the point a period back, round the seam, and its derivatives are that point's. Clamped,
// a pcurve that ran back across a closed B-spline's seam evaluated at the seam itself (Task 5 of
// the exact kernel's stage 2 met it reading a cylinder written as a B-spline). A direction that
// does not close still clamps.
TEST(NurbsSurfaceTest, AClosedSurfaceEvaluatesRoundItsSeam) {
    const auto cylinder = NurbsSurface::makeCylinder(Vec3(1, 2, 3), Vec3(0, 0, 1), 2.0, 3.0);
    const double period = cylinder.uMax() - cylinder.uMin();
    const double v = cylinder.vMin() + 0.4 * (cylinder.vMax() - cylinder.vMin());
    for (const double f : {0.03, 0.25, 0.5, 0.97}) {
        const double u = cylinder.uMin() + f * period;
        for (const double turns : {-2.0, -1.0, 1.0, 3.0}) {
            const double far = u + turns * period;
            EXPECT_LT((cylinder.evaluate(far, v) - cylinder.evaluate(u, v)).length(), 1e-12);
            const auto a = cylinder.derivatives(far, v);
            const auto b = cylinder.derivatives(u, v);
            EXPECT_LT((a.du - b.du).length(), 1e-9);
            EXPECT_LT((a.dv - b.dv).length(), 1e-9);
            EXPECT_LT((cylinder.normal(far, v) - cylinder.normal(u, v)).length(), 1e-12);
        }
    }
    // Along the axis it does not close: past its end is its end.
    EXPECT_LT((cylinder.evaluate(cylinder.uMin(), cylinder.vMax() + 1.0) -
               cylinder.evaluate(cylinder.uMin(), cylinder.vMax()))
                  .length(),
              1e-12);
}

// A search lying along a patch's edge finds a point further along it: its step across the edge,
// a hair outward, is shortened alone. Shortened with it, the step along the edge came to
// nothing, the search stopped where it began, and a straight edge of a box written by Horizon
// read as standing up to 8 off its own face.
TEST(NurbsSurfaceTest, APointOnAPatchsEdgeIsFoundFromBesideIt) {
    const NurbsSurface patch({{Vec3(0, 0, 0), Vec3(0, 20, 0)}, {Vec3(10, 0, 0), Vec3(10, 20, 0)}},
                             {{1, 1}, {1, 1}}, {0, 0, 1, 1}, {0, 0, 1, 1}, 1, 1);
    // From where a walk along the edge u = 0 had come, a hair inside it.
    const auto [u, v] = patch.project(Vec3(0, 7.5, 0), 0x1.4ap-194, 0x1.a000000000007p-2);
    EXPECT_NEAR(u, 0.0, 1e-12);
    EXPECT_NEAR(v, 0.375, 1e-12);
    for (const double from : {0.9, 0.5, 0.1}) {
        const auto [u2, v2] = patch.project(Vec3(0, 20.0 * (1.0 - from), 0), 1e-30, from);
        EXPECT_NEAR(u2, 0.0, 1e-12);
        EXPECT_NEAR(v2, 1.0 - from, 1e-12);
    }
}

namespace {

/// NurbsSurface::project's search as it was before it stopped at rounding's width, from (@p u,
/// @p v): it stood still only where its step in u and v was a 1e-15 of their ranges. @p steps
/// receives how many it took (100: it never stood still).
std::pair<double, double> everyStep(const NurbsSurface& s, const Vec3& point, double u, double v,
                                    int& steps) {
    const double u0 = s.uMin(), u1 = s.uMax();
    const double v0 = s.vMin(), v1 = s.vMax();
    const auto keep = [](double t, double lo, double hi, bool closed) {
        if (!closed) return std::clamp(t, lo, hi);
        const double period = hi - lo;
        t = lo + std::fmod(t - lo, period);
        return t < lo ? t + period : t;
    };
    const auto shorten = [](double t, double d, double lo, double hi, bool closed, double& alpha) {
        if (closed || d == 0.0) return;
        if (t + d < lo && t > lo) alpha = std::min(alpha, 0.5 * (t - lo) / -d);
        if (t + d > hi && t < hi) alpha = std::min(alpha, 0.5 * (hi - t) / d);
    };
    u = keep(u, u0, u1, s.closedU());
    v = keep(v, v0, v1, s.closedV());
    steps = 0;
    for (int iter = 0; iter < 100; ++iter) {
        ++steps;
        const SurfacePoint at = s.evaluateWithDerivatives(u, v);
        const Vec3 r = at.point - point;
        const double a = at.du.dot(at.du);
        const double b = at.du.dot(at.dv);
        const double c = at.dv.dot(at.dv);
        const double damping = 1e-14 * (a + c) + 1e-300;
        const double det = (a + damping) * (c + damping) - b * b;
        if (!(det > 0.0)) break;
        const double gu = r.dot(at.du);
        const double gv = r.dot(at.dv);
        double du = -((c + damping) * gu - b * gv) / det;
        double dv = -((a + damping) * gv - b * gu) / det;
        double alphaU = 1.0;
        double alphaV = 1.0;
        shorten(u, du, u0, u1, s.closedU(), alphaU);
        shorten(v, dv, v0, v1, s.closedV(), alphaV);
        du *= alphaU;
        dv *= alphaV;
        const bool still = std::abs(du) <= 1e-15 * (u1 - u0) && std::abs(dv) <= 1e-15 * (v1 - v0);
        u = keep(u + du, u0, u1, s.closedU());
        v = keep(v + dv, v0, v1, s.closedV());
        if (still) break;
    }
    return {u, v};
}

/// Whether @p a and @p b are the same doubles, bit for bit.
bool sameBits(std::pair<double, double> a, std::pair<double, double> b) {
    return a.first == b.first && a.second == b.second &&
           std::signbit(a.first) == std::signbit(b.first) &&
           std::signbit(a.second) == std::signbit(b.second);
}

}  // namespace

// A search far from the origin stops once its steps are rounding's width: on a cylinder written as
// a rational B-spline (a quarter circle a span, its knots doubled) placed a hundred from the
// origin, a point at a quarter's end was crossed back and forth across the knot (the surface's
// speed in u changes there) by rounding's width, the step never small enough in u to stand still,
// for all of its 100 steps; elsewhere such a search wandered in rounding's width as long. Round
// the circle and up the cylinder, each search from the point before (as a pcurve's samples are
// placed), and across its seam, the point found is as near as every step of the search finds, to
// rounding, and the same point to rounding. A search given the surface where it starts takes its
// first step from it and ends where it would without it, to the last bit; given it where it starts
// outside the domain, it evaluates the surface again where it is taken to, as without.
TEST(NurbsSurfaceTest, ASearchFarFromTheOriginStopsAtRoundingsWidth) {
    const Vec3 centre(100.0, -40.0, 30.0);
    const NurbsSurface cylinder = NurbsSurface::makeCylinder(centre, Vec3(0, 0, 1), 0.45, 4.0);
    ASSERT_TRUE(cylinder.closedU());
    int wandered = 0;
    for (const double height : {0.0, 0.3, 1.7, 4.0}) {
        double u = cylinder.uMin();
        double v = cylinder.vMin();
        for (int k = 0; k <= 64; ++k) {
            const double angle = kTwoPi * k / 64.0;
            const Vec3 point =
                centre + Vec3(0.45 * std::cos(angle), 0.45 * std::sin(angle), height);
            int steps = 0;
            const auto all = everyStep(cylinder, point, u, v, steps);
            if (steps == 100) ++wandered;
            const auto found = cylinder.project(point, u, v);
            // Rounding's width: a few units in the last place of the point's coordinates.
            const double width = 16.0 * 2.2e-16 * (2.0 * point.length());
            const Vec3 there = cylinder.evaluate(found.first, found.second);
            const Vec3 thereAll = cylinder.evaluate(all.first, all.second);
            EXPECT_LE((there - point).length(), (thereAll - point).length() + width)
                << "k " << k << " height " << height;
            EXPECT_LE((there - thereAll).length(), width) << "k " << k << " height " << height;
            const SurfacePoint at = cylinder.evaluateWithDerivatives(u, v);
            EXPECT_TRUE(sameBits(cylinder.project(point, u, v, &at), found))
                << "given where it starts";
            u = found.first;
            v = found.second;
        }
        // From a period past the domain: evaluated where it is taken to, as without.
        const Vec3 point = centre + Vec3(0.45, 0.0, height);
        const double outside = cylinder.uMax() + 0.25;
        const SurfacePoint there = cylinder.evaluateWithDerivatives(outside, v);
        EXPECT_TRUE(sameBits(cylinder.project(point, outside, v, &there),
                             cylinder.project(point, outside, v)));
    }
    EXPECT_GT(wandered, 0) << "every step of some searches is rounding's";
}
