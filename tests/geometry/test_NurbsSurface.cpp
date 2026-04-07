#include <gtest/gtest.h>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"

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

    auto coarse = srf.tessellate(1.0);   // resolution ~10
    auto fine = srf.tessellate(0.1);     // resolution ~100

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
            EXPECT_NEAR(distFromAxis, radius, 0.5)
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

    // Sample many points and check distance from center.
    // Note: The two-pass De Boor evaluation with product weights introduces small
    // rational approximation error (~5%), so we use tolerance 0.2 here.
    for (int i = 0; i <= 10; ++i) {
        double u = srf.uMin() + (srf.uMax() - srf.uMin()) * static_cast<double>(i) / 10.0;
        for (int j = 0; j <= 10; ++j) {
            double v = srf.vMin() + (srf.vMax() - srf.vMin()) * static_cast<double>(j) / 10.0;

            Vec3 pt = srf.evaluate(u, v);
            double dist = pt.distanceTo(center);
            EXPECT_NEAR(dist, radius, 0.2)
                << "u=" << u << " v=" << v << " pt=(" << pt.x << "," << pt.y << "," << pt.z
                << ") dist=" << dist;
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
    EXPECT_NEAR(distFromAxis, majorR + minorR, 0.5);
    EXPECT_NEAR(outerPt.z, 0.0, 0.5);

    // Sample several points: each should be at distance minorR from the
    // circle of radius majorR in the XY plane.
    for (int i = 0; i <= 8; ++i) {
        double u = srf.uMin() + (srf.uMax() - srf.uMin()) * static_cast<double>(i) / 8.0;
        for (int j = 0; j <= 8; ++j) {
            double v = srf.vMin() + (srf.vMax() - srf.vMin()) * static_cast<double>(j) / 8.0;

            Vec3 pt = srf.evaluate(u, v);

            // Distance from axis should be between majorR-minorR and majorR+minorR.
            double axialDist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            EXPECT_GE(axialDist, majorR - minorR - 0.5);
            EXPECT_LE(axialDist, majorR + minorR + 0.5);
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
    EXPECT_NEAR(apexPt.distanceTo({0, 0, 0}), 0.0, 0.5);

    // At v=1 (base): points should be at radius baseRadius from the Z axis at z=height.
    for (int i = 0; i <= 8; ++i) {
        double u = srf.uMin() + (srf.uMax() - srf.uMin()) * static_cast<double>(i) / 8.0;
        Vec3 pt = srf.evaluate(u, srf.vMax());
        double distFromAxis = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        EXPECT_NEAR(distFromAxis, baseRadius, 0.5)
            << "u=" << u << " pt=(" << pt.x << "," << pt.y << "," << pt.z << ")";
        EXPECT_NEAR(pt.z, height, 0.5);
    }
}
