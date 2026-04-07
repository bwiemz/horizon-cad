#include <gtest/gtest.h>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Tolerance.h"

#include <cmath>

using namespace hz::geo;
using namespace hz::math;

// ---------------------------------------------------------------------------
// Helper: uniform knot vector for a Bezier curve (clamped, no internal knots).
// For n control points of degree p, knots = [0,...,0, 1,...,1]
// with (p+1) zeros and (p+1) ones.
// ---------------------------------------------------------------------------
static std::vector<double> bezierKnots(int n, int degree) {
    std::vector<double> knots(n + degree + 1);
    for (int i = 0; i <= degree; ++i) knots[i] = 0.0;
    for (int i = degree + 1; i < n; ++i) knots[i] = static_cast<double>(i - degree) / (n - degree);
    for (int i = n; i < n + degree + 1; ++i) knots[i] = 1.0;
    return knots;
}

// ===========================================================================
// 1. Construct a linear (degree-1) NURBS curve
// ===========================================================================
TEST(NurbsCurveTest, ConstructLinear) {
    std::vector<Vec3> pts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> wts = {1.0, 1.0};
    auto knots = bezierKnots(2, 1);  // {0,0,1,1}

    NurbsCurve crv(pts, wts, knots, 1);
    EXPECT_EQ(crv.degree(), 1);
    EXPECT_EQ(crv.controlPointCount(), 2);
}

// ===========================================================================
// 2. Construct a quadratic (degree-2) NURBS curve
// ===========================================================================
TEST(NurbsCurveTest, ConstructQuadratic) {
    std::vector<Vec3> pts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    std::vector<double> wts = {1.0, 1.0, 1.0};
    auto knots = bezierKnots(3, 2);  // {0,0,0,1,1,1}

    NurbsCurve crv(pts, wts, knots, 2);
    EXPECT_EQ(crv.degree(), 2);
    EXPECT_EQ(crv.controlPointCount(), 3);
}

// ===========================================================================
// 3. Evaluate linear curve endpoints: t=0 -> P[0], t=1 -> P[n-1]
// ===========================================================================
TEST(NurbsCurveTest, EvaluateLinearEndpoints) {
    std::vector<Vec3> pts = {{1, 2, 3}, {7, 8, 9}};
    std::vector<double> wts = {1.0, 1.0};
    auto knots = bezierKnots(2, 1);

    NurbsCurve crv(pts, wts, knots, 1);

    auto p0 = crv.evaluate(0.0);
    EXPECT_NEAR(p0.x, 1.0, 1e-9);
    EXPECT_NEAR(p0.y, 2.0, 1e-9);
    EXPECT_NEAR(p0.z, 3.0, 1e-9);

    auto p1 = crv.evaluate(1.0);
    EXPECT_NEAR(p1.x, 7.0, 1e-9);
    EXPECT_NEAR(p1.y, 8.0, 1e-9);
    EXPECT_NEAR(p1.z, 9.0, 1e-9);
}

// ===========================================================================
// 4. Evaluate linear curve midpoint: t=0.5 -> average of P[0] and P[1]
// ===========================================================================
TEST(NurbsCurveTest, EvaluateLinearMidpoint) {
    std::vector<Vec3> pts = {{0, 0, 0}, {10, 20, 0}};
    std::vector<double> wts = {1.0, 1.0};
    auto knots = bezierKnots(2, 1);

    NurbsCurve crv(pts, wts, knots, 1);

    auto mid = crv.evaluate(0.5);
    EXPECT_NEAR(mid.x, 5.0, 1e-9);
    EXPECT_NEAR(mid.y, 10.0, 1e-9);
    EXPECT_NEAR(mid.z, 0.0, 1e-9);
}

// ===========================================================================
// 5. Evaluate quadratic Bezier endpoints
// ===========================================================================
TEST(NurbsCurveTest, EvaluateQuadraticEndpoints) {
    std::vector<Vec3> pts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    std::vector<double> wts = {1.0, 1.0, 1.0};
    auto knots = bezierKnots(3, 2);

    NurbsCurve crv(pts, wts, knots, 2);

    auto p0 = crv.evaluate(0.0);
    EXPECT_NEAR(p0.x, 0.0, 1e-9);
    EXPECT_NEAR(p0.y, 0.0, 1e-9);

    auto p1 = crv.evaluate(1.0);
    EXPECT_NEAR(p1.x, 10.0, 1e-9);
    EXPECT_NEAR(p1.y, 0.0, 1e-9);
}

// ===========================================================================
// 6. Evaluate quadratic Bezier midpoint: (0,0)-(5,10)-(10,0) at t=0.5 -> (5,5,0)
// ===========================================================================
TEST(NurbsCurveTest, EvaluateQuadraticMidpoint) {
    std::vector<Vec3> pts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    std::vector<double> wts = {1.0, 1.0, 1.0};
    auto knots = bezierKnots(3, 2);

    NurbsCurve crv(pts, wts, knots, 2);

    auto mid = crv.evaluate(0.5);
    EXPECT_NEAR(mid.x, 5.0, 1e-9);
    EXPECT_NEAR(mid.y, 5.0, 1e-9);
    EXPECT_NEAR(mid.z, 0.0, 1e-9);
}

// ===========================================================================
// 7. Evaluate cubic Bezier endpoints: 4 control points, degree 3
// ===========================================================================
TEST(NurbsCurveTest, EvaluateCubicEndpoints) {
    std::vector<Vec3> pts = {{0, 0, 0}, {1, 3, 0}, {4, 3, 0}, {5, 0, 0}};
    std::vector<double> wts = {1.0, 1.0, 1.0, 1.0};
    auto knots = bezierKnots(4, 3);  // {0,0,0,0,1,1,1,1}

    NurbsCurve crv(pts, wts, knots, 3);

    auto p0 = crv.evaluate(0.0);
    EXPECT_NEAR(p0.x, 0.0, 1e-9);
    EXPECT_NEAR(p0.y, 0.0, 1e-9);

    auto p1 = crv.evaluate(1.0);
    EXPECT_NEAR(p1.x, 5.0, 1e-9);
    EXPECT_NEAR(p1.y, 0.0, 1e-9);
}

// ===========================================================================
// 8. Weighted evaluation: a high weight on the middle control point pulls
//    the curve toward it.
// ===========================================================================
TEST(NurbsCurveTest, WeightedEvaluation) {
    std::vector<Vec3> pts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    auto knots = bezierKnots(3, 2);

    // Uniform weights -> midpoint y = 5.0
    NurbsCurve uniform(pts, {1.0, 1.0, 1.0}, knots, 2);
    auto midUniform = uniform.evaluate(0.5);

    // High weight on middle control point -> midpoint y > 5.0
    NurbsCurve weighted(pts, {1.0, 10.0, 1.0}, knots, 2);
    auto midWeighted = weighted.evaluate(0.5);

    EXPECT_GT(midWeighted.y, midUniform.y);
    // With w=10 on the middle point, the midpoint should be pulled much closer
    // to (5, 10, 0).
    EXPECT_GT(midWeighted.y, 8.0);
}

// ===========================================================================
// 9. Parameter domain: tMin and tMax come from the knot vector
// ===========================================================================
TEST(NurbsCurveTest, ParameterDomain) {
    std::vector<Vec3> pts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    std::vector<double> wts = {1.0, 1.0, 1.0};
    auto knots = bezierKnots(3, 2);

    NurbsCurve crv(pts, wts, knots, 2);

    EXPECT_DOUBLE_EQ(crv.tMin(), 0.0);
    EXPECT_DOUBLE_EQ(crv.tMax(), 1.0);

    // Non-standard domain: shift knots
    std::vector<double> knots2 = {2.0, 2.0, 2.0, 5.0, 5.0, 5.0};
    NurbsCurve crv2(pts, wts, knots2, 2);

    EXPECT_DOUBLE_EQ(crv2.tMin(), 2.0);
    EXPECT_DOUBLE_EQ(crv2.tMax(), 5.0);
}

// ===========================================================================
// 10. Derivative of a linear curve should be constant
// ===========================================================================
TEST(NurbsCurveTest, DerivativeLinear) {
    std::vector<Vec3> pts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> wts = {1.0, 1.0};
    auto knots = bezierKnots(2, 1);

    NurbsCurve crv(pts, wts, knots, 1);

    // Derivative of a line from (0,0,0) to (10,0,0) over [0,1] should be (10,0,0).
    for (double t : {0.1, 0.3, 0.5, 0.7, 0.9}) {
        auto d = crv.derivative(t);
        EXPECT_NEAR(d.x, 10.0, 0.1);
        EXPECT_NEAR(d.y, 0.0, 0.1);
        EXPECT_NEAR(d.z, 0.0, 0.1);
    }
}

// ===========================================================================
// 11. Derivative matches an independent finite-difference check (cubic)
// ===========================================================================
TEST(NurbsCurveTest, DerivativeMatchesFiniteDifference) {
    std::vector<Vec3> pts = {{0, 0, 0}, {1, 3, 0}, {4, 3, 0}, {5, 0, 0}};
    std::vector<double> wts = {1.0, 1.0, 1.0, 1.0};
    auto knots = bezierKnots(4, 3);

    NurbsCurve crv(pts, wts, knots, 3);

    const double t = 0.3;
    const double h = 1e-5;
    Vec3 fdDeriv = (crv.evaluate(t + h) - crv.evaluate(t - h)) * (1.0 / (2.0 * h));
    Vec3 analyticDeriv = crv.derivative(t);

    EXPECT_NEAR(analyticDeriv.x, fdDeriv.x, 1e-2);
    EXPECT_NEAR(analyticDeriv.y, fdDeriv.y, 1e-2);
    EXPECT_NEAR(analyticDeriv.z, fdDeriv.z, 1e-2);
}

// ===========================================================================
// 12. Second derivative matches finite difference of first derivatives
// ===========================================================================
TEST(NurbsCurveTest, SecondDerivativeMatchesFiniteDifference) {
    std::vector<Vec3> pts = {{0, 0, 0}, {1, 3, 0}, {4, 3, 0}, {5, 0, 0}};
    std::vector<double> wts = {1.0, 1.0, 1.0, 1.0};
    auto knots = bezierKnots(4, 3);

    NurbsCurve crv(pts, wts, knots, 3);

    const double t = 0.5;
    const double h = 1e-4;
    Vec3 d1Plus = crv.derivative(t + h, 1);
    Vec3 d1Minus = crv.derivative(t - h, 1);
    Vec3 fdSecond = (d1Plus - d1Minus) * (1.0 / (2.0 * h));
    Vec3 analyticSecond = crv.derivative(t, 2);

    EXPECT_NEAR(analyticSecond.x, fdSecond.x, 0.5);
    EXPECT_NEAR(analyticSecond.y, fdSecond.y, 0.5);
    EXPECT_NEAR(analyticSecond.z, fdSecond.z, 0.5);
}

// ===========================================================================
// 13. Tessellate a straight line -> minimal points
// ===========================================================================
TEST(NurbsCurveTest, TessellateLinearGivesFewPoints) {
    std::vector<Vec3> pts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> wts = {1.0, 1.0};
    auto knots = bezierKnots(2, 1);

    NurbsCurve crv(pts, wts, knots, 1);
    auto poly = crv.tessellate(0.01);

    // Should have at least start and end.
    ASSERT_GE(poly.size(), 2u);
    EXPECT_NEAR(poly.front().x, 0.0, 1e-9);
    EXPECT_NEAR(poly.back().x, 10.0, 1e-9);

    // A line shouldn't need many points (the minimum recursion depth forces some
    // subdivisions, but still limited).
    EXPECT_LE(poly.size(), 20u);
}

// ===========================================================================
// 14. Tessellate a curved shape produces more points than a line
// ===========================================================================
TEST(NurbsCurveTest, TessellateCurvedHasMorePoints) {
    // Line
    std::vector<Vec3> linePts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> lineWts = {1.0, 1.0};
    NurbsCurve line(linePts, lineWts, bezierKnots(2, 1), 1);
    auto linePoly = line.tessellate(0.001);

    // Curved (quadratic with high curvature)
    std::vector<Vec3> curvePts = {{0, 0, 0}, {5, 20, 0}, {10, 0, 0}};
    std::vector<double> curveWts = {1.0, 1.0, 1.0};
    NurbsCurve curve(curvePts, curveWts, bezierKnots(3, 2), 2);
    auto curvePoly = curve.tessellate(0.001);

    EXPECT_GT(curvePoly.size(), linePoly.size());
}

// ===========================================================================
// 15. Knot insertion preserves curve shape
// ===========================================================================
TEST(NurbsCurveTest, KnotInsertionPreservesShape) {
    std::vector<Vec3> pts = {{0, 0, 0}, {1, 3, 0}, {4, 3, 0}, {5, 0, 0}};
    std::vector<double> wts = {1.0, 1.0, 1.0, 1.0};
    auto knots = bezierKnots(4, 3);

    NurbsCurve original(pts, wts, knots, 3);
    NurbsCurve refined = original.insertKnot(0.5);

    // The refined curve should have one more control point.
    EXPECT_EQ(refined.controlPointCount(), original.controlPointCount() + 1);

    // Same degree.
    EXPECT_EQ(refined.degree(), original.degree());

    // Evaluate both at 11 sample points — shapes must match.
    for (int i = 0; i <= 10; ++i) {
        const double t = static_cast<double>(i) / 10.0;
        Vec3 pOrig = original.evaluate(t);
        Vec3 pRef = refined.evaluate(t);
        EXPECT_NEAR(pOrig.x, pRef.x, 1e-8);
        EXPECT_NEAR(pOrig.y, pRef.y, 1e-8);
        EXPECT_NEAR(pOrig.z, pRef.z, 1e-8);
    }
}

// ===========================================================================
// 16. Degree elevation preserves curve shape
// ===========================================================================
TEST(NurbsCurveTest, DegreeElevationPreservesShape) {
    std::vector<Vec3> pts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    std::vector<double> wts = {1.0, 1.0, 1.0};
    auto knots = bezierKnots(3, 2);

    NurbsCurve original(pts, wts, knots, 2);
    NurbsCurve elevated = original.elevateDegree();

    // Elevated curve should be one degree higher.
    EXPECT_EQ(elevated.degree(), original.degree() + 1);

    // Evaluate both at 11 sample points — shapes must match.
    for (int i = 0; i <= 10; ++i) {
        const double t = static_cast<double>(i) / 10.0;
        Vec3 pOrig = original.evaluate(t);
        Vec3 pElev = elevated.evaluate(t);
        EXPECT_NEAR(pOrig.x, pElev.x, 1e-8);
        EXPECT_NEAR(pOrig.y, pElev.y, 1e-8);
        EXPECT_NEAR(pOrig.z, pElev.z, 1e-8);
    }
}

// ===========================================================================
// Task 4: Closest-Point Projection & Arc-Length Parameterization
// ===========================================================================

TEST(NurbsCurveTest, ClosestPointOnLine) {
    // Line from (0,0,0) to (10,0,0). Point at (5,10,0). Closest t=0.5.
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0};
    std::vector<double> knots = {0, 0, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 1);
    double t = curve.closestPoint(Vec3(5, 10, 0));
    EXPECT_NEAR(t, 0.5, 1e-4);
}

TEST(NurbsCurveTest, ClosestPointOnCubic) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {3, 10, 0}, {7, 10, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0, 1.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 3);
    double t0 = curve.closestPoint(Vec3(0, 0, 0));
    EXPECT_NEAR(t0, 0.0, 0.01);
    double t1 = curve.closestPoint(Vec3(10, 0, 0));
    EXPECT_NEAR(t1, 1.0, 0.01);
}

TEST(NurbsCurveTest, ArcLengthLinear) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0};
    std::vector<double> knots = {0, 0, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 1);
    double len = curve.arcLength(0.0, 1.0);
    EXPECT_NEAR(len, 10.0, 1e-4);
}

TEST(NurbsCurveTest, ParameterAtLengthLinear) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0};
    std::vector<double> knots = {0, 0, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 1);
    double t = curve.parameterAtLength(5.0);
    EXPECT_NEAR(t, 0.5, 1e-4);
}

// ===========================================================================
// Task 5: Exact Conic Factory Functions
// ===========================================================================

TEST(NurbsCurveTest, CircleExactRepresentation) {
    NurbsCurve circle = NurbsCurve::makeCircle(Vec3(0, 0, 0), 5.0);
    // All points at distance 5 from center.
    for (int i = 0; i <= 100; ++i) {
        double t = circle.tMin() + (circle.tMax() - circle.tMin()) * i / 100.0;
        Vec3 p = circle.evaluate(t);
        double dist = std::sqrt(p.x * p.x + p.y * p.y);
        EXPECT_NEAR(dist, 5.0, 1e-6) << "at t=" << t;
    }
}

TEST(NurbsCurveTest, ArcExactRepresentation) {
    // Quarter circle
    NurbsCurve arc = NurbsCurve::makeArc(Vec3(0, 0, 0), 10.0, 0.0, hz::math::kHalfPi);
    Vec3 start = arc.evaluate(arc.tMin());
    EXPECT_NEAR(start.x, 10.0, 1e-6);
    EXPECT_NEAR(start.y, 0.0, 1e-6);
    Vec3 end = arc.evaluate(arc.tMax());
    EXPECT_NEAR(end.x, 0.0, 1e-6);
    EXPECT_NEAR(end.y, 10.0, 1e-6);
    // All points at radius 10.
    for (int i = 0; i <= 50; ++i) {
        double t = arc.tMin() + (arc.tMax() - arc.tMin()) * i / 50.0;
        Vec3 p = arc.evaluate(t);
        double dist = std::sqrt(p.x * p.x + p.y * p.y);
        EXPECT_NEAR(dist, 10.0, 1e-6);
    }
}

TEST(NurbsCurveTest, EllipseExactRepresentation) {
    NurbsCurve ellipse = NurbsCurve::makeEllipse(Vec3(0, 0, 0), 10.0, 5.0);
    for (int i = 0; i <= 100; ++i) {
        double t = ellipse.tMin() + (ellipse.tMax() - ellipse.tMin()) * i / 100.0;
        Vec3 p = ellipse.evaluate(t);
        double val = (p.x * p.x) / 100.0 + (p.y * p.y) / 25.0;
        EXPECT_NEAR(val, 1.0, 1e-5) << "at t=" << t;
    }
}

TEST(NurbsCurveTest, SemicircleArc) {
    // 180-degree arc should have start at (r,0) and end at (-r,0)
    NurbsCurve arc = NurbsCurve::makeArc(Vec3(0, 0, 0), 5.0, 0.0, hz::math::kPi);
    Vec3 start = arc.evaluate(arc.tMin());
    EXPECT_NEAR(start.x, 5.0, 1e-6);
    Vec3 end = arc.evaluate(arc.tMax());
    EXPECT_NEAR(end.x, -5.0, 1e-6);
    EXPECT_NEAR(end.y, 0.0, 1e-5);
}

