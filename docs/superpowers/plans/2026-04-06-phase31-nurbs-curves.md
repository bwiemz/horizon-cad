# Phase 31: NURBS Curve Library Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Full NURBS curve implementation — the universal curve representation for CAD. This is the first piece of the custom geometry kernel.

**Architecture:** `NurbsCurve` in `hz::geo` (the existing geometry module) implements De Boor evaluation, knot insertion, degree elevation, derivative computation, closest-point projection, arc-length parameterization, and adaptive tessellation. Factory functions produce exact NURBS representations of circles, arcs, and ellipses.

**Tech Stack:** C++20, Google Test, existing `hz::math` (Vec2, Vec3, Tolerance)

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 4.1

---

## Spec Compliance Check

| Spec Requirement | Plan Task | Status |
|---|---|---|
| `NurbsCurve`: control points (Vec3), weights, knot vector, degree | Task 1 | ✅ |
| De Boor's algorithm for evaluation at parameter t | Task 1 | ✅ |
| Handle knot multiplicities without division by zero | Task 1 | ✅ |
| Boehm's knot insertion | Task 3 | ✅ |
| Degree elevation | Task 3 | ✅ |
| Derivative evaluation (1st and 2nd order) | Task 2 | ✅ |
| Arc-length parameterization via Newton iteration | Task 4 | ✅ |
| Closest-point projection via Newton iteration | Task 4 | ✅ |
| Exact conic representation: circles, arcs, ellipses as NURBS | Task 5 | ✅ |
| Adaptive tessellation (curvature-based) | Task 2 | ✅ |
| Tests: line as degree-1 NURBS, circle as rational NURBS | Tasks 1, 5 | ✅ |
| Tests: derivatives match finite-difference | Task 2 | ✅ |
| Tests: closest-point converges | Task 4 | ✅ |
| Tests: conic accuracy < Tolerance::kLinear | Task 5 | ✅ |
| User-facing: upgrade spline tool for NURBS weight editing | Task 6 | ✅ |

---

## File Map

| Action | File | Responsibility |
|--------|------|---------------|
| Create | `src/geometry/include/horizon/geometry/curves/NurbsCurve.h` | NURBS curve class |
| Create | `src/geometry/src/curves/NurbsCurve.cpp` | Core algorithms |
| Modify | `src/geometry/CMakeLists.txt` | Add NurbsCurve.cpp |
| Create | `tests/geometry/test_NurbsCurve.cpp` | Comprehensive tests |
| Create | `tests/geometry/CMakeLists.txt` | Geometry test target |
| Modify | `tests/CMakeLists.txt` | Add geometry subdirectory |

---

## Task 1: NurbsCurve — Data Structure + De Boor Evaluation

The foundation: the curve class with point evaluation via De Boor's algorithm.

**Files:**
- Create: `src/geometry/include/horizon/geometry/curves/NurbsCurve.h`
- Create: `src/geometry/src/curves/NurbsCurve.cpp`
- Create: `tests/geometry/test_NurbsCurve.cpp`
- Create: `tests/geometry/CMakeLists.txt`
- Modify: `src/geometry/CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write failing tests for construction and evaluation**

Create `tests/geometry/test_NurbsCurve.cpp`:
```cpp
#include <gtest/gtest.h>
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/math/Vec3.h"
#include "horizon/math/Tolerance.h"

using namespace hz::geo;
using namespace hz::math;

// --- Construction ---

TEST(NurbsCurveTest, ConstructLinear) {
    // Degree-1 NURBS = a straight line from (0,0,0) to (10,0,0).
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0};
    std::vector<double> knots = {0, 0, 1, 1};  // Clamped
    NurbsCurve curve(ctrlPts, weights, knots, 1);
    EXPECT_EQ(curve.degree(), 1);
    EXPECT_EQ(curve.controlPointCount(), 2);
}

TEST(NurbsCurveTest, ConstructQuadratic) {
    // Degree-2 NURBS with 3 control points.
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 2);
    EXPECT_EQ(curve.degree(), 2);
}

// --- Evaluation (De Boor) ---

TEST(NurbsCurveTest, EvaluateLinearEndpoints) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0};
    std::vector<double> knots = {0, 0, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 1);

    Vec3 p0 = curve.evaluate(0.0);
    EXPECT_NEAR(p0.x, 0.0, 1e-10);

    Vec3 p1 = curve.evaluate(1.0);
    EXPECT_NEAR(p1.x, 10.0, 1e-10);
}

TEST(NurbsCurveTest, EvaluateLinearMidpoint) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0};
    std::vector<double> knots = {0, 0, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 1);

    Vec3 mid = curve.evaluate(0.5);
    EXPECT_NEAR(mid.x, 5.0, 1e-10);
    EXPECT_NEAR(mid.y, 0.0, 1e-10);
}

TEST(NurbsCurveTest, EvaluateQuadraticEndpoints) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 2);

    Vec3 p0 = curve.evaluate(0.0);
    EXPECT_NEAR(p0.x, 0.0, 1e-10);
    EXPECT_NEAR(p0.y, 0.0, 1e-10);

    Vec3 p1 = curve.evaluate(1.0);
    EXPECT_NEAR(p1.x, 10.0, 1e-10);
    EXPECT_NEAR(p1.y, 0.0, 1e-10);
}

TEST(NurbsCurveTest, EvaluateQuadraticMidpoint) {
    // For a symmetric quadratic Bezier, midpoint is at (5, 5, 0)
    // (average of control points with Bezier weights)
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 2);

    Vec3 mid = curve.evaluate(0.5);
    EXPECT_NEAR(mid.x, 5.0, 1e-10);
    EXPECT_NEAR(mid.y, 5.0, 1e-10);  // Quadratic Bezier at t=0.5
}

TEST(NurbsCurveTest, EvaluateCubicEndpoints) {
    // Cubic NURBS with 4 control points.
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {3, 10, 0}, {7, 10, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0, 1.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 3);

    Vec3 p0 = curve.evaluate(0.0);
    EXPECT_NEAR(p0.x, 0.0, 1e-10);
    Vec3 p1 = curve.evaluate(1.0);
    EXPECT_NEAR(p1.x, 10.0, 1e-10);
}

TEST(NurbsCurveTest, WeightedEvaluation) {
    // Rational NURBS: weight on middle control point pulls curve toward it.
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    std::vector<double> weightsNormal = {1.0, 1.0, 1.0};
    std::vector<double> weightsHigh = {1.0, 5.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};

    NurbsCurve curveNormal(ctrlPts, weightsNormal, knots, 2);
    NurbsCurve curveHigh(ctrlPts, weightsHigh, knots, 2);

    Vec3 midNormal = curveNormal.evaluate(0.5);
    Vec3 midHigh = curveHigh.evaluate(0.5);

    // Higher weight pulls midpoint closer to control point (5, 10)
    EXPECT_GT(midHigh.y, midNormal.y);
}

TEST(NurbsCurveTest, ParameterDomain) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0};
    std::vector<double> knots = {0, 0, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 1);
    EXPECT_DOUBLE_EQ(curve.tMin(), 0.0);
    EXPECT_DOUBLE_EQ(curve.tMax(), 1.0);
}
```

- [ ] **Step 2: Create geometry test infrastructure**

Create `tests/geometry/CMakeLists.txt`:
```cmake
add_executable(hz_geometry_tests
    test_NurbsCurve.cpp
)

target_link_libraries(hz_geometry_tests
    PRIVATE
        Horizon::Geometry
        GTest::gtest
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(hz_geometry_tests)
```

Add `add_subdirectory(geometry)` to `tests/CMakeLists.txt`.

- [ ] **Step 3: Implement NurbsCurve.h**

```cpp
#pragma once

#include "horizon/math/Vec3.h"
#include <vector>

namespace hz::geo {

/// Non-Uniform Rational B-Spline curve.
class NurbsCurve {
public:
    NurbsCurve(std::vector<math::Vec3> controlPoints,
               std::vector<double> weights,
               std::vector<double> knots,
               int degree);

    // Accessors
    int degree() const { return m_degree; }
    int controlPointCount() const { return static_cast<int>(m_controlPoints.size()); }
    const std::vector<math::Vec3>& controlPoints() const { return m_controlPoints; }
    const std::vector<double>& weights() const { return m_weights; }
    const std::vector<double>& knots() const { return m_knots; }

    double tMin() const;
    double tMax() const;

    /// Evaluate the curve at parameter t using De Boor's algorithm.
    [[nodiscard]] math::Vec3 evaluate(double t) const;

    /// Evaluate derivative of given order at parameter t.
    [[nodiscard]] math::Vec3 derivative(double t, int order = 1) const;

    /// Adaptive tessellation based on curvature tolerance.
    [[nodiscard]] std::vector<math::Vec3> tessellate(double tolerance = 0.01) const;

    /// Find the parameter t of the closest point on the curve to the given point.
    [[nodiscard]] double closestPoint(const math::Vec3& point, double tol = 1e-8) const;

    /// Compute arc length from tStart to tEnd.
    [[nodiscard]] double arcLength(double tStart, double tEnd, int segments = 100) const;

    /// Find parameter t at a given arc length from tStart.
    [[nodiscard]] double parameterAtLength(double length, double tStart = -1.0) const;

    /// Insert a knot (Boehm's algorithm). Returns new curve without changing shape.
    [[nodiscard]] NurbsCurve insertKnot(double t) const;

    /// Elevate degree by 1. Returns new curve without changing shape.
    [[nodiscard]] NurbsCurve elevateDegree() const;

    // --- Factory functions for exact conics ---

    /// Create a NURBS curve exactly representing a full circle.
    static NurbsCurve makeCircle(const math::Vec3& center, double radius,
                                  const math::Vec3& normal = math::Vec3::UnitZ);

    /// Create a NURBS curve exactly representing a circular arc.
    static NurbsCurve makeArc(const math::Vec3& center, double radius,
                               double startAngle, double endAngle,
                               const math::Vec3& normal = math::Vec3::UnitZ);

    /// Create a NURBS curve exactly representing an ellipse.
    static NurbsCurve makeEllipse(const math::Vec3& center,
                                   double semiMajor, double semiMinor,
                                   double rotation = 0.0,
                                   const math::Vec3& normal = math::Vec3::UnitZ);

private:
    std::vector<math::Vec3> m_controlPoints;
    std::vector<double> m_weights;
    std::vector<double> m_knots;
    int m_degree;

    /// Find the knot span index for parameter t.
    int findKnotSpan(double t) const;

    /// Compute the non-vanishing basis functions at parameter t.
    std::vector<double> basisFunctions(int span, double t) const;

    /// De Boor evaluation with homogeneous coordinates (for rational curves).
    math::Vec3 deBoor(double t) const;

    /// Recursive adaptive tessellation helper.
    void tessellateRecursive(double t0, double t1,
                              const math::Vec3& p0, const math::Vec3& p1,
                              double tolerance,
                              std::vector<math::Vec3>& result) const;
};

}  // namespace hz::geo
```

- [ ] **Step 4: Implement NurbsCurve.cpp — De Boor evaluation**

Key algorithm: **De Boor's algorithm** for rational NURBS.

For rational B-splines, work in homogeneous coordinates:
1. Multiply each control point by its weight: `P_h[i] = (P[i].x * w[i], P[i].y * w[i], P[i].z * w[i], w[i])`
2. Apply De Boor's algorithm in 4D homogeneous space
3. Divide result by the 4th coordinate (weight): `result = (x/w, y/w, z/w)`

**findKnotSpan(t):** Binary search for the knot span `[knots[i], knots[i+1])` containing t. Handle edge case: when `t == tMax()`, return the last valid span (n - p - 1 where n = number of control points, p = degree).

**basisFunctions(span, t):** Compute the `p+1` non-vanishing basis functions N_{span-p,p}(t) through N_{span,p}(t) using the triangular evaluation table. Guard against division by zero when knots are repeated.

**deBoor(t):** 
1. Find span
2. Compute basis functions
3. For each basis function, multiply by the homogeneous control point
4. Sum and divide by total weight

- [ ] **Step 5: Add NurbsCurve.cpp to CMakeLists**

Add `src/curves/NurbsCurve.cpp` to `src/geometry/CMakeLists.txt`.

- [ ] **Step 6: Build and run tests**

Expected: All existing 231 tests + new NurbsCurve tests pass.

- [ ] **Step 7: Commit**

```bash
git add src/geometry/include/horizon/geometry/curves/NurbsCurve.h \
        src/geometry/src/curves/NurbsCurve.cpp \
        src/geometry/CMakeLists.txt \
        tests/geometry/test_NurbsCurve.cpp \
        tests/geometry/CMakeLists.txt \
        tests/CMakeLists.txt
git commit -m "feat(geometry): add NurbsCurve with De Boor evaluation

Rational B-spline curve with homogeneous-coordinate De Boor algorithm.
Supports arbitrary degree, non-uniform knots, weighted control points."
```

---

## Task 2: Derivatives + Adaptive Tessellation

- [ ] **Step 1: Write tests for derivatives and tessellation**

Append to `tests/geometry/test_NurbsCurve.cpp`:
```cpp
// --- Derivatives ---

TEST(NurbsCurveTest, DerivativeLinear) {
    // Derivative of a straight line is constant.
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0};
    std::vector<double> knots = {0, 0, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 1);

    Vec3 d0 = curve.derivative(0.0);
    Vec3 d1 = curve.derivative(0.5);
    Vec3 d2 = curve.derivative(1.0);
    EXPECT_NEAR(d0.x, 10.0, 1e-6);
    EXPECT_NEAR(d1.x, 10.0, 1e-6);
    EXPECT_NEAR(d2.x, 10.0, 1e-6);
}

TEST(NurbsCurveTest, DerivativeMatchesFiniteDifference) {
    // Cubic curve — verify derivative matches finite difference.
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {3, 10, 0}, {7, 10, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0, 1.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 3);

    double t = 0.3;
    double h = 1e-7;
    Vec3 analytical = curve.derivative(t);
    Vec3 numerical = (curve.evaluate(t + h) - curve.evaluate(t - h)) * (0.5 / h);

    EXPECT_NEAR(analytical.x, numerical.x, 1e-3);
    EXPECT_NEAR(analytical.y, numerical.y, 1e-3);
}

TEST(NurbsCurveTest, SecondDerivativeMatchesFiniteDifference) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {3, 10, 0}, {7, 10, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0, 1.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 3);

    double t = 0.5;
    double h = 1e-5;
    Vec3 analytical = curve.derivative(t, 2);
    Vec3 numerical = (curve.derivative(t + h) - curve.derivative(t - h)) * (0.5 / h);

    EXPECT_NEAR(analytical.x, numerical.x, 0.1);
    EXPECT_NEAR(analytical.y, numerical.y, 0.1);
}

// --- Tessellation ---

TEST(NurbsCurveTest, TessellateLinearGivesTwoPoints) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0};
    std::vector<double> knots = {0, 0, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 1);

    auto points = curve.tessellate(0.01);
    EXPECT_GE(points.size(), 2u);
    EXPECT_NEAR(points.front().x, 0.0, 1e-10);
    EXPECT_NEAR(points.back().x, 10.0, 1e-10);
}

TEST(NurbsCurveTest, TessellateCurvedHasMorePoints) {
    // Curved NURBS should produce more tessellation points than a straight line.
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 2);

    auto coarse = curve.tessellate(1.0);
    auto fine = curve.tessellate(0.01);
    EXPECT_GT(fine.size(), coarse.size());
}
```

- [ ] **Step 2: Implement derivative evaluation**

Derivative of a rational NURBS curve uses the quotient rule on the homogeneous form:
```
C(t) = A(t) / w(t) where A(t) is the numerator (3D) and w(t) is the weight function

C'(t) = (A'(t) * w(t) - A(t) * w'(t)) / w(t)^2
```

For the implementation, compute derivatives of the non-rational B-spline in homogeneous coordinates, then apply the quotient rule.

- [ ] **Step 3: Implement adaptive tessellation**

Recursive subdivision:
1. Start with the full parameter range [tMin, tMax]
2. Evaluate midpoint
3. Check if the chord (straight line from start to end) deviates from the curve at the midpoint by more than `tolerance`
4. If yes, subdivide into two halves and recurse
5. If no, the chord is accurate enough — add the endpoint

- [ ] **Step 4: Build and run tests**

- [ ] **Step 5: Commit**

```bash
git add src/geometry/src/curves/NurbsCurve.cpp tests/geometry/test_NurbsCurve.cpp
git commit -m "feat(geometry): add NURBS derivative evaluation and adaptive tessellation

1st and 2nd order derivatives via quotient rule on homogeneous coords.
Curvature-based adaptive tessellation with tolerance parameter."
```

---

## Task 3: Knot Insertion + Degree Elevation

- [ ] **Step 1: Write tests**

```cpp
TEST(NurbsCurveTest, KnotInsertionPreservesShape) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 2);

    NurbsCurve refined = curve.insertKnot(0.5);
    EXPECT_EQ(refined.controlPointCount(), 4);  // One more control point

    // Shape must be identical at several sample points.
    for (double t = 0.0; t <= 1.0; t += 0.1) {
        Vec3 p1 = curve.evaluate(t);
        Vec3 p2 = refined.evaluate(t);
        EXPECT_NEAR(p1.x, p2.x, 1e-10);
        EXPECT_NEAR(p1.y, p2.y, 1e-10);
    }
}

TEST(NurbsCurveTest, DegreeElevationPreservesShape) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {5, 10, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 2);

    NurbsCurve elevated = curve.elevateDegree();
    EXPECT_EQ(elevated.degree(), 3);

    for (double t = 0.0; t <= 1.0; t += 0.1) {
        Vec3 p1 = curve.evaluate(t);
        Vec3 p2 = elevated.evaluate(t);
        EXPECT_NEAR(p1.x, p2.x, 1e-8);
        EXPECT_NEAR(p1.y, p2.y, 1e-8);
    }
}
```

- [ ] **Step 2: Implement Boehm's knot insertion**

Boehm's algorithm: given a knot to insert at parameter `t_new`:
1. Find the knot span `k` such that `knots[k] <= t_new < knots[k+1]`
2. Compute new control points by linear interpolation:
   `Q[i] = (1 - alpha_i) * P[i-1] + alpha_i * P[i]`
   where `alpha_i = (t_new - knots[i]) / (knots[i+p] - knots[i])`
3. New knot vector = old knots with t_new inserted

Work in homogeneous coordinates for rational curves.

- [ ] **Step 3: Implement degree elevation**

For a Bezier segment (single span): elevate by multiplying with the Bernstein polynomial identity.
For multi-span NURBS: decompose into Bezier segments (knot insertion to max multiplicity), elevate each, recombine.

Simpler approach for initial implementation: use the standard degree elevation formula that works directly on the B-spline control points without decomposition. The control points increase from n to n+1, the knot vector gains one extra copy of each unique knot.

- [ ] **Step 4: Build and run tests**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(geometry): add NURBS knot insertion and degree elevation"
```

---

## Task 4: Closest-Point Projection + Arc-Length Parameterization

- [ ] **Step 1: Write tests**

```cpp
TEST(NurbsCurveTest, ClosestPointOnLine) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0};
    std::vector<double> knots = {0, 0, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 1);

    // Point above the midpoint
    double t = curve.closestPoint(Vec3(5, 10, 0));
    EXPECT_NEAR(t, 0.5, 1e-4);
}

TEST(NurbsCurveTest, ClosestPointOnCubic) {
    std::vector<Vec3> ctrlPts = {{0, 0, 0}, {3, 10, 0}, {7, 10, 0}, {10, 0, 0}};
    std::vector<double> weights = {1.0, 1.0, 1.0, 1.0};
    std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1};
    NurbsCurve curve(ctrlPts, weights, knots, 3);

    // Point at origin — closest should be near t=0
    double t = curve.closestPoint(Vec3(0, 0, 0));
    EXPECT_NEAR(t, 0.0, 0.01);

    // Point at (10, 0, 0) — closest should be near t=1
    double t2 = curve.closestPoint(Vec3(10, 0, 0));
    EXPECT_NEAR(t2, 1.0, 0.01);
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

    // Half the arc length should correspond to t=0.5.
    double t = curve.parameterAtLength(5.0);
    EXPECT_NEAR(t, 0.5, 1e-4);
}
```

- [ ] **Step 2: Implement closest-point projection**

Newton iteration on `f(t) = (C(t) - P) · C'(t) = 0`:
1. Sample the curve at N uniform points to find a good initial guess
2. Newton: `t_{n+1} = t_n - f(t_n) / f'(t_n)` where `f'(t) = C'(t)·C'(t) + (C(t)-P)·C''(t)`
3. Clamp t to [tMin, tMax]
4. Converge when `|f(t)| < tolerance`

- [ ] **Step 3: Implement arc-length parameterization**

Arc length by numerical integration (Gauss-Legendre or Simpson's rule on `||C'(t)||`).
Parameter at length by Newton iteration: find `t` such that `arcLength(tMin, t) = targetLength`.

- [ ] **Step 4: Build and run tests**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(geometry): add NURBS closest-point projection and arc-length parameterization"
```

---

## Task 5: Exact Conic Factory Functions

- [ ] **Step 1: Write tests for circle, arc, ellipse**

```cpp
TEST(NurbsCurveTest, CircleExactRepresentation) {
    NurbsCurve circle = NurbsCurve::makeCircle(Vec3(0, 0, 0), 5.0);
    
    // Sample at many points — all should be distance 5 from center.
    for (int i = 0; i <= 100; ++i) {
        double t = circle.tMin() + (circle.tMax() - circle.tMin()) * i / 100.0;
        Vec3 p = circle.evaluate(t);
        double dist = std::sqrt(p.x * p.x + p.y * p.y);
        EXPECT_NEAR(dist, 5.0, Tolerance::kLinear)
            << "at t=" << t << " point=(" << p.x << "," << p.y << ")";
    }
}

TEST(NurbsCurveTest, ArcExactRepresentation) {
    // Quarter circle (0 to 90 degrees)
    NurbsCurve arc = NurbsCurve::makeArc(Vec3(0, 0, 0), 10.0, 0.0, M_PI / 2.0);

    Vec3 start = arc.evaluate(arc.tMin());
    EXPECT_NEAR(start.x, 10.0, 1e-8);
    EXPECT_NEAR(start.y, 0.0, 1e-8);

    Vec3 end = arc.evaluate(arc.tMax());
    EXPECT_NEAR(end.x, 0.0, 1e-8);
    EXPECT_NEAR(end.y, 10.0, 1e-8);

    // All points at radius 10.
    for (int i = 0; i <= 50; ++i) {
        double t = arc.tMin() + (arc.tMax() - arc.tMin()) * i / 50.0;
        Vec3 p = arc.evaluate(t);
        double dist = std::sqrt(p.x * p.x + p.y * p.y);
        EXPECT_NEAR(dist, 10.0, Tolerance::kLinear);
    }
}

TEST(NurbsCurveTest, EllipseExactRepresentation) {
    NurbsCurve ellipse = NurbsCurve::makeEllipse(Vec3(0, 0, 0), 10.0, 5.0);
    
    // Check that points satisfy x²/a² + y²/b² = 1
    for (int i = 0; i <= 100; ++i) {
        double t = ellipse.tMin() + (ellipse.tMax() - ellipse.tMin()) * i / 100.0;
        Vec3 p = ellipse.evaluate(t);
        double val = (p.x * p.x) / (10.0 * 10.0) + (p.y * p.y) / (5.0 * 5.0);
        EXPECT_NEAR(val, 1.0, 1e-6)
            << "at t=" << t << " point=(" << p.x << "," << p.y << ")";
    }
}
```

- [ ] **Step 2: Implement makeCircle**

A full circle as a 9-control-point rational NURBS (degree 2):
- 9 control points at 0°, 45°, 90°, ..., 360° (wrapping back)
- Weights: 1, cos(45°), 1, cos(45°), 1, cos(45°), 1, cos(45°), 1
- Knot vector: {0,0,0, 0.25,0.25, 0.5,0.5, 0.75,0.75, 1,1,1}

This is the standard rational quadratic NURBS circle representation.

- [ ] **Step 3: Implement makeArc**

Similar to circle but with fewer spans depending on the angle. For a quarter circle: 3 control points. For a semicircle: 5. For > 180°: multiple spans.

General approach: split the angle into arcs ≤ 90° each. Each 90° arc is a 3-point rational quadratic.

- [ ] **Step 4: Implement makeEllipse**

Same as circle but scale the control points by (semiMajor, semiMinor) instead of (radius, radius). Apply rotation if specified.

- [ ] **Step 5: Build and run tests**

- [ ] **Step 6: Commit**

```bash
git commit -m "feat(geometry): add exact NURBS conic factory functions (circle, arc, ellipse)"
```

---

## Task 6: Upgrade Spline Tool for NURBS Weight Editing

**Files:**
- Modify: `src/ui/src/SplineTool.cpp` (or equivalent)
- Modify: `src/ui/include/horizon/ui/` (if needed for new tool header)

- [ ] **Step 1: Read the existing spline tool**

Find and read the spline drawing tool. Understand how it creates DraftSpline entities.

- [ ] **Step 2: Add NURBS weight editing to grip manager**

When a NURBS-type spline is selected, the grip manager should show weight handles alongside control points. Scrolling or dragging a weight handle changes the weight of the associated control point.

This is a UI enhancement — read the GripManager to understand how grips work, then add a weight-editing mode.

For the initial implementation, a simple approach: when editing a spline's control point, holding Shift while dragging adjusts the weight instead of the position.

- [ ] **Step 3: Build and test manually**

Launch horizon.exe, draw a spline, select it, edit control point weights.

- [ ] **Step 4: Run all tests**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(ui): upgrade spline tool with NURBS weight editing"
```

---

## Task 7: Final Phase Commit

- [ ] **Step 1: Run complete test suite**

Report exact test count.

- [ ] **Step 2: Commit**

```bash
git add -A
git commit -m "Phase 31: NURBS curve library with De Boor evaluation and exact conics

- NurbsCurve in hz::geo with rational B-spline evaluation
- De Boor algorithm with homogeneous coordinates for weighted curves
- 1st and 2nd order derivative evaluation via quotient rule
- Adaptive curvature-based tessellation
- Boehm's knot insertion and degree elevation
- Closest-point projection via Newton iteration
- Arc-length parameterization
- Exact NURBS representations: circle, arc, ellipse
- Spline tool upgraded with NURBS weight editing"
```
