# Exact Kernel, Stage 1: Foundations — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the kernel exact geometry to build on — typed surfaces and curves, trim curves on faces, a tolerance model, robust predicates, a geometric validator, and watertight meshing of trimmed faces at the read point — with nothing in the app changing yet.

**Architecture:** New typed surfaces (`geo::Surface`: plane, cylinder, cone, sphere, torus, NURBS) and curves (`geo::Curve3`: line, circle, ellipse, NURBS; pcurves on the existing `geo::Curve2D`) live in `hz_geometry`. The half-edge B-rep gains exact fields beside today's (a face's surface and sense, an edge's curve and tolerance, a half-edge's pcurve), validated by `topo::validateExact`. `model::TrimmedFaceMesher` meshes exact faces watertight, and `BoundaryMesh::extractFacePolygons` uses it for faces that have an exact surface, so every consumer can read exact solids. No feature builds exact solids yet: `model::exact` primitives exist for tests and for stage 3.

**Tech Stack:** C++20, CMake (preset `linux-system-qt`), GoogleTest, `hz::math` vectors and matrices.

**Spec:** `docs/superpowers/specs/2026-11-24-exact-kernel-design.md` (this plan is its stage 1, "Foundations").

## Global Constraints

- C++20 that GCC 11, MSVC and Apple clang build; in-house code only, no new third-party library (spec: Constraints).
- Every CI gate stays green: GCC/MSVC/Apple clang builds, ASan, TSan, clang-tidy-15 (`bugprone-*`, `performance-*`), clang-format-15 over `src` and `tests`, coverage, fuzz, STEP interop.
- Nothing in the app changes in this stage: every existing test passes unchanged; no feature, file format or UI is touched.
- Kernel code holds no global mutable state (it runs on workers).
- Tolerances come from `math::ModelTolerance` (Task 1), never ad hoc epsilons, except where a step states its own.
- Namespaces as the project uses them: `hz::math`, `hz::geo`, `hz::topo`, `hz::model`.
- Formatting: `.clang-format` (Google-based, 4 spaces, 100 columns); run `~/.local/bin/clang-format-15 -i` on changed `.h`/`.cpp` (never on `CMakeLists.txt`).

## Conventions every task relies on

- **Parametrisations** (`Frame` = origin `o`, axes `x`, `y`, `z`):
  - Plane: `S(u,v) = o + u·x + v·y`; u, v unbounded.
  - Cylinder (radius r): `S(u,v) = o + r(cos u·x + sin u·y) + v·z`; u in [0, 2π) periodic, v unbounded.
  - Cone (radius r at v = 0, half-angle α, as STEP's `CONICAL_SURFACE`): `S(u,v) = o + (r + v·tan α)(cos u·x + sin u·y) + v·z`; u periodic, v unbounded (apex at v = −r / tan α).
  - Sphere (radius r): `S(u,v) = o + r cos v (cos u·x + sin u·y) + r sin v·z`; u periodic, v in [−π/2, π/2].
  - Torus (major R, minor r): `S(u,v) = o + (R + r cos v)(cos u·x + sin u·y) + r sin v·z`; u and v periodic.
  - Line: `C(t) = o + t·d`, d unit (t is distance). Circle: `C(t) = o + r(cos t·x + sin t·y)`, t in [0, 2π) periodic. Ellipse: `C(t) = o + a cos t·x + b sin t·y`.
- **Transforms keep parameters.** For a similarity `m` (rigid, uniformly scaled, possibly reflecting), `s.transformed(m)->evaluate(u,v) == m.transformPoint(s.evaluate(u,v))`, likewise for curves. A frame is moved axis by axis, so after a reflection it is left-handed; radii scale by the scale factor.
- **Orientation.** A face's outward normal is `du × dv` of its surface, negated when `Face::reversed`. Material is to the left of each half-edge seen from outside, so in (u,v) the outer loop is counter-clockwise when the face is not reversed (clockwise when it is) and inner loops the opposite way.
- **Edges.** `Edge::exactCurve` runs from `halfEdge->origin` at `curveStart` to `halfEdge->twin->origin` at `curveEnd` (`curveEnd < curveStart` reverses it; for a closed curve one full period). A half-edge's pcurve runs from its origin at `pcurveStart` to `next->origin` at `pcurveEnd`. A pcurve and its 3D curve need not share parameters: they agree in shape (Task 9 checks it).
- **Tolerance 0** on a vertex or edge means the model's resolution.

## Review Focus

1. A point at or just across a periodic seam (u = −1e-12, u = 2π + 1e-12) inverts to u in [0, 2π), and the two sides of a seam stay distinguishable in pcurves — Task 3.
2. A point on a cylinder's, cone's or torus's axis, or at a sphere's centre, inverts to finite parameters (u = 0), never NaN — Tasks 3 and 4.
3. Parts of 0.001 mm and of 100 m: tolerances scale, predicates stay right, meshes stay watertight — Tasks 1 and 12.
4. Triangulation input with duplicate or collinear boundary points, or a hole close to the outer loop — Task 11.
5. A sliver face (a plate's 100 × 0.01 side) meshes with no inverted triangles and stays watertight — Task 12.

## Before Task 1

- [ ] Configure the build in the worktree:

```bash
cd /home/brandon/Projects/horizon-cad/.claude/worktrees/exact-kernel
env VCPKG_ROOT=/home/brandon/vcpkg PATH="$HOME/.local/bin:$PATH" cmake --preset linux-system-qt
```

Expected: "Configuring done". Build a test target with
`env PATH="$HOME/.local/bin:$PATH" cmake --build build/linux-system-qt -j"$(nproc)" --target <target>`.

---

### Task 1: The tolerance model

**Files:**
- Create: `src/math/include/horizon/math/ModelTolerance.h`
- Test: `tests/math/test_ModelTolerance.cpp` (add to `tests/math/CMakeLists.txt`)

**Interfaces:**
- Produces:
  ```cpp
  namespace hz::math {
  struct ModelTolerance {
      double linear = 1e-7;    ///< the modelling resolution
      double angular = 1e-10;  ///< radians
      static ModelTolerance forExtent(double extent);
      double effective(double entityTolerance) const;  // max(linear, entityTolerance)
      bool within(double distance, double entityTolerance = 0.0) const;
      bool coincident(double distance, double tolA = 0.0, double tolB = 0.0) const;
  };
  }
  ```

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(ModelTolerance, ScalesWithThePartsSize) {
    EXPECT_DOUBLE_EQ(ModelTolerance::forExtent(100.0).linear, 1e-5);
    EXPECT_DOUBLE_EQ(ModelTolerance::forExtent(1e-3).linear, 1e-10);
    // Never finer than 64 units in the last place of the extent.
    const double huge = 1e300;
    EXPECT_GE(ModelTolerance::forExtent(huge).linear, 64 * (std::nextafter(huge, INFINITY) - huge));
}
TEST(ModelTolerance, AMeaninglessExtentIsTakenAsOne) {
    for (double e : {0.0, -5.0, std::nan(""), INFINITY})
        EXPECT_DOUBLE_EQ(ModelTolerance::forExtent(e).linear, 1e-7) << e;
}
TEST(ModelTolerance, EntitiesCoincideWhenTheirTolerancesMeet) {
    const ModelTolerance t = ModelTolerance::forExtent(10.0);  // 1e-6
    EXPECT_DOUBLE_EQ(t.effective(0.0), 1e-6);
    EXPECT_DOUBLE_EQ(t.effective(1e-4), 1e-4);
    EXPECT_TRUE(t.within(1e-6));
    EXPECT_FALSE(t.within(1.1e-6));
    EXPECT_TRUE(t.coincident(2e-6));            // 1e-6 + 1e-6
    EXPECT_TRUE(t.coincident(1.5e-4, 1e-4, 0));  // 1e-4 + 1e-6
    EXPECT_FALSE(t.coincident(2.1e-6));
}
```

- [ ] **Step 2: Run, expect a compile failure** (`hz_math_tests`: ModelTolerance.h not found).
- [ ] **Step 3: Implement** — header-only; `linear = max(1e-7 · extent, 64 · (nextafter(extent, ∞) − extent))`, extent taken as 1 when not finite and positive.
- [ ] **Step 4: Run** `build/linux-system-qt/tests/math/hz_math_tests --gtest_filter='ModelTolerance*'` — PASS.
- [ ] **Step 5: Commit** — `feat(math): a tolerance model scaled to the part`.

---

### Task 2: Robust predicates

**Files:**
- Create: `src/math/include/horizon/math/RobustPredicates.h`, `src/math/src/RobustPredicates.cpp` (add to the math library's sources)
- Test: `tests/math/test_RobustPredicates.cpp`

**Interfaces:**
- Produces:
  ```cpp
  namespace hz::math {
  /// +1 when c is left of the line a→b (counter-clockwise), −1 right, 0 on it — exactly.
  int orient2d(const Vec2& a, const Vec2& b, const Vec2& c);
  /// +1 when d is below the plane through a, b, c (they counter-clockwise seen from above), −1 above, 0 on it — exactly.
  int orient3d(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d);
  /// +1 when d is inside the circle through a, b, c (counter-clockwise), −1 outside, 0 on it — exactly.
  int incircle(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d);
  enum class Containment { Outside, Inside, OnBoundary };
  /// Where p is relative to the closed polygon (no closing duplicate), by winding number with orient2d.
  Containment classifyPoint(const std::vector<Vec2>& polygon, const Vec2& p);
  }
  ```

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(RobustPredicates, OrientationIsExactWhereDoublesLie) {
    // Shewchuk's classic: points near a line, offset by single ulps. A naive
    // determinant gives inconsistent signs; the exact one never does.
    const Vec2 a(0.5, 0.5), b(12.0, 12.0), c(24.0, 24.0);
    EXPECT_EQ(orient2d(a, b, c), 0);
    for (int i = 0; i < 256; ++i) {
        const Vec2 p(0.5 + i * std::ldexp(1.0, -53), 0.5);
        const int s = orient2d(p, b, c);
        EXPECT_EQ(s, i == 0 ? 0 : -1) << i;  // p moves right of the line y = x
    }
}
TEST(RobustPredicates, Orient3dAndIncircleSigns) {
    EXPECT_EQ(orient3d({0,0,0}, {1,0,0}, {0,1,0}, {0,0,-1}), 1);
    EXPECT_EQ(orient3d({0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}), -1);
    EXPECT_EQ(orient3d({0,0,0}, {1,0,0}, {0,1,0}, {0.3,0.7,0}), 0);
    EXPECT_EQ(incircle({0,0}, {1,0}, {0,1}, {0.5,0.5}), 1);
    EXPECT_EQ(incircle({0,0}, {1,0}, {0,1}, {1,1}), 0);
    EXPECT_EQ(incircle({0,0}, {1,0}, {0,1}, {2,2}), -1);
}
TEST(RobustPredicates, PointsAgainstAPolygon) {
    const std::vector<Vec2> l{{0,0}, {2,0}, {2,1}, {1,1}, {1,2}, {0,2}};  // an L
    EXPECT_EQ(classifyPoint(l, {0.5, 0.5}), Containment::Inside);
    EXPECT_EQ(classifyPoint(l, {1.5, 1.5}), Containment::Outside);
    EXPECT_EQ(classifyPoint(l, {1.0, 1.5}), Containment::OnBoundary);
    EXPECT_EQ(classifyPoint(l, {2.0, 0.0}), Containment::OnBoundary);
}
```

- [ ] **Step 2: Run, expect compile failure.**
- [ ] **Step 3: Implement** with a floating-point filter (the determinant against Shewchuk's error bound) and, when the filter cannot decide, an exact evaluation in floating-point expansions (`two_sum`, `two_product` via `std::fma`, expansion sum and scale). The staged adaptive levels are not needed.
- [ ] **Step 4: Run** `hz_math_tests --gtest_filter='RobustPredicates*'` — PASS.
- [ ] **Step 5: Commit** — `feat(math): exact orientation and incircle predicates`.

---

### Task 3: The surface interface, planes and cylinders

**Files:**
- Create: `src/geometry/include/horizon/geometry/Frame.h` (`Frame`, `Interval`), `src/geometry/src/Frame.cpp`, `src/geometry/include/horizon/geometry/surfaces/Surface.h`, `src/geometry/src/surfaces/Surface.cpp`, `src/geometry/include/horizon/geometry/surfaces/AnalyticSurfaces.h`, `src/geometry/src/surfaces/AnalyticSurfaces.cpp`
- Test: `tests/geometry/test_AnalyticSurfaces.cpp`, with a local `expectNear(const Vec3&, const Vec3&, double tol = 1e-12)` helper comparing component-wise

**Interfaces:**
- Produces:
  ```cpp
  namespace hz::geo {
  struct Frame {
      math::Vec3 origin, x{1,0,0}, y{0,1,0}, z{0,0,1};
      /// z along axis, x along xHint made perpendicular (any perpendicular when parallel), y = z × x.
      static Frame fromAxis(const math::Vec3& origin, const math::Vec3& axis,
                            const math::Vec3& xHint = {1,0,0});
      Frame transformed(const math::Mat4& m) const;  // each axis moved and normalised
      double scaleOf(const math::Mat4& m) const;     // m's uniform scale
  };
  struct Interval { double min = 0.0; double max = 0.0; };  // ±infinity allowed
  enum class SurfaceKind { Plane, Cylinder, Cone, Sphere, Torus, Nurbs };
  struct SurfaceDerivatives { math::Vec3 point, du, dv, duu, duv, dvv; };
  class Surface {
  public:
      virtual ~Surface() = default;
      virtual SurfaceKind kind() const = 0;
      virtual math::Vec3 evaluate(double u, double v) const = 0;
      virtual SurfaceDerivatives derivatives(double u, double v) const = 0;
      /// Unit du × dv; the zero vector where it is undefined (a pole, an apex).
      virtual math::Vec3 normal(double u, double v) const;
      virtual Interval uRange() const = 0;
      virtual Interval vRange() const = 0;
      virtual bool periodicU() const = 0;  // period = the range's width
      virtual bool periodicV() const = 0;
      /// Parameters of the nearest point of the surface; a periodic u in [min, max).
      virtual math::Vec2 invert(const math::Vec3& point) const = 0;
      virtual std::unique_ptr<Surface> transformed(const math::Mat4& m) const = 0;
  };
  class Plane : public Surface { public: explicit Plane(const Frame& frame); const Frame& frame() const; /* + overrides */ };
  class Cylinder : public Surface { public: Cylinder(const Frame& frame, double radius); double radius() const; const Frame& frame() const; /* + overrides */ };
  }
  ```

- [ ] **Step 1: Write the failing tests** — for each surface, a shared check over a grid of (u, v): `derivatives` agrees with central differences of `evaluate` (h = 1e-6, to 1e-6 relative); `invert(evaluate(u,v))` returns (u,v) (periodic u compared modulo 2π) to 1e-12 relative; and `transformed(m)` commutes with `m` for a rotation + translation, a uniform scale of 2.5 and a reflection in x. Plus:

```cpp
TEST(AnalyticSurfaces, ACylinderMatchesItsFormula) {
    const Cylinder c(Frame::fromAxis({1,2,3}, {0,0,1}), 4.0);
    expectNear(c.evaluate(0.0, 0.0), {5,2,3});
    expectNear(c.evaluate(M_PI/2, 7.0), {1,6,10});
    expectNear(c.normal(M_PI, 0.0), {-1,0,0});  // outward
    EXPECT_TRUE(c.periodicU());
    EXPECT_FALSE(c.periodicV());
    EXPECT_DOUBLE_EQ(c.uRange().max, 2*M_PI);
}
TEST(AnalyticSurfaces, InversionAtTheSeamStaysInRange) {
    const Cylinder c(Frame::fromAxis({0,0,0}, {0,0,1}), 1.0);
    for (double u : {-1e-12, 2*M_PI + 1e-12, 2*M_PI - 1e-12}) {
        const math::Vec2 uv = c.invert(c.evaluate(u, 0.5));
        EXPECT_GE(uv.x, 0.0); EXPECT_LT(uv.x, 2*M_PI) << u;
    }
}
TEST(AnalyticSurfaces, APointOnTheAxisInvertsToFiniteParameters) {
    const Cylinder c(Frame::fromAxis({0,0,0}, {0,0,1}), 1.0);
    const math::Vec2 uv = c.invert({0,0,3});
    EXPECT_DOUBLE_EQ(uv.x, 0.0); EXPECT_DOUBLE_EQ(uv.y, 3.0);
}
TEST(AnalyticSurfaces, AMirroredCylinderKeepsItsParametersAndTurnsItsNormal) {
    const Cylinder c(Frame::fromAxis({0,0,0}, {0,0,1}), 2.0);
    const math::Mat4 mirror = math::Mat4::scale({-1, 1, 1});
    const auto m = c.transformed(mirror);
    expectNear(m->evaluate(0.3, 1.0), mirror.transformPoint(c.evaluate(0.3, 1.0)));
    // du × dv now points into the cylinder: a face on it is `reversed`.
    EXPECT_LT(m->normal(0.0, 0.0).dot(m->evaluate(0.0, 0.0)), 0.0);
}
```

- [ ] **Step 2: Run, expect compile failure.**
- [ ] **Step 3: Implement** — closed forms throughout; the plane's ranges are ±infinity; `Surface::normal` defaults to normalised `du × dv` from `derivatives`. (Use `math::Mat4`'s existing scale factory or its equivalent for the mirror test.)
- [ ] **Step 4: Run** `hz_geometry_tests --gtest_filter='AnalyticSurfaces*'` — PASS.
- [ ] **Step 5: Commit** — `feat(geometry): typed surfaces, planes and cylinders`.

---

### Task 4: Cones, spheres and tori

**Files:**
- Modify: `AnalyticSurfaces.h/.cpp` (Task 3)
- Test: `tests/geometry/test_AnalyticSurfaces.cpp`

**Interfaces:**
- Consumes: `Surface`, `Frame` (Task 3).
- Produces: `Cone(const Frame&, double radius, double halfAngle)`, `Sphere(const Frame&, double radius)`, `Torus(const Frame&, double majorRadius, double minorRadius)`, each with accessors for its parameters and the `Surface` overrides; parametrisations as in Conventions.

- [ ] **Step 1: Write the failing tests** — the shared checks of Task 3 for each; and:

```cpp
const Cone kCone(Frame::fromAxis({0,0,0}, {0,0,1}), 2.0, M_PI/4);
TEST(AnalyticSurfaces, ConeSphereAndTorusMatchTheirFormulas) {
    expectNear(kCone.evaluate(0.0, 1.0), {3,0,1});          // r grows with v
    expectNear(kCone.evaluate(0.0, -2.0), {0,0,-2});         // the apex
    const Sphere s(Frame::fromAxis({1,1,1}, {0,0,1}), 3.0);
    expectNear(s.evaluate(0.0, M_PI/2), {1,1,4});           // the pole
    EXPECT_DOUBLE_EQ(s.vRange().min, -M_PI/2);
    const Torus t(Frame::fromAxis({0,0,0}, {0,0,1}), 5.0, 1.0);
    expectNear(t.evaluate(0.0, 0.0), {6,0,0});
    expectNear(t.evaluate(0.0, M_PI), {4,0,0});
    EXPECT_TRUE(t.periodicU()); EXPECT_TRUE(t.periodicV());
}
TEST(AnalyticSurfaces, CentresAndAxesInvertToFiniteParameters) {
    const Sphere s(Frame::fromAxis({0,0,0}, {0,0,1}), 1.0);
    const math::Vec2 c = s.invert({0,0,0});
    EXPECT_TRUE(std::isfinite(c.x) && std::isfinite(c.y));
    const math::Vec2 pole = s.invert({0,0,5});
    EXPECT_DOUBLE_EQ(pole.x, 0.0); EXPECT_DOUBLE_EQ(pole.y, M_PI/2);
    const Torus t(Frame::fromAxis({0,0,0}, {0,0,1}), 5.0, 1.0);
    const math::Vec2 axis = t.invert({0,0,2});
    EXPECT_TRUE(std::isfinite(axis.x) && std::isfinite(axis.y));
    EXPECT_NEAR(Sphere(Frame{}, 1.0).normal(0.0, M_PI/2).length(), 1.0, 1e-12)
        << "a sphere's pole has a normal (the limit along its meridians)";
    EXPECT_DOUBLE_EQ(kCone.normal(0.0, -2.0).length(), 0.0) << "a cone's apex has none";
}
```

- [ ] **Step 2–4:** run (fail), implement (closed-form inversion: the meridian plane through the point, then the nearest point of the profile line or circle), run `--gtest_filter='AnalyticSurfaces*'` — PASS.
- [ ] **Step 5: Commit** — `feat(geometry): cones, spheres and tori`.

---

### Task 5: NURBS surfaces as typed surfaces

**Files:**
- Modify: `src/geometry/include/horizon/geometry/surfaces/NurbsSurface.h`, `src/geometry/src/surfaces/NurbsSurface.cpp`
- Test: `tests/geometry/test_NurbsSurface.cpp`

**Interfaces:**
- Consumes: `Surface` (Task 3).
- Produces: `class NurbsSurface : public Surface` — its existing API unchanged, plus `kind() == SurfaceKind::Nurbs`; `derivatives(u, v)` with the second derivatives of the rational surface; `uRange()/vRange()` from `uMin..vMax`; `periodicU()/periodicV()` = `closedU()/closedV()`; `invert` via `locate`; `transformed` (control points moved; weights kept); and
  ```cpp
  NurbsSurface insertKnotU(double u) const;   // same surface, one more U knot
  NurbsSurface insertKnotV(double v) const;
  std::pair<NurbsSurface, NurbsSurface> splitU(double u) const;  // [uMin,u] and [u,uMax]
  std::pair<NurbsSurface, NurbsSurface> splitV(double v) const;
  ```

- [ ] **Step 1: Write the failing tests** — on `makeCylinder`, `makeSphere`, `makeTorus` and a degree-(3,2) non-rational patch: second derivatives agree with differences of first derivatives (h = 1e-6, 1e-5 relative); `insertKnotU/V` leaves `evaluate` unchanged on a 20 × 20 grid to 1e-12; each half of `splitU/V` evaluates as the original over its sub-range to 1e-12; `transformed` commutes with a rigid motion and a reflection; and the existing NurbsSurface tests still pass.
- [ ] **Step 2–4:** fail, implement (Boehm insertion applied per row/column in homogeneous coordinates; split = insert to full multiplicity then cut), run `hz_geometry_tests --gtest_filter='NurbsSurface*'` — PASS.
- [ ] **Step 5: Commit** — `feat(geometry): NURBS surfaces as typed surfaces, with knot insertion and splitting`.

---

### Task 6: Recognising Horizon's NURBS as typed surfaces

**Files:**
- Create: `src/geometry/include/horizon/geometry/surfaces/SurfaceRecognition.h`, `src/geometry/src/surfaces/SurfaceRecognition.cpp`
- Test: `tests/geometry/test_SurfaceRecognition.cpp`

**Interfaces:**
- Consumes: `NurbsSurface`, the analytic surfaces (Tasks 3–5).
- Produces:
  ```cpp
  namespace hz::geo {
  /// The typed surface @p surface exactly is (every sampled point within @p tolerance of it,
  /// and its parameters mapping one to one), or null. For surfaces Horizon makes
  /// (NurbsSurface::makePlane … makeCone); imported B-splines are not passed through it (spec, goal 7).
  std::unique_ptr<Surface> recognize(const NurbsSurface& surface, double tolerance);
  }
  ```

- [ ] **Step 1: Write the failing tests** — each factory (`makePlane`, `makeCylinder`, `makeCone`, `makeSphere`, `makeTorus`), at an arbitrary placement, is recognised as its kind with its radius/half-angle/frame within 1e-9; a 20 × 20 grid of the NURBS lies on the recognised surface within 1e-9; a NURBS bumped by 1e-4 at one control point, and a bicubic free-form patch, give null.
- [ ] **Step 2–4:** fail; implement (fit the candidate kind from sampled points and normals, then verify every sample of a dense grid within tolerance); run `--gtest_filter='SurfaceRecognition*'` — PASS.
- [ ] **Step 5: Commit** — `feat(geometry): recognise the quadrics Horizon's NURBS factories make`.

---

### Task 7: Typed curves and 2D NURBS for pcurves

**Files:**
- Create: `src/geometry/include/horizon/geometry/curves/Curve3.h`, `src/geometry/include/horizon/geometry/curves/AnalyticCurves.h`, `src/geometry/src/curves/AnalyticCurves.cpp`, `src/geometry/include/horizon/geometry/curves/Nurbs2D.h`, `src/geometry/src/curves/Nurbs2D.cpp`
- Modify: `NurbsCurve.h/.cpp` (implements `Curve3`)
- Test: `tests/geometry/test_Curve3.cpp`

**Interfaces:**
- Consumes: `Frame`, `Interval` (`geometry/Frame.h`, Task 3).
- Produces:
  ```cpp
  namespace hz::geo {
  enum class CurveKind { Line, Circle, Ellipse, Nurbs };
  struct CurveDerivatives { math::Vec3 point, d1, d2; };
  class Curve3 {
  public:
      virtual ~Curve3() = default;
      virtual CurveKind kind() const = 0;
      virtual math::Vec3 evaluate(double t) const = 0;
      virtual CurveDerivatives derivatives(double t) const = 0;
      virtual Interval range() const = 0;
      virtual bool periodic() const = 0;
      virtual double closestParameter(const math::Vec3& point) const = 0;  // periodic t in [min,max)
      virtual std::unique_ptr<Curve3> transformed(const math::Mat4& m) const = 0;
  };
  class LineCurve : public Curve3 { public: LineCurve(const math::Vec3& origin, const math::Vec3& direction); /* direction normalised */ };
  class CircleCurve : public Curve3 { public: CircleCurve(const Frame& frame, double radius); };
  class EllipseCurve : public Curve3 { public: EllipseCurve(const Frame& frame, double majorRadius, double minorRadius); };
  /// A pcurve that is a NURBS: a 2D rational B-spline (a NurbsCurve with z = 0 inside).
  class Nurbs2D : public Curve2D {
  public:
      Nurbs2D(std::vector<math::Vec2> controlPoints, std::vector<double> weights,
              std::vector<double> knots, int degree);
  };
  }
  ```
  `NurbsCurve` gains `kind()`, `derivatives`, `range()` (`tMin..tMax`), `periodic()` (false), `closestParameter` (its `closestPoint`), `transformed`.

- [ ] **Step 1: Write the failing tests** — for line, circle, ellipse and `NurbsCurve::makeCircle`: `derivatives` against differences of `evaluate`; `closestParameter(evaluate(t)) == t` (modulo period) to 1e-12; `transformed` commutes with rigid motion, scale and reflection; a point at a circle's centre gives a finite parameter (0); `Nurbs2D` built as a quarter circle evaluates on the unit circle to 1e-12 and its `derivative(t, 1)` matches differences.
- [ ] **Step 2–4:** fail; implement; run `hz_geometry_tests --gtest_filter='Curve3*'` — PASS.
- [ ] **Step 5: Commit** — `feat(geometry): typed 3D curves, and NURBS pcurves`.

---

### Task 8: Exact fields in the B-rep, and exact primitives

**Files:**
- Modify: `src/topology/include/horizon/topology/HalfEdge.h`
- Create: `src/modeling/include/horizon/modeling/ExactPrimitives.h`, `src/modeling/src/ExactPrimitives.cpp`
- Test: `tests/modeling/test_ExactPrimitives.cpp`

**Interfaces:**
- Consumes: `Surface`, `Curve3`, `Curve2D`, `Frame` (Tasks 3–7).
- Produces — new fields (beside today's, which stay):
  ```cpp
  // Vertex
  double tolerance = 0.0;
  // HalfEdge
  std::shared_ptr<const geo::Curve2D> pcurve; double pcurveStart = 0.0; double pcurveEnd = 0.0;
  // Edge
  std::shared_ptr<const geo::Curve3> exactCurve; double curveStart = 0.0; double curveEnd = 0.0;
  double tolerance = 0.0;
  // Face
  std::shared_ptr<const geo::Surface> exactSurface; bool reversed = false;
  ```
  and
  ```cpp
  namespace hz::model::exact {
  std::unique_ptr<topo::Solid> makeBlock(const math::Vec3& min, const math::Vec3& max);
  /// base.origin at the bottom cap's centre, height along base.z; one seam edge at u = 0.
  std::unique_ptr<topo::Solid> makeCylinder(const geo::Frame& base, double radius, double height);
  /// A block with a round hole through it along z, centred at holeCentre (x, y).
  std::unique_ptr<topo::Solid> makePlateWithHole(const math::Vec3& min, const math::Vec3& max,
                                                 const math::Vec2& holeCentre, double holeRadius);
  }
  ```
  Faces are named as `PrimitiveFactory` names its faceted ones, without facet suffixes (`cylinder/side`, `cylinder/top`, …); the hole's face `hole/wall`.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(ExactPrimitives, TheirTopologyIsWholeFaces) {
    const auto block = exact::makeBlock({0,0,0}, {2,3,4});
    EXPECT_EQ(block->vertexCount(), 8u); EXPECT_EQ(block->edgeCount(), 12u);
    EXPECT_EQ(block->faceCount(), 6u);
    const auto cyl = exact::makeCylinder(geo::Frame::fromAxis({0,0,0}, {0,0,1}), 5.0, 10.0);
    EXPECT_EQ(cyl->vertexCount(), 2u); EXPECT_EQ(cyl->edgeCount(), 3u);  // two rims, one seam
    EXPECT_EQ(cyl->faceCount(), 3u);
    const auto plate = exact::makePlateWithHole({0,0,0}, {40,40,10}, {20,20}, 5.0);
    EXPECT_EQ(plate->vertexCount(), 10u); EXPECT_EQ(plate->edgeCount(), 15u);
    EXPECT_EQ(plate->faceCount(), 7u);
    for (const auto* s : {block.get(), cyl.get(), plate.get()}) {
        EXPECT_TRUE(s->isValid()) << s->validationReport();
        EXPECT_TRUE(s->checkEulerFormula());
    }
}
TEST(ExactPrimitives, EveryPieceHasItsExactGeometry) {
    const auto cyl = exact::makeCylinder(geo::Frame::fromAxis({0,0,0}, {0,0,1}), 5.0, 10.0);
    for (const auto& f : cyl->faces()) ASSERT_NE(f.exactSurface, nullptr);
    for (const auto& e : cyl->edges()) {
        ASSERT_NE(e.exactCurve, nullptr);
        EXPECT_NE(e.halfEdge->pcurve, nullptr); EXPECT_NE(e.halfEdge->twin->pcurve, nullptr);
    }
    // The side is a cylinder of radius 5; a rim is a circle of radius 5.
    // (Spot checks; Task 9's validator checks everything.)
}
```

- [ ] **Step 2: Run, expect compile failure.**
- [ ] **Step 3: Implement** — build each solid with `Solid::alloc*` and the conventions (orientation, edge and pcurve ranges); the cylinder's seam edge has both half-edges in the side face, with pcurves `u = 0` and `u = 2π`. Existing code that copies faces is untouched here (Task 10).
- [ ] **Step 4: Run** `hz_modeling_tests --gtest_filter='ExactPrimitives*'`, then the whole `hz_topology_tests` and `hz_modeling_tests` — PASS (nothing else changed).
- [ ] **Step 5: Commit** — `feat(topology): exact geometry on faces, edges and half-edges; exact primitives`.

---

### Task 9: The exact validator

**Files:**
- Create: `src/topology/include/horizon/topology/ExactValidator.h`, `src/topology/src/ExactValidator.cpp`
- Test: `tests/modeling/test_ExactValidator.cpp` (it builds its solids with Task 8's primitives)

**Interfaces:**
- Consumes: the exact fields (Task 8), `ModelTolerance`, `classifyPoint`, `orient2d` (Tasks 1–2).
- Produces:
  ```cpp
  namespace hz::topo {
  enum class ExactIssueKind { MissingGeometry, VertexOffCurve, PcurveOffCurve, PcurveOffVertex,
                              LoopNotClosed, LoopWrongOrientation, LoopSelfIntersects,
                              LoopsCross, HoleOutsideOuterLoop };
  struct ExactIssue { ExactIssueKind kind; const Face* face = nullptr; const Edge* edge = nullptr;
                      std::string detail; };
  /// Every way @p solid's exact geometry disagrees with its topology; empty when it agrees.
  std::vector<ExactIssue> validateExact(const Solid& solid, const math::ModelTolerance& tolerance);
  }
  ```
  Checks, each on 17 samples per pcurve (its two ends and 15 between): every alive face, non-seam edge and half-edge has its geometry; an edge's curve meets its vertices at `curveStart`/`curveEnd`; each pcurve, mapped onto its face's surface, lies on the edge's curve and meets the half-edge's vertices, within the edge's effective tolerance; consecutive pcurves of a loop join in (u, v) (periodic parameters compared modulo their period); loop orientation by signed area in (u, v) per the Conventions; no loop crosses itself or another (sampled segments, `orient2d`); inner loops inside the outer one.

- [ ] **Step 1: Write the failing tests** — the three primitives at arbitrary placements give no issues; then one mutation per kind, each giving exactly that kind: a moved vertex (`VertexOffCurve`), a pcurve swapped for one at `v + 0.1` (`PcurveOffCurve`), a face's `exactSurface` cleared (`MissingGeometry`), a face's `reversed` flipped (`LoopWrongOrientation`), a hole's centre moved outside the plate (`HoleOutsideOuterLoop`).
- [ ] **Step 2–4:** fail; implement; run `hz_modeling_tests --gtest_filter='ExactValidator*'` — PASS.
- [ ] **Step 5: Commit** — `feat(topology): a validator for exact geometry`.

---

### Task 10: Copies and transforms carry exact geometry

**Files:**
- Modify: `src/modeling/src/Pattern.cpp` (`Pattern::transformed`, `Pattern::append`)
- Test: `tests/modeling/test_ExactPrimitives.cpp`

**Interfaces:**
- Consumes: `Surface::transformed`, `Curve3::transformed`, `validateExact`.
- Produces: `Pattern::transformed` and `append` copy `exactSurface` (moved), `reversed` (toggled under a reflection, which reverses loops as the code already does), `exactCurve` (moved; start/end swapped where the edge's direction flips), pcurves (unchanged in (u, v); start/end swapped where a half-edge is reversed), and tolerances (scaled).

- [ ] **Step 1: Write the failing test**

```cpp
TEST(ExactPrimitives, TransformedAndMirroredCopiesStayValid) {
    const auto plate = exact::makePlateWithHole({0,0,0}, {40,40,10}, {20,20}, 5.0);
    const math::ModelTolerance tol = math::ModelTolerance::forExtent(60.0);
    for (const math::Mat4& m : {math::Mat4::translation({3,4,5}) * math::Mat4::rotationZ(0.7),
                                math::Mat4::scale({-1,1,1}), math::Mat4::scale({2,2,2})}) {
        const auto copy = model::Pattern::transformed(*plate, m);
        EXPECT_TRUE(topo::validateExact(*copy, tol).empty());
    }
}
```

- [ ] **Step 2–4:** fail; implement; run `hz_modeling_tests` whole — PASS (existing Pattern tests unchanged).
- [ ] **Step 5: Commit** — `feat(modeling): placed and mirrored copies keep their exact geometry`.

---

### Task 11: Constrained Delaunay triangulation in 2D

**Files:**
- Create: `src/geometry/include/horizon/geometry/Triangulate2D.h`, `src/geometry/src/Triangulate2D.cpp`
- Test: `tests/geometry/test_Triangulate2D.cpp`

**Interfaces:**
- Consumes: `orient2d`, `incircle`, `classifyPoint` (Task 2).
- Produces:
  ```cpp
  namespace hz::geo {
  struct Triangulation2D {
      std::vector<math::Vec2> points;                     // boundary points first, in input order
      std::vector<std::array<std::uint32_t, 3>> triangles;  // counter-clockwise
  };
  /// The region inside @p outer and outside each of @p holes (closed polylines, no closing
  /// duplicate), with @p interior points added where they fall strictly inside it. Every
  /// boundary segment is an edge of the result, unsplit (callers rely on it to stay
  /// watertight); boundary points within @p merge of each other are one point.
  Triangulation2D triangulateRegion(const std::vector<math::Vec2>& outer,
                                    const std::vector<std::vector<math::Vec2>>& holes,
                                    const std::vector<math::Vec2>& interior = {},
                                    double merge = 0.0);
  }
  ```

- [ ] **Step 1: Write the failing tests** — for a square, an L, a square with a square hole, a square with a hole 1e-6 from its outer edge, a 100 × 0.01 sliver, and an outer loop with three collinear points and a duplicated point: triangle areas sum to the region's area (to 1e-12 relative); every boundary segment appears as exactly one triangle edge; no triangle has a negative or zero area; every interior point inside the region is a vertex; the result is Delaunay (no point strictly inside a triangle's circumcircle, by `incircle`) except across boundary segments.
- [ ] **Step 2–4:** fail; implement (Bowyer–Watson with constraint recovery by edge flips, then removal of triangles outside the region by `classifyPoint` of their centroids); run `hz_geometry_tests --gtest_filter='Triangulate2D*'` — PASS.
- [ ] **Step 5: Commit** — `feat(geometry): constrained Delaunay triangulation of a region`.

---

### Task 12: Meshing trimmed faces

**Files:**
- Create: `src/modeling/include/horizon/modeling/TrimmedFaceMesher.h`, `src/modeling/src/TrimmedFaceMesher.cpp`
- Test: `tests/modeling/test_TrimmedFaceMesher.cpp`

**Interfaces:**
- Consumes: exact fields and primitives (Task 8), `triangulateRegion` (Task 11), `Surface`, `Curve3`.
- Produces:
  ```cpp
  namespace hz::model {
  struct MeshOptions {
      double chordTolerance = 0.01;  // largest distance from the mesh to the surface
      double maxAngle = 0.35;        // radians between neighbouring triangle normals on a face
      std::size_t maxTrianglesPerFace = 200000;
  };
  struct FaceMesh {
      const topo::Face* face = nullptr;
      std::vector<math::Vec3> points;
      std::vector<math::Vec3> normals;                      // the surface's, outward
      std::vector<std::array<std::uint32_t, 3>> triangles;  // wound outward
  };
  class TrimmedFaceMesher {
  public:
      /// Every exact face of @p solid meshed. Each edge is discretised once, and both its
      /// faces use those very points, so the meshes meet without gaps.
      static std::vector<FaceMesh> meshSolid(const topo::Solid& solid, const MeshOptions& options);
  };
  }
  ```

- [ ] **Step 1: Write the failing tests**

```cpp
// For the three primitives, at 1 mm scale and scaled by 1e-3 and by 1e4:
//  - watertight: every boundary segment of every face mesh appears in exactly two
//    face meshes, once each way (the points are the same Vec3 values);
//  - chord error: the surface point at each triangle's (u,v) centroid and edge
//    midpoints is within chordTolerance of the triangle;
//  - outward: each triangle's normal has a positive dot with the surface normal at
//    its centroid (no inverted triangles), including on the plate's 0.01-wide sliver
//    side in a 100 x 0.01 x 50 block;
//  - area: the cylinder's side within 1e-3 relative of 2*pi*r*h at tolerance 1e-3 r.
TEST(TrimmedFaceMesher, ACylinderMeshesWatertightToTheTolerance) { ... }
TEST(TrimmedFaceMesher, APlateWithAHoleMeshesWatertight) { ... }
TEST(TrimmedFaceMesher, ASliverFaceHasNoInvertedTriangles) { ... }
TEST(TrimmedFaceMesher, TinyAndHugePartsMeshAlike) { ... }
```

- [ ] **Step 2–4:** fail; implement (edge samples from curvature and the chord tolerance; each face's boundary in (u, v) from its pcurves at those samples, the seam's two sides apart; interior points on a (u, v) grid spaced from the surface's curvature; `triangulateRegion`; refine triangles whose chord error exceeds the tolerance by inserting their (u, v) centroid, up to `maxTrianglesPerFace`); run `hz_modeling_tests --gtest_filter='TrimmedFaceMesher*'` — PASS.
- [ ] **Step 5: Commit** — `feat(modeling): watertight meshes of trimmed exact faces`.

---

### Task 13: Exact faces at the read point

**Files:**
- Modify: `src/modeling/include/horizon/modeling/BoundaryMesh.h`, `src/modeling/src/BoundaryMesh.cpp`
- Test: `tests/modeling/test_ExactReadPoint.cpp`

**Interfaces:**
- Consumes: `TrimmedFaceMesher::meshSolid` (Task 12).
- Produces: `BoundaryPolygon` gains `std::shared_ptr<const geo::Surface> exactSurface`; `extractFacePolygons(const topo::Solid&)` meshes every face with an `exactSurface` at a chord tolerance of 1e-4 × the solid's bounding-box diagonal and returns one polygon per triangle (its face's `topoId` and `exactSurface`); a new overload `extractFacePolygons(const topo::Solid&, double chordTolerance)` takes the tolerance. Faces without one are read as today.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(ExactReadPoint, MassPropertiesOfExactSolidsAreTheirShapes) {
    const auto cyl = exact::makeCylinder(geo::Frame::fromAxis({0,0,0}, {0,0,1}), 5.0, 10.0);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*cyl).volume, M_PI * 25 * 10, M_PI * 250 * 1e-3);
    const auto plate = exact::makePlateWithHole({0,0,0}, {40,40,10}, {20,20}, 5.0);
    const double v = 40.0 * 40 * 10 - M_PI * 25 * 10;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*plate).volume, v, v * 1e-3);
}
TEST(ExactReadPoint, EveryPolygonNamesItsFace) {
    const auto cyl = exact::makeCylinder(geo::Frame::fromAxis({0,0,0}, {0,0,1}), 5.0, 10.0);
    std::set<std::string> names;
    for (const auto& p : BoundaryMesh::extractFacePolygons(*cyl)) {
        EXPECT_EQ(p.points.size(), 3u);
        names.insert(p.topoId.tag());
    }
    EXPECT_EQ(names.size(), 3u);  // side, top, bottom
}
TEST(ExactReadPoint, FacetedSolidsReadAsBefore) {
    const auto faceted = PrimitiveFactory::makeCylinder(5.0, 10.0);  // today's, 32 facets
    EXPECT_EQ(BoundaryMesh::extractFacePolygons(*faceted).size(), faceted->faceCount());
}
```

- [ ] **Step 2–4:** fail; implement; run the whole `hz_modeling_tests`, `hz_topology_tests`, `hz_geometry_tests`, `hz_math_tests` — PASS.
- [ ] **Step 5: Commit** — `feat(modeling): exact faces read as meshes where every consumer reads faces`.

---

### Task 14: Stage gate

- [ ] **Step 1:** Full suite: `QT_QPA_PLATFORM=offscreen ctest --test-dir build/linux-system-qt -j"$(nproc)"` — "100% tests passed".
- [ ] **Step 2:** Format: `find src tests -name '*.h' -o -name '*.cpp' | xargs ~/.local/bin/clang-format-15 --dry-run --Werror` — no output.
- [ ] **Step 3:** The clang-tidy gate on every changed `src/*.cpp`, with clang-tidy-15's check list and a compile database stripped of `-mno-direct-extern-access` (see the memory note on clang-tidy versions); no local `const` returned by value.
- [ ] **Step 4:** Update `docs/superpowers/notes/2026-11-23-competitive-gaps.md` (the kernel row: stage 1 done) and the spec's stage 1 line with what landed.
- [ ] **Step 5:** Push `feat/exact-kernel`, open the PR ("Exact kernel, stage 1: foundations"), watch CI to green.

## Deferred from stage 1 (on purpose)

- **Singular faces** (a sphere's poles, a cone's apex) as topology: their surfaces exist (Task 4), but how a face meets a pole (degenerate edges or not) is decided in stage 3, with the exact sphere and cone primitives.
- **`IntersectionCurve`** belongs to the intersection library (stage 4).
- **The faceted bridge** for unconverted operations (generalised `facetCurved`) comes with the first exact solids in the feature tree (stage 2).
- **Exact mass properties** by integration over trimmed surfaces (stage 2); here mass comes from the read-point mesh.
