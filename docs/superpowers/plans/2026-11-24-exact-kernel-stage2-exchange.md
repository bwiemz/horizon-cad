# Exact Kernel, Stage 2 (Exact Exchange) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** STEP import keeps faces exact. Rendering, mass properties and STEP export use those exact faces, and every modelling operation still gets the faceted copy it understands.

**Architecture:**
- The STEP reader fills the stage-1 exact fields beside today's NURBS carriers. It builds pcurves itself, inserts seams where a periodic face has none, and refuses, with the reason, a solid it cannot make exact.
- `ImportedBodyFeature` keeps handing operations its faceted copy: today's `facetCurved` of the carriers, which is the spec's "faceted copy for unconverted operations".
- The build also carries the part's **exact counterpart** while only imported bodies have made the part. The document hands that counterpart to rendering, mass and export, and drops it at the first feature that changes the part.
- Mass properties integrate exactly over trimmed faces. The writer emits typed STEP surfaces.

**Tech Stack:** C++20, CMake (preset `linux-system-qt` locally), GoogleTest, libFuzzer (CI), Python + OpenCASCADE (`cadquery-ocp`) as the CI oracle.

**Spec:** `docs/superpowers/specs/2026-11-24-exact-kernel-design.md`. This plan covers stage 2 ("Exact exchange") and the stage-1 items the spec carries to stage 2.

## Global Constraints

- C++20, in-house, GPL-3.0-or-later; no new third-party geometry library (spec: Constraints).
- Every CI gate stays green: GCC 11, MSVC and Apple clang builds, ASan, TSan, clang-tidy-15 (`--checks='-*,bugprone-*,performance-*'` with ci.yml's backlog exclusions), clang-format-15 over `src` and `tests`, coverage, fuzzing, and the OpenCASCADE STEP interop job.
- Kernel code holds no global mutable state; imports and rebuilds run on workers.
- Tolerances come from `math::ModelTolerance` and `topo::exactTolerance(solid)`, never ad hoc epsilons, except where a step states its own.
- "Fallbacks that hide failure" are an anti-goal. An exact import or meshing that fails says why. It never quietly runs faceted code in place of the exact result.
- Owner decision 1b: a model this stage cannot make exact fails with its reason. The corpus files that newly fail are listed in this stage's PR and worked down.
- Spec goal 7:
  - a `CYLINDRICAL_SURFACE` comes back as one;
  - a `B_SPLINE_SURFACE` comes back as the same B-spline, even when it is a quadric;
  - imported B-splines are never recognised as typed surfaces.
- Spec goal 6: volumes, areas and centres of mass to 1e-9 relative for solids bounded by closed-form geometry.
- Existing tests keep passing, or are re-baselined on purpose, with each change named in the PR.
- Old native files (formats 1 to 31) still open. The native format stays 31; the migration to 32 is stage 3.
- Never clang-format a `CMakeLists.txt`.
- No local `const` returned by value (CI's `performance-no-automatic-move`).
- No gtest macro as the unbraced body of an `if`. GCC 11's `-Werror=dangling-else` fails it.

## Conventions (stage 1's hold; these are added)

- **STEP parametrisations equal ours.**
  - PLANE, CYLINDRICAL_SURFACE, CONICAL_SURFACE, SPHERICAL_SURFACE and TOROIDAL_SURFACE map one to one onto `Plane`, `Cylinder`, `Cone`, `Sphere` and `Torus`, with frame `Frame::fromAxis(location, axis, ref_direction)` and scale factors 1. Cone semi-angles are read through the file's plane-angle unit.
  - A LINE `(pnt, VECTOR(dir, magnitude))` is `LineCurve(pnt, dir, magnitude)`; the writer writes `magnitude = scale`, which folds Ruling 5's scale in.
  - CIRCLE and ELLIPSE map to `CircleCurve` and `EllipseCurve`.
- **A face's sense** is `Face::reversed = !(same_sense)`, combined with the bound and shell flags the carrier code already combines. The carrier keeps its U-reversal, and the exact surface is never reversed.
- **Pcurves are built, not read.** The reader computes every pcurve with `geo::pcurveOn` (Task 4) and ignores PCURVE entities. The writer writes none; STEP readers, OpenCASCADE among them, compute their own.
- **The exact counterpart** carries the faceted part's logical names. Its faces are `<featureID>/face:<i>` and its edges are named by the same pass, so `logicalFace` and `logicalEdge` of a faceted entity name the exact one, and a pick on the exact mesh resolves in the faceted part.

## Review Focus

1. **A real exporter's edges stand off their surfaces** within the file's UNCERTAINTY_MEASURE (gaps around 1e-5). The import succeeds, with each edge's tolerance grown to cover its gap, and the validator stays clean. *(Task 5)*
2. **A periodic face that is a band between two rims with no seam**, including Solid Edge's case of two loops winding the same way. It imports exact, with a seam inserted, and its volume is exact. *(Task 6)*
3. **A file in inches or metres.** Its exact geometry is scaled with it: radii, frames and a line's magnitude. A round trip back to millimetres keeps each surface. *(Tasks 5 and 8)*
4. **A face whose `same_sense` is false, and a void shell (BREP_WITH_VOIDS).** The outward normals are right, the void's volume subtracts, and the total is positive. *(Tasks 5 and 7)*
5. **An imported part followed by a feature (a cut or a fillet).** The operation runs on the faceted copy exactly as on master, the exact counterpart drops, and nothing about the faceted behaviour changes. *(Task 9)*

---

### Task 1: Guards carried from stage 1

**Files:**
- Modify: `src/math/include/horizon/math/ModelTolerance.h`
- Modify: `src/geometry/src/Triangulate2D.cpp`
- Modify: `src/topology/include/horizon/topology/ExactValidator.h`, `src/topology/src/ExactValidator.cpp`
- Create: `tests/fuzz/fuzz_triangulate.cpp`, plus seed inputs in `tests/fuzz/corpus/triangulate/`
- Modify: `tests/fuzz/CMakeLists.txt`
- Test: `tests/math/test_ModelTolerance.cpp`, `tests/geometry/test_Triangulate2D.cpp`, `tests/modeling/test_ExactValidator.cpp`

**Interfaces:**
- Produces:
  - `ModelTolerance::forExtent` clamps a non-finite or huge extent to 1e100, the predicates' upper range.
  - `triangulateRegion` sets `problem` to "the points spread too little for exact predicates" when the boundary's non-zero spread is under 1e-60.
  - `ExactIssueKind::RangeTooLong`: an edge whose curve range exceeds one period of its periodic curve, or a loop whose unwrapped extent exceeds a period on a periodic axis.

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(ModelTolerance, AnInfiniteExtentGivesAFiniteResolution) {
    EXPECT_TRUE(std::isfinite(math::ModelTolerance::forExtent(DBL_MAX).resolution));
    EXPECT_EQ(math::ModelTolerance::forExtent(DBL_MAX).resolution,
              math::ModelTolerance::forExtent(1e100).resolution);
}
TEST(Triangulate2D, PointsSpreadTooLittleAreSaidToBe) {
    const auto t = geo::triangulateRegion({{0,0},{1e-62,0},{1e-62,1e-62},{0,1e-62}}, {});
    EXPECT_NE(t.problem.find("spread too little"), std::string::npos);
}
TEST(ExactValidator, AnEdgeOverMoreThanOnePeriodIsRangeTooLong) {
    // an exact cylinder whose top rim's curveEnd is moved a further 2π on: RangeTooLong
}
TEST(ExactValidator, ALoopWiderThanAPeriodIsRangeTooLong) {
    // a cylinder side whose rim pcurves span u over 3π: RangeTooLong
}
```

- [ ] **Step 2: Run them, expect failure.** Command: `hz_math_tests`, `hz_geometry_tests`, `hz_modeling_tests --gtest_filter='ExactValidator*'`.
- [ ] **Step 3: Implement.**
  - Add the fuzz target `hz_fuzz_triangulate`, which reads loops from bytes and asserts the contract: every boundary segment is an edge or `problem` is non-empty, and every triangle has positive orient2d. Register it with `hz_add_fuzz_target`, with five seed inputs.
- [ ] **Step 4: Run everything, then run the full ctest.** Expect PASS.
- [ ] **Step 5: Commit:** `fix(kernel): stage 1's carried guards — huge extents, tiny spreads, ranges over a period; a triangulation fuzzer`.

---

### Task 2: Expansions on the stack

**Files:** Modify `src/math/src/RobustPredicates.cpp`. Test: `tests/math/test_RobustPredicates.cpp`.

**Interfaces:** Unchanged; `orient2d`, `orient3d` and `incircle` keep their signatures and results.

- [ ] **Step 1:** Record the baseline. Time 10⁶ `incircle` calls on a cocircular 200×200 lattice and 10⁶ random ones at `-O2`. Use a probe in the scratchpad, not a test.
- [ ] **Step 2:** Implement. Expansions use fixed-capacity arrays sized to each predicate's maximum length (Shewchuk's bounds), with no heap allocation. Add a test that the 200×200 cocircular lattice's `incircle` signs equal the previous implementation's on 10⁴ sampled quadruples (recorded as a golden vector in the test).
- [ ] **Step 3:** Run `hz_math_tests`, then the full ctest; expect PASS. Re-time the probe and put both timings in the report. Target: the cocircular lattice at least 3× faster.
- [ ] **Step 4: Commit:** `perf(math): exact predicates keep their expansions on the stack`.

---

### Task 3: Meshing without re-triangulating, and graded on cones

**Files:**
- Modify: `src/geometry/include/horizon/geometry/Triangulate2D.h`, `src/geometry/src/Triangulate2D.cpp`, `src/modeling/src/TrimmedFaceMesher.cpp`, `src/modeling/src/MeshEdgeSampler.cpp`
- Test: `tests/geometry/test_Triangulate2D.cpp`, `tests/modeling/test_TrimmedFaceMesher.cpp`

**Interfaces:**
- Produces:

  ```cpp
  namespace hz::geo {
  /// A constrained Delaunay triangulation that takes more interior points after it is built.
  class RegionTriangulation {
  public:
      RegionTriangulation(const std::vector<math::Vec2>& outer,
                          const std::vector<std::vector<math::Vec2>>& holes,
                          const std::vector<math::Vec2>& interior = {}, double merge = 0.0);
      /// Adds a point strictly inside the region; false (and nothing changes) outside it, on a
      /// boundary segment, or within merge of a point. Constraints and the region are kept.
      bool insert(const math::Vec2& point);
      const Triangulation2D& result() const;  // Ruling 10's contract; points appended in order
  };
  Triangulation2D triangulateRegion(...);  // unchanged; builds a RegionTriangulation
  }
  ```

- The mesher's refinement rounds call `insert` instead of re-triangulating.
- The interior grid's spacing along a cone's or torus's u follows the local radius (`r + v·tan α`, or `R + r cos v`) instead of the largest radius.

- [ ] **Step 1: Write the failing tests.**
  - `RegionTriangulation.InsertingKeepsTheContract`: insert 500 seeded points (`hz::test::Uniform`). After each 50 insertions, `checkRegion`'s properties hold (area, constraints, Delaunay except across constraints, positive triangles, inner edges used twice, points used).
  - `TrimmedFaceMesher.AFrustumIsMeshedWithFewTriangles`: a frustum (r 6.5 to 3.2, h 8) at 1e-4 × its diagonal gives at most 4000 triangles (today 9526), stays watertight, stays within tolerance, and keeps the neighbour bound.
  - `TrimmedFaceMesher.ATorusIsMeshedWithFewTriangles`: R10 r3 at 1e-4 × its diagonal gives at most 20000 triangles (today 33480).
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run everything.**
  - Run the full ctest, and the mesher's mutation-killing tests unchanged.
  - Record `-O2` timings for the cylinder r5 h10, the plate, the frustum and the torus R10 r3 at 1e-4 × diagonal. Targets: frustum ≤ 50 ms, torus ≤ 60 ms.
- [ ] **Step 5: Commit:** `perf(modeling): meshes refine in place and grade cones and tori by their radius`.

---

### Task 4: Pcurves from a curve on a surface

**Files:** Create `src/geometry/include/horizon/geometry/Pcurve.h`, `src/geometry/src/Pcurve.cpp`. Test: `tests/geometry/test_Pcurve.cpp`.

**Interfaces:**
- Produces:

  ```cpp
  namespace hz::geo {
  struct Pcurve {
      std::shared_ptr<const Curve2D> curve;
      double start = 0.0;
      double end = 0.0;
      double deviation = 0.0;  ///< largest |S(curve(s)) - C(t)| measured
      std::string problem;     ///< empty when it lies within the tolerance
  };
  /// The curve in @p surface's (u, v) that @p curve follows from t0 to t1, within @p tolerance:
  /// continuous across a seam (u unwrapped), start at t0's point and end at t1's.
  Pcurve pcurveOn(const Surface& surface, const Curve3& curve, double t0, double t1,
                  double tolerance);
  }
  ```

- **Closed forms** (deviation ≤ 1e-12 relative):
  - a line on a plane, as a `Line2D`;
  - a circle on a plane, as a `Circle2D` when the plane's scale is uniform, otherwise a rational `Nurbs2D`;
  - an ellipse on a plane, as a rational `Nurbs2D`;
  - a line along a cylinder's or cone's generator (u constant), as a `Line2D`;
  - a parallel circle on a cylinder, cone, sphere or torus (v constant), as a `Line2D` in u;
  - a meridian on a sphere or torus (u constant), as a `Line2D` in v. This also covers a meridian through a sphere's pole: its u comes from the meridian, not from inverting the pole.
- **Otherwise:**
  - Sample the curve, adaptively by its curvature and the surface's.
  - Invert each sample, starting from the previous one's (u, v) so it stays continuous across seams.
  - Fit a cubic `Nurbs2D` by chord length.
  - Measure and refine until the deviation is at most the tolerance, up to 2048 samples; otherwise set `problem`.
- **A reversed range** (t1 < t0) gives a pcurve running the other way.

- [ ] **Step 1: Write the failing tests.**
  - `Pcurve.EachClosedFormIsExact`: each closed-form pair above, at a tilted frame, with deviation ≤ 1e-12 × size, and endpoints that map onto `C(t0)` and `C(t1)`.
  - `Pcurve.ARimAcrossTheSeamIsContinuous`: a cylinder rim from t = 3π/2 to t = 5π/2 has an unwrapped pcurve with no jump.
  - `Pcurve.ASplineOnASplineIsWithinTolerance`: a B-spline curve lying on a B-spline patch (built as the patch's image of a 2D spline) gives deviation ≤ tolerance.
  - `Pcurve.ACurveOffTheSurfaceSaysSo`: the same curve lifted 10 × tolerance off the patch gives a non-empty `problem` naming the deviation.
  - `Pcurve.AReversedRangeRunsTheOtherWay`.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them, expect PASS.**
- [ ] **Step 5: Commit:** `feat(geometry): pcurves of a curve on a surface, closed form where it has one`.

---

### Task 5: The STEP reader builds exact faces

**Files:**
- Modify: `src/fileio/src/StepFormat.cpp` (`SolidBuilder`: `readFaceGeometry` ~2030, `readEdgeGeometry` ~1885, `buildFace` ~2316, `buildSolids` ~3470)
- Modify: `src/fileio/include/horizon/fileio/ImportReport.h`, if a field is needed
- Create: `tests/fileio/StepTestText.h`. Move the STEP text builders the new tests reuse (`externalStyleCylinder` from test_StepFormat.cpp:470, and `bandCylinder` and `alibreTube` from test_StepCurved.cpp:896 and :1069) here as `inline` functions, and include it from the old files.
- Test: `tests/fileio/test_StepExact.cpp` (new), `tests/fileio/test_StepCurved.cpp`

**Interfaces:**
- Consumes: `pcurveOn` (Task 4), `validateExact` and `exactTolerance` (stage 1).
- Produces: every solid read from STEP has, beside its carriers (`Face::surface`, `Edge::curve`, unchanged):
  - `Face::exactSurface` and `reversed`;
  - `Edge::exactCurve`, `curveStart`, `curveEnd` and `tolerance`;
  - each `HalfEdge::pcurve` with its `pcurveStart` and `pcurveEnd`.
- **Surface kinds:**
  - typed surfaces for PLANE, CYLINDRICAL, CONICAL, SPHERICAL and TOROIDAL;
  - the read `NurbsSurface` itself for B_SPLINE_SURFACE;
  - for SURFACE_OF_LINEAR_EXTRUSION, the carrier NURBS. That kind comes back as a B-spline; this is the one kind exception, documented in the header.
- **Edge ranges:**
  - Edge ranges come from inverting the vertices on the curve, following `same_sense`.
  - A closed edge gets one full period.
- **Tolerances:**
  - The model tolerance is the file's UNCERTAINTY_MEASURE when given, otherwise `forExtent` of the solid.
  - Each edge's tolerance grows to cover its pcurve's measured deviation and its vertices' gaps (tolerant modelling).
  - Where any edge's tolerance grew beyond the model resolution, `ImportReport::converted` says `"solid <i>: exact to <largest edge tolerance> mm, as the file gives its edges"` (spec goal 6: "the report says which").
- **A solid whose exact form `validateExact` rejects is not read.** `ImportReport::skipped` gets `"solid <i>: <issue detail>"`, and nothing faceted takes its place (decision 1b). Today's reasons for skipping stay as they are.

- [ ] **Step 1: Write the failing tests** in `test_StepExact.cpp`. They use STEP text built in the test, like `test_StepCurved`'s helpers, plus `externalStyleCylinder()`.
  - `StepExact.ACylinderComesInAsThreeExactFaces`: kinds Plane, Cylinder, Plane; radius and frame equal the file's; `validateExact` is empty; the read point meshes it.
  - `StepExact.EachSurfaceKindKeepsItsKind`: a cone, a sphere, a torus and a B-spline patch closed into a solid each come in as their own kind. The B-spline is not recognised as a quadric.
  - `StepExact.ASurfaceFacingAgainstItsFaceIsReversed`: same_sense .F. gives `reversed == true`, and outward normals point out.
  - `StepExact.EdgesOffTheirSurfaceWithinTheFileTolerance`: Review Focus 1. Rims shifted 5e-6 off the cylinder, with UNCERTAINTY 1e-5, import with edge tolerance ≥ 5e-6, and the validator is clean. `ImportReport::converted` says "exact to".
  - `StepExact.AFileInInchesIsScaledExactly`: Review Focus 3. Radius ×25.4 and a line's magnitude ×25.4.
  - `StepExact.AVoidShellIsInward`: Review Focus 4. A box with a spherical void has the void's faces outward-from-material.
  - `StepExact.ASolidThatCannotBeMadeExactIsLeftOutWithWhy`: a face whose rim lies 1e-2 off its cylinder, with UNCERTAINTY 1e-6, is skipped, with the reason in `ImportReport::skipped`.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run the tests, then the full ctest.**
  - Run `hz_step_acceptance tests/interop/step <scratch>` and put, for each corpus file, its exact solids against total solids in the report.
  - `test_StepCurved`'s faceted-bridge tests (`AnImportedBodyIsBuiltInFacets` and others) still pass. They test the faceted copy, which this task does not change.
- [ ] **Step 5: Commit:** `feat(fileio): STEP solids come in with exact faces, or are left out with why`.

---

### Task 6: Seams where a periodic face has none

**Files:** Create `src/modeling/include/horizon/modeling/SeamInsertion.h`, `src/modeling/src/SeamInsertion.cpp`. Modify `StepFormat.cpp` to call it before validation. Test: `tests/modeling/test_SeamInsertion.cpp`, `tests/fileio/test_StepExact.cpp`.

**Interfaces:**
- Produces:

  ```cpp
  namespace hz::model {
  /// Gives every face of @p solid whose surface is periodic in u, and whose loops wind round it,
  /// a seam: each winding loop's edges are split where they cross u0 (the outer loop's first
  /// vertex's u), and a seam edge along the surface's u0 iso-curve joins the splits, used once
  /// each way. Exact fields and carriers are both split. Empty on success; else why.
  std::string insertSeams(topo::Solid& solid, const math::ModelTolerance& tolerance);
  }
  ```

- The seam's curve:
  - a `LineCurve` on a cylinder or cone;
  - a `CircleCurve` meridian on a torus. A torus face winding in v as well needs its u seam and its v seam, and gets both.
- A face that already has a seam is untouched.

- [ ] **Step 1: Write the failing tests.**
  - `SeamInsertion.ABandGetsASeamAndStaysExact`: `test_StepCurved`'s `bandCylinder` read and given seams has a clean validator, meshes, and has volume within 1e-9 of πr²h.
  - `SeamInsertion.LoopsWindingTheSameWayAreSeamed`: Solid Edge's same-winding band, from `alibreTube` or the Solid Edge fixture in `test_StepCurved`.
  - `SeamInsertion.ABandWithAHoleIsSeamed`.
  - `SeamInsertion.ASeamedFaceIsUntouched`.
  - Also `StepExact.ABandComesInExact`: Review Focus 2, through the reader.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them; run the full ctest; re-run `hz_step_acceptance` and update the report's count.**
- [ ] **Step 5: Commit:** `feat(modeling): a periodic face with no seam is given one`.

---

### Task 7: Exact mass properties

**Files:**
- Create: `src/modeling/src/ExactMass.h`, `src/modeling/src/ExactMass.cpp`
- Modify: `src/modeling/include/horizon/modeling/MassProperties.h`, `src/modeling/src/MassProperties.cpp`, `src/modeling/src/IdealMassProperties.cpp`
- Test: `tests/modeling/test_ExactMass.cpp`

**Interfaces:**
- Produces:
  - `MassProperties` gains `std::string problem;`, set whenever `valid` is false and a reason is known (carried).
  - `MassPropertiesCalculator::compute(solid)`, for a solid with exact faces, integrates them exactly. It replaces Ruling 22's read-point mesh for mass.
  - `computeIdeal` on such a solid returns the same numbers, with `estimatedError ≤ 1e-9 × volume` and `withinTolerance = true`.
  - Faces without exact geometry are summed by today's polygon formulas, so a mixed solid works.
- The method, which the signature does not determine:
  - Volume, first and second moments come by the divergence theorem. Each is a sum over faces of ∫∫_R g(S) · (S_u × S_v)_k du dv over the face's trimmed region R in its unwrapped parameters (`topo::unwrapLoop`), with the sign from `reversed`.
  - Each such integral reduces by Green's theorem to ∮_∂R G(u, v) dv along the face's pcurves, where G(u, v) = ∫_{u_ref}^{u} g(s, v) ds is taken by Gauss–Legendre.
  - The line integral is taken by adaptive Gauss–Kronrod in each pcurve's parameter, to a relative accuracy of 1e-11 per face.
  - Area is the same integral, with |S_u × S_v| as the integrand.

- [ ] **Step 1: Write the failing tests** (`test_ExactMass.cpp`).
  - `ExactMass.ACylinderIsExact`: `exact::makeCylinder` r5 h10 at a tilted frame. Volume, area, centre and inertia within 1e-9 relative of the closed forms.
  - `ExactMass.APlateWithAHoleIsExact`: volume and area 1e-9 relative.
  - `ExactMass.ImportedSphereTorusAndFrustumAreExact`: STEP text read by Task 5. Volume within 1e-9 of 4/3πr³, 2π²Rr² and πh/3(R² + Rr + r²).
  - `ExactMass.AMirroredCylinderIsPositive`: through `Pattern::transformed` with a reflection.
  - `ExactMass.AVoidSubtracts`: Review Focus 4.
  - `ExactMass.ARationalSplineCylinderIsExact`: a B_SPLINE cylinder read from STEP (rational circle), 1e-9.
  - `ExactMass.AFaceItCannotIntegrateGivesWhy`: an exact face with a pcurve removed gives `valid == false` and a non-empty `problem`.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them, then the full ctest.**
- [ ] **Step 5: Commit:** `feat(modeling): mass properties integrate exact faces exactly`.

---

### Task 8: The STEP writer writes exact faces

**Files:** Modify `src/fileio/src/StepFormat.cpp` (`writeSolid` ~873, `writeSurface` ~258, `writeCurve` ~235). Test: `tests/fileio/test_StepExact.cpp`, `tests/fileio/test_PartFormat.cpp`.

**Interfaces:**
- Produces, for a face with `exactSurface`:
  - PLANE, CYLINDRICAL_SURFACE, CONICAL_SURFACE, SPHERICAL_SURFACE or TOROIDAL_SURFACE (an AXIS2_PLACEMENT_3D from the frame), or B_SPLINE_SURFACE_WITH_KNOTS through today's `writeSurface`;
  - ADVANCED_FACE `same_sense = !reversed`.
- Produces, for an edge with `exactCurve`:
  - LINE (VECTOR magnitude = the line's scale), CIRCLE, ELLIPSE or B_SPLINE_CURVE_WITH_KNOTS;
  - EDGE_CURVE `same_sense` from whether curveStart < curveEnd runs with the edge.
- Faces without exact geometry go through today's paths unchanged. No PCURVE is written.
- `ImportedBodyFeature`'s embedded STEP text, in NativeFormat, therefore keeps it exact.

- [ ] **Step 1: Write the failing tests.**
  - `StepExact.ARoundTripChangesNoSurface`. For each solid of the Task 5 and Task 6 tests, and the exact primitives:
    - `fromString(toString(s))` has the same face count;
    - each face's surface is the same kind, with frame and radii equal within the model resolution;
    - the volume is equal within 1e-9;
    - lines keep their magnitude.
  - `StepExact.TypedSurfacesAreWrittenAsTheirKind`: the written text contains CYLINDRICAL_SURFACE and no B_SPLINE_SURFACE for an exact cylinder.
  - `PartFormat.AnImportedBodyStaysExactThroughThePartFile`: an `ImportedBodyFeature` saved and reopened has an exact solid whose faces keep their kinds.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them, then the full ctest.**
- [ ] **Step 5: Commit:** `feat(fileio): exact faces are written to STEP as their kind`.

---

### Task 9: The part's exact counterpart

**Files:**
- Modify: `src/document/include/horizon/document/FeatureTree.h` (`BuildResult` ~989, `ImportedBodyFeature` ~925), `src/document/src/FeatureTree.cpp` (`ImportedBodyFeature::execute` ~1364, `applyFeature` ~1756, `buildBodies` ~2303, `combine` ~1622)
- Modify: `src/document/include/horizon/document/Document.h` and `Document.cpp`
- Test: `tests/document/test_ExactCounterpart.cpp`

**Interfaces:**
- Produces:
  - `BuildResult` gains `std::unique_ptr<topo::Solid> exactSolid;` and `std::string exactNote;`.
  - `Document` gains `const topo::Solid* exactSolid() const;`, which is null when there is none. `applyBuild` takes it, and `setSolid` clears it.
  - `ImportedBodyFeature` gains `std::unique_ptr<topo::Solid> exactBody() const`: its solid with faces renamed `<featureID>/face:<i>` and edges named by the same pass as the faceted output.
- The build keeps `exactSolid` while every feature so far is an `ImportedBodyFeature` making a new body. Several are joined with `Pattern::append`, which carries exact fields. Any other feature, or a Boolean combine, clears it.
- At the end the build meshes `exactSolid` through `BoundaryMesh::extractFacePolygons(solid, tol, &problem)`. On a problem it clears `exactSolid` and sets `exactNote`: `"shown from facets: <problem>"`. The faceted solid is what the user has, so nothing is lost.
- `execute`'s faceted output, the bridge, is unchanged.

- [ ] **Step 1: Write the failing tests.**
  - `ExactCounterpart.AnImportedBodyHasOne`: the document's `exactSolid()` has 3 faces for a cylinder. Its faces' names equal `logicalFace` of the faceted solid's facets, as a set.
  - `ExactCounterpart.AFeatureAfterItDropsIt`: Review Focus 5. An extrude-cut after the import leaves `exactSolid() == nullptr`, and `solid()` is byte-for-byte the faceted result master gives (compare `faceCount`, volume and names).
  - `ExactCounterpart.TwoImportedBodiesAreBothExact`.
  - `ExactCounterpart.OneThatCannotBeMeshedIsDroppedWithANote`: tamper with a face so the read point reports a problem; `exactSolid() == nullptr` and `exactNote` names it.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them, then the full ctest.**
- [ ] **Step 5: Commit:** `feat(document): an imported part carries its exact counterpart beside its facets`.

---

### Task 10: Rendering, mass and export read the exact counterpart

**Files:**
- Modify the call sites:
  - `src/ui/src/MainWindow.cpp`: ~1572 (tessellate), ~3608, ~3722 and ~3738 (mass), and the status line for `exactNote`;
  - `src/document/src/DocumentManager.cpp` ~252 and ~304;
  - `src/fileio/src/NativeFormat.cpp` ~1019 (cached mesh);
  - `src/ui/src/ExchangeCommands.cpp` ~429 (export);
  - `src/mcp/src/McpSession.cpp` ~468 and ~497;
  - `src/ai/src/CadTools.cpp` ~347 and `CadQueryTools.cpp` ~478 (mass).
- Modify `src/modeling/src/SolidTessellator.cpp`, `BooleanOp.cpp` ~174 and `InterferenceChecker.cpp` ~52.
- Test: `tests/ui/test_ExactDisplay.cpp` (new, in `hz_ui_tests`), `tests/ai/…` (the mass tool), `tests/modeling/test_SolidTessellator.cpp`.

**Interfaces:**
- Consumes: `Document::exactSolid()` (Task 9).
- Produces:
  - Each listed site reads `exactSolid() ? exactSolid() : solid()`.
  - `SolidTessellator::tessellate(const topo::Solid&, double tolerance = 0.1, std::string* problem = nullptr)` emits **one faceTag per face** (carried). It goes through the read point's problem overload, so it never throws.
  - `BooleanOp` and `InterferenceChecker` catch the read point's `std::runtime_error` and fail with its message (carried).

- [ ] **Step 1: Write the failing tests.**
  - `ExactDisplay.AnImportedCylinderIsMeshedFromItsExactFaces`: `DocumentManager`'s mesh for the part has 3 distinct face tags, and its triangles lie on the cylinder within 1e-4 × diagonal.
  - `ExactDisplay.APickOnTheExactMeshNamesTheFacetedFace`: the side's tag resolves, through `logicalFace`, to faces of `solid()`.
  - `ExactDisplay.ExportWritesTheExactCounterpart`: the exported text has CYLINDRICAL_SURFACE.
  - `CadTools.MassOfAnImportedPartIsExact`: the AI mass tool's volume is within 1e-9 of πr²h.
  - `SolidTessellator.OneTagPerFace`.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them; run the full ctest, with window tests on `QT_QPA_PLATFORM=offscreen`.**
- [ ] **Step 5: Commit:** `feat(ui): an imported part is shown, measured and exported from its exact faces`.

---

### Task 11: The corpus and the oracle

**Files:**
- Modify: `tests/interop/StepAcceptance.cpp`/`.h` (summary fields), `tests/interop/step/manifest.json`, `tests/interop/test_StepAcceptance.cpp`
- Modify: `tools/interop/occt_check.py`
- Modify: any test re-baselined, each named in the commit message

**Interfaces:**
- Produces:
  - The summary gains per solid: `exact` (bool), `exactVolume`, and `exactProblem`.
  - The manifest gains `exact_failures` per file. That is the stage's regression list: each entry holds the solid and why. A listed failure that starts passing is an error, as `known_gaps` already works.
  - `occt_check.py` compares our exact volume with OpenCASCADE's within 1e-6 relative, or the manifest's tolerance. It compares surface kinds between the original and our export (`occt_faces`) as counts per kind, with SURFACE_OF_LINEAR_EXTRUSION allowed to become a B-spline.

- [ ] **Step 1:** Run the acceptance tool and the oracle locally if `cadquery-ocp` installs, otherwise in CI on a draft PR. Write every file's exact failures into the manifest, with the reasons the tool prints.
- [ ] **Step 2:** Add the checks. Re-baseline any test that asserted a faceted import result the exact path changed, and name each in the commit.
- [ ] **Step 3:** Run the full ctest and `hz_interop_tests`; expect PASS.
- [ ] **Step 4: Commit:** `test(interop): the corpus checks exact imports against OpenCASCADE; the regression list`.

---

### Task 12: Stage gate

- [ ] **Step 1:** Full suite: `QT_QPA_PLATFORM=offscreen ctest --test-dir build/linux-system-qt -j"$(nproc)"`. Expect "100% tests passed".
- [ ] **Step 2:** Format: `find src tests -name '*.h' -o -name '*.cpp' | xargs ~/.local/bin/clang-format-15 --dry-run --Werror`. Expect no output.
- [ ] **Step 3:** The CI-equivalent clang-tidy gate on every changed `src/*.cpp`. Use a compile database stripped of `-mno-direct-extern-access` and checks `-*,bugprone-*,performance-*`. Only checks that clang-tidy-15 has and that are not in ci.yml's backlog count. Read file lists with `while read`, since the shell is zsh. Expect no gated findings.
- [ ] **Step 4:** Docs.
  - Add the spec's stage 2 "*Landed:*" note, the carried list for stage 3, and the regression list's size.
  - Update the competitive-gaps note's kernel row (stage 2 done).
  - Add the CHANGELOG line.
- [ ] **Step 5:** Final whole-branch review and its fix wave (per the SDD skill). Then rebase on master, push, open "Exact kernel, stage 2: exact exchange" with the regression list in its body, and watch CI to green.

## Decisions this plan makes (the controller's, for the owner to check)

- **The exact counterpart, not exact parts everywhere.** Operations and every reader of ideal labels keep getting today's faceted part: mates, silhouettes, sketch-on-face, fillets and drawings. Only rendering, mass and export read the exact counterpart. That is the spec's bridge, at the least risk to features that work today. It widens stage by stage until the faceted part is deleted in stage 7.
- **The bridge stays `facetCurved` of the carriers** in stage 2, because every imported exact face keeps its carrier. Generalising `facetCurved` to faces with no carrier is stage 3, where exact construction makes such faces.
- **Pcurves are built, never read from the file.** One code path; STEP's pcurve parametrisations of analytic surfaces vary between exporters.
- **SURFACE_OF_LINEAR_EXTRUSION round-trips as a B-spline.** There is no typed extrusion surface yet; this is the one exception to goal 7, documented.
- **Old part files' embedded STEP is NURBS only**, so an imported body saved before stage 2 reopens with exact B-spline faces, not typed ones. That is still exact, and files saved between stages are best effort (decision 2b).
- **Deferred to stage 3:**
  - non-similar transforms converting copies to NURBS, since no stage-2 path transforms an exact solid other than similarly;
  - orientation of mixed exact and faceted solids, since stage 2 makes none;
  - the per-face mesh cache, since the display caches per build already and exact construction is what rebuilds exact faces repeatedly.
