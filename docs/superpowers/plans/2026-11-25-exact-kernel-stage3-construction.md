# Exact Kernel, Stage 3 (Exact Construction) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Primitives, extrude, revolve (with holes) and sheet metal build exact faces. A part made only by them, and by imports, is exact: its volume, its display, its export and its drawings. Old parts migrate to native format 32 on open, saying what changed.

**Architecture:**
- Each converted feature builds an **exact tool** first.
- Its faceted output, which every unconverted operation reads, is the spec's bridge: `facetExact` of that exact tool, at the feature's own segment count and chord tolerance. That reproduces master's facet counts, and the logical names on facets and chords equal the exact faces' and edges' names.
- The build keeps the part's exact solid (stage 2's `ExactCounterpart`) while every body so far came from a converted feature or an exact import, as a new body. A Join, Cut or Intersect, and every unconverted feature, drops it until the stage that converts them.
- The document hands the exact solid to the display, mass, export and, new in this stage, drawings. Everything else keeps reading the faceted solid, as in stage 2.
- Old parts (format 31 or older) are migrated on open:
  - features named the old ways become Stable, and their references are reported;
  - facet and chord names are carried to their logical names;
  - a low segment count is reported as now round.

**Tech Stack:** C++20, CMake (preset `linux-system-qt` locally), GoogleTest, libFuzzer (CI), Python + OpenCASCADE (`cadquery-ocp`, CI only) as the oracle.

**Spec:** `docs/superpowers/specs/2026-11-24-exact-kernel-design.md`. This plan covers stage 3 ("Exact construction"), the items stage 2 carried to stage 3 (the spec's stage 2 entry), and stage 1's stage-3 items. The stage 2 rulings are in `docs/superpowers/notes/2026-11-25-exact-kernel-stage2-rulings.md`.

## Global Constraints

- C++20, in-house, GPL-3.0-or-later; no new third-party geometry library (spec: Constraints).
- Every CI gate stays green on the PR: GCC 11, MSVC and Apple clang (libc++) builds, ASan (`-Werror`), TSan, clang-tidy-15 (`--checks='-*,bugprone-*,performance-*'` with ci.yml's backlog exclusions), clang-format-15 over `src` and `tests`, coverage, fuzzing, and the OpenCASCADE STEP interop job.
- clang 15 rejects a structured binding named inside a lambda. Never capture or use one there; use `.first`/`.second` or named locals.
- Kernel code holds no global mutable state; builds and imports run on workers.
- Tolerances come from `math::ModelTolerance` and `topo::exactTolerance(solid)`, never ad hoc epsilons, except where a step states its own.
- Anti-goals, verbatim from the spec:
  - "Fallbacks that hide failure. An exact operation that fails says why; it never quietly runs the faceted code instead."
  - "Failures nobody tracks." Every model master built that this stage fails on is listed in the PR with why.
  - "Silent changes to saved parts. An old part opened after a change says what changed."
- Owner decision 1b: a converted feature whose exact build fails **fails with its reason**. It never runs master's faceted construction in its place.
- Spec goal 6: volumes, areas and centres of mass to 1e-9 relative for solids bounded by closed-form geometry.
- Existing tests keep passing, or are re-baselined on purpose, each named in its commit and in the PR.
- Native format becomes 32 (Task 11). Formats 1 to 31 still open.
- Never clang-format a `CMakeLists.txt`.
- No local `const` returned by value (CI's `performance-no-automatic-move`).
- No gtest macro as the unbraced body of an `if` (GCC 11's `-Werror=dangling-else`).

## Conventions (stages 1–2's hold; these are added)

- **Converted features:** `PrimitiveFeature`, `ExtrudeFeature`, `RevolveFeature`, `HoleFeature` and `SheetMetalFeature`, under Stable naming. `ImportedBodyFeature` is exact as in stage 2. A feature still named Positional or FromGeometry builds as master does; only code and tests that set those schemes reach that path after Task 11's migration.
- **Exact names are master's logical names.**
  - An exact face's tag equals `model::logicalFace` of the tags master's Stable path gives that face's facets, scoped to the feature, for example `<fid>/side` and `<fid>/side:<entity>`.
  - Exact edges are named by `model::nameEdgesLogically` on the exact solid.
  - Vertices are not named, since nothing refers to one (agent map; see Decisions).
- **The bridge:** `model::facetExact(exact, FacetResolution)`.
  - Its facets are `<face>/facet:<k>`, and a face that stays one facet keeps its tag. Its chords are `<edge>/chord:<k>`.
  - Facets carry `analyticSurface = exactSurface->toNurbs(...)`, one per face. Chords carry `analyticCurve` likewise.
  - Chord counts follow `FacetResolution::stepsFor`, master's `ProfileResolution` rule, so a feature's faceted output has master's face, edge and vertex counts.
- **Constructed exact faces have no NURBS carrier** (`Face::surface` empty), as the spec's topology section intends ("no separate ideal"). Imports keep theirs.
- **Pole convention (stage 1 carry):** no degenerate edges. A loop reaches a pole or apex by a jump in (u, v) between two pcurves, as imported spheres do.
- **Frames:** an exact primitive's seam lies at u = 0 along its frame's x, where master's ring starts, so the bridge's vertices are master's.

## Review Focus

1. **A sketch on an inclined face, or on a rotated sketch plane,** extruded and revolved. The exact faces pass through the bridge's vertices within 1e-9 × size, and the exact volume equals the closed form. *(Task 5)*
2. **A profile with a tangent line–arc join and a 0.01 mm edge on a 100 mm part.** The exact extrude validates without growing any tolerance. *(Task 5)*
3. **A full revolve whose profile has a sloped edge meeting the axis (an apex) and an arc meeting it (a pole),** and the same profile revolved 90° and 359.9°. Each validates, meshes and integrates exactly. *(Task 6)*
4. **An old (format 31) Stable part** with a fillet on `…/chord:3` of an extruded arc's rim, and an assembly mate on a cylinder facet `…/facet:7`. It opens and the fillet builds as on master. The mate resolves, and the report says nothing about either. *(Tasks 11 and 12)*
5. **A primitive and an import as two new bodies.** Editing the primitive re-meshes only its body. An extrude-cut afterwards drops the exact part, and the faceted result equals master's. *(Tasks 4 and 8)*

---

### Task 1: Typed geometry as NURBS; non-similar transforms and mirrors

**Files:**
- Modify: `src/geometry/include/horizon/geometry/surfaces/{Surface.h,AnalyticSurfaces.h,NurbsSurface.h}`, `src/geometry/src/surfaces/*.cpp`
- Modify: `src/geometry/include/horizon/geometry/curves/{Curve3.h,AnalyticCurves.h}`, `src/geometry/src/curves/*.cpp`
- Modify: `src/modeling/src/Pattern.cpp` (`cloneInto` ~121, `transformed` ~624)
- Test: `tests/geometry/test_ToNurbs.cpp` (new), `tests/modeling/test_ExactTransform.cpp` (new)

**Interfaces:**
- Produces:
  - `virtual std::shared_ptr<NurbsSurface> Surface::toNurbs(double u0, double u1, double v0, double v1) const`. It is the exact rational NURBS of the surface over that parameter box, implemented for Plane, Cylinder, Cone, Sphere, Torus and NurbsSurface (a copy restricted by splitting). Its parametrisation may differ from the typed one.
  - `virtual std::shared_ptr<NurbsCurve> Curve3::toNurbs(double t0, double t1) const` for Line, Circle, Ellipse and NurbsCurve.
  - `static NurbsSurface NurbsSurface::makeExtrusion(const NurbsCurve& c, const math::Vec3& offset)`: degree (p, 1), exact.
  - `static NurbsSurface NurbsSurface::makeRevolution(const NurbsCurve& profile, const math::Vec3& axisPoint, const math::Vec3& axisDir, double angle)`: exact and rational, with u round the axis.
  - `Pattern::transformed(const Solid&, const Mat4&, std::string* problem)` (new overload). Under a non-similar transform it converts each face's typed surface and each edge's typed curve to NURBS over their ranges, transforms them, refits every pcurve with `geo::pcurveOn`, and grows tolerances as the reader does. On failure the copy has no exact fields and `problem` says why. The old overload calls it and ignores the problem.

- [ ] **Step 1: Write the failing tests.**
  - `ToNurbs.EachTypedSurfaceIsItsNurbs`: for Plane, Cylinder (r = 3), Cone (half-angle 0.3), Sphere (r = 2) and Torus (R = 5, r = 1), over a box inside one period:
    - 400 NURBS samples each lie on the typed surface within 1e-12 × size;
    - the four corners equal the typed surface's corners within 1e-12 × size.
  - `ToNurbs.EachTypedCurveIsItsNurbs`: the same for Line, Circle and Ellipse, with the ends at `t0` and `t1`.
  - `NurbsSurface.AnExtrusionIsExact`: a rational quarter circle extruded by (0, 0, 4) has every sample at radius 1 within 1e-12.
  - `NurbsSurface.ARevolutionIsExact`: a quarter circle about an axis through its centre gives every sample at distance r within 1e-12.
  - `ExactTransform.ANonUniformScaleKeepsTheSolidExact`: `exact::makeCylinder` scaled (2, 1, 1) has `hasExactFaces` true and an empty `validateExact`. Its `ExactMass` volume is 2πr²h within 1e-9.
  - `ExactTransform.AMirroredConeIsValid`: an exact frustum and an apex cone (read from `step_text::cone()` / `apexCone()`), each mirrored by `scale(-1, 1, 1)`, have an empty `validateExact`. Their volume equals the original's within 1e-9.
- [ ] **Step 2: Run them, expect failure.** Command: `hz_geometry_tests --gtest_filter='ToNurbs*:NurbsSurface.A*'`, `hz_modeling_tests --gtest_filter='ExactTransform*'`.
- [ ] **Step 3: Probe the mirrored cone first.** Print `unwrapLoop`'s result per half-edge for the mirrored cone, and put the cause in the report. Then implement. The fix goes where the cause is, for example `Cone::normal` in a left-handed frame or the apex jump's side, not in the test.
- [ ] **Step 4: Run them, then the full ctest.** Expect PASS.
- [ ] **Step 5: Commit:** `feat(geometry): typed surfaces and curves convert to NURBS; a non-similar copy stays exact; a mirrored cone validates`.

---

### Task 2: The bridge from exact faces

**Files:**
- Create: `src/modeling/include/horizon/modeling/FacetResolution.h`
- Modify: `src/modeling/src/RingStack.h` and `.cpp` (`ProfileResolution` becomes an alias of `FacetResolution`, with one `stepsFor`)
- Modify: `src/modeling/include/horizon/modeling/Faceting.h`, `src/modeling/src/Faceting.cpp`, `src/modeling/src/Pattern.cpp` (`bodyShells` ~539)
- Test: `tests/modeling/test_FacetExact.cpp` (new)

**Interfaces:**
- Consumes: `toNurbs` (Task 1).
- Produces:
  - `struct FacetResolution { int segmentsPerTurn = 32; double chordTolerance = 0.0; int stepsFor(double radius, double sweep) const; };`. This is master's rule, moved.
  - `FacetedSolid facetExact(const topo::Solid& exact, const FacetResolution& resolution)`.
    - Edges are chorded on their exact curves: a circle or arc by `stepsFor(radius, sweep)` at uniform angle, a line as one chord, and an ellipse or NURBS by turning angle 2π/segmentsPerTurn with sag at most `chordTolerance` when that is positive.
    - Curved faces are gridded uniformly in their own parameters; a periodic face's columns take the largest count of its edges.
    - Planar faces stay one face with their inner loops.
    - Names and labels follow the Conventions. Caller-chosen naming of edges follows, as `ImportedBodyFeature::execute` does.
  - `facetCurved` (carriers) is unchanged for faces with a carrier.
  - A curved face with neither carrier nor exact surface fails both functions with `"face <tag> has no surface to facet"`, never the corner polygon it gives today.
  - `Pattern::bodyShells` measures a shell with no carriers by its exact signed volume (`ExactMass`).

- [ ] **Step 1: Write the failing tests.**
  - `FacetExact.ACylinderAtThirtyTwoIsMastersPrism`: `exact::makeCylinder` (r = 5, h = 10) at {32, 0} gives:
    - face, edge and vertex counts equal to `PrimitiveFactory::makeCylinder(5, 10, 32)`'s (34, 96, 64);
    - the vertex set equal within 1e-12;
    - the volume equal within 1e-12 relative.
  - `FacetExact.FacetsNameTheirExactFace`:
    - the set of `logicalFace(facet tag)` equals the exact solid's face tags;
    - after `nameEdgesLogically` on both, the set of `logicalEdge(chord tag)` equals the exact edge tags.
  - `FacetExact.AChordToleranceGivesEachArcItsOwnCount`: on `exact::makePlateWithHole` holes of r = 2 and r = 20 at chordTolerance 0.01, each rim's chord count is `stepsFor(r, 2π)`.
  - `FacetExact.ACurvedFaceWithNoSurfaceIsRefused`: a cylinder whose side lost its `exactSurface` gives an `error` naming the face. `facetCurved` gives the same.
  - `FacetExact.AnExactBodyWithAVoidIsMeasured`: an exact block with an exact reversed block inside, joined by `Pattern::append`, gives `bodyShells` = one body with one void, and the volume difference is exact within 1e-9.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.** Every existing `facetCurved` caller and the corpus tests must be unchanged. Run `hz_interop_tests` and confirm the manifest's faceted volumes did not move.
- [ ] **Step 4: Run them, then the full ctest.** Expect PASS.
- [ ] **Step 5: Commit:** `feat(modeling): the bridge facets exact faces at a feature's own resolution; a face with no surface is refused, not outlined`.

---

### Task 3: Exact primitives, poles and apexes

**Files:**
- Modify: `src/modeling/include/horizon/modeling/ExactPrimitives.h`, `src/modeling/src/ExactPrimitives.cpp`
- Modify as the cause requires: `src/modeling/src/TrimmedFaceMesher.cpp`, `MeshEdgeSampler.cpp`, `src/topology/src/LoopUnwrap.cpp` (`signedArea` ~134, a pole jump closed by its arc, not a chord), `src/topology/src/ExactValidator.cpp` (`sameUv` ~395), `src/modeling/src/ExactMass.cpp`
- Test: `tests/modeling/test_ExactPrimitives.cpp`, `test_TrimmedFaceMesher.cpp`, `test_ExactMass.cpp`

**Interfaces:**
- Produces (namespace `hz::model::exact`; every face, edge, pcurve and tolerance set; faces named as master's primitives' logical names before scoping, for example `cylinder/side`):
  - `makeBox(const geo::Frame&, double w, double h, double d)`
  - `makeSphere(const geo::Frame&, double r)`: one face, one seam from pole to pole.
  - `makeCone(const geo::Frame&, double r0, double r1, double h)`: an apex when a radius is 0; a `Cylinder` side when the radii are equal, as master tags it.
  - `makeTorus(const geo::Frame&, double R, double r)`: a `Torus` when R > r; otherwise (a horn or spindle) a NURBS surface of revolution (`makeRevolution`), so stage 6's `Torus::invert` limit is never reached.
  - `makeSphereSector(const geo::Frame&, double r, double angle)`: the lune that a partial revolve of a semicircle makes, with two half-disc caps. It exists so the pole work has a builder to test; the revolve of Task 6 makes the same shape.
- Mesher, validator and mass accept a face bounded by two meridians meeting at two poles (the spec's regression class "a ball split into two faces by two meridians"), and a cone sector with an apex.

- [ ] **Step 1: Write the failing tests.**
  - `ExactPrimitives.EachIsValidAndExact`, for box 2×3×4, sphere r = 3, frustum (2, 1, 4), apex cone (2, 0, 4), torus (5, 1) and spindle (1, 2):
    - `validateExact` is empty and the read point meshes without a problem;
    - the face counts are 6, 1, 3, 2, 1 and 1;
    - the `ExactMass` volume is within 1e-9 relative of the closed form: wdh, 4/3πr³, πh/3(r0² + r0r1 + r1²), πr²h/3 and 2π²Rr²; the spindle's is by Pappus over its circular segment.
  - `ExactPrimitives.NamesAreMastersLogicalNames`: each primitive's face tags equal `logicalFace` of `nameFacetsLogically(PrimitiveFactory::make…(…))`'s tags, as a set. This uses the faceted factory, which stays.
  - `ExactPrimitives.ASphereSectorIsValidAndExact`: angle π/2, r = 3. It has 3 faces, volume πr³/3 within 1e-9, and meshes with no problem. Repeat at angle 0.01 and 2π − 0.01.
  - `ExactPrimitives.EveryEdgeIsNamed`: after `nameEdgesLogically` the edge tags are unique and non-empty.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.** The lune's failure is found by probing first, with what fails and why in the report. Pole jumps close by the arc round the pole in `signedArea`, and `sameUv` measures arc, not chord, where the surface comes to a point.
- [ ] **Step 4: Run them, then the full ctest and `hz_interop_tests`.** The corpus's `exact_failures` may only shrink. A file that now reads exact is moved out of the list in this commit.
- [ ] **Step 5: Commit:** `feat(modeling): exact primitives, and faces that meet at poles between meridians`.

---

### Task 4: The build carries an exact part; primitives build exactly

**Files:**
- Modify: `src/document/include/horizon/document/FeatureTree.h` (`Feature` ~69, `PrimitiveFeature` ~856, `ImportedBodyFeature` ~926), `src/document/src/FeatureTree.cpp` (`applyFeature` ~1776, `PrimitiveFeature::execute` ~1237, `buildWithDiagnostics` ~2368)
- Modify: `src/document/src/ExactCounterpart.h` and `.cpp` (`follow` ~366, `make` ~384, `madeFrom` ~189)
- Test: `tests/document/test_ExactConstruction.cpp` (new, in `hz_document_tests`), `tests/document/test_ExactCounterpart.cpp`

**Interfaces:**
- Consumes: `facetExact` and `FacetResolution` (Task 2), and the `exact::` primitives (Task 3).
- Produces:
  - `struct FeatureOutput { std::unique_ptr<topo::Solid> faceted; std::unique_ptr<topo::Solid> exact; };`
  - `virtual FeatureOutput Feature::executeWithExact(const BuildContext&, std::unique_ptr<topo::Solid> input, std::string* reason) const`. The default returns `{executeIn(...), nullptr}`. The tree calls this in place of `executeIn`.
  - `virtual model::FacetResolution Feature::facetResolution() const`. It is master's faceting rule for that feature:
    - a primitive returns `{segments(), 0}`, one count from its governing radius, as `PrimitiveFeature` computes it today;
    - an extrude or revolve returns `{segments, chordTolerance}`, as its `ProfileResolution` was;
    - sheet metal is in Task 7.
  - A converted feature's `executeWithExact`:
    - builds the exact tool, scoped to its feature ID;
    - if that fails, fails with `"could not be built exactly: <why>"` and leaves `faceted` null;
    - otherwise sets `faceted = facetExact(*exact, facetResolution())` with `nameEdgesLogically`. Its `executeIn` returns that `.faceted`.
  - `ImportedBodyFeature::executeWithExact` returns `{execute(...), exactBody()}`, which keeps stage 2's faceted copy.
  - `ExactFollower::follow(const Feature&, bool first, const topo::Solid* exactTool)` keeps the part's exact solid.
    - It joins each exact tool with `Pattern::append` while the feature is the first body or a `NewBody`.
    - Any other operation, or a feature with no exact tool, drops it.
    - `madeFrom` becomes the exact part's `fingerprint`.
  - `PrimitiveFeature` (Stable) builds through `exact::` in its placement frame. Its other schemes build as master does.

- [ ] **Step 1: Write the failing tests.**
  - `ExactConstruction.ACylinderPrimitiveIsThreeExactFaces`: r = 5, h = 10 gives:
    - `exactSolid()->faceCount() == 3`;
    - the mass tool's volume π·25·10 within 1e-9;
    - `solid()->faceCount() == 34`;
    - `logicalFace` of `solid()`'s tags equal to the exact tags.
  - `ExactConstruction.TwoNewBodiesAreBothExact` (two primitives) and `ExactConstruction.AnImportAndAPrimitiveAreBothExact`.
  - `ExactConstruction.ACutAfterItDropsTheExactPart` (Review Focus 5): `exactSolid() == nullptr`. `solid()`'s face count, volume (within 1e-12 relative) and tag set equal master's for the same document. Record master's values in the test before changing the code.
  - `ExactConstruction.AFailedExactBuildFailsTheFeature`: a torus whose tube radius is 0 makes the feature fail. Its reason starts "could not be built exactly", and there is no faceted solid.
  - `ExactConstruction.APositionalPrimitiveBuildsAsMasterDid`: names `cylinder/side<i>`, and no exact part.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them; run the full ctest with window tests on `QT_QPA_PLATFORM=offscreen`.** Re-baseline any test that asserted no exact counterpart, or a faceted mass, for a part made of primitives. Name each in the commit.
- [ ] **Step 5: Commit:** `feat(document): a part made of primitives is exact; the faceted part is its bridge copy`.

---

### Task 5: Exact extrude

**Files:**
- Create: `src/modeling/src/ExactProfile.h` and `.cpp`. A validated profile region becomes oriented 2D loops of `Line2D`/`Circle2D`/arcs, each with its entity's source name, and their 3D curves on the sketch plane.
- Modify: `src/modeling/include/horizon/modeling/Extrude.h`, `src/modeling/src/Extrude.cpp`, `src/document/src/FeatureTree.cpp` (`ExtrudeFeature::executeIn` ~228), `src/ui/src/PartCommands.cpp` (probe extrusions ~397)
- Test: `tests/modeling/test_ExactExtrude.cpp` (new), `tests/document/test_ExactConstruction.cpp`

**Interfaces:**
- Consumes: Tasks 1, 2 and 4.
- Produces:
  - `static std::unique_ptr<topo::Solid> Extrude::exact(const std::vector<std::shared_ptr<draft::DraftEntity>>& profile, const draft::SketchPlane& plane, const math::Vec3& direction, double distance, const std::string& featureID, std::string* reason)`.
    - It makes one face per profile edge: a line gives a `Plane`; an arc or circle gives a `Cylinder` along the sketch normal and a NURBS `makeExtrusion` along any other direction.
    - Caps are planar, with a hole's loops as inner loops.
    - Each region, and an island inside a hole, is its own shell.
    - No `BooleanOp`.
    - Names: master's Stable logical names (`side:<entity>` and master's cap names).
  - Under Stable naming, `Extrude::execute(...)` returns `facetExact(exact(...), {segments, chordTolerance})` with `nameEdgesLogically`. Other schemes build as master does.
  - `ExtrudeFeature` builds through `executeWithExact`. Blind, Symmetric, ThroughAll, ThroughAllBoth and UpToFace keep computing their plane and distance as today.

- [ ] **Step 1: Record master's Stable names and counts** for the test profiles in the tests, as literals, before any code changes: face, edge and vertex counts and the `logicalFace` tag set.
- [ ] **Step 2: Write the failing tests.**
  - `ExactExtrude.ARectangleIsABlock`: 6 faces, volume exact.
  - `ExactExtrude.ASlotIsExact`: two lines of 20 and two arcs of r = 5, height 3. It has 6 faces and volume (20·10 + 25π)·3 within 1e-9.
  - `ExactExtrude.APlateWithTwoHolesHasInnerLoops`: each cap has 2 inner loops and the volume is exact. The faceted copy's counts equal master's recorded ones.
  - `ExactExtrude.AnObliqueArcIsANurbsExtrusion`: volume = area × distance × (direction·normal) within 1e-9.
  - `ExactExtrude.AnIslandInAHoleIsItsOwnShell`.
  - `ExactExtrude.NamesAreMastersLogicalNames`: tags equal the recorded set.
  - `ExactExtrude.OnAnInclinedPlane` (Review Focus 1): a sketch plane tilted 30° about x and 20° about z. The bridge's vertices lie on the exact faces within 1e-9 × size, and the volume is exact.
  - `ExactExtrude.ATangentJoinAndATinyEdge` (Review Focus 2): a 100 mm profile with a line tangent to an arc and a 0.01 mm edge. `validateExact` is empty, and no edge tolerance is above `ModelTolerance` resolution.
  - `ExactConstruction.SymmetricAndUpToFaceExtrudesAreExact`.
- [ ] **Step 3: Run them, expect failure.**
- [ ] **Step 4: Implement.**
- [ ] **Step 5: Run them; run the full ctest.** Re-baseline only what the bridge changed on purpose: a plate with holes is no longer a Boolean of cutters, so `/piece:` names on its hole walls go. Name each test in the commit.
- [ ] **Step 6: Commit:** `feat(modeling): extrusions are exact — planes, cylinders and NURBS extrusions; holes as inner loops, not Booleans`.

---

### Task 6: Exact revolve and holes

**Files:**
- Modify: `src/modeling/include/horizon/modeling/Revolve.h`, `src/modeling/src/Revolve.cpp`, `src/modeling/src/ExactProfile.*`
- Modify: `src/document/src/FeatureTree.cpp` (`RevolveFeature::execute` ~388, `HoleFeature::executeIn` ~2180, `holeName` ~2161), `src/ui/src/PartCommands.cpp` (probe revolves ~483)
- Test: `tests/modeling/test_ExactRevolve.cpp` (new), `tests/document/test_ExactConstruction.cpp`

**Interfaces:**
- Produces:
  - `static std::unique_ptr<topo::Solid> Revolve::exact(profile, plane, axisPoint, axisDirection, double angle, const std::string& featureID, std::string* reason)`. Per profile edge:
    - a line parallel to the axis gives a `Cylinder`;
    - a line perpendicular to it gives a `Plane` (a disc or ring);
    - a line on the axis gives no face;
    - any other line gives a `Cone`, with an apex where it meets the axis;
    - an arc centred on the axis gives a `Sphere`;
    - an arc off the axis gives a `Torus` when R > r, and otherwise `makeRevolution`.
  - A partial turn adds planar `cap_start` and `cap_end`.
  - A full turn's flat band is one disc or ring face, as master's Stable path makes it.
  - Holes in the profile become inner loops.
  - Names are master's Stable logical names.
  - Under Stable naming, `Revolve::execute` returns the bridge copy, as Extrude does.
  - `HoleFeature` builds its tool through `Revolve::exact`, with `holeName` applied to the exact tags (`top`, `wall`, `bottom`, `bore`, `boreFloor`, `sink`). It stays a Cut, so the part after it is faceted, as on master.

- [ ] **Step 1: Record master's Stable names and counts** for the test profiles, as in Task 5.
- [ ] **Step 2: Write the failing tests** (volumes within 1e-9 relative):
  - `ExactRevolve.ARectangleAgainstTheAxisIsACylinderWithDiscs`: 3 faces, πr²h.
  - `ExactRevolve.ATriangleAgainstTheAxisIsAnApexCone`: πr²h/3.
  - `ExactRevolve.ASemicircleIsASphere`: 1 face, 4/3πr³.
  - `ExactRevolve.AnOffAxisCircleIsATorus`: R = 5, r = 1, 2π²Rr².
  - `ExactRevolve.ASpindleArcIsANurbsRevolution`: by Pappus.
  - `ExactRevolve.AQuarterTurnHasCaps`: the rectangle gives 5 faces and πr²h/4.
  - `ExactRevolve.APoleAndAnApexAtAnyAngle` (Review Focus 3): a profile with a sloped edge and an arc both meeting the axis, at 2π, π/2 and 359.9°. Each validates, meshes and integrates exactly.
  - `ExactRevolve.NamesAreMastersLogicalNames`.
  - `ExactConstruction.EachHoleToolIsExact`: Simple, Counterbore and Countersink tools are valid, with exact volumes. The drilled part's faceted solid equals master's recorded counts and volume within 1e-9.
- [ ] **Step 3: Run them, expect failure.**
- [ ] **Step 4: Implement.**
- [ ] **Step 5: Run them; run the full ctest.** Re-baseline only on purpose, naming each test.
- [ ] **Step 6: Commit:** `feat(modeling): revolves and hole tools are exact — cones to apexes, spheres to poles, tori`.

---

### Task 7: Sheet metal bends are exact

**Files:**
- Modify: `src/modeling/include/horizon/modeling/SheetMetalSolid.h`, `src/modeling/src/SheetMetalSolid.cpp`, `src/document/src/SheetMetalFeature.cpp` (~211)
- Test: `tests/document/test_SheetMetalFeature.cpp`, `tests/modeling/test_SheetMetalSolid.cpp`

**Interfaces:**
- Produces:
  - `static std::vector<std::shared_ptr<draft::DraftEntity>> SheetMetalSolid::profile(const SheetMetalStrip&, const SheetMetalParams&)`: lines, with each bend as an inner and an outer `DraftArc`, in the sketch's 2D.
  - `SheetMetalFeature` builds through `Extrude::exact` with `facetResolution() = {4 × segmentsPerBend, 0}`, so a 90° bend keeps master's 8 chords.
  - `crossSection` and the flat pattern are unchanged.

- [ ] **Step 1: Write the failing tests.**
  - `SheetMetalFeature.ABendIsTwoCylinders`: one 90° bend, inner radius 2, thickness 1, width 10.
    - The exact part has `Cylinder` faces of r = 2 and r = 3.
    - Its volume = (flat lengths × 1 + (π/4)(3² − 2²)) × 10 within 1e-9.
    - The faceted copy's bend has 8 chords per arc.
  - `SheetMetalFeature.ABendOtherThanNinetyDegrees` (60°): the faceted copy has `stepsFor` chords. Re-baseline master's 8, named.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them; run the full ctest.**
- [ ] **Step 5: Commit:** `feat(sheetmetal): bends are exact cylinders`.

---

### Task 8: Each body's mesh is kept

**Files:**
- Modify: `src/document/src/ExactCounterpart.h` and `.cpp`
- Test: `tests/document/test_ExactCounterpart.cpp`

**Interfaces:**
- Produces:
  - `ExactCounterpart` gains `std::vector<BodyMesh> bodies`, where `struct BodyMesh { std::uint64_t fingerprint; std::shared_ptr<const geo::MeshData> mesh; std::string note; };`, one per exact tool joined.
  - `make()` reuses each body whose fingerprint equals one in `previous`. The part's mesh is the bodies' meshes concatenated, with face tags as they are.
  - `std::size_t bodiesMeshed` records how many bodies the last `make()` meshed.
  - The cross-check (`holdToFacets`) and the inward check run per body, and their notes per body join as today's.

- [ ] **Step 1: Write the failing tests.**
  - `ExactCounterpart.EditingOneBodyMeshesOnlyIt` (Review Focus 5): an import plus a primitive as new bodies. Change the primitive's radius and rebuild with `previous`:
    - `bodiesMeshed == 1`;
    - the import's `mesh` pointer is the previous one.
  - `ExactCounterpart.AnUnchangedRebuildMeshesNothing`: `bodiesMeshed == 0`.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them, then the full ctest.**
- [ ] **Step 5: Commit:** `perf(document): an exact part keeps each body's mesh across rebuilds`.

---

### Task 9: Drawings from the exact part

**Files:**
- Modify: `src/ui/src/DrawingWorkbench.cpp` (`partSolid` ~274), `src/fileio/src/DrawingDocumentIO.cpp` (`load` ~640, `build` ~168 callers), and the section view callers (`DrawingDocumentIO.cpp` ~761, `DrawingWorkbench.cpp` ~1663)
- Test: `tests/fileio/test_DrawingExact.cpp` (new)

**Interfaces:**
- Consumes: `Document::exactSolid()`, `facetExact`.
- Produces:
  - `model::Drawing` sources come from `exactSolid()` when there is one, through `facetExact(*exact, {32, tol})` with `nameEdgesLogically`. `tol = 0.01 / s` mm, where `s` is the largest view scale on the sheet (paper ÷ model), so no chord sags more than 0.01 mm on paper.
  - Otherwise they come from `solid()`, as master does.
  - Dimensions, callouts, GD&T and datums resolve against that copy through `logicalEdge`, as today.

- [ ] **Step 1: Write the failing tests.**
  - `DrawingExact.ACylindersSilhouetteIsSmooth`: an r = 50 cylinder primitive part on a sheet at scale 1.
    - The front view's silhouette edges lie within 0.01 of x = ±50; master's lie 0.12 inside.
    - The top view's rim points are within 0.01 of r = 50.
  - `DrawingExact.ATwoToOneViewHalvesTheTolerance`.
  - `DrawingExact.AFacetedPartIsDrawnAsBefore`: a part with a cut has edge counts and kinds equal to master's recorded ones.
  - `DrawingExact.ARadiusDimensionIsExact`: a radius dimension on the rim reads 50.000.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.** Time `DrawingProjection::project` for the cylinder part's four views and put it in the report. If it exceeds 1 s in Debug, report it before going on.
- [ ] **Step 4: Run them, then the full ctest.**
- [ ] **Step 5: Commit:** `feat(drawings): views of an exact part are drawn from its exact faces, smooth to 0.01 mm on paper`.

---

### Task 10: The exact part reaches every caller

**Files:**
- Modify:
  - `src/scripting/src/ScriptContext.cpp` (~38, ~139);
  - `src/document/include/horizon/document/AssemblyDocument.h` (`ComponentInterference` ~134) and `src/document/src/AssemblyDocument.cpp` (~226);
  - `src/ui/src/AssemblyWorkbench.cpp` (~632);
  - `src/document/src/ExactCounterpart.*` and `src/document/include/horizon/document/Document.h`;
  - `src/ui/src/MainWindow.cpp` (`updateExactNote` ~1616, mass dialog ~3646 and ~3798);
  - `src/fileio/include/horizon/fileio/{StlExport.h,GltfExport.h}` and their `.cpp`;
  - `src/ui/src/ExchangeCommands.cpp` (~393, ~505);
  - `src/fileio/src/AssemblyStepExport.cpp` (~53);
  - `src/document/src/DocumentManager.cpp` (~150, ~275).
- Test: `tests/scripting/…` (mass), `tests/document/test_AssemblyDocument.cpp`, `tests/ui/test_ExactDisplay.cpp`, `tests/fileio/test_AssemblyStepExport.cpp`

**Interfaces:**
- Produces:
  - Python `mass_properties` and `analysisRefusal` read `exactOrFacetedSolid()`.
  - `ComponentInterference` gains `std::string problem`, copied from `InterferencePair::problem`. The workbench shows it in place of "overlap could not be measured".
  - The exact note becomes `enum class ExactNoteKind { None, ShownFromFacets, NotChecked, Disagrees }` plus a detail string. `Document::exactNoteKind()` is added. `MainWindow` shows the kind through `tr()`, with the detail as the tooltip, and nothing user-facing is built from English in document code.
  - The mass dialog integrates an exact part on a `BackgroundTask` under the same rule as the faceted path: `RebuildMode::Always`, or `Auto` with `kWorkerIdealFaces` or more curved faces.
  - `StlExport::save(path, const geo::MeshData&)` and `GltfExport::saveMesh(path, const geo::MeshData&)`. `ExchangeCommands` passes `exactMesh()` when it was made at the export's tolerance.
  - `gatherAssemblyStep` writes each component's `exactOrFacetedSolid()`. Components to be exported are resolved with the counterpart (`BuildOptions{}`), not `facetedOnly()`.

- [ ] **Step 1: Write the failing tests.**
  - `Scripting.MassOfAPrimitivePartIsExact`: π·25·10 within 1e-9.
  - `AssemblyDocument.AnInterferenceSaysWhyItWasNotMeasured`.
  - `ExactDisplay.TheNoteIsTranslatable`: the label's text comes from `tr()` for each kind.
  - `ExactDisplay.ExactMassRunsOnAWorkerWhenAsked`: under `RebuildMode::Always` the dialog's result arrives through the task.
  - `ExactDisplay.StlExportReusesTheKeptMesh`: the tessellation counter does not move.
  - `AssemblyStepExport.TwoPrimitivePartsExportExact`: the text has CYLINDRICAL_SURFACE.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them; run the full ctest with scripting built.** Build scripting with the pybind11 venv, as CI does.
- [ ] **Step 5: Commit:** `feat: scripts, interference, exports and the mass dialog read the exact part; its note is translated`.

---

### Task 11: Format 32 — old parts migrate on open

**Files:**
- Create: `src/document/include/horizon/document/Migration.h`, `src/document/src/Migration.cpp`
- Modify: `src/fileio/src/NativeFormat.cpp` (`kFormatVersion` ~106 to 32, `loadDocumentChecked` ~2119 passes the file's version to `loadDocumentRoot` ~1267, tessellation cache ~1061)
- Modify: `src/document/include/horizon/document/FeatureTree.h`. Each feature lists and rewrites its stored references through `virtual std::vector<StoredReference> references() const` and `virtual void setReference(std::size_t, std::string)`.
- Test: `tests/fileio/test_Migration.cpp` (new), `tests/fileio/test_PartFormat.cpp`, `tests/fileio/test_ImportExport.cpp`

**Interfaces:**
- Produces:
  - `struct StoredReference { enum class Kind { Face, Edge } kind; std::string name; std::string what; };`. `what` is for the report, for example "the face sketch 'Sketch 2' is on".
  - It covers sketch faces, extrude and hole up-to faces, hole faces, mirror plane faces, shell removed faces, fillet and chamfer edges, and projected source edges.
  - `io::ImportReport migrateToExact(Document&, int fileVersion)` runs for `fileVersion <= 31` after load:
    1. **Old naming.** Each feature named Positional or FromGeometry becomes Stable. Each reference that names a face or edge such a feature made (judged by the name's feature prefix and that feature's old scheme) is removed and reported in `skipped`: `"<feature>: its reference to <name> is not carried over (named the old way)"`.
    2. **Facet names.** Each other reference with `/facet:` or `/chord:` is rewritten through `logicalFace`/`logicalEdge`, with no entry.
    3. **Low segment counts.** Each converted feature whose effective segments per turn are under 12 is reported in `approximated`: `"<feature>: drawn as a <n>-sided prism before; round now"`.
    4. **Stale caches.** The tessellation cache of a format 31 or older file is dropped.
  - Files are written as version 32.
  - `loadDocumentChecked` merges the migration's report into the load's, so `skipped` entries make the part read in part. The existing rule then refuses to save over it without asking.

- [ ] **Step 1: Write the failing tests.**
  - `Migration.AStablePartCarriesItsFacetNames` (Review Focus 4, part side): a format 31 part with a fillet on `…/chord:3` of an extruded arc's rim opens with an empty report. The fillet builds, and its result equals master's recorded volume within 1e-9.
  - `Migration.APositionalPartsReferencesAreReported`: a format 18 part with a Positional extrude and a fillet on `edge7`. The report's `skipped` names the reference, the document is read in part, and the extrude is now Stable.
  - `Migration.ASixSidedCylinderIsNowRound`: `approximated` holds "drawn as a 6-sided prism before; round now", and the exact volume is πr²h.
  - `Migration.SavedPartsAreVersion32`, and `Migration.AStaleCacheIsDropped`.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them; run the full ctest.** Update fixtures that hard-code the version on purpose, naming them.
- [ ] **Step 5: Commit:** `feat(fileio): format 32 — an old part is migrated on open and says what changed`.

---

### Task 12: References that map to nothing are reported at open

**Files:**
- Modify: `src/ui/src/MainWindow.cpp` (`showOpened` ~2307, after the first build of a migrated part), `src/document/src/DocumentManager.cpp` (~272, assembly components), `src/fileio/src/DrawingDocumentIO.cpp` (`build` ~780, `lost` into `leftOut`), `src/ui/src/DrawingWorkbench.cpp` (~437)
- Test: `tests/ui/test_ImportExport.cpp`, `tests/document/test_AssemblyDocument.cpp`, `tests/fileio/test_DrawingDocumentIO.cpp`

**Interfaces:**
- Produces:
  - A migrated part whose first build fails puts the failure into `leftOut` as `"after the exact rebuild: <message>"` and is shown in the "Not Everything Was Read" report.
  - An assembly of format 31 or older checks its mates after its components resolve. Each mate that `AssemblyMates::gather` cannot resolve goes into its `leftOut`.
  - A sheet's `lost` references go into `DrawingDocumentSpec::leftOut` (and so the report), not only a status message.

- [ ] **Step 1: Write the failing tests.**
  - `ImportExport.AMigratedPartThatNoLongerBuildsIsReported`.
  - `AssemblyDocument.AnOldMateThatResolvesToNothingIsReported`.
  - `AssemblyDocument.AnOldMateOnAFacetResolves` (Review Focus 4, assembly side): a mate on `…/facet:7` of a cylinder gives an empty report and a resolved mate.
  - `DrawingDocumentIO.ALostDimensionIsReported`.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them; run the full ctest with window tests on offscreen.**
- [ ] **Step 5: Commit:** `feat: what an old part, assembly or sheet can no longer find is reported when it opens`.

---

### Task 13: Measures that do not miss a peak

**Files:**
- Modify: `src/geometry/src/Pcurve.cpp` (`measureAlong` ~823, `standOff` ~1236), `src/topology/src/ExactValidator.cpp` (`measurePcurve` ~110, `pcurveGaps` ~550), `src/modeling/src/ExactMass.cpp` (budget ~77, floor ~1416)
- Test: `tests/geometry/test_Pcurve.cpp`, `tests/modeling/test_ExactValidator.cpp`, `tests/modeling/test_ExactMass.cpp`

**Interfaces:**
- Produces:
  - **One measuring function** for a curve's gap from its pcurve, shared by `pcurveOn` and the validator (the stage 2 lesson): sampled as today, then refined by a golden-section search between each sample and its neighbours where the gap rises, so that a peak between samples is found to 1% of its height.
  - **Mass budget:** `ExactMass`'s per-face evaluation budget scales with the face's knot spans (4·10⁶ for up to 64 spans, and proportionally beyond).
  - **Accuracy floor:** an estimate that cannot reach the request at Gauss–Kronrod's floor says so in the result's note (`"its error estimate stops at <e>"`), instead of reading as within the request.

- [ ] **Step 1: Write the failing tests.**
  - `Pcurve.APeakBetweenSamplesIsFound`: the tilted band rim of Ruling 23 (tilt 8e-4). `standOff` is within 1% of the true peak, 3.7e-7 above the old reading.
  - `ExactValidator.APeakBetweenSamplesStrays`: the same rim, with its tolerance set to the old reading, reports a stray.
  - `ExactMass.ADenseSplineFaceSettles`: a 150 × 150 knot B-spline face integrates without "did not settle".
  - `ExactMass.AThinPartSaysWhereItsEstimateStops`.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them; run the full ctest and `hz_interop_tests`.** The corpus may change only by tolerances growing to the peaks now found. Each moved "exact to" goes in the report. Move a newly failing solid onto `exact_failures` with its reason, and name it in the commit.
- [ ] **Step 5: Commit:** `fix(kernel): gaps are measured at their peaks, not only at samples; dense faces get their mass budget`.

---

### Task 14: Seams for bands that wind both ways, or alike

**Files:**
- Modify: `src/modeling/src/SeamInsertion.cpp` (`insertSeams` ~1032, `seamAlong` ~990)
- Test: `tests/modeling/test_SeamInsertion.cpp`, `tests/interop/step/manifest.json`

**Interfaces:**
- Produces:
  - A torus band whose loops wind round it both ways gets both seams (u and v) and validates exact. That is Ruling 29's class.
  - A closed B-spline band whose two loops wind the same way: probe the corpus's Onshape and Solid Edge old-part bodies first, and put the cause in the report. If the cause is the loops' sense, that sense is corrected and the band seamed. Otherwise it stays refused, with a reason that names the cause found.

- [ ] **Step 1: Probe and write the failing tests.**
  - `SeamInsertion.ATorusBandWindingBothWaysGetsTwoSeams`: a helix-cut torus band validates, meshes, and integrates to its closed form within 1e-9.
  - One test per cause the probe finds.
- [ ] **Step 2: Run them, expect failure.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run them; run the full ctest and `hz_interop_tests`.** Bodies that now read exact leave the manifest's `exact_failures` and `test_OldParts`' kept-as-it-was list in this commit.
- [ ] **Step 5: Commit:** `feat(modeling): seams for bands that wind round both ways, or alike`.

---

### Task 15: Old parts open within budget

**Files:**
- Modify: `src/geometry/src/Pcurve.cpp` (`generalPcurve` ~1067), `src/fileio/src/StepFormat.cpp` (pcurves ~4636, validation ~4729 and ~4753), `src/topology/src/ExactValidator.cpp`
- Test: the timing harness stage 2 used (rebuilt in the scratchpad, not a test); `tests/interop/test_OldParts.cpp` for behaviour

**Interfaces:**
- Produces:
  - Pcurves on B-spline faces are fitted from the edge's own samples, projected once with each projection's warm start taken from its neighbour's.
  - The validator's second pass (Ruling 66) re-checks only the edges and vertices the top-up grew, never the whole solid.
  - Behaviour is unchanged: `test_OldParts` and the manifest read alike.

- [ ] **Step 1: Baseline.** Rebuild stage 2's harness: master's reader at `90a758f7` and this branch, built alike (Debug, same flags), on the same texts, back to back, best of 3, twice. Record per-part open time for the 18 old exports and the 64 old-part bodies.
- [ ] **Step 2: Implement.**
- [ ] **Step 3: Measure again.** Target: every old export reads within 2× master (spec budget 2), and the largest old part opens in under 0.5 s in Debug. Put the table in the report. A part still over target is recorded as a ruling with its cost, not hidden.
- [ ] **Step 4: Run the full ctest and `hz_interop_tests`.** Expect PASS, with the manifest unchanged.
- [ ] **Step 5: Commit:** `perf(fileio): old parts open faster — pcurves on B-spline faces and one validation`.

---

### Task 16: The regression set, re-baselines and the stage's list

**Files:**
- Modify: `tests/interop/step/manifest.json` (as needed), and any test re-baselined (each named)
- Modify: `docs/superpowers/specs/2026-11-24-exact-kernel-design.md` (stage 3 regression list, draft)

- [ ] **Step 1: Run the regression set** on this branch and on master, and compare:
  - every test;
  - the samples (`src/app/samples/MakeSamples.cpp`: build each and record exact or faceted, volume and face count);
  - the STEP corpus;
  - `test_AdversarialModels`;
  - the beta tasks in `docs/BETA.md` that build parts.
- [ ] **Step 2: List every model master built that now fails,** with the reason the exact builder gave. This goes into the spec's stage 3 regression list, per the anti-goal "failures nobody tracks". Fix the ones with a clear cause in this task. A fix outside this task's files becomes a ledger ruling.
- [ ] **Step 3: Budgets** (spec: Performance budgets; Release, same flags, back to back, best of 3). Rebuild of the bracket sample must be no slower than master at 32 facets per turn. Record the number. If it is slower, profile it and fix it, or record a ruling with its cost.
- [ ] **Step 4: Run the full ctest.** Expect "100% tests passed".
- [ ] **Step 5: Commit:** `test: stage 3's regression set — what master built and stage 3 refuses, with why`.

---

### Task 17: Stage gate

- [ ] **Step 1: Full suite.** Run `QT_QPA_PLATFORM=offscreen ctest --test-dir build/linux-system-qt -j"$(nproc)"`. Expect "100% tests passed"; `known-red.md` is empty.
- [ ] **Step 2: Format.** Run `find src tests -name '*.h' -o -name '*.cpp' | xargs ~/.local/bin/clang-format-15 --dry-run --Werror`. Expect no output.
- [ ] **Step 3: Tidy.** Run the CI-equivalent clang-tidy gate on every changed `src/*.cpp`.
  - Use a compile database stripped of `-mno-direct-extern-access`, with checks `-*,bugprone-*,performance-*`.
  - Only checks clang-tidy-15 has, and that are not in ci.yml's backlog, count.
  - Read the file list with `while read` (the shell is zsh).
  - Expect no gated findings.
- [ ] **Step 4: Docs.**
  - The spec's stage 3 *Landed* note, its regression list and the carried list for stages 4–7.
  - The competitive-gaps note's kernel row.
  - The CHANGELOG line.
  - The README's format version, if it names one.
- [ ] **Step 5: Final review and PR.** Final whole-branch review and its fix wave (per the SDD skill). Then rebase on master, push, open "Exact kernel, stage 3: exact construction" with the regression list and the owner questions in its body, and watch CI to green. Open a draft PR after Task 4 lands, so CI's compilers see the branch early (the stage 2 lesson).

---

## Decisions this plan makes (the controller's, for the owner to check)

- **Two solids still.** The part's faceted solid is the bridge copy of its exact part.
  - Booleans, fillets, shell, draft, patterns, mirror, mates, sketch on a face, picks and the AI's face lists keep reading facets until their stages.
  - Because the bridge uses each feature's own segment count, master's facet counts and logical names hold, so their results do not move.
- **A part is exact only while every body is.** There are no mixed exact and faceted solids this stage, so the "mixed solids" carry goes to stage 5, whose Booleans make the first ones.
- **Each body's mesh is kept, not each face's.** The mesher samples an edge from both faces' plans, so a face's mesh depends on its neighbours; bodies are independent. This closes the per-face cache carry in a form that holds.
- **Vertices are not named.** Nothing stores a vertex reference (the stage-1 carry asked for it). One is named the day something refers to one.
- **Horn and spindle tori are NURBS surfaces of revolution,** in primitives and revolves. A typed `Torus` is made only when R > r, so nothing new reaches `Torus::invert`'s limit, which is stage 6.
- **Drawings are drawn through the bridge at 0.01 mm on paper.** Projected edges stay straight segments, fine enough at the sheet's scale. True 2D arcs and ellipses in views, and in DXF exports of sheets, are carried to a later stage.
- **Old naming schemes are converted at open,** and their references are reported, never guessed (spec: Migration). The legacy builders stay only for code and tests that set those schemes, and stage 7 deletes them.
- **Low segment counts are reported as approximated, which does not block saving.** A lost reference is reported as left out, which does block it.
- **A fillet's "fewer than 4 arc segments" report waits for stage 6.** Fillets are not exact yet, so their segments still shape them.
- **An imported body whose exact form fails keeps Ruling 10's behaviour after migration:** it opens from its facets, with a note. It is not a converted feature's failure, and its facets are its bridge.
- **One PR,** as stages 1 and 2 were. Migration must land with exact construction, or an old six-sided "cylinder" would turn round silently.
- **`segments` and `chordTolerance` no longer shape the exact part, but they still shape its bridge copy** until Booleans are exact (stage 5), so that features after them keep master's results. The display meshes the exact part at the display tolerance. A per-feature coarse display (spec: Migration) waits until the bridge's last user goes.
- **Pole cases construction never makes stay on the regression list:** a hole touching a pole, a zero-length edge along one, and an edge a hair past an apex. Only imports make them. A recognised sphere's frame z is moot, because construction sets every frame and imports are never recognised (goal 7).
- **Kept from stage 2 unless the owner says otherwise:** the display's 0.35 rad angle floor, and Ruling 67's acceptance (Task 15 now works it down).
- **Carried, unchanged:** the platform-dependent facet grid in `trimmedFacets` (stage 7 deletes the bridge), the validator's O(n²) checks (stage 5), and the spindle torus's `Torus::invert` (stage 6).
