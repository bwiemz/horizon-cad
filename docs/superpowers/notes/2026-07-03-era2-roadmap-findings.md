# Era 2 roadmap findings (Phases 50 & 52)

Notes captured while working through Era 2, to keep the roadmap status honest
and hand off the two items that warrant dedicated, careful efforts rather than a
rushed change.

## Phase 50 — STEP AP242: dependency deferred → ✅ DONE in-house

The roadmap calls for integrating **STEPcode** (BSD) for EXPRESS/Part-21
parsing. STEPcode is **not available as a vcpkg port** (no
`versions/s-/stepcode.json` in the vcpkg registry), so it cannot be added the
way scripting's pybind11/Python was. Bringing it in means a FetchContent/git
submodule build of STEPcode's own CMake project (which also code-generates C++
from EXPRESS schemas) — a heavy, non-binary-cached build that would add
substantial Windows CI time and flakiness.

**Resolution (implemented):** neither STEPcode nor OpenCASCADE. Horizon's
kernel is uniformly NURBS-backed, so AP242 export maps 1:1 onto
`(RATIONAL_)B_SPLINE_CURVE/SURFACE_WITH_KNOTS` with zero loss, and a compact
in-house Part-21 tokenizer + entity resolver covers import. `hz::io::StepFormat`
writes MANIFOLD_SOLID_BREP + product structure and reconstructs `topo::Solid`
(half-edge structure rebuilt from EDGE_LOOP/ORIENTED_EDGE, twins linked with a
strict 2-use manifold check, `Solid::isValid()` gate). External-file interop
covers LINE / CIRCLE edge geometry and PLANE / CYLINDRICAL_SURFACE face
geometry (untrimmed-patch limitation documented in the header). This keeps CI
dependency-free — the same trade Phase 57 made with the in-house FEA solver
instead of TetGen/CalculiX.

Found in passing: Phase-36 Boolean results are not seam-stitched at the A/B
boundary (documented "may not be perfectly watertight"), so their STEP
re-import correctly *fails* the manifold check —
`StepFormat.BooleanSeamDefectIsDetectedOnReimport` pins that behaviour until
SSI face splitting lands.

## Phase 52 — assembly-solver scaling: dense → sparse ✅ DONE

The Phase-42 `AssemblySolver` was correct but O(N³), missing the roadmap's
"100-part assembly, 200+ mates, solve < 1 s" target — a chained 100-part stack
did not finish in tens of seconds (debug). Two O(N³) hot spots in
`AssemblySolver::solve`:

1. **Numerical Jacobian** recomputed *all* residuals for *every* unknown column.
2. **Dense normal-equations solve** `(JᵀJ + λI).ldlt()` each iteration.

**Fix (implemented):** the Jacobian is now assembled per mate — each mate's rows
touch only its two components' 12 columns, differenced in place — into an
`Eigen::SparseMatrix`, and the LM step solves `JᵀJ + λI` with
`Eigen::SimplicialLDLT`. The finite-difference values are identical to the dense
version, so every existing `AssemblySolver`/`AssemblyMates` test still passes
with the same convergence. Measured (debug, unoptimized Eigen):

| N components | before | after |
|---|---|---|
| 40 | 3.33 s | 78 ms |
| 100 | ~50 s | 471 ms |
| 200 | — | 1.33 s |

100 parts / 198 mates now solves in **~0.47 s even in debug** — comfortably under
1 s (release is far faster). The post-solve rank analysis (redundancy + DOF) is
still a dense O(N³) QR — kept dense so its rank threshold matches the
established DOF results — and is now gated behind `setComputeDiagnostics(bool)`
(default on); large-assembly callers turn it off. Covered by
`AssemblySolverTest.LargeAssemblySolvesQuickly` (NDEBUG-guarded < 1 s bound).

## Phase 52 — Boolean robustness & rebuild perf ✅ VALIDATED

Two more Era-2 stabilization criteria are now checked by regression guards:

- **Boolean robustness under coordinate perturbation.** `BooleanOp` was measured
  across all three types (Union / Subtract / Intersect) over a spread of random
  overlapping box configurations — from deep overlap to near-coincident faces.
  Every operation either produced a **valid, Euler-consistent** B-Rep or gave up
  cleanly (returned `nullptr`); none crashed or emitted invalid topology. This is
  the roadmap's "no crashes, no invalid topology" bar. Guarded by
  `BooleanOpTest.RandomOverlapProducesValidTopology`.
  - Along the way, the classifier was found to re-tessellate the whole opposing
    solid per face centroid (12 tessellations for a box-vs-box op); hoisting that
    to 2 (`ExactPredicates::tessellateSolid` + `classifyPointAgainstMesh`) made
    Booleans ~5.5× faster, which also keeps the robustness sweep affordable.

- **50-feature rebuild < 5 s.** A 50-feature part rebuilds in ~3 ms (debug), far
  under the 5 s target — the tree replay is not a bottleneck. Guarded by
  `PerfTest.FeatureTreeRebuild` (now 50 features, single-rebuild < 5 s bound).

**Remaining Phase-52 work:** STEP round-trip (blocked on the Phase 50 dependency
above) and the 100-part < 2 GB memory bound (not yet instrumented) are the last
open Era-2 stabilization items.

## Done in passing

- Scripting API (Phase 47) now exposes Phase-49 mass properties:
  `doc.mass_properties(density)` → `MassProperties{volume, surface_area,
  center_of_mass, mass, density, valid}`.
