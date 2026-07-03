# Era 2 roadmap findings (Phases 50 & 52)

Notes captured while working through Era 2, to keep the roadmap status honest
and hand off the two items that warrant dedicated, careful efforts rather than a
rushed change.

## Phase 50 — STEP AP242: dependency deferred

The roadmap calls for integrating **STEPcode** (BSD) for EXPRESS/Part-21
parsing. STEPcode is **not available as a vcpkg port** (no
`versions/s-/stepcode.json` in the vcpkg registry), so it cannot be added the
way scripting's pybind11/Python was. Bringing it in means a FetchContent/git
submodule build of STEPcode's own CMake project (which also code-generates C++
from EXPRESS schemas) — a heavy, non-binary-cached build that would add
substantial Windows CI time and flakiness.

**Recommendation:** do Phase 50 as a focused effort. Gate it behind
`HZ_ENABLE_STEP` with quiet detection (mirroring `HZ_ENABLE_SCRIPTING`) so the
default build never depends on it, and validate the Windows path deliberately.
Alternatively evaluate OpenCASCADE's STEP reader (which *is* in vcpkg) as the
import/export backend instead of STEPcode.

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

**Remaining Phase-52 work:** Boolean robustness under random coordinate
perturbation (the other half of Era-2 stabilization) is still open.

## Done in passing

- Scripting API (Phase 47) now exposes Phase-49 mass properties:
  `doc.mass_properties(density)` → `MassProperties{volume, surface_area,
  center_of_mass, mass, density, valid}`.
