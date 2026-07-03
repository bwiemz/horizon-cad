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

## Phase 52 — assembly-solver scaling: dense → sparse needed

The Phase-42 `AssemblySolver` is correct but does not meet the roadmap's
"100-part assembly, 200+ mates, solve < 1 s" target. Measured solve time on a
chained stack (2 coincident mates per adjacent pair, debug build):

| N components | time |
|---|---|
| 10 | 54 ms |
| 20 | 399 ms |
| 30 | 1.47 s |
| 40 | 3.33 s |

Growth is ~N³, so N=100 is ~50 s (debug); even a ~10× release speedup leaves it
well above 1 s. Two O(N³) hot spots in `AssemblySolver::solve`:

1. **Numerical Jacobian** (forward differences) recomputes *all* residuals for
   *every* unknown column, though perturbing component k's DOF only affects
   mates that touch component k.
2. **Dense normal-equations solve** (`(JᵀJ + λI).ldlt()`), an O(unknowns³) dense
   factorization each iteration.

**Recommendation (bounded but not a rush job):** exploit the block sparsity —
build the Jacobian per-mate touching only its two components' 12 columns, and
solve with an Eigen sparse factorization (`SimplicialLDLT` on the sparse
`JᵀJ`, or a sparse QR on `J`). This is localized to the solve loop but needs
careful re-testing of all assembly-solver cases, so it deserves its own change.

## Done in passing

- Scripting API (Phase 47) now exposes Phase-49 mass properties:
  `doc.mass_properties(density)` → `MassProperties{volume, surface_area,
  center_of_mass, mass, density, valid}`.
