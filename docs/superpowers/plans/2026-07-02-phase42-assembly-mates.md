# Phase 42: Assembly Mates Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Position parts relative to each other via geometric mate constraints, solved by a 6-DOF-per-component Newton-Raphson solver with kinematic graph pre-analysis.

**Architecture:** `hz::doc` gains `Mate` (type + two `MateReference`s carrying component id + face `TopologyID` + optional value) stored on `AssemblyDocument` and serialized in `.hzasm`. `hz::model` gains `MateGeometry` (extracts a mate frame — origin, direction, radius — from a resolved part's B-Rep face via TopologyID, transformed by the instance placement) and `AssemblySolver`: each free component contributes 6 unknowns (translation + rotation vector); mates contribute residual equations (coincident plane 3, concentric axis 4, distance 3, angle 1, parallel 2, perpendicular 1, tangent 2; Fixed grounds a component). Newton-Raphson with Levenberg-Marquardt damping and a numerically differentiated Jacobian, mirroring `hz::cstr::SketchSolver`. Pre-analysis walks the mate graph: components with no path to a grounded component are reported under-constrained; Jacobian rank (column-pivoted QR) reports redundant mates and remaining DOF.

**Tech Stack:** C++20, Eigen, Qt6, Google Test

**Spec Reference:** docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md — Section 5.2

---

## Spec Compliance Check

| # | Spec Requirement | Plan Task | Status |
|---|-----------------|-----------|--------|
| 1 | Mate types: Coincident, Concentric, Distance, Angle, Parallel, Perpendicular, Tangent, Fixed | Task 1, Task 3 | ✅ |
| 2 | Assembly solver extends Newton-Raphson architecture, 6 DOF per part | Task 3 | ✅ |
| 3 | Kinematic graph pre-analysis before numerical solve | Task 3 | ✅ |
| 4 | Rigid sub-group / grounding reduction feeds a reduced system | Task 3 (grounded components eliminated from unknowns) | ✅ |
| 5 | Detect and report redundant constraints | Task 3 (Jacobian rank analysis) | ✅ |
| 6 | DOF visualization (draggable free parts) — solver reports per-component DOF; interactive dragging deferred with viewport 3D manipulation work | Task 3, Task 4 | ✅ |
| 7 | Mate references use TopologyIDs, resolved via genealogy on rebuild | Task 1, Task 2 | ✅ |

---

## Task 1: Mate model + serialization

**Files:** Modify `src/document/include/horizon/document/AssemblyDocument.h`, `src/document/src/AssemblyDocument.cpp`, `src/fileio/src/NativeFormat.cpp`; tests `tests/document/test_AssemblyDocument.cpp`, `tests/fileio/test_AssemblyFormat.cpp`

- [x] `MateType` enum (8 types), `MateReference { componentId, faceId (TopologyID) }`, `Mate { id, type, a, b, value }`
- [x] `AssemblyDocument::addMate/removeMate/mate(id)/mates()` with unique-id bookkeeping and dirty tracking
- [x] `.hzasm` v16 gains a `mates` array (type string, refs, value); round-trip tests
- [x] Commit: `feat(document): assembly mate model with .hzasm serialization`

## Task 2: Mate geometry extraction

**Files:** Create `src/modeling/include/horizon/modeling/MateGeometry.h`, `src/modeling/src/MateGeometry.cpp`; tests `tests/modeling/test_MateGeometry.cpp`

- [x] `MateFrame { origin, direction, radius, kind (Planar|Cylindrical) }`
- [x] Extract from a `topo::Solid` face found by TopologyID: planar faces → centroid + surface normal; cylindrical faces → axis point + axis direction + radius (sampled ring fit)
- [x] `transformed(Mat4)` to place the frame by the component transform
- [x] Commit: `feat(modeling): mate frame extraction from B-Rep faces`

## Task 3: Assembly solver with kinematic pre-analysis

**Files:** Create `src/modeling/include/horizon/modeling/AssemblySolver.h`, `src/modeling/src/AssemblySolver.cpp`; tests `tests/modeling/test_AssemblySolver.cpp`

- [x] Component state: translation + rotation-vector increment applied to a base `Mat4`; Fixed mates ground components (removed from unknowns)
- [x] Residual equations per mate type (see Architecture); numerical Jacobian; Newton-Raphson + LM damping; convergence to `Tolerance`-scale residuals
- [x] Pre-analysis: mate graph connectivity (ungrounded islands reported), redundancy via rank-revealing QR, per-component remaining DOF estimate
- [x] `AssemblySolveResult { status, iterations, residualNorm, redundantCount, componentDOF }`
- [x] Tests: plane-plane coincident snaps a translated box back; concentric aligns cylinder axes; distance/angle/parallel/perpendicular/tangent each verified against analytic expectation; redundant duplicate mate detected; ungrounded island reported; 3-part chain solves < 1s
- [x] Commit: `feat(modeling): 6-DOF assembly mate solver with kinematic pre-analysis`

## Task 4: Assembly UI integration

**Files:** Modify `src/ui/include/horizon/ui/MainWindow.h`, `src/ui/src/MainWindow.cpp`

- [x] "Add Mate..." action (assembly tabs): dialog picks two components + face tags + mate type + value; appends mate, resolves components, solves, applies transforms, rebuilds scene; solver status (including redundancy warnings) in the status bar
- [x] Mates solve on assembly load so saved assemblies come back positioned
- [x] Commit: `feat(ui): assembly mate creation and solve-on-load`

## Task 5: Integration test + Final Phase Commit + Push

**Files:** `tests/integration/test_MultiDocument.cpp` (extend or add `test_AssemblyMates.cpp`)

- [x] End-to-end: author part → assembly with two instances + coincident/distance mates → save → reload → solve → verify placements analytically
- [x] Run complete test suite. Report exact test count.
- [x] `git add -A && git commit -m "Phase 42: Assembly mates with kinematic pre-analysis" && git push -u origin claude/production-readiness-roadmap-29aacn`
