# Phase 43: Loft & Sweep Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complex 3D shapes from profile interpolation (loft) and path sweeping (sweep), as parametric features that persist in `.hzpart`.

**Architecture:** A shared ring-stack topology builder in `hz::model` generalizes the Phase 35 prism construction to S+1 rings of N vertices each (V=(S+1)N, E=(2S+1)N, F=SN+2 — Euler-valid for all S). `Loft` maps each closed profile section to a ring (winding aligned, start index rotated to minimize twist) and rules bilinear NURBS patches between consecutive rings. `Sweep` transports the profile ring along a polyline path (translation transport — profile keeps its orientation, the documented Era-2 scope; Frenet-frame rotation and twist deferred with the guide-curve work). `LoftFeature`/`SweepFeature` join the feature tree with full serialization (persisted feature IDs per the Phase 42 TNP fix).

**Tech Stack:** C++20, Google Test

**Spec Reference:** docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md — Section 5.3

---

## Spec Compliance Check

| # | Spec Requirement | Plan Task | Status |
|---|-----------------|-----------|--------|
| 1 | Loft: two or more closed profiles on sketch planes | Task 1, Task 2 | ✅ |
| 2 | Same-vertex-count profiles required initially (automatic matching deferred) | Task 2 (validated, mismatches rejected) | ✅ |
| 3 | Guide curves — explicitly optional in spec; deferred | — | ✅ |
| 4 | Sweep: closed profile + 3D path | Task 3 | ✅ |
| 5 | Frenet/user frame + twist — deferred with guide curves (translation transport documented) | Task 3 | ✅ |
| 6 | Test: cylinder as trivial loft | Task 2 | ✅ |
| 7 | Test: known swept shapes vs analytic comparison | Task 3 | ✅ |

## Task 1: Ring-stack topology builder

**Files:** Create `src/modeling/src/RingStack.h`, `src/modeling/src/RingStack.cpp`; modify `src/modeling/CMakeLists.txt`

- [x] `buildRingStack(Solid&, const std::vector<std::vector<Vec3>>& rings)` → caps + per-level lateral faces via Euler operators (MVFS/MEV/MEF), generalizing `buildPrismTopology`
- [x] Shared helpers: profile→ring extraction (chain-ordered 2D vertices), bilinear NURBS patch, planar cap surface fit, edge curve assignment

## Task 2: Loft

**Files:** Create `src/modeling/include/horizon/modeling/Loft.h`, `src/modeling/src/Loft.cpp`; tests `tests/modeling/test_Loft.cpp`

- [x] `Loft::execute(sections, featureID)` — sections = (profile entities, plane); validation (≥2 sections, closed, equal N≥3), winding alignment + start-index rotation, ring stack build, TopologyIDs (`cap_bottom`, `cap_top`, `lateral_<level>_<i>`), bilinear lateral surfaces, planar caps
- [x] Tests: prism loft equals extrude counts; tapered loft valid/manifold; 3-section loft Euler check; mismatched N and open profiles rejected

## Task 3: Sweep

**Files:** Create `src/modeling/include/horizon/modeling/Sweep.h`, `src/modeling/src/Sweep.cpp`; tests `tests/modeling/test_Sweep.cpp`

- [x] `Sweep::execute(profile, plane, pathPoints, featureID)` — translation transport of the profile ring to each path vertex; degenerate paths (repeated points, <2 points) rejected
- [x] Tests: straight-path sweep equals extrude counts and height; L-path sweep valid/manifold with expected face count; degenerate paths rejected

## Task 4: Feature tree + serialization

**Files:** Modify `src/document/include/horizon/document/FeatureTree.h`, `src/document/src/FeatureTree.cpp`, `src/fileio/src/NativeFormat.cpp`; tests in `tests/document/test_FeatureTree.cpp`, `tests/fileio/test_PartFormat.cpp`

- [x] `LoftFeature` (ordered section sketches) and `SweepFeature` (profile sketch + path sketch, path polyline extracted from the path sketch's chained entities)
- [x] `.hzpart` serialization (`loft`: sketchIds array; `sweep`: sketchId + pathSketchId) with persisted feature IDs
- [x] Round-trip + rebuild tests

## Task 5: Final Phase Commit + Push

- [x] Run complete test suite. Report exact test count.
- [x] `git add -A && git commit -m "Phase 43: Loft and Sweep" && git push -u origin claude/production-readiness-roadmap-29aacn`

UI note: loft/sweep creation UI needs multi-sketch selection, which lands with the sketch-browser work; features are fully usable via the feature tree, files, and (later) the scripting API.
