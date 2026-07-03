# Phase 44: Shell & Draft Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Thin-wall (shell) and tapered-wall (draft) features, with a self-intersection guard on shell and clean rejection of unsupported topology.

**Architecture:** Both live in `hz::model`. **Draft** is topology-preserving: it tilts a solid's lateral faces about a neutral plane by moving each lateral vertex along the mitered offset of its two incident lateral-face normals, scaled by the vertex's signed height times `tan(angle)`; caps and connectivity are untouched, planar surfaces are rebound. **Shell** hollows a solid by building an inner "cutter" — the solid's lateral profile offset inward by the wall thickness and extended through the removed cap(s) — then `BooleanOp::subtract`ing it, reusing the Phase 36 engine rather than hand-sewing topology. The self-intersection guard refuses when the wall thickness meets or exceeds the profile inradius (the inner offset would collapse). Both join the feature tree with serialization and persisted feature IDs.

**Tech Stack:** C++20, Google Test

**Spec Reference:** docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md — Section 5.4

---

## Spec Compliance Check

| # | Spec Requirement | Plan Task | Status |
|---|-----------------|-----------|--------|
| 1 | Shell: solid + wall thickness + faces to remove | Task 2 | ✅ |
| 2 | Shell: offset faces inward, remove open faces, connect outer/inner shells | Task 2 (inner-cutter Boolean subtract) | ✅ |
| 3 | Self-intersection detection: refuse when thickness exceeds curvature/inradius, clear error | Task 2 (inradius guard + reason) | ✅ |
| 4 | Draft: tilt selected faces by draft angle relative to pull direction | Task 1 | ✅ |
| 5 | Draft: rebuild adjacent topology | Task 1 (mitered vertex offset keeps lateral faces planar) | ✅ |

Era-2 scope note: shell targets prism-family solids (box / extrude / loft output with planar lateral faces and a removable cap) — the overwhelmingly common shell target. Unsupported configurations return nullptr with a diagnostic reason, matching the roadmap's incremental-hardening approach.

---

## Task 1: Draft

**Files:** Create `src/modeling/include/horizon/modeling/Draft.h`, `src/modeling/src/Draft.cpp`; tests `tests/modeling/test_Draft.cpp`; CMake

- [x] `Draft::execute(solid, pullDir, neutralPoint, angleRad)` — consumes and returns the solid (feature semantics)
- [x] Classify faces as lateral (normal ⟂ pull) vs cap; collect each vertex's incident lateral-face outward normals (Newell + solid-centroid orientation)
- [x] Move each lateral vertex by `δ/(1+n1·n2)·(n1+n2)` with `δ = height·tan(angle)`; rebind planar lateral + cap surfaces
- [x] Tests: box side draft tapers top ring, stays valid/manifold, expected top area; zero angle is a no-op; validity preserved

## Task 2: Shell

**Files:** Create `src/modeling/include/horizon/modeling/Shell.h`, `src/modeling/src/Shell.cpp`; tests `tests/modeling/test_Shell.cpp`; CMake

- [x] `Shell::execute(solid, thickness, removedFaceIds)` → `{solid, ok, message}`
- [x] Detect prism structure (two cap faces ∥ pull axis + planar laterals); inradius guard (`thickness < inradius`)
- [x] Build inner cutter: profile mitered-offset inward by thickness, extended past the removed cap; `BooleanOp::subtract`
- [x] Tests: box top-removed → cup (watertight, Euler, wall thickness by bbox), too-thick rejected, no-removed-face closed hollow, unsupported solid rejected

## Task 3: Feature tree + serialization

**Files:** Modify `FeatureTree.h/.cpp`, `NativeFormat.cpp`; tests in `test_FeatureTree.cpp`, `test_PartFormat.cpp`

- [x] `DraftFeature` (pull dir, neutral point, angle) and `ShellFeature` (thickness, removed face TopologyIDs) — both consume the previous feature's solid
- [x] `.hzpart` serialization with persisted feature IDs; removed faces stored as TopologyID tags
- [x] Build + round-trip tests

## Task 4: Final Phase Commit + Push

- [x] Run complete test suite. Report exact test count.
- [x] `git commit && git push`
