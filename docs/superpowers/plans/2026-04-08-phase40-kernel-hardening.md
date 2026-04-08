# Phase 40: Stabilization & Kernel Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stress-test and harden the kernel before building more features on top.

**Tech Stack:** C++20, Google Test

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 4.10

---

## Spec Compliance Check

| # | Spec Requirement | Plan Task | Status |
|---|---|---|---|
| 1 | Fuzz testing: random extrude/revolve/Boolean sequences | Task 1 | ✅ |
| 2 | Verify Euler formula and manifold after every operation | Task 1 | ✅ |
| 3 | TNP regression: edit sketch → rebuild → verify features resolve | Task 2 | ✅ |
| 4 | Performance profiling: benchmark Booleans, tessellation | Task 3 | ✅ |
| 5 | Regression suite: 50+ integration tests | Task 1 + Task 2 | ✅ |
| 6 | .hcad v14: serialize B-Rep + feature tree + NURBS + TopologyIDs | Task 4 | ✅ |
| 7 | Full save/load round-trip verification | Task 4 | ✅ |
| 8 | Memory audit: AddressSanitizer on Linux CI | Task 5 | ✅ |

---

## Task 1: Fuzz Testing + Integration Test Suite

Create 50+ integration tests covering the full pipeline:
- Extrude rectangle → verify Euler
- Extrude circle → verify Euler  
- Revolve rectangle → verify Euler
- Boolean union of two boxes → verify
- Boolean subtract → verify
- Fillet one edge → verify
- Chamfer one edge → verify
- Extrude → Boolean → Fillet pipeline
- Random extrude/revolve/Boolean sequences (20+ iterations)
- Degenerate inputs (zero distance, zero radius, empty profile)

## Task 2: TNP Regression Tests

- Create sketch → extrude → assign TopologyIDs
- Modify sketch parameters → rebuild → verify TopologyIDs resolve
- Add fillet referencing specific edge IDs → modify extrude → verify fillet finds edges after rebuild

## Task 3: Performance Benchmarks

- Boolean operation benchmark: time 10 box-box subtracts
- Tessellation benchmark: time tessellation of 100-face solid
- Feature tree rebuild: time 10-feature rebuild

## Task 4: .hcad Serialization of 3D Data

- Serialize FeatureTree to .hcad JSON
- Serialize TopologyIDs alongside B-Rep data
- Round-trip test: create → save → load → verify identical

## Task 5: Memory Audit + CI Enhancement

- Add AddressSanitizer build option to CMake
- Enable ASan in Linux CI for the test run
- Fix any leaks found

## Task 6: Final Era 1 Commit

```bash
git add -A  
git commit -m "Phase 40: Kernel hardening — Era 1 complete

- 50+ integration tests covering extrude/revolve/Boolean/fillet pipeline
- Fuzz testing with random operation sequences
- TNP regression: sketch edit → rebuild → TopologyID resolution
- Performance benchmarks for Boolean and tessellation
- .hcad serialization of feature tree and TopologyIDs
- AddressSanitizer enabled in Linux CI
- Era 1 geometry kernel complete and hardened"

git push origin master
```
