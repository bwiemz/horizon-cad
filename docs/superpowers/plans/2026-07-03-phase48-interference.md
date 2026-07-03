# Phase 48 — Collision Detection & Interference Checking

## Goal

Detect overlapping parts in an assembly.

## Design (`hz::model::InterferenceChecker`)

- **Input:** world-space solids (`const Solid*`); the assembly resolver positions
  components, so callers pass already-placed geometry.
- **Broad phase:** an `math::RTree` of solid AABBs → O(n log n) candidate pairs,
  deduped with a `set<pair>`.
- **Narrow phase (`solidsInterfere`):**
  - AABB quick-reject.
  - Tessellate each solid at a **size-relative** tolerance (`diag · 0.2`); the
    tessellator otherwise emits a dense uniform grid (~120k triangles for a flat
    box), which would make an all-pairs test quadratic and hang.
  - Build a per-solid **triangle R\*-tree** so the pair test is
    O(edges · log tris), not O(tris²).
  - Report interference when an edge of one mesh crosses a face of the other
    (segment/triangle, Möller–Trumbore), **or** one solid is contained in the
    other (point-in-solid by ray parity, using a generic non-axis-aligned ray).
- **Output:** `InterferencePair{indexA, indexB, overlapBounds}` where
  `overlapBounds` is the AABB overlap region — a localization box for
  visualization.

### Deviation from the roadmap (improvement)

The roadmap called for a Boolean intersection ("nonzero volume = interference").
The Phase-36 Boolean is not yet robust enough to gate a correctness result, so
the narrow phase uses a robust mesh-overlap test instead. The precise
interference *volume* solid (and the translucent-red viz + conflict panel) is a
follow-up, best done after Boolean hardening (Phase 52) and mass properties
(Phase 49).

## Tests

`test_InterferenceChecker` (9): overlapping / separated / perpendicular-lifted /
interlocking (edge-face crossing with no contained vertex) / contained (both
directions) pairs; multi-body `check` reporting only real pairs with the correct
overlap box; empty, single, and null-solid inputs; solid-bounds sanity.
