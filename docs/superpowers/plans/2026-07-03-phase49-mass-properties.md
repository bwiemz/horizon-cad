# Phase 49 — Measurement & Mass Properties

## Goal

Engineering measurements and material analysis.

## Design (`hz::model`)

### Mass properties (`MassPropertiesCalculator`, `MassProperties`, `Material`)

- **Integration source: the B-Rep, not the render tessellation.** The render
  tessellator winds curved-face triangles inconsistently *and* overlaps them
  (a cylinder's lateral faces tessellate to ~3× their true area), so it is
  unusable for volume/inertia integration. Instead the boundary is triangulated
  directly from each face's outer loop (fan from its first vertex) — exact for
  planar-faced solids and free of overlaps.
- **Orientation:** face loops aren't guaranteed outward-wound, so each triangle
  is oriented away from an interior reference point (the vertex average, inside
  any convex/star-shaped solid); the global inward/outward sign is normalized to
  a positive volume.
- **Quantities (Eberly's polyhedral-mass-properties integrals):** volume,
  surface area, volume-weighted centroid, and the inertia tensor about the
  center of mass. `Material` presets (steel, aluminum, titanium, ABS) supply a
  density; `mass = ρ·V` and the inertia tensor scales by ρ. With no material,
  ρ = 1 (unit-density / geometric tensor).

### Measurements (`measure::`)

`distance` (point-to-point), `angleBetween` (directions), `pointToSegment`, and
`segmentToSegment` (Ericson closest-approach). UI face/edge measurement resolves
the picked geometry to points/segments and calls these.

### Known limitations (follow-ups)

- Curved faces are approximated by their B-Rep loop polygon, so a cylinder's
  volume is that of its inscribed prism (centroid still exact by symmetry).
  Smooth-surface accuracy needs per-face NURBS integration.
- Inner loops (holes) are not yet subtracted; non-convex orientation relies on
  star-convexity w.r.t. the vertex average.

## Tests

- `test_MassProperties` (6): box volume/area/centroid (exact), translation
  invariance, box inertia tensor vs the m(h²+d²)/12 closed form, material mass +
  inertia scaling, preset densities, and a curved primitive (symmetry-exact
  centroid, volume bounded by the ideal round cylinder).
- `test_Measure` (6): point-to-point, angle (incl. magnitude independence and
  degenerate input), point-to-segment (interior/endpoint/degenerate), and
  segment-to-segment (crossing / skew / parallel).
