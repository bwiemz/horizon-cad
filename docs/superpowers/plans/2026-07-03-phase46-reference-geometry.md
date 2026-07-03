# Phase 46 — Reference Geometry (Datum Planes, Axes, Points)

## Goal

Construction geometry for complex modeling: datum planes, axes, and points that
live in the feature tree as **non-geometric** features and serve as inputs to
sketches and downstream features. They do not alter the solid body.

## Design

### Data types (`hz::model`, `ReferenceGeometry.h`)

Plain value structs — resolved coordinates, not live references (Era-2 scope):

- `DatumPlane { origin, normal, xAxis }` with `yAxis() = normal × xAxis` and
  `toSketchPlane()` → `draft::SketchPlane` so a `Sketch` can be built on it.
- `DatumAxis  { origin, direction }` — an infinite line.
- `DatumPoint { position }`.

### Construction methods (`hz::model::refgeo`)

Fallible constructions return `std::optional`.

**Planes**
- `planeOffset(base, d)` — parallel plane shifted `d` along the normal.
- `planeThroughPoints(p0,p1,p2)` — origin `p0`, xAxis→`p1`, normal `(p1-p0)×(p2-p0)`;
  `nullopt` if collinear.
- `planeAtAngle(base, hingeOrigin, hingeDir, angle)` — rotate `base` about a hinge
  line by `angle` (the datum contains the hinge).
- `planeMidplane(a, b)` — midplane between two (near-parallel) planes; averages
  origins and orientation-aligned normals.

**Axes**
- `axisThroughPoints(p0,p1)` — `nullopt` if coincident.
- `axisPlaneIntersection(a,b)` — line `n_a × n_b`; `nullopt` if parallel.
- `axisFromDirection(base, dir)` — cylinder axis captured directly.

**Points**
- `pointAt(pos)` — at a vertex.
- `pointCentroid(points)` — face-center; `nullopt` if empty.
- `pointLineIntersection(a,b)` — midpoint of the shortest segment between two
  lines (edge intersection); `nullopt` if parallel.

### Feature-tree integration

- `Feature::isConstruction()` (new virtual, default `false`).
- `DatumFeature` (kind Plane/Axis/Point) stores three `Vec3` slots + kind;
  `isConstruction()` returns `true`; `execute()` passes the input solid through
  unchanged. Factories `makePlane/makeAxis/makePoint`; accessors return the
  resolved `DatumPlane/DatumAxis/DatumPoint`.
- `FeatureTree::build()` / `buildWithDiagnostics()` **skip** construction features
  when producing the solid, so a leading datum does not make the build fail and
  rollback indices still line up with feature positions.

### Serialization (`.hzpart`)

`type:"datum"`, `datumKind:"plane|axis|point"`, `origin/dirA/dirB` arrays, plus
the persisted `featureID`. Round-trips before the single-sketch guard.

## Tests

- `test_ReferenceGeometry`: each construction method — offset, 3-point (+collinear
  reject), angle, midplane; axis through points / plane-intersection / parallel
  reject; centroid, line intersection; `toSketchPlane` frame orthonormality.
- Feature-tree: datum passthrough (leading datum + extrude still builds the solid;
  datum count in tree).
- `.hzpart` round-trip of a datum plane feature.
