# Parametric primitives (toward task #8)

## Goal

Make box/cylinder/sphere/cone/torus **parametric feature-tree features** rather
than one-shot scene-graph edits, so they persist in the document, rebuild, edit
by parameter, serialize, and are scriptable. This is the verifiable core of the
"wrap toolbar ops as tree features" task.

## Design

- **`PrimitiveFeature`** (`hz::doc`) — a base feature (ignores the input solid,
  like a sketch-create) with `Kind ∈ {Box, Cylinder, Sphere, Cone, Torus}` and
  three positional params. `execute()` delegates to `PrimitiveFactory`.
  `parameters()`/`setParameter()` expose kind-specific editable names
  (`width/height/depth`, `radius`, `bottomRadius/topRadius/height`,
  `majorRadius/minorRadius`), so primitives are parametrically editable.
- **Serialization** — `.hzpart` round-trips `primitiveKind` + `p0/p1/p2` with a
  persisted feature ID (before the single-sketch guard, like the other
  input-independent features).
- **Scripting** — `doc.add_box/add_cylinder/add_sphere/add_cone/add_torus`
  expose primitive creation to Python (a real user-facing consumer).

## Deferred: the toolbar UI wiring

The `MainWindow` toolbar handlers still write straight to the scene graph. They
do so deliberately — to allow **multiple independent bodies** in the view — while
the feature tree is linear/single-solid. Routing the toolbar through
`PrimitiveFeature` as-is would regress that multi-body behaviour (a second
primitive would replace the first on rebuild). Fully wiring the toolbar
therefore depends on **multi-body feature-tree support**, a larger architectural
change tracked separately. The feature type shipped here is the foundation that
work will build on, and is already usable via files and the scripting API.

## Tests

- `FeatureTreeTest`: box builds (6 faces, Euler-valid); all five kinds build
  valid Euler-consistent solids; parametric edit (`setParameter("width", 8)`
  changes the rebuilt extent; wrong-kind parameter rejected).
- `PartFormatTest.PrimitiveFeatureRoundTrip`: a cone round-trips kind + params
  and rebuilds.
- `ScriptEngineTest.ScriptBuildsPrimitive`: Python `doc.add_box(4,4,4)` →
  6 faces, volume 64.
