# Milestone 16 — Units and design intent (Phases 154–157)

Roadmap: [2026-09-25-professional-workflows-roadmap.md](../specs/2026-09-25-professional-workflows-roadmap.md).

## Baseline (checked in the code)

- **The model is in millimetres and radians, always.** That stays: a unit
  is how lengths are shown and typed, never how they are kept.
- **One display unit, an application preference.**
  - It is `Preferences::lengthUnit`, a `QString` ("mm", "cm", "m", "in",
    "ft"), stored in QSettings, with `decimals`.
  - Three places read it:
    - the cursor readout (`MainWindow::onMouseMoved`);
    - Measure Distance;
    - Measure Area.
  - Measure Angle prints degrees to two places, whatever the setting.
  - Mass Properties is hard-coded to mm, mm², mm³, cm³ and g.
- **Four tables list the five units:**
  - `Preferences.cpp`;
  - `DimensionStyle.cpp` (`millimetresPerUnit`);
  - the Dimension Style form's list in `MainWindow`;
  - DXF's `$INSUNITS` map, which is a table of codes and stays.
- **No file stores a unit**, except a dimension style's own.
  - NativeFormat is at version 19.
  - It reads older files by checking for each key and taking a default.
  - It bumps the version only when an older build would misread new
    content. An older build that ignores a unit misreads nothing, since
    the model is in millimetres.
- **Nothing typed accepts a unit.**
  - `FeatureForm::number` is a plain `QDoubleSpinBox`. Angle fields are
    `number` fields labelled "(degrees)", converted by each caller.
  - `TypedPoint::key` refuses letters and spaces.
  - `TypedLength` takes a bare number only.
  - The property panel's spin boxes, the array and block dialogs, and the
    constraint `QInputDialog`s take plain numbers.
- **`math::Expression`** (+ − × ÷ ^, functions, variables; bounded) exists.
  It has no units, and no form uses it. `ExpressionEngine` and
  `ParameterRegistry` hold a document's design variables, which are saved,
  but nothing lets a user make one (Phase 155).

## Phase 154: Document units

A unit per document, saved in the file. Every length field shows it and
takes it, as do the readouts and typing in the view. A typed value may
carry its own unit ("2 in", "1' 6\"", "30 deg", "0.5 rad"). Angles are
shown in degrees.

### 154a: the units, and the document's (as built)
The model is unchanged, in millimetres. What changed:
- **`math::Units`** (`Units.h`) is the one table.
  - `formatLength` is locale-free.
  - `parseLength` and `parseAngle` follow the grammar below.
  - Preferences, DimensionStyle and the Dimension Style form use it. DXF's
    `$INSUNITS` codes keep their own map.
- **`Document::lengthUnit` and `AssemblyDocument::lengthUnit`**:
  - Saved as `"units": {"length": "in"}` in NativeFormat's document and
    assembly roots, and in `.hzdwg`.
  - Read back by any version; millimetres when absent or not known.
  - `DocumentManager::setNewDocumentUnit` gives new documents the
    preference, "Unit for new documents".
  - An assembly's backing document mirrors its unit, so the view's tools
    read it there.
- **Edit ▸ Document Units** pushes `SetLengthUnitCommand` (the document, and
  its assembly if it backs one) as one undo step.
- **Readouts in the document's unit:**
  - the cursor readout;
  - Measure Distance and Area;
  - Measure Angle, with the decimals set;
  - Mass Properties: volume, area and centre in the unit; inertia in
    g·unit² (or unit⁵ per unit density); mass in g; density in g/cm³;
  - the assembly tree's mate distances;
  - interference volumes.

The design, as planned:
- **`math::Units`**, the one table:
  - `LengthUnit` (mm, cm, m, in, ft), with symbols, and millimetres per
    unit;
  - `formatLength`, locale-free;
  - `parseLength(text, unit)` into millimetres. One or more terms, each a
    number with an optional unit ("25.4", "2in", "1' 6\"", "1 ft 6 in"),
    under one sign. A bare number is in @p unit. A decimal point only, since
    a comma separates a typed point's coordinates.
  - `parseAngle(text)` into radians. A bare number is degrees; "°", "deg"
    and "rad" are accepted.
  - Refused: an unknown unit, an empty term, a non-finite value.
  - Preferences and DimensionStyle use it. The Dimension Style form lists
    it.
- **The document's unit**:
  - `Document::lengthUnit()` and `AssemblyDocument::lengthUnit()`.
  - Saved in NativeFormat's document and assembly roots, and in a drawing
    sheet's `.hzdwg`, as `"units": {"length": "in"}`.
  - A file without it (every file so far) reads as millimetres. No version
    bump.
  - A new document takes the preference, now labelled "Unit for new
    documents".
- **Edit ▸ Document Units…** sets it, as one undo step, and marks the
  document modified.
- **Readouts in it**:
  - the cursor readout;
  - Measure Distance and Measure Area;
  - Measure Angle, with the decimals setting;
  - Mass Properties: lengths, areas and volumes in the unit; mass in g;
    density in g/cm³; inertia in g·unit²;
  - the assembly tree's mate distances;
  - interference volumes.

### 154b: fields that show and take it (as built)
- **`QuantitySpinBox`** (Length or Angle):
  - Its `value()` is millimetres or degrees, as before, so callers and
    `FormFiller` read what they did.
  - It shows the unit ("1.000 in", "30.00°"), keeping ten decimals inside.
  - `validate` takes only a whole value in range; anything else waits, and
    the field goes back to its value.
  - `setUnit` re-shows a field kept across tabs.
  - Keyboard tracking is off: a value is taken when entered.
- **`FeatureForm(parent, title, unit)`** with `length()` and `angle()`:
  - every MainWindow form, and its length and angle fields: primitives,
    Extrude, Revolve, Fillet/Chamfer, Shell, Draft, patterns, datums,
    section plane, block base, Dimension Style;
  - the generic Edit Feature form (Length, Angle and point fields);
  - the assembly's Move, Rotate and Edit Mate;
  - Add Mate, now a distance field and an angle field, each enabled for
    its mate type;
  - a drawing's section offset.
  Angle labels lost "(degrees)": the field shows °.
- **The property panel:**
  - line, circle and arc geometry, text height, hatch spacing and ellipse
    axes are lengths in the document's unit, set as the selection shows;
  - the rotations are angles;
  - line width (a plot width) and block scale (a factor) stay plain.
- **Dialogs:** the rectangular and polar array dialogs take the unit, Insert
  Block's rotation is an angle, and the constraint value prompts
  (`QInputDialog`) are forms with a length or an angle.
- **Tests:** `FormAnswers::typed(name, text)` types into a number field and
  enters it.

The design, as planned:
- **`QuantitySpinBox`**, a `QDoubleSpinBox`:
  - Its `value()` stays in millimetres (or degrees, for an angle), so
    callers keep reading what they read today.
  - It shows the document's unit as a suffix, and accepts a typed unit.
  - Internally it keeps enough decimals that 0.001 in survives the round
    trip.
- **`FeatureForm::length(...)`** and **`FeatureForm::angle(...)`**:
  - Every length and angle field moves to them: primitives, Extrude,
    Revolve, Fillet/Chamfer, Shell, Draft, patterns, datums, section plane,
    block base, dimension style, assembly Move/Rotate/Mate, and drawing
    sections.
  - The generic Edit Feature form moves too.
  - `number` stays for plain numbers.
- **The property panel's spin boxes** (lengths and angles), the rectangular
  and polar array dialogs, Insert Block, and the constraint value dialogs.
- **Tests:** `FormAnswers::typed(name, text)` types text into a named number
  field, so a test can enter "2 in".

### 154c: typing in the view (as built)
- **`TypedUnits`**: `typeUnitKey` appends a letter, a space, `'`, `"`, `/` or
  `°` once something is typed. A letter first is still the view's, a
  shortcut. `takeBack` removes a whole UTF-8 character.
- **`TypedPoint`**:
  - Keys: the above.
  - `resolve` and `take` read each coordinate and length with
    `parseLength` in the document's unit, and each angle with `parseAngle`.
  - `prompt` shows the unit: "Point (in): 2,3".
  - `ViewportInputHandler` and the window pass the document's unit.
- **`TypedLength`** (fillet radius, chamfer distance) parses with
  `parseLength`. Its prompt shows the unit, as "Radius (in): …" and
  "[radius=0.75 mm]".
- **Rotate's typed angle** is parsed with `parseAngle`, so "0.5 rad" works.
  A text that is not an angle is dropped.

The design, as planned:
- **`TypedPoint`** accepts letters, spaces, quotes and "°". Each coordinate
  or length may carry a unit ("2in,3in", "@50 mm<30 deg"). A bare number is
  in the document's unit.
- **`TypedLength`** (fillet and chamfer radius) the same.
- **Rotate's typed angle** accepts "rad".
- **The prompts** show the unit.

### Done when
- A part in inches shows and takes inches everywhere, keeps them when saved
  and opened, and models exactly what the same part in millimetres models.
- "2 in", "50.8 mm" and "5.08 cm" typed into any length give one value.
- Older files open in millimetres, as they did.

## Phase 155: Variables and equations

The question is what a number means. The model is in millimetres and
radians; a document is shown in its unit; and a change of that unit must
change nothing modelled (154). So an expression is kept with its units
written, and a plain number means a plain number.

### 155a: expressions with units (as built)
- **`UnitExpr`**:
  - a number, or a bracketed expression, followed by a unit word: mm, cm,
    m, in, ft, deg, rad;
  - its plain value is in millimetres or radians;
  - parsed, printed and read from JSON. A unit word after a number used to
    be a parse error, so no existing expression changes meaning.
- **`evaluateQuantity`** gives a value and what it measures, as powers of
  length and angle. It refuses:
  - a length plus a number;
  - a unit on something that already has one;
  - a fractional power of a length;
  - the sine of a length;
  - an unknown variable or function;
  - a division by zero.
- **`normalized`** is the form in which an expression entered for a
  parameter is kept:
  - a plain number added to a length gets the document's unit written in
    ("wall + 1" in inches becomes "wall + 1 in");
  - a plain result for a length or an angle gets the unit ("2 * 3" becomes
    "(2 * 3) in").
  - Nothing kept then depends on the document's unit.

### 155b: a document's variables (as built)
- **`ParameterRegistry`**:
  - `definitions()` and `setDefinitions()` treat the variables as a whole;
  - `quantities()` evaluates each in dependency order, leaving out any that
    can't be (with why);
  - `check()` refuses a bad name (a unit, a function, pi), a bad
    expression, a loop, or anything that can't be evaluated.
- **`SetVariablesCommand`** changes them as one undo step, and marks the
  part to be built again.
- **Edit ▸ Variables** (`VariablesDialog`):
  - a table of name, expression and value, with the value worked out as it
    is typed;
  - OK is refused with the reason shown, and the dialog stays open.
- Saved as before (`designVariables`, with the expression). An older build
  reads an expression with a unit as unparseable and keeps the value.

### 155c: feature parameters as expressions (as built)
- **`Feature::parameterExpressions`** holds the kept expressions, by
  parameter. `expressionError` is set when one can't be evaluated.
- **`Document::applyExpressions`**, run first by `buildWithDiagnostics`
  (which `rebuildModel` and the worker's `RebuildJob` both call):
  - evaluates each expression against the variables;
  - checks it measures what its parameter is: a length, an angle, or a
    plain number for a count;
  - sets the parameter.
  - A failure fails that feature's build with the reason, and the part
    stands as it did before it.
- **NativeFormat** saves `expressions` beside the parameters. The
  parameters hold the values last worked out, which an older build reads.
- **`EditFeatureCommand`** takes expression changes (an empty one returns
  the parameter to a number), and undo restores all of them.
- **`QuantitySpinBox`**:
  - takes "=expression" through a resolver, and shows the expression
    without its outer brackets;
  - stepping or typing a number drops the expression;
  - Qt rounds a value to the field's places by multiplying, so the field
    recognises its expression's value within a tolerance.
- **The Edit Feature form** gives each length and angle field a resolver
  that normalizes in the document's unit, shows a stored expression, and
  passes changes to the command.

## Phase 156: Configurations

### As built
- **Before this,** `ConfigurationTable` existed, with numeric overrides
  written into the variables destructively, but nothing saved it or
  reached it.
- **`ConfigurationTable`** now holds expression overrides ("diameter" =
  "8 mm") and the active one ("" for none). `overlay()` lays a
  configuration over the variables without writing them, so going back
  needs nothing but choosing another.
- **`Document`**:
  - `effectiveDefinitions()` gives its own variables with the active
    configuration laid over them;
  - `variables()` gives those worked out, which builds and the Edit
    form's expressions use;
  - `variableResolver()` gives their values, which the constraint solver
    uses at all five call sites (grip, move, constraint, edit and property
    panel), so constraints follow the configuration too.
- **`SetConfigurationsCommand`** changes the table, or which configuration
  is active, as one undo step, and marks the part to be built again.
- **Files:** saved as `configurations` (active, and each name with its
  values). An active one that isn't there leaves none; an older build
  ignores it.
- **Edit ▸ Configurations** (`ConfigurationsDialog`):
  - a row for each configuration, and a column for each variable, headed
    with its own value;
  - a blank cell keeps that variable's own value;
  - OK is refused if two rows share a name, or a configuration laid over
    the variables can't be evaluated.
- **The feature tree** has a Configuration chooser above the features,
  hidden while there are none; choosing one builds the part in it.

## Phase 157: Sketches that follow

Three parts, each its own PR: a sketch follows its face (157a), part edges
projected into a sketch (157b), and extrude up to a face (157c).

### 157a: a sketch follows its face (as built)
- **Before this,** Sketch on a Face copied the face's plane into the sketch
  and dropped the face's name. When the part changed, the sketch stayed
  where it was drawn, and a boss drawn on a top face was left inside a
  taller part.
- **`model::FacePlane`** (new, in modeling):
  - `planeOf(face, outward)` gives a flat face's plane, through the middle
    of its outline and facing out of the part. A face on a curved carrier
    is not flat, and neither is a facet of a curved face (its ideal, a
    cylinder's side, is curved);
  - `wholeFaceName(tag)` removes the `/piece:<n>` a Boolean gives each
    piece of a face it splits;
  - `planeOfFace(solid, name)` finds all the pieces of a face named that
    way. They must share one plane. Otherwise it gives nullopt with "is not
    there" or "is no longer flat".
  - Sketch on a Face lists faces with the same `planeOf`, so a sketch is
    placed exactly where it was drawn while the part is unchanged.
- **`Sketch`** keeps:
  - the plane it was drawn on (`drawnPlane()`, what the file calls
    "plane");
  - the face it follows (`face()`, a whole name);
  - where a build last placed it (`placed()`).
  - `plane()` is where it is: the placement, else the drawn plane.
  - `placeOn(point, normal)` takes the drawn plane's origin and x axis
    straight onto the face's plane. A face that moves along its normal
    carries the sketch along unturned, and a face that grows leaves it
    where it is.
  - This is a pure function of the drawn plane and the face, so undoing a
    change puts the sketch back exactly.
  - `placement()` is the rigid move from the drawn plane to the placed one.
- **The build.** `Feature::sketches()` names the sketches a feature is made
  from (Extrude, Revolve, Loft's sections, Sweep's profile and path).
  - Before a feature is built, `applyFeature` (and `buildBodies`) places
    each sketch that follows a face, on the part as it stands.
  - Each sketch is placed once per build, by the first feature made from
    it. A pattern's rebuild of that feature, and a second feature from the
    sketch, use it where it was placed.
  - A face that is not there, or is no longer flat, fails the feature,
    naming the sketch and the face. The sketch stays where it was last
    placed.
- **Directions and axes are kept as drawn.** An extrusion's direction and a
  revolve's axis are stored in the drawn plane's frame, and the build takes
  them through `placement()`. A face that turns, turns them too. The Extrude
  and Revolve commands take the drawn plane's normal and axes.
- **A worker's build.** `BuildResult::placements` carries each placed
  sketch's plane by id, and `Document::applyBuild` gives it to the
  document's own sketches. So a part built from a copy draws, and edits,
  its sketches where they now are.
- **Files:**
  - a sketch saves "plane" (drawn), "face" and "placed";
  - format version 20, because an older build would leave the sketch on
    its drawn plane and build a different part.
- **The sketch list** says "on a face", and its tooltip names the face.
- Not done: a sketch on a datum plane does not follow the datum (a datum
  keeps coordinates, not what it was made from), and there is no command
  yet to move a sketch to another face or stop it following.

### 157b: part edges projected into a sketch (plan)
- **Construction geometry.** `DraftEntity` gets a construction flag.
  - It is saved, and drawn dashed and dim.
  - `ProfileValidator` skips it, so it is never part of a profile.
  - A command sets or clears the flag on the selected sketch entities, as
    one undo step.
- **Sketch ▸ Project Edges**, while a sketch is edited:
  - The part's edges are listed. Picking is off in sketch mode, so edges
    clicked before are checked.
  - Each chosen logical edge is taken straight onto the sketch plane:
    - a line for a straight edge;
    - a circle or arc for a circular one whose plane is parallel to the
      sketch;
    - otherwise a polyline through its chords.
  - They are construction by default, and keep the edge's name
    (`sourceEdge`).
- **They follow.** When a build places the sketch, each projected entity is
  projected again from the same part. It keeps its id, so constraints on it
  hold. An edge that is gone leaves the entity where it was, and says so.
- The constraint solver holds projected entities fixed.

### 157c: extrude up to a face (plan)
- **The data.** `ExtrudeFeature::Extent::UpToFace` and a face (by whole
  name). They are saved as "extent": 4 and "upToFace", and the format
  version becomes 21, because an older build would read 4 as Blind.
- **The build.** The face is found in the part before the extrude
  (`planeOfFace`).
  - A face parallel to the sketch gives the distance.
  - A face that is not parallel gives each profile point its own distance
    to the face's plane, making a slanted top. It is refused where a point
    would not reach the face.
  - A curved face is refused for now.
- **The Extrude form** gets "Up to a face" with a list of the part's flat
  faces, the one clicked before already chosen. The Edit form shows the
  face.

## Tracking

Each phase is its own PR (154 as three), with README rows and CHANGELOG
entries. A phase's section here is replaced by "as built" when it lands.
