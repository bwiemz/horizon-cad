# Where Horizon CAD stands against SolidWorks and AutoCAD (2026-11-23)

The goal: an open-source CAD application that can compete with SolidWorks
(3D parts, assemblies, drawings) and AutoCAD (2D drafting), with AI built
in. This note says, area by area, what a user of those products expects,
what Horizon has, and what to build next. It is written from the README's
Feature Maturity table and the code, not from marketing: "has" means a
user can reach it in the application.

| Area | What users of SolidWorks / AutoCAD expect | What Horizon has | Gap |
|------|------------------------------------------|------------------|-----|
| **AI** | Neither has an assistant that builds parts or edits drawings from a request. | Claude, GPT, Gemini and local models; tools for parts, assemblies, drawings (made and edited), views of the model; every change one undo step; `horizon-mcp` for outside AI clients. | **Ahead.** Keep widening the tools as features land. |
| **Geometry kernel** | Exact B-Rep (Parasolid / ACIS): true cylinders, fillets and NURBS, booleans on exact surfaces. | A B-Rep whose faces are facets that record their true surface; booleans on the facets. Volumes converge with resolution. Beside it, stage 1 of an exact kernel, which nothing in the application reads yet: typed analytic and NURBS surfaces and curves, pcurves, one tolerance model, exact predicates, an exact validator, constrained Delaunay triangulation, watertight meshing of trimmed faces. | **Largest gap**, being closed in-house in seven stages; stage 1 is done, and stage 2 (exact exchange) is the first the application sees. |
| **Part features** | Extrude, revolve, sweep, loft, fillet, chamfer, shell, draft, hole wizard, patterns, mirror, ribs, configurations. | All of these but ribs; configurations; variables. Fillet and loft are experimental on hard inputs. | Small in breadth; robustness on hard inputs. |
| **Sheet metal** | Base flange from an open sketch, edge flanges, hems, bend tables, flat pattern to DXF for cutting. | A folded-strip library (bends, K-factor, bend allowance, flat-pattern length), **not reachable** in the application. | **Large and cheap to start**: the maths exists. Next step. |
| **Assemblies** | Mates, subassemblies, patterns, interference, exploded views, BOM, motion. | All, motion as studies of a mate driven through a range. Experimental. | Moderate: flexible subassemblies; motion over time, with forces. |
| **Drawings** | Standard, section, detail and auxiliary views; dimensions; hole callouts; GD&T; BOM tables and balloons; sheet formats. | Standard, section and detail views; length/radius/diameter dimensions; title block; balloons and a parts list for assemblies; undo; PDF/SVG/DXF. GD&T exists in the model, not the UI. | Moderate: hole callouts, GD&T and ordinate dimensions in the UI, auxiliary views. |
| **2D drafting (AutoCAD)** | Lines to splines, hatches, dimensions, blocks, layers, linetypes, layouts and paper space, xrefs, DWG. | Stable 2D core: the entities, hatches, dimensions, blocks, layers, linetypes, constraints; DXF in and out; DWG read (2000 to 2018 formats) through LibreDWG's `dwg2dxf`, run as a program of its own, behind `HZ_ENABLE_DWG`. | DWG write; layouts/paper space. |
| **Exchange** | STEP, IGES, Parasolid, DWG/DXF, STL, 3MF. | STEP AP242 in and out (experimental), DXF, STL, glTF. | IGES, 3MF; STEP robustness. |
| **Simulation / CAM** | SolidWorks Simulation, CAMWorks. | Library-only prototypes (FEA on boxes; 2.5-axis paths). | Large; needs a tet mesher first. Later. |

## What to build next, in order

1. **Sheet metal in the application**: a base flange from an open sketch
   (thickness, bend radius, K-factor), its flat pattern exported to DXF for
   cutting, and an AI tool for it. The maths and the folded solid exist.
2. **Drawing annotations**: hole callouts and GD&T frames in the Drawing
   menu and the sheet tools (the model already holds GD&T).
3. ~~**DWG import**~~ (done: File ▸ Open and Import read DWG through
   LibreDWG's `dwg2dxf`; `HZ_ENABLE_DWG`, which CI builds on all three
   platforms, ships it. Next: turn it on in the release workflow, with the
   install check opening a DWG on each clean machine.) Through LibreDWG,
   behind a build option until it is proven on every platform CI builds.
4. ~~**Motion** for assemblies~~ (done: Assembly ▸ Motion Study drives a
   Distance or Angle mate through a range with the mate solver, not the
   serial-chain kinematics library, which a linkage is not; `study_motion`
   for the AI. Next: saving studies, several drivers, time and speed.)

The kernel gap is real and is being closed in-house, in seven stages, which
is a multi-year project, so it is not on this list of next steps. The design
is in [the exact kernel spec](../specs/2026-11-24-exact-kernel-design.md).
Stage 1 is done: typed analytic and NURBS surfaces and curves, pcurves, one
tolerance model, exact robust predicates, an exact validator, Pattern copies
that keep exact geometry, constrained Delaunay triangulation, and watertight
meshing of trimmed faces at `BoundaryMesh`'s read point. Nothing in the
application reads exact geometry until stage 2, exact exchange. Until then the
facets-with-true-surfaces kernel keeps volumes, masses and exports right to a
chosen accuracy, which is what most parts need.
