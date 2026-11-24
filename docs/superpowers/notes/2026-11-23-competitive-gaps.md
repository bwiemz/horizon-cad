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
| **Geometry kernel** | Exact B-Rep (Parasolid / ACIS): true cylinders, fillets and NURBS, booleans on exact surfaces. | A B-Rep whose faces are facets that record their true surface; booleans on the facets. Volumes converge with resolution. | **Largest gap**, and the costliest to close. Not a next step. |
| **Part features** | Extrude, revolve, sweep, loft, fillet, chamfer, shell, draft, hole wizard, patterns, mirror, ribs, configurations. | All of these but ribs; configurations; variables. Fillet and loft are experimental on hard inputs. | Small in breadth; robustness on hard inputs. |
| **Sheet metal** | Base flange from an open sketch, edge flanges, hems, bend tables, flat pattern to DXF for cutting. | A folded-strip library (bends, K-factor, bend allowance, flat-pattern length), **not reachable** in the application. | **Large and cheap to start**: the maths exists. Next step. |
| **Assemblies** | Mates, subassemblies, patterns, interference, exploded views, BOM, motion. | All, motion as studies of a mate driven through a range. Experimental. | Moderate: flexible subassemblies; motion over time, with forces. |
| **Drawings** | Standard, section, detail and auxiliary views; dimensions; hole callouts; GD&T; BOM tables and balloons; sheet formats. | Standard, section and detail views; length/radius/diameter dimensions; title block; balloons and a parts list for assemblies; undo; PDF/SVG/DXF. GD&T exists in the model, not the UI. | Moderate: hole callouts, GD&T and ordinate dimensions in the UI, auxiliary views. |
| **2D drafting (AutoCAD)** | Lines to splines, hatches, dimensions, blocks, layers, linetypes, layouts and paper space, xrefs, DWG. | Stable 2D core: the entities, hatches, dimensions, blocks, layers, linetypes, constraints; DXF in and out. | **DWG** (AutoCAD's own format) and layouts/paper space. DWG needs a library (LibreDWG is GPL, compatible with GPL-3.0-or-later). |
| **Exchange** | STEP, IGES, Parasolid, DWG/DXF, STL, 3MF. | STEP AP242 in and out (experimental), DXF, STL, glTF. | IGES, 3MF; STEP robustness. |
| **Simulation / CAM** | SolidWorks Simulation, CAMWorks. | Library-only prototypes (FEA on boxes; 2.5-axis paths). | Large; needs a tet mesher first. Later. |

## What to build next, in order

1. **Sheet metal in the application**: a base flange from an open sketch
   (thickness, bend radius, K-factor), its flat pattern exported to DXF for
   cutting, and an AI tool for it. The maths and the folded solid exist.
2. **Drawing annotations**: hole callouts and GD&T frames in the Drawing
   menu and the sheet tools (the model already holds GD&T).
3. **DWG import** through LibreDWG, behind a build option until it is proven
   on every platform CI builds.
4. ~~**Motion** for assemblies~~ (done: Assembly ▸ Motion Study drives a
   Distance or Angle mate through a range with the mate solver, not the
   serial-chain kinematics library, which a linkage is not; `study_motion`
   for the AI. Next: saving studies, several drivers, time and speed.)

The kernel gap is real and is not on this list: closing it is a multi-year
project. The facets-with-true-surfaces kernel keeps volumes, masses and
exports right to a chosen accuracy, which is what most parts need.
