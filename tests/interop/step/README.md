# The external STEP corpus

STEP files exported by twenty other CAD systems, each read by Horizon CAD as
a user's would be, and checked two ways:

- **Against its manifest, in every build** (`hz_interop_tests`,
  `tests/interop/test_StepAcceptance.cpp`). The file must be read, and
  must come through Horizon CAD unchanged:
  - kept as a part, and read back;
  - kept as an assembly of part files, and read back;
  - sent out as STEP again, and read back.

  Its bodies, volume and bounding box must be OpenCASCADE's. What it
  reports leaving out or approximating must be as many items as the
  manifest says.
- **Against OpenCASCADE, an independent reader, in CI** (the STEP interop
  job, `tools/interop/occt_check.py`). OpenCASCADE reads each original file,
  and must find the same solids, volume and box. It then reads Horizon CAD's
  export of the file, and must find a valid shape with as many solids.

The expected numbers are OpenCASCADE's, never Horizon CAD's own: a reader
that agrees with itself proves nothing.

## What is in it

| Exporter | File | What it exercises |
|---|---|---|
| Unigraphics 10.5 | `edm_ug_moon_buggy_asm.stp` | an assembly 4 levels deep, placed by MAPPED_ITEM, parts repeated; inches |
| Pro/ENGINEER | `edm_proe_vaccase_asm.stp` | an assembly, one part placed 5 times; inches |
| I-DEAS | `edm_ideas_card_cage.stp` | a surface model, no solid; a misused header |
| IronCAD | `edm_ironcad_impeller.stp` | B-spline faces throughout |
| EUCLID | `occt_euclid_screw.step` | a pre-release AP214 schema name; tori and cones |
| NX 2027, CATIA V5, Creo 9 | `nist_uuid_*_plate.stp` | one part from three systems, with PMI, AP242 edition 4 |
| SolidWorks 2016 | `irnas_tightener_asm.step` | an assembly with a part that has a void (BREP_WITH_VOIDS) |
| Onshape | `onshape_rocketsmith_upper_airframe.step` | metres, and an inch unit the exporter writes inverted |
| Fusion 360 | `fusion_smallrobotarm_axistot1.step` | a nested assembly, a part placed 6 times, colours |
| Inventor 2023 | `inventor_prusa_encl_door_hinge.stp` | 177 faces, B-splines; a wireframe beside the solid |
| Solid Edge | `solidedge_ercf_knife_holder.step` | countersinks (cones) and holes |
| FreeCAD | `freecad_thor_basetop.step` | a part with holes |
| build123d | `build123d_m4x10_screw.step` | a screw from a code-CAD library |
| KiCad | `kicad_kicon2019_pcb.step` | a circuit board: a nested assembly, colours |
| Shapr3D | `shapr3d_hkcam_lid.step` | the AP203 edition 2 schema name, through HOOPS |
| Rhino 8 | `rhino_axom_bearings.step` | a surface model in metres |
| Alibre | `alibre_jacktheripper_guide.stp` | centimetres |
| Creo 2015 | `creo_opencellular_emblem.stp` | a surface model, heavy styling |

The files are kept byte for byte as their exporters wrote them, CRLF line
ends included (`.gitattributes`).

## What it found

These are ranked by how many of the twenty files they affect, which is the
order to fix them in. `known_gaps` and `not_read` in the manifest pin each
one. A test fails once a gap closes, so the manifest is updated when it is
fixed. OpenCASCADE's reading confirmed each of them.

1. **A curved face not bounded by its surface's own edges** comes in as one
   flat facet, its outline. That holds for 8 files.
   - For 4 of them (Pro/E, SolidWorks, Onshape and Shapr3D), the facet is not
     flat. The part imports, then its Imported feature fails ("a flat face is
     not flat"), and it cannot be saved and opened again.
   - For others, the part measures wrong. The Onshape tube measures 132,720
     mm³ against OpenCASCADE's 712,990, and the Solid Edge part 61 mm³ in
     facets against 685.
   - The causes: a cylinder's side written as a band between two rims with
     no seam; edges a few micrometres off their surfaces; and faces of plain
     FACE_BOUNDs whose inner loop comes first. #175 fixes all three, and
     those files then agree with OpenCASCADE.
2. **Surface models**, with open shells and no solid, are not read at all.
   That holds for 3 files (Rhino, Creo 2015 and I-DEAS).
3. **Edge curves the reader does not build** stop a solid from coming in.
   That holds for 3 files: the Inventor 2023 part and the IronCAD impeller
   are not read, and one of the Unigraphics solids is skipped.
4. **A nested assembly not placed whole.** 4 of the KiCad board's 9 placed
   solids are not read; it is not yet known why.
5. **Placements by MAPPED_ITEM** are not read. Their parts come in where
   they were drawn, and this is reported. It affects 1 file, which has 23 of
   them. Three more of that file's solids are skipped for malformed loops.
   OpenCASCADE places 19 solids in it; Horizon CAD reads 9.
6. **BREP_WITH_VOIDS** affects 1 file. The part with a void is left out of
   the SolidWorks assembly, and **nothing reports it**. OpenCASCADE reads 6
   solids in it; Horizon CAD reads 4.
7. **A surface type the reader does not build** affects 1 file (EUCLID).
8. **An export that measures under the part:** OpenCASCADE reads Horizon
   CAD's export of the Shapr3D lid 0.4% under the part; not yet known why.

One difference is OpenCASCADE's, not Horizon CAD's: CATIA writes a
tessellated copy of the plate beside it (TESSELLATED_SOLID), which
OpenCASCADE reads as a second solid.

## Adding a file

1. Only a file whose licence allows it to be redistributed here, as test
   data in a GPL-3.0-or-later project. The licence must be stated in its
   repository, and must be public domain, CC0, CC-BY or CC-BY-SA, MIT, BSD,
   Apache, LGPL, GPL, MPL or CERN-OHL. Not "all rights reserved", NC or ND,
   and not a vendor's part model inside someone else's repository.
2. Download it from a URL pinned to a commit, and put it here unchanged.
3. Add its manifest entry:
   - `source` (that URL), `repository`, `license` (its SPDX id) and
     `license_file`;
   - `attribution`, as its licence asks;
   - `exporter`, `schema` and `unit` (from its FILE_NAME and FILE_SCHEMA);
   - `content`, what it exercises;
   - `expect: {}`.

   If its licence's text is not in `LICENSES/` yet, add it (GPL-3.0 is the
   repository's own `LICENSE`).
4. Push. The STEP interop job keeps what OpenCASCADE found, as `occt.json` in
   the run's `step-interop` artifact. Merge it into the manifest:
   `python3 tools/interop/occt_check.py merge occt.json tests/interop/step/manifest.json`.
5. Run `hz_interop_tests`. For each check the file fails for a reason you
   cannot fix now, add it to `known_gaps`, with why. The checks are
   `import-build`, `reopen`, `bodies`, `volume` and `bounds`, and for the
   OpenCASCADE job also `export`, `export-valid`, `export-bodies` and
   `export-volume`. If it is not read at all, set `not_read` to part of the
   error, and `why_not_read`. Set `expect.skipped` and `expect.approximated`
   to what it reports now.

## Provenance and licences

Every file's source, licence and attribution are in `manifest.json`. The
licence texts are in `LICENSES/`, and the GPL-3.0 files are under the
repository's own `LICENSE`. The files are:

- **NIST's Engineering Design Model Repository** (`edm_*`). NIST states this
  collection is in the public domain; it was gathered in the 1990s from
  industrial contributors. See Regli and Gaines, *Computer-Aided Design*
  29(12):895–905, 1997.
- **NIST's UUID dataset** (`nist_uuid_*`), under the NIST software licence,
  from NIST AMS 300-12. NIST edited these files by hand.
- **Open CASCADE's sample data** (`occt_euclid_screw.step`), under LGPL-2.1
  with the OCCT exception, © OPEN CASCADE SAS.
- **Open-hardware and code-CAD projects**, each named with its licence in the
  manifest:
  - Thor, by AngelLM (CC BY-SA 4.0);
  - OpenCellular, by the Telecom Infra Project (CC BY 4.0);
  - GoodEnoughCNC, by IRNAS (CERN OHL 1.2);
  - RocketSmith, © 2026 Peter Pak (MIT);
  - OpenPartVault, by jdegenstein (Apache-2.0);
  - hkcam, by brutella (Apache-2.0);
  - axom_data, © 2017–2024 Lawrence Livermore National Security, LLC
    (BSD-3-Clause);
  - SmallRobotArm, the Original Prusa Enclosure, ERCF v2, kicon2019 and
    JacktheRipperBot (GPL-3.0).
