# Old part bodies

Imported bodies as part files saved before stage 2 of the exact kernel hold
them: the STEP text NativeFormat embeds for an `ImportedBodyFeature`, written
by the writer of master at 54f72335 (each face a B-spline patch sized to its
facets). `tests/interop/test_OldParts.cpp` opens each as such a part file and
checks it opens exact, or, where no exact form holds it, as master opened it
(Ruling 10).

| File | Made from | Licence |
| --- | --- | --- |
| `solidedge_ercf_knife_holder.body.step` | `../step/solidedge_ercf_knife_holder.step`, read as File ▸ Import ▸ STEP read it, and its one body written by `StepFormat::toString` | as its source: ERCF v2, GPL-3.0 (see `../step/README.md`) |
| `onshape_rocketsmith_upper_airframe.body.step` | `../step/onshape_rocketsmith_upper_airframe.step`, the same way | as its source: RocketSmith, MIT (see `../step/README.md`) |
| `edm_ug_moon_buggy_asm.body15.step` | `../step/edm_ug_moon_buggy_asm.stp`, the same way: its body 15 (of 0 to 18, in the order the read gives them) | as its source: NIST Engineering Design Model Repository, public domain (see `../step/README.md`) |
