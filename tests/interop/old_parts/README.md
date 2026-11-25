# Old part bodies

Imported bodies as part files saved before stage 2 of the exact kernel hold
them: the STEP text NativeFormat embeds for an `ImportedBodyFeature`, written
by the writer of master at 54f72335 (each face a B-spline patch sized to its
facets). `tests/interop/test_OldParts.cpp` opens each as such a part file and
checks it opens as master opened it (Ruling 10).

| File | Made from | Licence |
| --- | --- | --- |
| `solidedge_ercf_knife_holder.body.step` | `../step/solidedge_ercf_knife_holder.step`, read as File ▸ Import ▸ STEP read it, and its one body written by `StepFormat::toString` | as its source: ERCF v2, GPL-3.0 (see `../step/README.md`) |
