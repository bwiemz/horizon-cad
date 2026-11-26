#pragma once

#include "horizon/fileio/ImportReport.h"

namespace hz::doc {
class Document;
}

namespace hz::io {

/// The newest native format whose parts were built from facets: a part of it or older is
/// migrated when it is read (migrateToExact). Format 32 records that a part is built exactly.
inline constexpr int kLastFacetedFormat = 31;

/// Migrate @p doc, a part just read from a file of format @p fileVersion, to format 32's exact
/// build (stage 3 of the exact kernel; spec: Migration), and say what changed. A file of format
/// 32 or newer is as this build wrote it: nothing is done, and nothing said.
///
/// - Each feature named the old ways (Positional, FromGeometry) is named Stable, as every
///   feature since Phase 139 is.
/// - Each face or edge kept by name (Feature::storedReferences(), the face a sketch is on, an
///   edge projected into one) whose name can no longer mean what it meant is dropped, never
///   guessed, and reported in `skipped`, so that the part is read in part (Document::leftOut):
///   "<owner>: its reference to <name>, <what>, is not carried over (<why>)". The owner is the
///   feature that keeps it (for a sketch's, the first feature made from the sketch), "feature
///   <k> (<its name>)", or "sketch '<name>'" for a sketch no feature is made from. Why:
///   - "a sheet's faces are named anew": a face or an edge of a sheet, which master named after
///     lines it made anew in each build (Ruling 28);
///   - "named the old way": one whose name a feature named the old ways made, which the name
///     carries (its id, or an old primitive's shared source: `box/top`);
///   - "built from pieces before": a piece, a region, a chord or a seam of an extrusion or a
///     revolve of a profile with holes or of several regions, which master built by Booleans
///     whose names the exact build has not (Ruling 23);
///   - "a flat face is now one face": a seam of a face square to the axis of a revolve through
///     part of a turn, which master cut into a quad a step and is one face now (Ruling 25).
///   Every other reference is kept as it is: the bridge numbers facets and chords as master did
///   (Ruling 21), so a facet's or a chord's name means the facet or the chord it meant, and is
///   not widened to its face or edge (Ruling 40).
/// - Each converted feature (a curved primitive, an extrusion with arcs, a revolve, a hole, a
///   sheet) drawn with fewer than 12 chords a turn is reported in `approximated`, which reads
///   the part whole: "<owner>: drawn as a 6-sided prism before; round now", in words for what it
///   is (a revolve "turned in <n> steps a turn", a sheet's bends "drawn with <n> chords each").
ImportReport migrateToExact(doc::Document& doc, int fileVersion);

}  // namespace hz::io
