#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "horizon/fileio/StepFormat.h"

namespace hz::doc {
class AssemblyDocument;
class Document;
class DocumentManager;
struct ExactCounterpart;
}  // namespace hz::doc

namespace hz::io {

/// What a STEP export of an assembly writes (Phase 153): each part once, from
/// its file, and each unsuppressed component a use of it, placed; the
/// components whose part could not be read, by name. Each part is written as
/// its own export writes it (Task 10): its exact counterpart where it has one
/// (Document::exactOrFacetedSolid), else its faceted solid; `held` holds them.
/// A subassembly's parts are placed in one level with the rest, their
/// transforms composed with its (Phase 159); a mirrored component's part is a
/// part of its own, its mirror (Phase 162).
struct AssemblyStepExport {
    std::vector<StepWritePart> parts;
    std::vector<StepOccurrence> occurrences;
    std::vector<std::string> unread;
    /// The solids `parts` writes, the export's own: a part's counterpart,
    /// shared (it is never changed), a copy's, or a mirror made of one. None
    /// is the solid of a part an assembly places, which the window may build
    /// again.
    std::vector<std::shared_ptr<const topo::Solid>> held;
};

/// An assembly's STEP export made ready on the window's thread
/// (prepareAssemblyStep), to be built and gathered on any thread
/// (finishAssemblyStep): each part file once, as what it is written from, and
/// each use of it. It holds nothing of the parts the assembly places, so the
/// window may change them while it is finished.
struct AssemblyStepJob {
    /// One part file. Written as it is (`ready`: the part's exact
    /// counterpart, shared, or a copy of its faceted solid, where the part is
    /// built as it is written: a tab's, or one with no counterpart by design),
    /// or from `copy`, a document of the export's own read from the part (a
    /// snapshot of the part open, or its file read again), built with its
    /// exact counterpart when the job is finished. The part itself is never
    /// built for it (Ruling 20).
    struct Source {
        std::shared_ptr<const topo::Solid> ready;
        std::shared_ptr<doc::Document> copy;
        /// The part's counterpart, reused by the copy's build where it was
        /// made from the exact part the copy has.
        std::shared_ptr<const doc::ExactCounterpart> previous;
    };
    /// One part written: its file's source, mirrored or not, by its name.
    struct Part {
        std::size_t source = 0;
        bool mirrored = false;
        std::string name;
    };
    std::vector<Source> sources;
    std::vector<Part> parts;
    /// Each use of a part: `part` an index into `parts`.
    std::vector<StepOccurrence> occurrences;
    std::vector<std::string> unread;
    /// How many copies finishing it builds: work for a worker, which may take
    /// as long as building the parts.
    std::size_t builds() const;
};

/// Ready @p assembly's STEP export, on the window's thread: each unsuppressed
/// component's part found open in @p documents (the component's, or a tab's)
/// or read again from its file (DocumentManager::readPart), and nothing
/// resolved but a subassembly not yet read, which is read as its drawing reads
/// it (Lightweight). A part is never built.
AssemblyStepJob prepareAssemblyStep(doc::AssemblyDocument& assembly,
                                    doc::DocumentManager& documents);

/// Finish @p job, on any thread: each copy built with its exact counterpart,
/// and each part written as it, or its faceted solid where it has none, a
/// mirrored part as its mirror. A part none could be had for (its copy's build
/// made no solid) is left out, and its uses are unread. Once @p cancelled is
/// set, it stops between copies, and what it gathers is empty.
AssemblyStepExport finishAssemblyStep(AssemblyStepJob job,
                                      const std::atomic<bool>* cancelled = nullptr);

/// Prepare and finish @p assembly's export here, for StepFormat::saveAssembly.
/// The Assembly menu's STEP export (on a worker, where the rebuild mode sends
/// work there) and horizon-mcp's both write what this gathers.
AssemblyStepExport gatherAssemblyStep(doc::AssemblyDocument& assembly,
                                      doc::DocumentManager& documents);

}  // namespace hz::io
