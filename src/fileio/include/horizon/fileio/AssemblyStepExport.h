#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/fileio/StepFormat.h"

namespace hz::doc {
class AssemblyDocument;
class DocumentManager;
}  // namespace hz::doc

namespace hz::io {

/// What a STEP export of an assembly writes (Phase 153): each part once, from
/// its file, and each unsuppressed component a use of it, placed; the
/// components whose part could not be read, by name. Each part is written as
/// its own export writes it (Task 10): its exact counterpart where it has one
/// (Document::exactOrFacetedSolid), held here, else the component's faceted
/// solid, valid while the assembly is not changed. A subassembly's parts are
/// placed in one level with the rest, their transforms composed with its
/// (Phase 159); a mirrored component's part is a part of its own, its mirror
/// (Phase 162), of its exact counterpart where it has one.
struct AssemblyStepExport {
    std::vector<StepWritePart> parts;
    std::vector<StepOccurrence> occurrences;
    std::vector<std::string> unread;
    /// The exact solids `parts` writes: each part's counterpart, shared (it is
    /// never changed), or a mirror made of it, so that they outlive a rebuild
    /// of their part.
    std::vector<std::shared_ptr<const topo::Solid>> held;
};

/// Gather @p assembly for StepFormat::saveAssembly, each component's part
/// read whole from @p documents first and built with its exact counterpart
/// (DocumentManager::resolveForExport). The Assembly menu's STEP export and
/// horizon-mcp's both write what this gathers.
AssemblyStepExport gatherAssemblyStep(doc::AssemblyDocument& assembly,
                                      doc::DocumentManager& documents);

}  // namespace hz::io
