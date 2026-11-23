#pragma once

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
/// components whose part could not be read, by name. The parts' solids are
/// the components' own: valid while the assembly is not changed. A
/// subassembly's parts are placed in one level with the rest, their
/// transforms composed with its (Phase 159); a mirrored component's part is a
/// part of its own, its mirror (Phase 162).
struct AssemblyStepExport {
    std::vector<StepWritePart> parts;
    std::vector<StepOccurrence> occurrences;
    std::vector<std::string> unread;
};

/// Gather @p assembly for StepFormat::saveAssembly, each component's part
/// read whole from @p documents first. The Assembly menu's STEP export and
/// horizon-mcp's both write what this gathers.
AssemblyStepExport gatherAssemblyStep(doc::AssemblyDocument& assembly,
                                      doc::DocumentManager& documents);

}  // namespace hz::io
