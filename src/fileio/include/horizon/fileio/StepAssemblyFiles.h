#pragma once

#include <atomic>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "horizon/fileio/StepFormat.h"

namespace hz::io {

/// The files a STEP assembly was kept as (Phase 153).
struct StepAssemblyFiles {
    /// The assembly (.hzasm).
    std::string assembly;
    /// A part file (.hzpart) for each of its parts, in order.
    std::vector<std::string> parts;
    /// The folder the parts were put in.
    std::string partsDir;
    /// The folders made for them, the parts' own first, then each above it
    /// that was not there either.
    std::vector<std::string> madeDirs;
};

/// Called as each part file is written: how many are, of how many.
using StepAssemblyProgress = std::function<void(std::size_t written, std::size_t total)>;

/// Keep @p read, a STEP file's parts and where it places them, as Horizon
/// files (Phase 153): each part a part file (.hzpart) in @p partsDir, made
/// if it is not there, its solids imported bodies from @p source; and an
/// assembly (.hzasm) at @p assemblyPath placing the parts as the STEP file
/// does, each component named as its use there. No part file is written
/// over: a name already taken is numbered ("Bolt 2"). The parts' solids are
/// moved out of @p read.
///
/// Each part is built and written in turn, which for a large file takes a
/// while: run it on a worker, told of each part written by @p progress. Once
/// @p cancelled is set, it stops before the next part, and returns false with
/// the error "cancelled". False, with why in @p error, when a file cannot be
/// written. Either way what it wrote is removed again (removeStepAssembly-
/// Files), so an import that does not finish leaves nothing behind; @p
/// written is what it wrote, all of it, when it returns true.
bool saveStepAssembly(StepAssembly& read, const std::string& assemblyPath,
                      const std::string& partsDir, const std::string& source,
                      StepAssemblyFiles* written = nullptr, std::string* error = nullptr,
                      const std::atomic<bool>* cancelled = nullptr,
                      const StepAssemblyProgress& progress = {});

/// Remove what saveStepAssembly wrote: @p files' parts and assembly, and the
/// folders made for them that nothing else is in. A file that is not there
/// is passed over.
void removeStepAssemblyFiles(const StepAssemblyFiles& files);

}  // namespace hz::io
