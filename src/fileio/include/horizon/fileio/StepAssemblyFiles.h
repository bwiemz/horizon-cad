#pragma once

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
};

/// Keep @p read, a STEP file's parts and where it places them, as Horizon
/// files (Phase 153): each part a part file (.hzpart) in @p partsDir, made
/// if it is not there, its solids imported bodies from @p source; and an
/// assembly (.hzasm) at @p assemblyPath placing the parts as the STEP file
/// does, each component named as its use there. No part file is written
/// over: a name already taken is numbered ("Bolt 2"). The parts' solids are
/// moved out of @p read. False, with why in @p error, when a file cannot be
/// written; any written by then are left, and listed in @p written.
bool saveStepAssembly(StepAssembly& read, const std::string& assemblyPath,
                      const std::string& partsDir, const std::string& source,
                      StepAssemblyFiles* written = nullptr, std::string* error = nullptr);

}  // namespace hz::io
