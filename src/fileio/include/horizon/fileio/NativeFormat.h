#pragma once

#include "horizon/drafting/DraftDocument.h"
#include <string>

namespace hz::io {

/// Save/load DraftDocument to/from JSON-based .hcad files.
class NativeFormat {
public:
    static bool save(const std::string& filePath,
                     const draft::DraftDocument& doc);

    static bool load(const std::string& filePath,
                     draft::DraftDocument& doc);
};

}  // namespace hz::io
