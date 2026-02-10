#pragma once

#include "horizon/document/Document.h"
#include <string>

namespace hz::io {

/// Save/load Document to/from JSON-based .hcad files.
class NativeFormat {
public:
    static bool save(const std::string& filePath,
                     const doc::Document& doc);

    static bool load(const std::string& filePath,
                     doc::Document& doc);
};

}  // namespace hz::io
