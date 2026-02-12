#pragma once

#include "horizon/document/Document.h"
#include <string>

namespace hz::io {

/// Import/export Document to/from DXF (AC1027/R2013) files.
/// Supports a practical subset of DXF entities for 2D CAD interop.
class DxfFormat {
public:
    /// Write document to a DXF file.
    static bool save(const std::string& filePath,
                     const doc::Document& doc);

    /// Read a DXF file and populate the document.
    static bool load(const std::string& filePath,
                     doc::Document& doc);
};

}  // namespace hz::io
