#pragma once

#include <string>

#include "horizon/document/Document.h"

namespace hz::io {

/// Import/export Document to/from DXF (AC1027/R2013) files.
/// Supports a practical subset of DXF entities for 2D CAD interop.
class DxfFormat {
public:
    /// Write document to a DXF file. On failure returns false, with the
    /// reason in `error` when given.
    static bool save(const std::string& filePath, const doc::Document& doc,
                     std::string* error = nullptr);

    /// Read a DXF file and populate the document. On failure returns false,
    /// with the reason in `error` when given; never throws.
    static bool load(const std::string& filePath, doc::Document& doc, std::string* error = nullptr);
};

}  // namespace hz::io
