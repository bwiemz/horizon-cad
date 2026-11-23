#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/fileio/ImportReport.h"

namespace hz::io {

/// What a DXF export could not write as the drawing has it.
struct DxfWriteReport {
    /// Entities not written, each with why: "entity 7: a circle with no
    /// radius", or a value that is not a finite number. Readers refuse such
    /// an entity, and some the whole file with it.
    std::vector<std::string> leftOut;
    /// Layers written under another name, as DXF allows one: without the
    /// characters it refuses, not empty, and apart from the others whatever
    /// their case.
    std::vector<std::string> renamed;

    bool empty() const { return leftOut.empty() && renamed.empty(); }
};

/// Import/export Document to/from DXF (AC1027/R2013) files.
/// Supports a practical subset of DXF entities for 2D CAD interop.
class DxfFormat {
public:
    /// Write document to a DXF file. On failure returns false, with the
    /// reason in `error` when given. With @p report, what the file could not
    /// hold as it is, and so left out or renamed, is listed there.
    static bool save(const std::string& filePath, const doc::Document& doc,
                     std::string* error = nullptr, DxfWriteReport* report = nullptr);

    /// Read a DXF file and populate the document. On failure returns false,
    /// with the reason in `error` when given; never throws.
    /// With @p report, the entities it did not read are counted there by type.
    static bool load(const std::string& filePath, doc::Document& doc, std::string* error = nullptr,
                     ImportReport* report = nullptr);

    /// As load(), from DXF text in memory.
    static bool loadFromString(const std::string& text, doc::Document& doc,
                               std::string* error = nullptr, ImportReport* report = nullptr);

    /// The most placements one import makes by flattening blocks inserted
    /// into blocks (default 2,000,000): each entity placed, and each block
    /// within a block walked. Nesting multiplies: ten inserts of a block of
    /// ten inserts, eight levels deep, is a hundred million entities from a
    /// few kilobytes. An import that would make more stops flattening there,
    /// and its report says what was left out.
    static void setMaxFlattenedEntities(size_t limit);
    static size_t maxFlattenedEntities();
};

}  // namespace hz::io
