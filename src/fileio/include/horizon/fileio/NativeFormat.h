#pragma once

#include <memory>
#include <string>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"

namespace hz::io {

/// Save/load Document to/from JSON-based native files.
///
/// Formats (all share the same JSON envelope, distinguished by "type"):
///  - .hcad   ("hcad")   — 2D drawing document
///  - .hzpart ("hzpart") — parametric part: sketches + feature tree +
///                         optional tessellation cache for lightweight loads
///  - .hzasm  ("hzasm")  — assembly: component references + transforms
class NativeFormat {
public:
    static bool save(const std::string& filePath, const doc::Document& doc);

    static bool load(const std::string& filePath, doc::Document& doc);

    /// Save an assembly document (.hzasm). Component part paths are stored
    /// relative to the assembly file when possible.
    static bool saveAssembly(const std::string& filePath, const doc::AssemblyDocument& asmDoc);

    static bool loadAssembly(const std::string& filePath, doc::AssemblyDocument& asmDoc);

    /// Read only the tessellation cache of a part file, without constructing
    /// a Document (lightweight component resolution). Returns nullptr when
    /// the file has no cache or cannot be read.
    static std::shared_ptr<render::MeshData> loadPartMesh(const std::string& filePath);

    // -- JSON envelope helpers (shared with BinaryFormat) ---------------------

    /// Serialize the document envelope to a JSON string. BinaryFormat stores
    /// this payload inside its FlatBuffers container, with the tessellation
    /// cache carried as typed binary vectors instead of JSON numbers.
    static std::string documentToJson(const doc::Document& doc, bool includeTessellation);

    /// Populate a Document from a JSON envelope string.
    static bool documentFromJson(const std::string& text, doc::Document& doc);

    /// Serialize the assembly envelope. @p filePath anchors relative component
    /// paths (pass the eventual on-disk location; empty keeps paths as-is).
    static std::string assemblyToJson(const doc::AssemblyDocument& asmDoc,
                                      const std::string& filePath);

    /// Populate an AssemblyDocument from a JSON envelope string.
    static bool assemblyFromJson(const std::string& text, doc::AssemblyDocument& asmDoc,
                                 const std::string& filePath);
};

}  // namespace hz::io
