#pragma once

#include <memory>
#include <string>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"

namespace hz::io {

/// FlatBuffers-based binary container for 3D documents (Phase 51).
///
/// The container wraps the canonical NativeFormat JSON envelope (feature
/// tree, sketches, entities, parameters) and carries the tessellation cache
/// as typed binary vectors.  loadPartMesh() reads those vectors without
/// parsing the JSON payload — the zero-copy fast path that lets assemblies
/// display components immediately while feature trees resolve in the
/// background.
///
/// Files carry the FlatBuffers identifier "HZBF" at offset 4; isBinaryFile()
/// distinguishes them from the JSON formats so both can share the .hzpart /
/// .hzasm extensions.
class BinaryFormat {
public:
    static bool save(const std::string& filePath, const doc::Document& doc);

    static bool load(const std::string& filePath, doc::Document& doc);

    static bool saveAssembly(const std::string& filePath, const doc::AssemblyDocument& asmDoc);

    static bool loadAssembly(const std::string& filePath, doc::AssemblyDocument& asmDoc);

    /// Read only the tessellation cache, without constructing a Document or
    /// parsing the JSON payload. Returns nullptr when the file has no cache
    /// or fails verification.
    static std::shared_ptr<geo::MeshData> loadPartMesh(const std::string& filePath);

    /// True when the file carries the "HZBF" FlatBuffers identifier.
    static bool isBinaryFile(const std::string& filePath);
};

}  // namespace hz::io
