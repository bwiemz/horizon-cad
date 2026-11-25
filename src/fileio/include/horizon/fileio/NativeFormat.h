#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/fileio/ImportReport.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::io {

/// An imported body as a copy of a document shares it with the document it was copied from (a
/// rebuild on a worker, ui::RebuildJob): the body as read, which is never changed once read, and
/// what its read said of it (doc::ImportedBodyFeature::note).
struct SharedImport {
    std::shared_ptr<const topo::Solid> solid;
    std::string note;
};

/// A document's shared imported bodies, by their feature's ID (doc::Feature::featureID()).
using SharedImports = std::unordered_map<std::string, SharedImport>;

/// Save/load Document to/from JSON-based native files.
///
/// Formats (all share the same JSON envelope, distinguished by "type"):
///  - .hcad   ("hcad")   — 2D drawing document
///  - .hzpart ("hzpart") — parametric part: sketches + feature tree +
///                         optional tessellation cache for lightweight loads
///  - .hzasm  ("hzasm")  — assembly: component references + transforms
///
/// Every load and save reports failure by returning false; when `error` is
/// given it receives a reason a user can act on ("the file does not exist",
/// "parse error at line 3, column 5: ..."). Loaders never throw: a malformed
/// file is a failed load, not an exception.
class NativeFormat {
public:
    static bool save(const std::string& filePath, const doc::Document& doc,
                     std::string* error = nullptr);

    /// Load a document. With @p report, each malformed item the load had to
    /// leave out (it skips them rather than refusing the whole file) is
    /// recorded there, with why.
    static bool load(const std::string& filePath, doc::Document& doc, std::string* error = nullptr,
                     ImportReport* report = nullptr);

    /// Save an assembly document (.hzasm). Component part paths are stored
    /// relative to the assembly file when possible.
    static bool saveAssembly(const std::string& filePath, const doc::AssemblyDocument& asmDoc,
                             std::string* error = nullptr);

    static bool loadAssembly(const std::string& filePath, doc::AssemblyDocument& asmDoc,
                             std::string* error = nullptr, ImportReport* report = nullptr);

    /// Read only the tessellation cache of a part file, without constructing
    /// a Document (lightweight component resolution). Returns nullptr when
    /// the file has no cache or cannot be read.
    static std::shared_ptr<geo::MeshData> loadPartMesh(const std::string& filePath);

    // -- JSON envelope helpers (shared with BinaryFormat) ---------------------

    /// Serialize the document envelope to a JSON string. BinaryFormat stores
    /// this payload inside its FlatBuffers container, with the tessellation
    /// cache carried as typed binary vectors instead of JSON numbers.
    ///
    /// With @p shared, a copy for this process alone: each imported body is
    /// not written as STEP but put in @p shared by its feature's ID, for
    /// documentFromJson to take as it is. Such a copy is no part file.
    static std::string documentToJson(const doc::Document& doc, bool includeTessellation,
                                      SharedImports* shared = nullptr);

    /// Populate a Document from a JSON envelope string. With @p shared, an
    /// imported body found there by its feature's ID is that body, the very
    /// one, and no STEP is read for it: a copy documentToJson made with them.
    static bool documentFromJson(const std::string& text, doc::Document& doc,
                                 std::string* error = nullptr, ImportReport* report = nullptr,
                                 const SharedImports* shared = nullptr);

    /// Serialize the assembly envelope. @p filePath anchors relative component
    /// paths (pass the eventual on-disk location; empty keeps paths as-is).
    static std::string assemblyToJson(const doc::AssemblyDocument& asmDoc,
                                      const std::string& filePath);

    /// Populate an AssemblyDocument from a JSON envelope string.
    static bool assemblyFromJson(const std::string& text, doc::AssemblyDocument& asmDoc,
                                 const std::string& filePath, std::string* error = nullptr,
                                 ImportReport* report = nullptr);
};

}  // namespace hz::io
