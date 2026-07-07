#pragma once

#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"

namespace hz::doc {

/// Owns all open documents and the references between them.
///
/// Responsibilities (roadmap section 5.1):
///  - open-document registry, deduplicated by canonical file path
///  - cross-document reference resolution (assembly components → parts)
///    in Lightweight (cached mesh only) or Resolved (full feature tree) mode
///  - external file-change notifications via modification-time polling
///
/// The fileio module depends on hz::doc, not the other way around, so the
/// manager receives its load functions by injection (the application wires
/// them to `hz::io::NativeFormat` at startup; tests inject their own).
class DocumentManager {
public:
    /// Load a full part/drawing document from a file. Returns false on failure.
    using PartLoader = std::function<bool(const std::string& path, Document& doc)>;

    /// Load only the cached tessellated mesh from a part file (nullptr if
    /// absent). Used for lightweight component resolution.
    using MeshLoader = std::function<std::shared_ptr<geo::MeshData>(const std::string& path)>;

    /// Load an assembly document from a file. Returns false on failure.
    using AssemblyLoader = std::function<bool(const std::string& path, AssemblyDocument& doc)>;

    /// Invoked from pollExternalChanges() for each watched file whose
    /// on-disk modification time changed since it was opened/saved.
    using ExternalChangeCallback = std::function<void(const std::string& path)>;

    DocumentManager() = default;

    // Non-copyable (owns document identity).
    DocumentManager(const DocumentManager&) = delete;
    DocumentManager& operator=(const DocumentManager&) = delete;

    // --- Loader injection ---

    void setPartLoader(PartLoader loader) { m_partLoader = std::move(loader); }
    void setMeshLoader(MeshLoader loader) { m_meshLoader = std::move(loader); }
    void setAssemblyLoader(AssemblyLoader loader) { m_assemblyLoader = std::move(loader); }

    // --- Document lifecycle ---

    /// Create a new, unsaved document of the given type (Drawing or Part).
    std::shared_ptr<Document> newDocument(DocumentType type);

    /// Create a new, unsaved assembly document.
    std::shared_ptr<AssemblyDocument> newAssembly();

    /// Open a part/drawing file. If the file is already open, returns the
    /// existing instance (deduplicated by canonical path). Returns nullptr
    /// on load failure or when no part loader is set.
    std::shared_ptr<Document> openPart(const std::string& path);

    /// Open an assembly file (deduplicated by canonical path). Returns
    /// nullptr on load failure or when no assembly loader is set.
    std::shared_ptr<AssemblyDocument> openAssembly(const std::string& path);

    /// Close (unregister) a document. Returns true if it was open.
    bool closeDocument(const std::shared_ptr<Document>& doc);
    bool closeAssembly(const std::shared_ptr<AssemblyDocument>& doc);

    const std::vector<std::shared_ptr<Document>>& documents() const { return m_documents; }
    const std::vector<std::shared_ptr<AssemblyDocument>>& assemblies() const {
        return m_assemblies;
    }

    /// Find an open part/drawing document by path (nullptr if not open).
    std::shared_ptr<Document> findByPath(const std::string& path) const;

    /// Record that `doc` was saved to its filePath(): registers the path for
    /// deduplication and refreshes the stored modification time so the save
    /// is not reported as an external change.
    void noteSaved(const std::shared_ptr<Document>& doc);
    void noteSaved(const std::shared_ptr<AssemblyDocument>& doc);

    // --- Cross-document references ---

    /// Resolve a component instance against its part file.
    ///
    /// Lightweight: fills `instance.cachedMesh` from the part's tessellation
    /// cache (or, if the part is already open with a built solid, from that
    /// solid) without loading the feature tree.
    /// Resolved: opens the full part document (deduplicated), rebuilding its
    /// model if needed, and fills both `resolvedPart` and `cachedMesh`.
    ///
    /// `assemblyDir` is used to resolve relative part paths.
    /// Returns true on success.
    bool resolveComponent(ComponentInstance& instance, ComponentState mode,
                          const std::string& assemblyDir = {});

    // --- External change notifications ---

    void setExternalChangeCallback(ExternalChangeCallback cb) { m_changeCallback = std::move(cb); }

    /// Compare on-disk modification times of all watched files against the
    /// recorded ones. Returns the changed paths and invokes the callback for
    /// each. The recorded times are updated so a change reports once.
    std::vector<std::string> pollExternalChanges();

private:
    static std::string canonicalPath(const std::string& path);
    void watchFile(const std::string& canonical);
    void unwatchFile(const std::string& canonical);

    PartLoader m_partLoader;
    MeshLoader m_meshLoader;
    AssemblyLoader m_assemblyLoader;
    ExternalChangeCallback m_changeCallback;

    std::vector<std::shared_ptr<Document>> m_documents;
    std::vector<std::shared_ptr<AssemblyDocument>> m_assemblies;
    std::map<std::string, std::weak_ptr<Document>> m_documentsByPath;
    std::map<std::string, std::weak_ptr<AssemblyDocument>> m_assembliesByPath;
    std::map<std::string, std::filesystem::file_time_type> m_watchedFiles;
};

}  // namespace hz::doc
