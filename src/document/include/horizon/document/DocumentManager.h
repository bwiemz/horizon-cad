#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <set>
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

    /// Create a new, unsaved document of the given type (Drawing or Part),
    /// in the unit for new documents.
    std::shared_ptr<Document> newDocument(DocumentType type);

    /// Create a new, unsaved assembly document, in the unit for new
    /// documents.
    std::shared_ptr<AssemblyDocument> newAssembly();

    /// The unit a new document or assembly is made in (Phase 154): the
    /// user's preference. One read from a file is in the file's.
    void setNewDocumentUnit(math::LengthUnit unit) { m_newUnit = unit; }
    math::LengthUnit newDocumentUnit() const { return m_newUnit; }

    /// Open a part/drawing file. If the file is already open, returns the
    /// existing instance (deduplicated by canonical path). Returns nullptr
    /// on load failure or when no part loader is set.
    std::shared_ptr<Document> openPart(const std::string& path);

    /// Register @p loaded, read from @p path elsewhere (on a worker), as
    /// openPart registers what it reads. If @p path is open already (opened
    /// meanwhile), that document is returned and @p loaded is dropped.
    /// Returns nullptr for a null @p loaded.
    std::shared_ptr<Document> adoptPart(const std::string& path, std::shared_ptr<Document> loaded);
    /// Register @p document, made or imported elsewhere (a DXF read on a
    /// worker), as newDocument registers the documents it makes.
    void adoptDocument(std::shared_ptr<Document> document);
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
    /// model if needed, and fills both `resolvedPart` and `cachedMesh`. A
    /// part opened only for components is held by them alone: it is found by
    /// its path while one holds it, and released when none does (it is not
    /// among documents()).
    ///
    /// Every instance of a part shares one mesh, tessellated or read once
    /// while any holds it: each had its own.
    ///
    /// The part's file is watched from then on (pollExternalChanges), even
    /// after a tab of it closes: an assembly placing it shows it changed.
    ///
    /// `assemblyDir` is used to resolve relative part paths.
    /// Returns true on success.
    ///
    /// A component whose file is an assembly (Phase 159) is read from its
    /// file for its components alone (`resolvedAssembly`), each of them
    /// resolved the same way, recursively: its mesh is theirs merged
    /// (AssemblyDocument::drawingMesh), and Resolved, its solid theirs
    /// gathered (`assemblySolid`). It is refused, and why said in @p why,
    /// when it places, at any depth, an assembly it is inside: one of
    /// @p within (canonical paths; the assembly it is placed in, if saved),
    /// or itself. A component of it that cannot be read is left out.
    bool resolveComponent(ComponentInstance& instance, ComponentState mode,
                          const std::string& assemblyDir = {}, std::string* why = nullptr,
                          const std::vector<std::string>& within = {});

    /// Whether @p a and @p b name the same file (canonically).
    static bool samePath(const std::string& a, const std::string& b);

    /// Watch @p path for changes on disk from now on, for as long as the
    /// manager lives (a part a drawing draws: pollExternalChanges reports it).
    void watch(const std::string& path);

    /// Forget the part at @p path read for components alone, so the next
    /// resolve reads it from its file again (it changed there). A part open
    /// in a tab is kept: its tab's document is the part. Components hold
    /// what they have until they let go of it.
    void releasePart(const std::string& path);

    // --- External change notifications ---

    void setExternalChangeCallback(ExternalChangeCallback cb) { m_changeCallback = std::move(cb); }

    /// Compare on-disk modification times of all watched files against the
    /// recorded ones. Returns the changed paths and invokes the callback for
    /// each. The recorded times are updated so a change reports once.
    std::vector<std::string> pollExternalChanges();

private:
    /// Open @p path, or find it open; @p keep registers it among
    /// documents() (a tab's), where a component's is found by path only.
    std::shared_ptr<Document> openPart(const std::string& path, bool keep);
    std::shared_ptr<Document> registerPart(const std::string& path,
                                           std::shared_ptr<Document> document, bool keep);
    /// The mesh made for @p key at @p version, if one still holds it; else
    /// what @p make makes, kept (weakly) for the next.
    std::shared_ptr<const geo::MeshData> sharedMesh(
        const std::string& key, std::uint64_t version,
        const std::function<std::shared_ptr<const geo::MeshData>()>& make);

    /// resolveComponent for an assembly file at @p fullPath (@p key its
    /// canonical path). @p cycle is set when it fails for placing an
    /// assembly it is inside.
    bool resolveSubassembly(ComponentInstance& instance, ComponentState mode,
                            const std::string& fullPath, const std::string& key,
                            const std::vector<std::string>& within, std::string* why, bool* cycle);

public:
    /// @p path made canonical, as the manager keys files (a path that does
    /// not exist, normalized).
    static std::string canonicalPath(const std::string& path);

private:
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
    /// The files components were resolved from: never unwatched.
    std::set<std::string> m_placedFiles;
    struct SharedMesh {
        std::weak_ptr<const geo::MeshData> mesh;
        std::uint64_t version = 0;
    };
    std::map<std::string, SharedMesh> m_meshes;
    math::LengthUnit m_newUnit = math::LengthUnit::Millimetre;
};

}  // namespace hz::doc
