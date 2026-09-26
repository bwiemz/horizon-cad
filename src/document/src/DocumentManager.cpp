#include "horizon/document/DocumentManager.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iterator>
#include <system_error>

#include "horizon/modeling/SolidTessellator.h"

namespace hz::doc {

namespace fs = std::filesystem;

std::string DocumentManager::canonicalPath(const std::string& path) {
    std::error_code ec;
    fs::path canonical = fs::weakly_canonical(fs::path(path), ec);
    if (ec) return path;
    return canonical.string();
}

bool DocumentManager::samePath(const std::string& a, const std::string& b) {
    return !a.empty() && !b.empty() && canonicalPath(a) == canonicalPath(b);
}

void DocumentManager::watch(const std::string& path) {
    const std::string key = canonicalPath(path);
    m_placedFiles.insert(key);
    if (m_watchedFiles.count(key) == 0) watchFile(key);
}

void DocumentManager::releasePart(const std::string& path) {
    const std::string key = canonicalPath(path);
    const auto it = m_documentsByPath.find(key);
    if (it == m_documentsByPath.end()) return;
    const auto open = it->second.lock();
    const bool inTab =
        open && std::find(m_documents.begin(), m_documents.end(), open) != m_documents.end();
    if (!inTab) m_documentsByPath.erase(it);
}

std::shared_ptr<Document> DocumentManager::newDocument(DocumentType type) {
    auto doc = std::make_shared<Document>();
    doc->setType(type);
    doc->setLengthUnit(m_newUnit);
    m_documents.push_back(doc);
    return doc;
}

std::shared_ptr<AssemblyDocument> DocumentManager::newAssembly() {
    auto doc = std::make_shared<AssemblyDocument>();
    doc->setLengthUnit(m_newUnit);
    m_assemblies.push_back(doc);
    return doc;
}

std::shared_ptr<Document> DocumentManager::openPart(const std::string& path) {
    return openPart(path, /*keep=*/true);
}

std::shared_ptr<Document> DocumentManager::openPart(const std::string& path, bool keep) {
    if (auto existing = findByPath(path)) return registerPart(path, std::move(existing), keep);
    if (!m_partLoader) return nullptr;
    auto doc = std::make_shared<Document>();
    if (!m_partLoader(path, *doc)) return nullptr;
    return registerPart(path, std::move(doc), keep);
}

std::shared_ptr<Document> DocumentManager::adoptPart(const std::string& path,
                                                     std::shared_ptr<Document> loaded) {
    if (auto existing = findByPath(path)) return registerPart(path, std::move(existing), true);
    return registerPart(path, std::move(loaded), true);
}

std::shared_ptr<Document> DocumentManager::registerPart(const std::string& path,
                                                        std::shared_ptr<Document> document,
                                                        bool keep) {
    if (!document) return nullptr;
    const std::string key = canonicalPath(path);
    auto& known = m_documentsByPath[key];
    if (known.lock() != document) {
        // Newly read: its file is watched, a component's part too (a change
        // to it is worth knowing of while an assembly shows it).
        document->setFilePath(path);
        document->setDirty(false);
        known = document;
        watchFile(key);
    }
    // Kept (a tab's): among the documents. A part opened for components
    // before is kept once a tab opens it too.
    if (keep && std::find(m_documents.begin(), m_documents.end(), document) == m_documents.end()) {
        m_documents.push_back(document);
    }
    return document;
}

void DocumentManager::adoptDocument(std::shared_ptr<Document> document) {
    if (document) m_documents.push_back(std::move(document));
}

std::shared_ptr<AssemblyDocument> DocumentManager::openAssembly(const std::string& path) {
    const std::string key = canonicalPath(path);

    auto it = m_assembliesByPath.find(key);
    if (it != m_assembliesByPath.end()) {
        if (auto existing = it->second.lock()) return existing;
        m_assembliesByPath.erase(it);
    }

    if (!m_assemblyLoader) return nullptr;

    auto doc = std::make_shared<AssemblyDocument>();
    if (!m_assemblyLoader(path, *doc)) return nullptr;
    doc->setFilePath(path);
    doc->setDirty(false);

    m_assemblies.push_back(doc);
    m_assembliesByPath[key] = doc;
    watchFile(key);
    return doc;
}

namespace {

// Erase every registry entry that resolves to `target` (identity check —
// the same document may be registered under a stale path after Save As,
// and a path key may have been overwritten by another document). Also
// removes expired entries in passing. Returns the erased keys.
template <typename T>
std::vector<std::string> eraseEntriesFor(std::map<std::string, std::weak_ptr<T>>& registry,
                                         const std::shared_ptr<T>& target) {
    std::vector<std::string> erased;
    for (auto it = registry.begin(); it != registry.end();) {
        auto locked = it->second.lock();
        if (!locked || locked == target) {
            erased.push_back(it->first);
            it = registry.erase(it);
        } else {
            ++it;
        }
    }
    return erased;
}

}  // namespace

bool DocumentManager::closeDocument(const std::shared_ptr<Document>& doc) {
    auto it = std::find(m_documents.begin(), m_documents.end(), doc);
    if (it == m_documents.end()) return false;
    m_documents.erase(it);

    for (const auto& key : eraseEntriesFor(m_documentsByPath, doc)) {
        unwatchFile(key);
    }
    return true;
}

bool DocumentManager::closeAssembly(const std::shared_ptr<AssemblyDocument>& doc) {
    auto it = std::find(m_assemblies.begin(), m_assemblies.end(), doc);
    if (it == m_assemblies.end()) return false;
    m_assemblies.erase(it);

    for (const auto& key : eraseEntriesFor(m_assembliesByPath, doc)) {
        unwatchFile(key);
    }
    return true;
}

std::shared_ptr<Document> DocumentManager::findByPath(const std::string& path) const {
    auto it = m_documentsByPath.find(canonicalPath(path));
    if (it == m_documentsByPath.end()) return nullptr;
    return it->second.lock();
}

void DocumentManager::noteSaved(const std::shared_ptr<Document>& doc) {
    if (!doc || doc->filePath().empty()) return;
    // Drop registrations under any previous path (Save As), then register
    // the current one.
    for (const auto& key : eraseEntriesFor(m_documentsByPath, doc)) {
        unwatchFile(key);
    }
    const std::string key = canonicalPath(doc->filePath());
    m_documentsByPath[key] = doc;
    watchFile(key);
}

void DocumentManager::noteSaved(const std::shared_ptr<AssemblyDocument>& doc) {
    if (!doc || doc->filePath().empty()) return;
    for (const auto& key : eraseEntriesFor(m_assembliesByPath, doc)) {
        unwatchFile(key);
    }
    const std::string key = canonicalPath(doc->filePath());
    m_assembliesByPath[key] = doc;
    watchFile(key);
}

bool DocumentManager::resolveComponent(ComponentInstance& instance, ComponentState mode,
                                       const std::string& assemblyDir, std::string* why,
                                       const std::vector<std::string>& within) {
    fs::path partPath(instance.partPath);
    if (partPath.is_relative() && !assemblyDir.empty()) {
        partPath = fs::path(assemblyDir) / partPath;
    }
    const std::string fullPath = partPath.string();

    const std::string key = canonicalPath(fullPath);
    // Watched while the manager lives: a tab of the part closing does not
    // stop an assembly placing it from seeing it change.
    m_placedFiles.insert(key);
    if (instance.isAssembly()) {
        if (m_watchedFiles.count(key) == 0) watchFile(key);
        bool cycle = false;
        return resolveSubassembly(instance, mode, fullPath, key, within, why, &cycle);
    }
    // A mesh from a part's model is as new as its last build, of that
    // document (the file read again is another, built as often maybe); one
    // from the file (its cache, or a model built from it apart) as the file.
    const auto modelMesh = [this, &key](const Document& part) {
        return sharedMesh(key + "#model/" + std::to_string(part.serial()), part.builds(), [&part] {
            return std::make_shared<const geo::MeshData>(
                model::SolidTessellator::tessellate(*part.solid()));
        });
    };
    std::error_code timeError;
    const auto fileTime = fs::last_write_time(fullPath, timeError);
    const auto fileVersion =
        timeError ? 0u : static_cast<std::uint64_t>(fileTime.time_since_epoch().count());

    if (mode == ComponentState::Resolved) {
        auto part = openPart(fullPath, /*keep=*/false);
        if (!part) return false;
        if (!part->solid() && part->featureTree().featureCount() > 0) {
            part->rebuildModel();
        }
        instance.resolvedPart = part;
        if (part->solid()) instance.cachedMesh = modelMesh(*part);
        instance.state = ComponentState::Resolved;
        return true;
    }

    // Lightweight resolution: never load (or keep) the feature tree.
    // Demoting a Resolved instance releases its reference to the full part
    // document — Lightweight means "cached tessellation + transform only".
    instance.resolvedPart.reset();
    // Its file watched, as a resolved part's is: an assembly shows it, and
    // should show it changed.
    if (m_watchedFiles.count(key) == 0) watchFile(key);

    if (instance.cachedMesh) {
        instance.state = ComponentState::Lightweight;
        return true;
    }

    // If the part happens to be open already with a built solid, reuse it.
    if (auto open = findByPath(fullPath); open && open->solid()) {
        instance.cachedMesh = modelMesh(*open);
        instance.state = ComponentState::Lightweight;
        return true;
    }

    // From the file: its tessellation cache, or else the full part loaded
    // into a temporary document (not registered as open) and tessellated.
    const auto fromFile = [this, &fullPath]() -> std::shared_ptr<const geo::MeshData> {
        if (m_meshLoader) {
            if (auto mesh = m_meshLoader(fullPath)) return mesh;
        }
        if (m_partLoader) {
            Document temp;
            if (m_partLoader(fullPath, temp)) {
                temp.rebuildModel();
                if (temp.solid()) {
                    return std::make_shared<const geo::MeshData>(
                        model::SolidTessellator::tessellate(*temp.solid()));
                }
            }
        }
        return nullptr;
    };
    if (auto mesh = sharedMesh(key + "#file", fileVersion, fromFile)) {
        instance.cachedMesh = std::move(mesh);
        instance.state = ComponentState::Lightweight;
        return true;
    }
    return false;
}

bool DocumentManager::resolveSubassembly(ComponentInstance& instance, ComponentState mode,
                                         const std::string& fullPath, const std::string& key,
                                         const std::vector<std::string>& within, std::string* why,
                                         bool* cycle) {
    const auto nameOf = [](const std::string& path) { return fs::path(path).stem().string(); };
    // It places itself, at some depth: the chain of assemblies down to it.
    if (std::find(within.begin(), within.end(), key) != within.end()) {
        *cycle = true;
        if (why != nullptr) {
            std::string chain;
            const auto first = std::find(within.begin(), within.end(), key);
            for (auto it = first; it != within.end(); ++it) chain += nameOf(*it) + " > ";
            *why = nameOf(key) + " is placed inside itself (" + chain + nameOf(key) + ")";
        }
        return false;
    }
    if (!m_assemblyLoader) {
        if (why != nullptr) *why = nameOf(key) + " is an assembly, and none can be read here";
        return false;
    }
    auto sub = std::make_shared<AssemblyDocument>();
    if (!m_assemblyLoader(fullPath, *sub)) {
        if (why != nullptr) *why = nameOf(key) + " could not be read";
        return false;
    }
    if (sub->filePath().empty()) sub->setFilePath(fullPath);
    std::vector<std::string> inside = within;
    inside.push_back(key);
    const std::string dir = fs::path(fullPath).parent_path().string();
    for (auto& child : sub->components()) {
        if (child.suppressed) continue;
        fs::path childPath(child.partPath);
        if (childPath.is_relative()) childPath = fs::path(dir) / childPath;
        const std::string childKey = canonicalPath(childPath.string());
        m_placedFiles.insert(childKey);
        if (child.isAssembly()) {
            if (m_watchedFiles.count(childKey) == 0) watchFile(childKey);
            bool childCycle = false;
            if (!resolveSubassembly(child, mode, childPath.string(), childKey, inside, why,
                                    &childCycle) &&
                childCycle) {
                *cycle = true;
                return false;
            }
            continue;
        }
        // A part it places that cannot be read is left out, as an
        // assembly's own are.
        resolveComponent(child, mode, dir);
    }
    instance.resolvedAssembly = sub;
    instance.resolvedPart.reset();
    instance.cachedMesh = sub->drawingMesh();
    instance.assemblySolid.reset();
    if (mode == ComponentState::Resolved) {
        instance.assemblySolid =
            sub->drawingSolid([](const ComponentInstance& c) { return c.ownSolid(); });
    }
    instance.state = mode;
    if (instance.cachedMesh == nullptr && why != nullptr) {
        *why = nameOf(key) + " places nothing that can be shown";
    }
    return instance.cachedMesh != nullptr;
}

std::shared_ptr<const geo::MeshData> DocumentManager::sharedMesh(
    const std::string& key, std::uint64_t version,
    const std::function<std::shared_ptr<const geo::MeshData>()>& make) {
    // Let go of the entries no instance holds any more.
    for (auto it = m_meshes.begin(); it != m_meshes.end();) {
        it = it->second.mesh.expired() ? m_meshes.erase(it) : std::next(it);
    }
    if (const auto found = m_meshes.find(key); found != m_meshes.end()) {
        if (auto mesh = found->second.mesh.lock(); mesh && found->second.version == version) {
            return mesh;
        }
    }
    auto mesh = make();
    if (mesh) m_meshes[key] = SharedMesh{mesh, version};
    return mesh;
}

void DocumentManager::watchFile(const std::string& canonical) {
    std::error_code ec;
    auto time = fs::last_write_time(canonical, ec);
    if (!ec) m_watchedFiles[canonical] = time;
}

void DocumentManager::unwatchFile(const std::string& canonical) {
    if (m_placedFiles.count(canonical) == 0) m_watchedFiles.erase(canonical);
}

std::vector<std::string> DocumentManager::pollExternalChanges() {
    std::vector<std::string> changed;
    for (auto& [path, recordedTime] : m_watchedFiles) {
        std::error_code ec;
        auto currentTime = fs::last_write_time(path, ec);
        if (ec) continue;
        if (currentTime != recordedTime) {
            recordedTime = currentTime;
            changed.push_back(path);
        }
    }
    if (m_changeCallback) {
        for (const auto& path : changed) m_changeCallback(path);
    }
    return changed;
}

}  // namespace hz::doc
