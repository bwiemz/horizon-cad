#include "horizon/document/DocumentManager.h"

#include <algorithm>
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

std::shared_ptr<Document> DocumentManager::newDocument(DocumentType type) {
    auto doc = std::make_shared<Document>();
    doc->setType(type);
    m_documents.push_back(doc);
    return doc;
}

std::shared_ptr<AssemblyDocument> DocumentManager::newAssembly() {
    auto doc = std::make_shared<AssemblyDocument>();
    m_assemblies.push_back(doc);
    return doc;
}

std::shared_ptr<Document> DocumentManager::openPart(const std::string& path) {
    const std::string key = canonicalPath(path);

    auto it = m_documentsByPath.find(key);
    if (it != m_documentsByPath.end()) {
        if (auto existing = it->second.lock()) return existing;
        m_documentsByPath.erase(it);
    }

    if (!m_partLoader) return nullptr;

    auto doc = std::make_shared<Document>();
    if (!m_partLoader(path, *doc)) return nullptr;
    doc->setFilePath(path);
    doc->setDirty(false);

    m_documents.push_back(doc);
    m_documentsByPath[key] = doc;
    watchFile(key);
    return doc;
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
                                       const std::string& assemblyDir) {
    fs::path partPath(instance.partPath);
    if (partPath.is_relative() && !assemblyDir.empty()) {
        partPath = fs::path(assemblyDir) / partPath;
    }
    const std::string fullPath = partPath.string();

    if (mode == ComponentState::Resolved) {
        auto part = openPart(fullPath);
        if (!part) return false;
        if (!part->solid() && part->featureTree().featureCount() > 0) {
            part->rebuildModel();
        }
        instance.resolvedPart = part;
        if (part->solid()) {
            instance.cachedMesh = std::make_shared<render::MeshData>(
                model::SolidTessellator::tessellate(*part->solid()));
        }
        instance.state = ComponentState::Resolved;
        return true;
    }

    // Lightweight resolution: never load (or keep) the feature tree.
    // Demoting a Resolved instance releases its reference to the full part
    // document — Lightweight means "cached tessellation + transform only".
    instance.resolvedPart.reset();

    if (instance.cachedMesh) {
        instance.state = ComponentState::Lightweight;
        return true;
    }

    // If the part happens to be open already with a built solid, reuse it.
    if (auto open = findByPath(fullPath); open && open->solid()) {
        instance.cachedMesh =
            std::make_shared<render::MeshData>(model::SolidTessellator::tessellate(*open->solid()));
        instance.state = ComponentState::Lightweight;
        return true;
    }

    if (m_meshLoader) {
        if (auto mesh = m_meshLoader(fullPath)) {
            instance.cachedMesh = std::move(mesh);
            instance.state = ComponentState::Lightweight;
            return true;
        }
    }

    // Fallback: the part file has no tessellation cache. Load the full part
    // into a temporary document (not registered as open) and tessellate.
    if (m_partLoader) {
        Document temp;
        if (m_partLoader(fullPath, temp)) {
            temp.rebuildModel();
            if (temp.solid()) {
                instance.cachedMesh = std::make_shared<render::MeshData>(
                    model::SolidTessellator::tessellate(*temp.solid()));
                instance.state = ComponentState::Lightweight;
                return true;
            }
        }
    }
    return false;
}

void DocumentManager::watchFile(const std::string& canonical) {
    std::error_code ec;
    auto time = fs::last_write_time(canonical, ec);
    if (!ec) m_watchedFiles[canonical] = time;
}

void DocumentManager::unwatchFile(const std::string& canonical) {
    m_watchedFiles.erase(canonical);
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
