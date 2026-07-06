#include "horizon/pdm/SyncEngine.h"

#include <algorithm>
#include <filesystem>
#include <set>

namespace hz::pdm {

namespace fs = std::filesystem;

namespace {

constexpr char kArchiveSuffix[] = ".hzarchive";

/// docIds become filesystem path components: restrict them to a safe
/// character set so endpoint- or caller-supplied ids cannot escape the
/// vault root ("..", separators, drive letters).
bool isValidDocId(const std::string& docId) {
    if (docId.empty() || docId == "." || docId == "..") return false;
    for (const char c : docId) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                        (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-';
        if (!ok) return false;
    }
    return true;
}

/// docIds of `<docId>.hzarchive` directories under @p root.
std::vector<std::string> listArchives(const std::string& root) {
    std::vector<std::string> ids;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        if (!entry.is_directory()) continue;
        const std::string name = entry.path().filename().string();
        if (name.size() > sizeof(kArchiveSuffix) - 1 &&
            name.compare(name.size() - (sizeof(kArchiveSuffix) - 1), sizeof(kArchiveSuffix) - 1,
                         kArchiveSuffix) == 0) {
            ids.push_back(name.substr(0, name.size() - (sizeof(kArchiveSuffix) - 1)));
        }
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

}  // namespace

// ---------------------------------------------------------------------------
// FileSystemEndpoint
// ---------------------------------------------------------------------------

FileSystemEndpoint::FileSystemEndpoint(std::string rootDir) : m_root(std::move(rootDir)) {}

std::string FileSystemEndpoint::archiveDir(const std::string& docId) const {
    return (fs::path(m_root) / (docId + kArchiveSuffix)).string();
}

std::vector<std::string> FileSystemEndpoint::listDocuments() {
    return listArchives(m_root);
}

int FileSystemEndpoint::revisionCount(const std::string& docId) {
    RevisionArchive archive(archiveDir(docId));
    archive.load();
    return archive.latestIndex() + 1;
}

bool FileSystemEndpoint::fetchRevision(const std::string& docId, int index, RevisionInfo& info,
                                       std::string& content) {
    RevisionArchive archive(archiveDir(docId));
    archive.load();
    if (index < 0 || index > archive.latestIndex()) return false;
    info = archive.history()[static_cast<size_t>(index)];
    return archive.contentAt(index, content);
}

bool FileSystemEndpoint::pushRevision(const std::string& docId, const RevisionInfo& info,
                                      const std::string& content) {
    std::error_code ec;
    fs::create_directories(m_root, ec);
    RevisionArchive archive(archiveDir(docId));
    archive.load();
    return archive.commit(content, info.author, info.message) >= 0;
}

// ---------------------------------------------------------------------------
// SyncEngine
// ---------------------------------------------------------------------------

SyncEngine::SyncEngine(std::string localRoot, SyncEndpoint& endpoint, VaultManifest* remoteLocks,
                       std::string user)
    : m_localRoot(std::move(localRoot)),
      m_endpoint(endpoint),
      m_remoteLocks(remoteLocks),
      m_user(std::move(user)) {}

std::vector<std::string> SyncEngine::listLocalDocuments() const {
    return listArchives(m_localRoot);
}

SyncReport SyncEngine::sync() {
    SyncReport report;
    std::set<std::string> docs;
    for (const auto& id : listLocalDocuments()) docs.insert(id);
    for (const auto& id : m_endpoint.listDocuments()) docs.insert(id);
    for (const auto& id : docs) syncOne(id, report);
    return report;
}

SyncReport SyncEngine::syncDocument(const std::string& docId) {
    SyncReport report;
    syncOne(docId, report);
    return report;
}

void SyncEngine::syncOne(const std::string& docId, SyncReport& report) {
    if (!isValidDocId(docId)) {
        report.conflicts.push_back("invalid:" + docId);
        return;
    }

    RevisionArchive local((fs::path(m_localRoot) / (docId + kArchiveSuffix)).string());
    local.load();
    const int localCount = local.latestIndex() + 1;
    const int remoteCount = m_endpoint.revisionCount(docId);

    // Histories must agree on their shared prefix (append-only invariant):
    // any shared index with a different content hash is a divergence, and the
    // document is skipped on both sides — sync never merges.
    const int shared = std::min(localCount, remoteCount);
    for (int i = 0; i < shared; ++i) {
        RevisionInfo remoteInfo;
        std::string remoteContent;
        if (!m_endpoint.fetchRevision(docId, i, remoteInfo, remoteContent)) {
            report.ok = false;
            return;
        }
        if (remoteInfo.contentHash != local.history()[static_cast<size_t>(i)].contentHash) {
            report.conflicts.push_back(docId);
            return;
        }
    }

    if (localCount > remoteCount) {
        // Push the local tail — unless the remote lock belongs to someone else.
        if (m_remoteLocks != nullptr && !m_user.empty() &&
            m_remoteLocks->isLockedByOther(docId, m_user)) {
            report.conflicts.push_back("locked:" + docId);
            return;
        }
        for (int i = remoteCount; i < localCount; ++i) {
            // Guard against a concurrent pusher: the remote must still be
            // exactly where we expect before each append. This narrows (it
            // cannot fully close) the shared-folder race window — check-out
            // locking via the VaultManifest is the intended way to serialize
            // multi-machine writes to one document.
            if (m_endpoint.revisionCount(docId) != i) {
                report.conflicts.push_back("raced:" + docId);
                return;
            }
            std::string content;
            if (!local.contentAt(i, content) ||
                !m_endpoint.pushRevision(docId, local.history()[static_cast<size_t>(i)], content)) {
                report.ok = false;
                return;
            }
            ++report.pushed;
        }
    } else if (remoteCount > localCount) {
        // Fetch the remote tail. Replication re-commits, so indexes and
        // content hashes are preserved; timestamps record replication time.
        for (int i = localCount; i < remoteCount; ++i) {
            RevisionInfo info;
            std::string content;
            if (!m_endpoint.fetchRevision(docId, i, info, content)) {
                report.ok = false;
                return;
            }
            // A torn/partial remote read must not enter local history:
            // the fetched bytes have to hash to what the remote manifest
            // declared.
            if (RevisionArchive::hashContent(content) != info.contentHash) {
                report.conflicts.push_back("corrupt:" + docId);
                return;
            }
            if (local.commit(content, info.author, info.message) < 0) {
                report.ok = false;
                return;
            }
            // RevisionArchive::commit dedupes content identical to the head:
            // a remote history containing consecutive duplicates (external
            // tooling, partially synced shares) would silently misalign every
            // following index and permanently deadlock the document in
            // conflict. Detect the skip and stop cleanly instead.
            if (local.latestIndex() != i) {
                report.conflicts.push_back("corrupt:" + docId);
                return;
            }
            ++report.fetched;
        }
    }
}

}  // namespace hz::pdm
