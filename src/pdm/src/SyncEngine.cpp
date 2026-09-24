#include "horizon/pdm/SyncEngine.h"

#include <algorithm>
#include <filesystem>
#include <set>

namespace hz::pdm {

namespace fs = std::filesystem;

namespace {

constexpr char kArchiveSuffix[] = ".hzarchive";

/// Whether a local and a remote revision, each already verified against its
/// own hash, hold the same content. Hashes of one kind are compared directly.
/// When one side still has the FNV-1a hash of a pre-Phase-119 archive, the
/// remote bytes must match the local hash too.
bool sameContent(const std::string& localHash, const std::string& remoteHash,
                 const std::string& remoteContent) {
    if (localHash.size() == remoteHash.size()) return localHash == remoteHash;
    return RevisionArchive::hashMatches(remoteContent, localHash);
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
    if (archive.isCorrupt()) return -1;
    return archive.latestIndex() + 1;
}

bool FileSystemEndpoint::fetchRevision(const std::string& docId, int index, RevisionInfo& info,
                                       std::string& content) {
    RevisionArchive archive(archiveDir(docId));
    archive.load();
    if (index < 0 || index > archive.latestIndex()) return false;
    info = archive.history()[static_cast<size_t>(index)];
    // Bytes that do not match their hash are still returned: the engine
    // checks them, and reports corruption rather than a transport failure.
    return archive.read(index, content) != RevisionArchive::ReadStatus::Missing;
}

bool FileSystemEndpoint::pushRevision(const std::string& docId, const RevisionInfo& info,
                                      const std::string& content) {
    if (!RevisionArchive::hashMatches(content, info.contentHash)) return false;

    std::error_code ec;
    fs::create_directories(m_root, ec);
    RevisionArchive archive(archiveDir(docId));
    archive.load();
    if (archive.isCorrupt()) return false;

    // The revision must land as the next one: commit() skips content equal
    // to the head, and another writer may have appended meanwhile.
    const int expected = archive.latestIndex() + 1;
    if (archive.commit(content, info.author, info.message) != expected) return false;

    // Read it back, so a write the share tore or dropped is not reported as
    // pushed.
    std::string stored;
    return archive.read(expected, stored) == RevisionArchive::ReadStatus::Ok;
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
    // A history that cannot be trusted on either side is left alone: it
    // would read as empty, and replicating into it would overwrite it.
    if (local.isCorrupt() || remoteCount < 0) {
        report.conflicts.push_back("corrupt:" + docId);
        return;
    }

    // Histories must agree on their shared prefix (append-only invariant):
    // any shared index with a different content hash is a divergence, and the
    // document is skipped on both sides — sync never merges. Each side's bytes
    // are checked against their own hash on the way, so a revision that rotted
    // after it was synced is reported as corrupt, not passed over because the
    // two manifests still agree.
    const int shared = std::min(localCount, remoteCount);
    for (int i = 0; i < shared; ++i) {
        RevisionInfo remoteInfo;
        std::string remoteContent;
        if (!m_endpoint.fetchRevision(docId, i, remoteInfo, remoteContent)) {
            report.ok = false;
            return;
        }
        std::string localContent;
        const RevisionArchive::ReadStatus localRead = local.read(i, localContent);
        if (localRead == RevisionArchive::ReadStatus::Missing) {
            report.ok = false;
            return;
        }
        if (localRead == RevisionArchive::ReadStatus::Corrupt ||
            !RevisionArchive::hashMatches(remoteContent, remoteInfo.contentHash)) {
            report.conflicts.push_back("corrupt:" + docId);
            return;
        }
        if (!sameContent(local.history()[static_cast<size_t>(i)].contentHash,
                         remoteInfo.contentHash, remoteContent)) {
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
            // Only content that matches its hash leaves this machine.
            std::string content;
            const RevisionArchive::ReadStatus status = local.read(i, content);
            if (status == RevisionArchive::ReadStatus::Corrupt) {
                report.conflicts.push_back("corrupt:" + docId);
                return;
            }
            if (status != RevisionArchive::ReadStatus::Ok ||
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
            if (!RevisionArchive::hashMatches(content, info.contentHash)) {
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
