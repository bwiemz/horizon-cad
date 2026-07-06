#pragma once

#include <string>
#include <vector>

#include "horizon/pdm/RevisionArchive.h"
#include "horizon/pdm/VaultManifest.h"

namespace hz::pdm {

/// A replication target for vault revision histories (Phase 69, roadmap
/// §7.5). The first transport is a file-system directory (self-hosted shared
/// folder or mounted cloud drive); an HTTP endpoint against the roadmap's
/// REST vault server can implement the same interface later.
class SyncEndpoint {
public:
    virtual ~SyncEndpoint() = default;

    /// Document ids present on the endpoint.
    virtual std::vector<std::string> listDocuments() = 0;

    /// Number of revisions stored for @p docId (0 when absent).
    virtual int revisionCount(const std::string& docId) = 0;

    /// Fetch revision @p index of @p docId. False when absent/unreadable.
    virtual bool fetchRevision(const std::string& docId, int index, RevisionInfo& info,
                               std::string& content) = 0;

    /// Append one revision to @p docId's history. False on failure.
    virtual bool pushRevision(const std::string& docId, const RevisionInfo& info,
                              const std::string& content) = 0;
};

/// SyncEndpoint over a directory of `<docId>.hzarchive` stores — the same
/// layout the local vault uses, so any shared/synced folder becomes a remote.
class FileSystemEndpoint : public SyncEndpoint {
public:
    explicit FileSystemEndpoint(std::string rootDir);

    std::vector<std::string> listDocuments() override;
    int revisionCount(const std::string& docId) override;
    bool fetchRevision(const std::string& docId, int index, RevisionInfo& info,
                       std::string& content) override;
    bool pushRevision(const std::string& docId, const RevisionInfo& info,
                      const std::string& content) override;

private:
    std::string archiveDir(const std::string& docId) const;
    std::string m_root;
};

/// Result of one sync pass.
struct SyncReport {
    int pushed = 0;                      ///< revisions uploaded
    int fetched = 0;                     ///< revisions downloaded
    std::vector<std::string> conflicts;  ///< docIds left untouched
    bool ok = true;                      ///< false on transport failure
};

/// Local-first vault synchronization (Phase 69).
///
/// Horizon stays fully functional offline; sync replicates whole revision
/// histories between the local vault (a directory of `<docId>.hzarchive`
/// stores) and an endpoint. Replication is append-only and hash-verified:
/// histories may only extend one another. A divergence — the same revision
/// index carrying different content hashes on the two sides — is reported as
/// a conflict and that document is left untouched on BOTH sides; sync never
/// merges (pessimistic check-out locking is the mechanism that prevents
/// divergence in the first place).
///
/// When a remote VaultManifest is provided, pushes honour its locks: a
/// document checked out by another user is skipped and reported.
class SyncEngine {
public:
    /// @p localRoot   directory of local `<docId>.hzarchive` stores.
    /// @p endpoint    replication target (must outlive the engine).
    /// @p remoteLocks optional shared lock manifest; with @p user, pushes to
    ///                documents locked by others are refused.
    SyncEngine(std::string localRoot, SyncEndpoint& endpoint, VaultManifest* remoteLocks = nullptr,
               std::string user = {});

    /// Synchronize every document present on either side.
    SyncReport sync();

    /// Synchronize one document.
    SyncReport syncDocument(const std::string& docId);

private:
    void syncOne(const std::string& docId, SyncReport& report);
    std::vector<std::string> listLocalDocuments() const;

    std::string m_localRoot;
    SyncEndpoint& m_endpoint;
    VaultManifest* m_remoteLocks;
    std::string m_user;
};

}  // namespace hz::pdm
