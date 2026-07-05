#pragma once

#include <string>

namespace hz::pdm {

/// The lock state of one document in the vault.
struct LockState {
    bool locked = false;    ///< whether the document is checked out
    std::string owner;      ///< the user holding the lock ("" if free)
    std::string timestamp;  ///< ISO-8601 UTC time the lock was taken
};

/// A shared, file-system-backed lock manifest for a multi-user vault (Phase 60).
///
/// Implements pessimistic locking: at most one user may hold a given document at
/// a time. The manifest is a JSON map (document id -> lock state) living in a
/// shared folder (NFS/SMB/cloud-synced) — no server process. Every mutating
/// operation re-reads the manifest from disk before writing, so it respects
/// locks another user took since this handle last looked (last-writer-wins on
/// the file itself, which shared filesystems serialize).
class VaultManifest {
public:
    /// Bind to the manifest file at @p manifestPath (created on first check-out).
    explicit VaultManifest(std::string manifestPath);

    /// Current lock state of @p docId, read fresh from disk.
    LockState status(const std::string& docId) const;

    /// The lock holder of @p docId, or "" if it is free.
    std::string lockOwner(const std::string& docId) const;

    /// Whether @p docId is locked by someone other than @p user.
    bool isLockedByOther(const std::string& docId, const std::string& user) const;

    /// Check out @p docId for @p user. Succeeds (returns true) if the document is
    /// free or already held by @p user; fails if held by someone else. Persists
    /// the lock on success.
    bool checkOut(const std::string& docId, const std::string& user);

    /// Check in (release) @p docId on behalf of @p user. Succeeds only if @p user
    /// currently holds the lock. Persists the release on success.
    bool checkIn(const std::string& docId, const std::string& user);

    /// Force-release @p docId regardless of owner (administrative override).
    void breakLock(const std::string& docId);

private:
    std::string m_path;
};

}  // namespace hz::pdm
