#pragma once

#include <string>

namespace hz::pdm {

/// Whether @p docId can name a document in a vault: letters, digits, '.', '_'
/// and '-', and not "." or "..". A document id becomes a file or directory
/// name, so an id must not be able to leave the vault (separators, "..",
/// drive letters).
bool isValidDocId(const std::string& docId);

/// The lock state of one document in the vault.
struct LockState {
    bool locked = false;    ///< whether the document is checked out
    std::string owner;      ///< the user holding the lock ("" if free or unreadable)
    std::string timestamp;  ///< ISO-8601 UTC time the lock was taken
    /// The document is locked, but by whom cannot be read: its lock file is
    /// unreadable, empty or malformed, or the lock directory cannot be read.
    /// Such a lock counts as held by someone else until it is broken.
    bool unreadable = false;
};

/// The check-out locks of a multi-user vault (Phase 60), kept in a shared
/// folder (NFS, SMB, a synced drive) with no server process.
///
/// Implements pessimistic locking: at most one user may hold a given document
/// at a time. Each checked-out document is one file, `<docId>.lock` in the lock
/// directory, recording who holds it and since when (Phase 119). Taking a lock
/// is creating that file exclusively, which the file system does as one step,
/// so of two users racing for a document exactly one gets it; the lock record
/// used before, a JSON map rewritten by whoever changed it last, let both win.
///
/// Fails closed. A lock that cannot be read counts as held by someone else, a
/// lock directory that cannot be read makes every document read as locked,
/// and an invalid document id (isValidDocId) is refused. A vault from before
/// Phase 119 kept its locks in one JSON file; a lock directory path that is a
/// file is such a vault, and all its documents read as locked, so none is
/// taken twice across the upgrade.
class VaultManifest {
public:
    /// Bind to the lock directory @p lockDir, created on the first check-out.
    explicit VaultManifest(std::string lockDir);

    /// Current lock state of @p docId, read fresh from disk.
    LockState status(const std::string& docId) const;

    /// The lock holder of @p docId, or "" if it is free or its lock is
    /// unreadable (status() tells the two apart).
    std::string lockOwner(const std::string& docId) const;

    /// Whether @p docId is locked by someone other than @p user, or by someone
    /// unknown.
    bool isLockedByOther(const std::string& docId, const std::string& user) const;

    /// Check out @p docId for @p user (a non-empty name). Succeeds (returns
    /// true) if the document was free and the lock was taken, or is already
    /// held by @p user; fails if anyone else holds it, its lock is unreadable,
    /// or the lock cannot be written.
    bool checkOut(const std::string& docId, const std::string& user);

    /// Check in (release) @p docId on behalf of @p user. Succeeds only if @p user
    /// currently holds the lock and it was removed.
    bool checkIn(const std::string& docId, const std::string& user);

    /// Force-release @p docId regardless of owner (administrative override),
    /// including a lock that cannot be read. Returns whether the document is
    /// now free.
    bool breakLock(const std::string& docId);

private:
    std::string lockPath(const std::string& docId) const;

    std::string m_dir;
};

}  // namespace hz::pdm
