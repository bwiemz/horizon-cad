#pragma once

#include <string>
#include <vector>

namespace hz::pdm {

/// Metadata for one committed revision of a document.
struct RevisionInfo {
    int index = 0;            ///< 0-based revision number (commit order)
    std::string timestamp;    ///< ISO-8601 UTC time of the commit
    std::string author;       ///< who made the change
    std::string message;      ///< commit comment
    std::string contentHash;  ///< stable hash of the content (change detection)
};

/// A file-system-backed revision store for a single document — the local
/// version-control archive (Phase 59 PDM).
///
/// Each commit snapshots the document's serialized content (an opaque string —
/// typically the native JSON form of the feature tree + parameters) as a blob
/// alongside a JSON manifest of revision metadata, in an `.hzarchive` directory.
/// A commit whose content is byte-identical to the current head is a no-op and
/// returns the existing head index, so repeated saves without edits don't grow
/// the history.
class RevisionArchive {
public:
    /// Bind to (but do not yet read) the archive directory @p archiveDir.
    explicit RevisionArchive(std::string archiveDir);

    /// Load an existing archive's manifest, if the directory contains one.
    /// Returns true if a manifest was read; false (leaving the archive empty) if
    /// none exists yet. A malformed manifest is treated as empty.
    bool load();

    /// Snapshot @p content as a new revision. Creates the archive directory if
    /// needed and persists both the blob and the updated manifest. Returns the
    /// new revision index, or the current head index if @p content is unchanged,
    /// or -1 on I/O failure.
    int commit(const std::string& content, const std::string& author, const std::string& message);

    /// All revisions in commit order (oldest first).
    const std::vector<RevisionInfo>& history() const { return m_revisions; }

    /// Index of the newest revision, or -1 if the archive is empty.
    int latestIndex() const { return static_cast<int>(m_revisions.size()) - 1; }

    /// Read back the content blob of revision @p index into @p out. Returns false
    /// if the index is out of range or the blob cannot be read.
    bool contentAt(int index, std::string& out) const;

    /// Stable 64-bit FNV-1a content hash, as a 16-char lowercase hex string.
    static std::string hashContent(const std::string& content);

private:
    std::string m_dir;
    std::vector<RevisionInfo> m_revisions;

    std::string manifestPath() const;
    std::string blobPath(int index) const;
    bool writeManifest() const;
};

}  // namespace hz::pdm
