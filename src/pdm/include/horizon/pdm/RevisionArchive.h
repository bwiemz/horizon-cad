#pragma once

#include <string>
#include <vector>

namespace hz::pdm {

/// Metadata for one committed revision of a document.
struct RevisionInfo {
    int index = 0;          ///< 0-based revision number (commit order)
    std::string timestamp;  ///< ISO-8601 UTC time of the commit
    std::string author;     ///< who made the change
    std::string message;    ///< commit comment
    /// The content's hash: SHA-256, 64 hex digits. A revision committed
    /// before Phase 119 keeps the 64-bit FNV-1a hash it was written with, 16
    /// hex digits.
    std::string contentHash;
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
///
/// The archive fails closed (Phase 119). Content is verified against the hash
/// its manifest records whenever it is read, and an archive whose manifest
/// cannot be trusted reads as corrupt, not as empty, and refuses commits, so a
/// damaged history is reported instead of silently overwritten.
class RevisionArchive {
public:
    /// What reading a revision's content found.
    enum class ReadStatus {
        Ok,       ///< the content, matching its recorded hash
        Missing,  ///< no such revision, or its blob cannot be read
        Corrupt,  ///< the blob was read but does not match its recorded hash
    };

    /// Bind to (but do not yet read) the archive directory @p archiveDir.
    explicit RevisionArchive(std::string archiveDir);

    /// Read the archive's manifest from disk. Returns true if one was read.
    /// False leaves the archive empty: either it has no manifest yet, or the
    /// archive cannot be trusted, which isCorrupt() then reports.
    bool load();

    /// Whether the archive on disk cannot be trusted: its manifest exists but
    /// cannot be read, is not valid JSON, or does not describe a sequence of
    /// revisions numbered from 0 with well-formed hashes; or revision blobs
    /// exist with no manifest. Set by load().
    bool isCorrupt() const { return m_corrupt; }

    /// Snapshot @p content as a new revision. Re-reads the manifest first, so
    /// the revision is appended to the history on disk even if another handle
    /// committed since this one loaded. Creates the archive directory if
    /// needed and replaces the blob and manifest atomically. Returns the new
    /// revision index; the current head index if @p content is unchanged; or
    /// -1 if the archive is corrupt, its head cannot be read to compare
    /// against, or the write fails.
    int commit(const std::string& content, const std::string& author, const std::string& message);

    /// All revisions in commit order (oldest first).
    const std::vector<RevisionInfo>& history() const { return m_revisions; }

    /// Index of the newest revision, or -1 if the archive is empty.
    int latestIndex() const { return static_cast<int>(m_revisions.size()) - 1; }

    /// Read the content of revision @p index into @p out and check it against
    /// the hash the manifest records. @p out holds the bytes read whether they
    /// match (Ok) or not (Corrupt), and is empty when the status is Missing.
    ReadStatus read(int index, std::string& out) const;

    /// read(), true only for content that matches its recorded hash.
    bool contentAt(int index, std::string& out) const { return read(index, out) == ReadStatus::Ok; }

    /// The hash commit() records for @p content: SHA-256, 64 lowercase hex
    /// digits.
    static std::string hashContent(const std::string& content);

    /// Whether @p content matches @p recordedHash, a hash an archive recorded:
    /// SHA-256 (64 hex digits), or the FNV-1a of an archive written before
    /// Phase 119 (16 hex digits). False for anything else.
    static bool hashMatches(const std::string& content, const std::string& recordedHash);

private:
    std::string m_dir;
    std::vector<RevisionInfo> m_revisions;
    bool m_corrupt = false;

    std::string manifestPath() const;
    std::string blobPath(int index) const;
    bool writeManifest() const;
};

}  // namespace hz::pdm
