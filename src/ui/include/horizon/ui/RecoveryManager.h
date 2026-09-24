#pragma once

#include <QDateTime>
#include <QLockFile>
#include <QString>
#include <memory>
#include <vector>

namespace hz::doc {
class AssemblyDocument;
class Document;
}  // namespace hz::doc

namespace hz::ui {

/// Autosave snapshots and crash recovery.
///
/// Each running session owns `<root>/<session id>/`, held by a lock file for as
/// long as the session lives. Modified documents are written there as native
/// snapshots, each with a sidecar recording where it came from. A clean exit
/// removes the directory. A directory whose lock is stale — its process is
/// gone — was left by a session that crashed, and its snapshots are offered
/// back on the next start.
class RecoveryManager {
public:
    /// A document a crashed session left behind.
    struct Entry {
        QString snapshotPath;  ///< The native file to load.
        QString originalPath;  ///< Where the document was saved, or empty if never.
        QString title;         ///< Its tab caption.
        QString type;          ///< "hcad", "hzpart" or "hzasm".
        QDateTime savedAt;     ///< When the snapshot was taken.
    };

    /// Takes a session directory under `root` (created if needed) and locks it.
    explicit RecoveryManager(const QString& root);

    /// A clean exit: this session's snapshots are no longer needed.
    ~RecoveryManager();

    RecoveryManager(const RecoveryManager&) = delete;
    RecoveryManager& operator=(const RecoveryManager&) = delete;

    /// False when the session directory could not be created or locked; this
    /// session then writes no snapshots (autosave is best effort, never an
    /// error), though it can still recover what earlier sessions left.
    bool isActive() const { return m_active; }

    const QString& sessionDirectory() const { return m_sessionDir; }

    /// Write `doc` as the snapshot for `key` (one per open document). DXF
    /// drawings are snapshotted in native format, which loses nothing.
    bool snapshot(quint64 key, const doc::Document& doc, const QString& title,
                  const QString& originalPath);
    bool snapshot(quint64 key, const doc::AssemblyDocument& assembly, const QString& title,
                  const QString& originalPath);

    /// The document was saved or closed: its snapshot is obsolete.
    void remove(quint64 key);

    /// Snapshots left by sessions that did not exit cleanly. Their directories
    /// are locked by this manager from here on, so two sessions starting at
    /// once do not both recover them.
    std::vector<Entry> claimOrphans();

    /// Delete the directories claimed by claimOrphans() — after their
    /// documents were recovered, or when the user discards them.
    void discardClaimedOrphans();

private:
    bool writeSnapshot(quint64 key, const std::string& json, const QString& type,
                       const QString& title, const QString& originalPath);

    QString m_root;
    QString m_sessionDir;
    std::unique_ptr<QLockFile> m_lock;
    bool m_active = false;
    std::vector<std::unique_ptr<QLockFile>> m_claimedLocks;
    std::vector<QString> m_claimedDirs;
};

}  // namespace hz::ui
