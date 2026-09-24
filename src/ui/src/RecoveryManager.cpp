#include "horizon/ui/RecoveryManager.h"

#include <spdlog/spdlog.h>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QUuid>
#include <algorithm>
#include <filesystem>
#include <string>
#include <string_view>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/fileio/AtomicFile.h"
#include "horizon/fileio/NativeFormat.h"

namespace hz::ui {

namespace {

constexpr const char* kLockName = "session.lock";
const QStringList kSnapshotTypes = {QStringLiteral("hcad"), QStringLiteral("hzpart"),
                                    QStringLiteral("hzasm")};

std::filesystem::path toPath(const QString& path) {
    return io::pathFromUtf8(path.toStdString());
}

/// A lock that only a dead process's leftovers can be taken from. Age alone
/// never makes a lock stale: a session can run for days.
std::unique_ptr<QLockFile> makeLock(const QString& directory) {
    auto lock = std::make_unique<QLockFile>(QDir(directory).filePath(kLockName));
    lock->setStaleLockTime(0);
    return lock;
}

}  // namespace

RecoveryManager::RecoveryManager(const QString& root) : m_root(root) {
    const QString id = QUuid::createUuid().toString(QUuid::WithoutBraces);
    m_sessionDir = QDir(root).filePath(id);
    if (!QDir().mkpath(m_sessionDir)) {
        m_problem = QStringLiteral("cannot create %1").arg(QDir::toNativeSeparators(m_sessionDir));
        spdlog::warn("Autosave disabled: {}", m_problem.toStdString());
        return;
    }
    m_lock = makeLock(m_sessionDir);
    if (!m_lock->tryLock(0)) {
        m_problem = QStringLiteral("cannot lock %1").arg(QDir::toNativeSeparators(m_sessionDir));
        spdlog::warn("Autosave disabled: {}", m_problem.toStdString());
        return;
    }
    m_active = true;
}

RecoveryManager::~RecoveryManager() {
    // Orphans claimed but neither recovered nor discarded stay for next time.
    m_claimedLocks.clear();
    if (m_active) {
        m_lock->unlock();
        QDir(m_sessionDir).removeRecursively();
    }
}

bool RecoveryManager::snapshot(quint64 key, const doc::Document& doc, const QString& title,
                               const QString& originalPath, int recoveries) {
    const QString type =
        doc.type() == doc::DocumentType::Part ? QStringLiteral("hzpart") : QStringLiteral("hcad");
    return writeSnapshot(key, io::NativeFormat::documentToJson(doc, /*includeTessellation=*/false),
                         type, title, originalPath, recoveries);
}

bool RecoveryManager::snapshot(quint64 key, const doc::AssemblyDocument& assembly,
                               const QString& title, const QString& originalPath,
                               int recoveries) {
    // Component paths are kept as they are in memory, not made relative to
    // the snapshot's own directory.
    return writeSnapshot(key, io::NativeFormat::assemblyToJson(assembly, std::string()),
                         QStringLiteral("hzasm"), title, originalPath, recoveries);
}

bool RecoveryManager::writeSnapshot(quint64 key, const std::string& json, const QString& type,
                                    const QString& title, const QString& originalPath,
                                    int recoveries) {
    if (!m_active) return false;
    const QString base = QDir(m_sessionDir).filePath(QString::number(key));
    const auto failed = [&](const std::string& error) {
        m_problem = QString::fromStdString(error);
        spdlog::warn("Autosave of '{}' failed: {}", title.toStdString(), error);
        return false;
    };

    std::string error;
    if (!io::writeFileAtomically(toPath(base + '.' + type), json, &error)) return failed(error);
    // A document whose type changed (Save As a part) must not leave its old
    // snapshot behind — removed only now, so a failed write keeps the old one.
    for (const QString& other : kSnapshotTypes) {
        if (other != type) QFile::remove(base + '.' + other);
    }
    // The sidecar goes second: a snapshot without one is ignored, never a
    // sidecar pointing at a half-written snapshot.
    QJsonObject meta{
        {QStringLiteral("originalPath"), originalPath},
        {QStringLiteral("title"), title},
        {QStringLiteral("type"), type},
        {QStringLiteral("savedAt"), QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)},
    };
    if (recoveries > 0) meta.insert(QStringLiteral("recoveries"), recoveries);
    const QByteArray metaJson = QJsonDocument(meta).toJson();
    if (!io::writeFileAtomically(
            toPath(base + QStringLiteral(".json")),
            std::string_view(metaJson.constData(), static_cast<size_t>(metaJson.size())), &error)) {
        return failed(error);
    }
    m_problem.clear();
    return true;
}

void RecoveryManager::remove(quint64 key) {
    if (!m_active) return;
    const QString base = QDir(m_sessionDir).filePath(QString::number(key));
    QFile::remove(base + QStringLiteral(".json"));
    for (const QString& type : kSnapshotTypes) QFile::remove(base + '.' + type);
}

std::vector<RecoveryManager::Entry> RecoveryManager::claimOrphans() {
    std::vector<Entry> entries;
    const QString own = QFileInfo(m_sessionDir).absoluteFilePath();
    const QFileInfoList sessions =
        QDir(m_root).entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    for (const QFileInfo& session : sessions) {
        const QString dir = session.absoluteFilePath();
        if (dir == own) continue;
        if (std::find(m_claimedDirs.begin(), m_claimedDirs.end(), dir) != m_claimedDirs.end()) {
            continue;
        }
        // Taking the lock succeeds only when its owner is gone (or never
        // wrote one): a running session keeps its documents.
        auto lock = makeLock(dir);
        if (!lock->tryLock(0)) continue;

        const QDir sessionDir(dir);
        size_t found = 0;
        for (const QFileInfo& sidecar :
             sessionDir.entryInfoList({QStringLiteral("*.json")}, QDir::Files, QDir::Name)) {
            QFile file(sidecar.absoluteFilePath());
            if (!file.open(QIODevice::ReadOnly)) continue;
            const QJsonObject meta = QJsonDocument::fromJson(file.readAll()).object();
            const QString type = meta.value(QStringLiteral("type")).toString();
            if (!kSnapshotTypes.contains(type)) continue;
            const QString snapshot = sessionDir.filePath(sidecar.completeBaseName() + '.' + type);
            if (!QFileInfo::exists(snapshot)) continue;
            entries.push_back(Entry{
                snapshot,
                meta.value(QStringLiteral("originalPath")).toString(),
                meta.value(QStringLiteral("title")).toString(),
                type,
                QDateTime::fromString(meta.value(QStringLiteral("savedAt")).toString(),
                                      Qt::ISODateWithMs),
                std::max(0, meta.value(QStringLiteral("recoveries")).toInt()),
            });
            ++found;
        }

        if (found == 0) {
            // A crashed session with nothing modified: just clean it up.
            lock->unlock();
            QDir(dir).removeRecursively();
            continue;
        }
        m_claimedDirs.push_back(dir);
        m_claimedLocks.push_back(std::move(lock));
    }
    return entries;
}

void RecoveryManager::noteRecoveryAttempt() {
    for (const QString& dir : m_claimedDirs) {
        const QDir sessionDir(dir);
        for (const QFileInfo& sidecar :
             sessionDir.entryInfoList({QStringLiteral("*.json")}, QDir::Files, QDir::Name)) {
            QFile file(sidecar.absoluteFilePath());
            if (!file.open(QIODevice::ReadOnly)) continue;
            QJsonObject meta = QJsonDocument::fromJson(file.readAll()).object();
            file.close();
            if (meta.isEmpty()) continue;
            meta.insert(QStringLiteral("recoveries"),
                        std::max(0, meta.value(QStringLiteral("recoveries")).toInt()) + 1);
            const QByteArray json = QJsonDocument(meta).toJson();
            std::string error;
            if (!io::writeFileAtomically(
                    toPath(sidecar.absoluteFilePath()),
                    std::string_view(json.constData(), static_cast<size_t>(json.size())),
                    &error)) {
                spdlog::warn("Could not mark {} as being recovered: {}",
                             sidecar.absoluteFilePath().toStdString(), error);
            }
        }
    }
}

void RecoveryManager::discardClaimedOrphans() {
    for (size_t i = 0; i < m_claimedDirs.size(); ++i) {
        m_claimedLocks[i]->unlock();
        QDir(m_claimedDirs[i]).removeRecursively();
    }
    m_claimedLocks.clear();
    m_claimedDirs.clear();
}

}  // namespace hz::ui
