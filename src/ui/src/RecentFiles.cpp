#include "horizon/ui/RecentFiles.h"

#include <QDir>
#include <QFileInfo>
#include <QSettings>

namespace hz::ui {

namespace {

const char* const kKey = "files/recent";

QString normalized(const QString& path) {
    const QFileInfo info(path);
    const QString canonical = info.canonicalFilePath();  // empty once the file is gone
    return QDir::cleanPath(canonical.isEmpty() ? info.absoluteFilePath() : canonical);
}

bool samePath(const QString& a, const QString& b) {
#ifdef Q_OS_WIN
    return a.compare(b, Qt::CaseInsensitive) == 0;
#else
    return a == b;
#endif
}

}  // namespace

QStringList RecentFiles::list() {
    return QSettings().value(kKey).toStringList();
}

void RecentFiles::add(const QString& path) {
    if (path.isEmpty()) return;
    const QString entry = normalized(path);
    QStringList files = list();
    files.removeIf([&entry](const QString& f) { return samePath(f, entry); });
    files.prepend(entry);
    while (files.size() > kMax) files.removeLast();
    QSettings().setValue(kKey, files);
}

void RecentFiles::remove(const QString& path) {
    const QString entry = normalized(path);
    QStringList files = list();
    files.removeIf([&](const QString& f) { return samePath(f, entry) || samePath(f, path); });
    QSettings().setValue(kKey, files);
}

void RecentFiles::clear() {
    QSettings().remove(kKey);
}

}  // namespace hz::ui
