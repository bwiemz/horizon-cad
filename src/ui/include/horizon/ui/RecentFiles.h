#pragma once

#include <QString>
#include <QStringList>

namespace hz::ui {

/// The files most recently opened or saved, newest first, kept in the
/// application settings ("files/recent") so File ▸ Open Recent survives a
/// restart.
class RecentFiles {
public:
    /// How many are kept.
    static constexpr int kMax = 10;

    static QStringList list();

    /// Put @p path first, as an absolute path, dropping any older mention of
    /// the same file and anything past kMax.
    static void add(const QString& path);

    static void remove(const QString& path);
    static void clear();
};

}  // namespace hz::ui
