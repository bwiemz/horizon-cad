#pragma once

#include <QString>

namespace hz::ui {

/// Send the default spdlog logger to a rotating log file in `logDirectory`
/// (created if needed; three files of 5 MB) as well as to stderr, and route
/// Qt's own messages (qWarning, qCritical, ...) into it. Every line is flushed
/// as it is logged, so a crash does not lose the lines that explain it (a
/// crash report quotes them).
///
/// Returns the path of the log file, or an empty string when the file could
/// not be opened — logging then continues on stderr only.
QString initializeLogging(const QString& logDirectory);

/// The log file set up by initializeLogging(), or empty.
QString logFilePath();

/// Flush the log and stop routing Qt messages into it. The logger itself stays
/// usable until the process exits, so late messages are not lost or fatal.
/// Safe to call more than once.
void shutdownLogging();

}  // namespace hz::ui
