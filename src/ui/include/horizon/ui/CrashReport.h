#pragma once

#include <QString>
#include <QStringList>
#include <cstdint>

namespace hz::ui::crash {

/// Where crash reports are kept: `HZ_CRASH_DIR` if set (a test's), else
/// "crashes" beside the recovery files in the application's data folder.
QString reportDirectory();

/// Install the crash handler (Phase 167). A crash in native code (a fatal
/// signal: SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT; on Windows, an
/// unhandled structured exception) writes a report into @p directory: what
/// happened, a backtrace, the version and platform (@p about), the log
/// file's path (@p logFile) and the log's last lines, read as the crash
/// happens. On Windows a minidump goes beside it. The process then dies of
/// the signal as it would have.
///
/// The handler uses only calls safe in a signal handler, into a path and a
/// text made here, and runs on a stack of its own, so a stack overflow is
/// reported too. One thread writes the report; another that crashes
/// meanwhile dies of its own signal. Nothing is sent anywhere. Returns false
/// when the directory cannot be made; there is no handler then.
bool install(const QString& directory, const QString& logFile, const QString& about);

/// The reports a crash left in @p directory not yet shown, oldest first.
QStringList pendingReports(const QString& directory);

/// The report at @p path, as the crash wrote it: what to attach to an issue.
QString readReport(const QString& path);

/// Mark the report at @p path shown: it is kept, but not offered again.
/// When it cannot be renamed it is deleted, so it is not offered at every
/// start; false when neither could be done.
bool markShown(const QString& path);

/// Delete all but the newest @p keep shown reports in @p directory, and the
/// minidumps beside them, so they do not pile up. Returns how many went.
int prune(const QString& directory, int keep = 10);

/// Crash now, as a fault in native code would: for a test of the handler.
[[noreturn]] void crashForTest();

namespace detail {

/// The return addresses of the code a signal stopped at @p pc, by its chain
/// of frame pointers from @p fp: @p pc, then each caller's. The chain is
/// followed only while it is aligned, rises, and stays in [@p low, @p high),
/// the thread's stack, so a broken chain ends the walk instead of the
/// process. Returns how many of @p capacity were written. Safe in a signal
/// handler: it reads memory and nothing else.
int walkFramePointers(std::uintptr_t pc, std::uintptr_t fp, std::uintptr_t low, std::uintptr_t high,
                      void** frames, int capacity);

}  // namespace detail

}  // namespace hz::ui::crash
