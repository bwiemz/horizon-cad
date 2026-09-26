// Crash reports (Phase 167): what a crash leaves, and how it is read back.

#include <gtest/gtest.h>
#include <spdlog/sinks/null_sink.h>
#include <spdlog/spdlog.h>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QStringList>
#include <QTemporaryDir>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "horizon/ui/CrashReport.h"
#include "horizon/ui/Logging.h"

namespace crash = hz::ui::crash;

namespace {

void writeFile(const QString& path, const QByteArray& contents) {
    QFile file(path);
    ASSERT_TRUE(file.open(QIODevice::WriteOnly | QIODevice::Truncate)) << path.toStdString();
    file.write(contents);
}

/// Run the crashing child (crash_child.cpp) in @p how, with its reports in
/// @p reports and its log at @p log; the one report it left, read back, or
/// empty (a test failure recorded).
QString crashChild(const QString& reports, const QString& log, const QString& how) {
    QProcess child;
    child.start(QStringLiteral(HZ_CRASH_CHILD), {reports, log, how});
    if (!child.waitForFinished(60'000)) {
        ADD_FAILURE() << "the child did not finish: " << child.errorString().toStdString();
        child.kill();
        return {};
    }
    // It died of the crash, as it would have without the handler.
    EXPECT_EQ(child.exitStatus(), QProcess::CrashExit);
    const QStringList pending = crash::pendingReports(reports);
    if (pending.size() != 1) {
        ADD_FAILURE() << "reports: " << QDir(reports).entryList().join(' ').toStdString();
        return {};
    }
    return crash::readReport(pending.front());
}

}  // namespace

TEST(CrashReportTest, ACrashLeavesAReportOfWhatHappened) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString reports = dir.filePath(QStringLiteral("crashes"));
    const QString log = dir.filePath(QStringLiteral("horizon.log"));
    writeFile(log, "opened bracket.hcad\nfilleting edge 12\n");

    const QString report = crashChild(reports, log, QStringLiteral("raise"));
    ASSERT_FALSE(report.isEmpty());
    const QStringList pending = crash::pendingReports(reports);
#if defined(_WIN32)
    EXPECT_TRUE(report.contains(QStringLiteral("exception 0xc0000005"))) << report.toStdString();
    // A minidump beside it, for a debugger.
    const QFileInfo dump(pending.front().chopped(4) + QStringLiteral(".dmp"));
    EXPECT_TRUE(dump.exists());
    EXPECT_GT(dump.size(), 0);
#else
    EXPECT_TRUE(report.startsWith(QStringLiteral("Horizon CAD stopped: SIGSEGV")))
        << report.toStdString();
#if !defined(__APPLE__)
    // Sent by raise(), not a fault: no address to give, so none made up.
    // macOS gives a sent SIGSEGV a fault's code, so there the two look alike.
    EXPECT_FALSE(report.contains(QStringLiteral("At address"))) << report.toStdString();
#endif
#endif
    EXPECT_TRUE(report.contains(QStringLiteral("Horizon CAD test child"))) << report.toStdString();
    EXPECT_TRUE(report.contains(QStringLiteral("Log: ") + log)) << report.toStdString();
    // A backtrace that goes past where the signal stopped the thread: a
    // frame for that, and for its callers (the raise, the crash, main...).
    const qsizetype frames = report.indexOf(QStringLiteral("Backtrace:\n"));
    ASSERT_GE(frames, 0) << report.toStdString();
    const qsizetype tail = report.indexOf(QStringLiteral("The log's last lines"), frames);
    const QString trace = report.mid(frames, tail < 0 ? -1 : tail - frames);
    EXPECT_GE(trace.count(QStringLiteral("0x")), 3) << report.toStdString();
    // And what the log said last: what led up to it.
    EXPECT_TRUE(report.contains(QStringLiteral("filleting edge 12"))) << report.toStdString();
}

// The walk macOS's reports take through the frame pointers, over a stack
// made here: it follows the chain, and stops where the chain ends, falls,
// leaves the stack or is misaligned, or there is no more room.
TEST(CrashReportTest, TheFramePointerWalkFollowsTheChainAndStopsWhereItBreaks) {
    using hz::ui::crash::detail::walkFramePointers;
    std::array<std::uintptr_t, 16> stack{};
    const auto at = [&stack](std::size_t i) { return reinterpret_cast<std::uintptr_t>(&stack[i]); };
    const std::uintptr_t low = at(0);
    const std::uintptr_t high = low + sizeof(stack);
    const auto address = [](std::uintptr_t a) { return reinterpret_cast<void*>(a); };
    // Three frames, each the caller's frame pointer then its return address.
    stack[2] = at(6);
    stack[3] = 0x1111;
    stack[6] = at(10);
    stack[7] = 0x2222;
    stack[10] = 0;  // the outermost
    stack[11] = 0x3333;
    std::array<void*, 8> frames{};
    ASSERT_EQ(walkFramePointers(0xAAAA, at(2), low, high, frames.data(), 8), 4);
    EXPECT_EQ(frames[0], address(0xAAAA)) << "where the signal stopped it";
    EXPECT_EQ(frames[1], address(0x1111));
    EXPECT_EQ(frames[2], address(0x2222));
    EXPECT_EQ(frames[3], address(0x3333));

    EXPECT_EQ(walkFramePointers(0xAAAA, at(2), low, high, frames.data(), 2), 2) << "no more room";
    EXPECT_EQ(walkFramePointers(0xAAAA, at(2), low, high, frames.data(), 0), 0);
    EXPECT_EQ(walkFramePointers(0xAAAA, high, low, high, frames.data(), 8), 1) << "off the stack";
    EXPECT_EQ(walkFramePointers(0xAAAA, at(2) + 1, low, high, frames.data(), 8), 1) << "misaligned";
    EXPECT_EQ(walkFramePointers(0xAAAA, at(15), low, high, frames.data(), 8), 1)
        << "a frame that runs off the stack's end";
    stack[6] = at(2);  // a chain that falls back on itself
    EXPECT_EQ(walkFramePointers(0xAAAA, at(2), low, high, frames.data(), 8), 3);
    stack[3] = 0;  // no address to return to
    EXPECT_EQ(walkFramePointers(0xAAAA, at(2), low, high, frames.data(), 8), 1);
}

TEST(CrashReportTest, ABadMemoryAccessIsReportedWithItsAddress) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString report =
        crashChild(dir.filePath(QStringLiteral("crashes")),
                   dir.filePath(QStringLiteral("horizon.log")), QStringLiteral("fault"));
#if defined(_WIN32)
    EXPECT_TRUE(report.contains(QStringLiteral("exception 0xc0000005"))) << report.toStdString();
#else
    // SIGSEGV, or SIGBUS where the system says so of an unmapped address.
    EXPECT_TRUE(report.startsWith(QStringLiteral("Horizon CAD stopped: SIG")))
        << report.toStdString();
    EXPECT_TRUE(report.contains(QStringLiteral("At address 0x10\n"))) << report.toStdString();
#endif
}

#if !defined(_WIN32)
// On Windows the filter runs on what is left of the stack, which is not
// always enough; the handler here has a stack of its own.
TEST(CrashReportTest, AStackOverflowIsReportedToo) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString report =
        crashChild(dir.filePath(QStringLiteral("crashes")),
                   dir.filePath(QStringLiteral("horizon.log")), QStringLiteral("overflow"));
    EXPECT_TRUE(report.startsWith(QStringLiteral("Horizon CAD stopped: SIGSEGV")))
        << report.toStdString();
    EXPECT_TRUE(report.contains(QStringLiteral("Backtrace:"))) << report.toStdString();
}
#endif

// The log's last lines are read as the crash happens: whole lines, the last
// few kilobytes of a long log.
TEST(CrashReportTest, AReportHoldsTheLogsLastLines) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString log = dir.filePath(QStringLiteral("horizon.log"));
    QByteArray lines;
    for (int i = 1; i <= 500; ++i) {
        lines += QStringLiteral("entry %1 of the log\n").arg(i, 3, 10, QLatin1Char('0')).toUtf8();
    }
    writeFile(log, lines);

    const QString report =
        crashChild(dir.filePath(QStringLiteral("crashes")), log, QStringLiteral("raise"));
    const qsizetype tail = report.indexOf(QStringLiteral("The log's last lines:\n"));
    ASSERT_GE(tail, 0) << report.toStdString();
    const QString quoted = report.mid(tail).section(QLatin1Char('\n'), 1);
    EXPECT_TRUE(quoted.startsWith(QStringLiteral("entry "))) << "a whole line first";
    EXPECT_TRUE(quoted.contains(QStringLiteral("entry 500 of the log")));
    EXPECT_FALSE(quoted.contains(QStringLiteral("entry 001 of the log"))) << "the tail only";
}

// A report is read as it is: a path it names is not opened. A report planted
// in the folder cannot put another file into what the user attaches to an
// issue.
TEST(CrashReportTest, AReportIsReadAsItIsAndNothingItNames) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString secret = dir.filePath(QStringLiteral("secret.txt"));
    writeFile(secret, "not for an issue\n");
    const QString path = dir.filePath(QStringLiteral("crash-20260926-120000-7.txt"));
    writeFile(
        path,
        ("Horizon CAD stopped: SIGSEGV, a bad memory access\n\nLog: " + secret + "\n").toUtf8());

    const QString report = crash::readReport(path);
    EXPECT_TRUE(report.startsWith(QStringLiteral("Horizon CAD stopped: SIGSEGV")));
    EXPECT_FALSE(report.contains(QStringLiteral("not for an issue")));
    EXPECT_TRUE(crash::readReport(dir.filePath(QStringLiteral("gone.txt"))).isEmpty());
}

TEST(CrashReportTest, AShownReportIsKeptButNotOfferedAgain) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    writeFile(dir.filePath(QStringLiteral("crash-20260926-120000-7.txt")), "first");
    writeFile(dir.filePath(QStringLiteral("crash-20260926-130000-8.txt")), "second");
    writeFile(dir.filePath(QStringLiteral("crash-20260925-120000-6.txt.shown")), "shown");
    writeFile(dir.filePath(QStringLiteral("notes.txt")), "not a report");

    QStringList pending = crash::pendingReports(dir.path());
    ASSERT_EQ(pending.size(), 2);  // oldest first
    EXPECT_TRUE(pending[0].endsWith(QStringLiteral("crash-20260926-120000-7.txt")));
    EXPECT_TRUE(pending[1].endsWith(QStringLiteral("crash-20260926-130000-8.txt")));

    ASSERT_TRUE(crash::markShown(pending[0]));
    pending = crash::pendingReports(dir.path());
    ASSERT_EQ(pending.size(), 1);
    EXPECT_TRUE(pending[0].endsWith(QStringLiteral("crash-20260926-130000-8.txt")));
    EXPECT_TRUE(QFile::exists(dir.filePath(QStringLiteral("crash-20260926-120000-7.txt.shown"))));

    // One that cannot be renamed (here a folder has the name) is deleted,
    // so it is not offered at every start.
    ASSERT_TRUE(QDir(dir.path()).mkdir(QStringLiteral("crash-20260926-130000-8.txt.shown")));
    EXPECT_TRUE(crash::markShown(pending[0]));
    EXPECT_TRUE(crash::pendingReports(dir.path()).isEmpty());
}

TEST(CrashReportTest, OnlyTheNewestShownReportsAreKept) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    for (int day = 10; day < 15; ++day) {
        const QString base = QStringLiteral("crash-202609%1-120000-7").arg(day);
        writeFile(dir.filePath(base + QStringLiteral(".txt.shown")), "shown");
        writeFile(dir.filePath(base + QStringLiteral(".dmp")), "dump");
    }
    writeFile(dir.filePath(QStringLiteral("crash-20260901-120000-7.txt")), "not yet shown");

    EXPECT_EQ(crash::prune(dir.path(), 2), 3);
    const QStringList left = QDir(dir.path()).entryList(QDir::Files, QDir::Name);
    EXPECT_EQ(left,
              (QStringList{"crash-20260901-120000-7.txt",  // an unshown one stays
                           "crash-20260913-120000-7.dmp", "crash-20260913-120000-7.txt.shown",
                           "crash-20260914-120000-7.dmp", "crash-20260914-120000-7.txt.shown"}));
    EXPECT_EQ(crash::prune(dir.path(), 2), 0);
}

// The log's last lines are in its file when a crash comes: each line is
// flushed as it is logged, since a crash cannot flush it.
TEST(CrashReportTest, EveryLogLineIsInTheFileAsItIsLogged) {
    QString text;
    {
        QTemporaryDir dir;
        ASSERT_TRUE(dir.isValid());
        const QString file = hz::ui::initializeLogging(dir.path());
        ASSERT_FALSE(file.isEmpty());
        spdlog::info("a line a crash report quotes");
        QFile log(file);
        if (log.open(QIODevice::ReadOnly)) text = QString::fromUtf8(log.readAll());
        // Let the file go, so the directory can be removed (Windows).
        hz::ui::shutdownLogging();
        spdlog::set_default_logger(std::make_shared<spdlog::logger>(
            "tests", std::make_shared<spdlog::sinks::null_sink_mt>()));
    }
    EXPECT_TRUE(text.contains(QStringLiteral("a line a crash report quotes")))
        << text.toStdString();
}
