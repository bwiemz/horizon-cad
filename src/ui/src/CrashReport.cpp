#include "horizon/ui/CrashReport.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <QStringList>
#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#if defined(_WIN32)
#include <cwchar>
#include <iterator>
#include <string>
// windows.h before dbghelp.h, which needs its types: not sorted. And no
// min/max macros from it, which std::max would expand.
#ifndef NOMINMAX
#define NOMINMAX
#endif
// clang-format off
#include <windows.h>
#include <dbghelp.h>
// clang-format on
#else
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#if defined(__GLIBC__) || defined(__APPLE__)
#include <execinfo.h>
#define HZ_HAVE_BACKTRACE 1
#endif
#if defined(__APPLE__)
#include <mach/mach.h>
#include <pthread.h>
#include <sys/ucontext.h>
#endif
#endif

namespace hz::ui::crash {

namespace {

// Made when the handler is installed, and only read in it: a handler may
// not allocate, nor call much of anything.
char g_reportPath[4096];
char g_header[4096];
std::size_t g_headerLength = 0;
// The log's tail is copied into the report as the crash happens, so the
// report holds what led up to it, and nothing reads a path out of a report
// later. How much: about the last 40 lines.
constexpr std::size_t kLogTail = 4096;
char g_logTail[kLogTail];
// One thread writes the report. A second that faults meanwhile (or a fault
// in the handler) finds it taken, and dies of its own signal.
std::atomic_flag g_handling = ATOMIC_FLAG_INIT;
constexpr char kLogTailHeading[] = "\nThe log's last lines:\n";

/// Where the tail's whole lines begin in @p data (@p size bytes read from
/// the end of the log): after its first newline, the part line before it
/// dropped; all of it when the read began at the file's start.
std::size_t wholeLinesFrom(const char* data, std::size_t size, bool fromStart) {
    if (fromStart) return 0;
    for (std::size_t i = 0; i < size; ++i) {
        if (data[i] == '\n') return i + 1;
    }
    return size;
}

/// @p value in hexadecimal into @p out ("0x..."), without allocating: what a
/// handler may do. @p out holds at least 19 characters.
std::size_t formatHex(std::uintptr_t value, char* out) {
    static const char kDigits[] = "0123456789abcdef";
    char digits[16];
    std::size_t count = 0;
    do {
        digits[count++] = kDigits[value & 0xF];
        value >>= 4;
    } while (value != 0 && count < sizeof(digits));
    out[0] = '0';
    out[1] = 'x';
    for (std::size_t i = 0; i < count; ++i) out[2 + i] = digits[count - 1 - i];
    out[2 + count] = '\n';
    return 3 + count;
}

#if defined(_WIN32)

wchar_t g_reportPathW[4096];
wchar_t g_dumpPathW[4096];
wchar_t g_logPathW[4096];

void writeAll(HANDLE file, const char* text, std::size_t length) {
    DWORD written = 0;
    WriteFile(file, text, static_cast<DWORD>(length), &written, nullptr);
}

/// The log's last lines into @p file, read as the crash happens.
void writeLogTail(HANDLE file) {
    if (g_logPathW[0] == L'\0') return;
    HANDLE log = CreateFileW(g_logPathW, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                             OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (log == INVALID_HANDLE_VALUE) return;
    LARGE_INTEGER size{};
    if (GetFileSizeEx(log, &size) != 0) {
        const LONGLONG start = std::max<LONGLONG>(0, size.QuadPart - LONGLONG{kLogTail});
        LARGE_INTEGER at{};
        at.QuadPart = start;
        DWORD read = 0;
        if (SetFilePointerEx(log, at, nullptr, FILE_BEGIN) != 0 &&
            ReadFile(log, g_logTail, static_cast<DWORD>(kLogTail), &read, nullptr) != 0) {
            const std::size_t from = wholeLinesFrom(g_logTail, read, start == 0);
            writeAll(file, kLogTailHeading, sizeof(kLogTailHeading) - 1);
            writeAll(file, g_logTail + from, read - from);
        }
    }
    CloseHandle(log);
}

LONG WINAPI onException(EXCEPTION_POINTERS* info) {
    if (g_handling.test_and_set()) return EXCEPTION_CONTINUE_SEARCH;
    HANDLE file = CreateFileW(g_reportPathW, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
                              FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file != INVALID_HANDLE_VALUE) {
        static const char kStopped[] = "Horizon CAD stopped: exception ";
        writeAll(file, kStopped, sizeof(kStopped) - 1);
        char hex[24];
        const auto code = info != nullptr && info->ExceptionRecord != nullptr
                              ? info->ExceptionRecord->ExceptionCode
                              : 0u;
        writeAll(file, hex, formatHex(code, hex));
        writeAll(file, g_header, g_headerLength);
        static const char kFrames[] = "\nBacktrace (addresses; the .dmp beside this has more):\n";
        writeAll(file, kFrames, sizeof(kFrames) - 1);
        void* frames[62];
        const USHORT count = CaptureStackBackTrace(0, 62, frames, nullptr);
        for (USHORT i = 0; i < count; ++i) {
            writeAll(file, hex, formatHex(reinterpret_cast<std::uintptr_t>(frames[i]), hex));
        }
        writeLogTail(file);
        CloseHandle(file);
    }
    HANDLE dump = CreateFileW(g_dumpPathW, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
                              FILE_ATTRIBUTE_NORMAL, nullptr);
    if (dump != INVALID_HANDLE_VALUE) {
        MINIDUMP_EXCEPTION_INFORMATION exception{};
        exception.ThreadId = GetCurrentThreadId();
        exception.ExceptionPointers = info;
        exception.ClientPointers = FALSE;
        MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), dump, MiniDumpNormal,
                          info != nullptr ? &exception : nullptr, nullptr, nullptr);
        CloseHandle(dump);
    }
    // Let Windows end it as it would have (and report it, if it does).
    return EXCEPTION_CONTINUE_SEARCH;
}

#else

// A stack of its own: a stack overflow leaves none to run the handler on.
alignas(16) char g_altStack[64 * 1024];
char g_logPath[4096];

void writeAll(int fd, const char* text, std::size_t length) {
    while (length > 0) {
        const ssize_t n = ::write(fd, text, length);
        if (n <= 0) return;
        text += n;
        length -= static_cast<std::size_t>(n);
    }
}

void writeText(int fd, const char* text) {
    writeAll(fd, text, std::strlen(text));
}

const char* signalName(int sig) {
    switch (sig) {
        case SIGSEGV:
            return "SIGSEGV, a bad memory access";
        case SIGBUS:
            return "SIGBUS, a bad memory access";
        case SIGFPE:
            return "SIGFPE, an arithmetic fault";
        case SIGILL:
            return "SIGILL, an illegal instruction";
        case SIGABRT:
            return "SIGABRT, an abort";
        default:
            return "a fatal signal";
    }
}

/// The log's last lines into @p fd, read as the crash happens (open, lseek
/// and read are safe in a signal handler).
void writeLogTail(int fd) {
    if (g_logPath[0] == '\0') return;
    const int log = ::open(g_logPath, O_RDONLY | O_CLOEXEC);
    if (log < 0) return;
    const off_t end = ::lseek(log, 0, SEEK_END);
    const off_t start = end > static_cast<off_t>(kLogTail) ? end - static_cast<off_t>(kLogTail) : 0;
    if (end >= 0 && ::lseek(log, start, SEEK_SET) == start) {
        const ssize_t n = ::read(log, g_logTail, kLogTail);
        if (n > 0) {
            const auto read = static_cast<std::size_t>(n);
            const std::size_t from = wholeLinesFrom(g_logTail, read, start == 0);
            writeAll(fd, kLogTailHeading, sizeof(kLogTailHeading) - 1);
            writeAll(fd, g_logTail + from, read - from);
        }
    }
    ::close(log);
}

/// Whether the kernel raised the signal for a fault, which has an address,
/// not a process (raise, kill), whose si_code is SI_USER or the like: 0 or
/// below on Linux, but 0x10001 and above on macOS. macOS gives a SIGSEGV or
/// SIGBUS that was sent a fault's code all the same, so there such a one
/// reads as a fault, at whatever address the thread last faulted at.
bool raisedByAFault(const siginfo_t* info) {
    if (info == nullptr || info->si_code <= 0) return false;
#if defined(__APPLE__)
    return info->si_code < SI_USER;
#else
    return true;
#endif
}

#ifdef HZ_HAVE_BACKTRACE
/// The crashed thread's return addresses, from where the signal stopped it.
/// glibc's backtrace() unwinds through the signal's frame. macOS's gives up
/// at once when called off the thread's stack, as the handler, on a stack of
/// its own, is (backtrace_from_fp too): there the frame pointers are followed
/// by hand from the interrupted frame, within the thread's stack.
int backtraceOf(void* context, void** frames, int capacity) {
#if defined(__APPLE__)
    const auto* uc = static_cast<const ucontext_t*>(context);
    if (uc == nullptr || uc->uc_mcontext == nullptr) return 0;
#if defined(__arm64__) || defined(__aarch64__)
    const auto pc = static_cast<std::uintptr_t>(arm_thread_state64_get_pc(uc->uc_mcontext->__ss));
    const auto fp = static_cast<std::uintptr_t>(arm_thread_state64_get_fp(uc->uc_mcontext->__ss));
#else
    const auto pc = static_cast<std::uintptr_t>(uc->uc_mcontext->__ss.__rip);
    const auto fp = static_cast<std::uintptr_t>(uc->uc_mcontext->__ss.__rbp);
#endif
    pthread_t self = pthread_self();
    const auto high = reinterpret_cast<std::uintptr_t>(pthread_get_stackaddr_np(self));
    const std::uintptr_t low = high - pthread_get_stacksize_np(self);
    return crash::detail::walkFramePointers(pc, fp, low, high, frames, capacity);
#else
    (void)context;
    return ::backtrace(frames, capacity);
#endif
}
#endif

void onSignal(int sig, siginfo_t* info, void* context) {
    // Taken: another thread is writing the report, or this is a fault in
    // the handler. The default is back for this signal; die of it.
    if (g_handling.test_and_set()) return;
    const int fd = ::open(g_reportPath, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (fd >= 0) {
        writeText(fd, "Horizon CAD stopped: ");
        writeText(fd, signalName(sig));
        writeText(fd, "\n");
        // A fault the kernel raised has an address; one sent (raise, kill)
        // has none, only the sender's pid there.
        if (raisedByAFault(info) && (sig == SIGSEGV || sig == SIGBUS)) {
            char hex[24];
            writeText(fd, "At address ");
            writeAll(fd, hex, formatHex(reinterpret_cast<std::uintptr_t>(info->si_addr), hex));
        }
        writeAll(fd, g_header, g_headerLength);
#ifdef HZ_HAVE_BACKTRACE
        writeText(fd, "\nBacktrace:\n");
        void* frames[64];
        const int count = backtraceOf(context, frames, 64);
        ::backtrace_symbols_fd(frames, count, fd);
#endif
        writeLogTail(fd);
        ::close(fd);
    }
    // Die of it, as without the handler: SA_RESETHAND put the default back.
    ::raise(sig);
}

#endif

}  // namespace

int detail::walkFramePointers(std::uintptr_t pc, std::uintptr_t fp, std::uintptr_t low,
                              std::uintptr_t high, void** frames, int capacity) {
    if (capacity < 1) return 0;
    int count = 0;
    frames[count++] = reinterpret_cast<void*>(pc);
    // A frame: the caller's frame pointer, then the address to return to.
    constexpr std::uintptr_t kRecord = 2 * sizeof(std::uintptr_t);
    while (count < capacity && fp >= low && fp <= high && high - fp >= kRecord &&
           fp % sizeof(std::uintptr_t) == 0) {
        const auto* record = reinterpret_cast<const std::uintptr_t*>(fp);
        const std::uintptr_t next = record[0];
        const std::uintptr_t returnTo = record[1];
        if (returnTo == 0) break;
        frames[count++] = reinterpret_cast<void*>(returnTo);
        if (next <= fp) break;  // a caller's frame is higher up the stack
        fp = next;
    }
    return count;
}

QString reportDirectory() {
    QString forced = qEnvironmentVariable("HZ_CRASH_DIR");
    if (!forced.isEmpty()) return forced;
    return QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation) +
           QStringLiteral("/crashes");
}

bool install(const QString& directory, const QString& logFile, const QString& about) {
    if (!QDir().mkpath(directory)) return false;
    const QString name =
        QStringLiteral("crash-%1-%2.txt")
            .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd-HHmmss")))
            .arg(QCoreApplication::applicationPid());
    const QString report = QDir(directory).filePath(name);
    const QByteArray path = QFile::encodeName(report);
    if (static_cast<std::size_t>(path.size()) >= sizeof(g_reportPath)) return false;
    std::memcpy(g_reportPath, path.constData(), static_cast<std::size_t>(path.size()) + 1);

    QByteArray header =
        (QStringLiteral("\n") + about + QStringLiteral("\nLog: ") + logFile + QStringLiteral("\n"))
            .toUtf8();
    header.truncate(static_cast<qsizetype>(sizeof(g_header) - 1));
    std::memcpy(g_header, header.constData(), static_cast<std::size_t>(header.size()));
    g_headerLength = static_cast<std::size_t>(header.size());

#if defined(_WIN32)
    const std::wstring wide = QDir::toNativeSeparators(report).toStdWString();
    const std::wstring dumpWide =
        QDir::toNativeSeparators(report.chopped(4) + QStringLiteral(".dmp")).toStdWString();
    if (wide.size() >= std::size(g_reportPathW) || dumpWide.size() >= std::size(g_dumpPathW)) {
        return false;
    }
    std::wmemcpy(g_reportPathW, wide.c_str(), wide.size() + 1);
    std::wmemcpy(g_dumpPathW, dumpWide.c_str(), dumpWide.size() + 1);
    const std::wstring logWide = QDir::toNativeSeparators(logFile).toStdWString();
    g_logPathW[0] = L'\0';
    if (!logFile.isEmpty() && logWide.size() < std::size(g_logPathW)) {
        std::wmemcpy(g_logPathW, logWide.c_str(), logWide.size() + 1);
    }
    SetUnhandledExceptionFilter(onException);
#else
    const QByteArray log = QFile::encodeName(logFile);
    g_logPath[0] = '\0';
    if (!logFile.isEmpty() && static_cast<std::size_t>(log.size()) < sizeof(g_logPath)) {
        std::memcpy(g_logPath, log.constData(), static_cast<std::size_t>(log.size()) + 1);
    }
#ifdef HZ_HAVE_BACKTRACE
    // The first backtrace loads what it needs; not first in the handler.
    void* warm[1];
    ::backtrace(warm, 1);
#endif
    stack_t stack{};
    stack.ss_sp = g_altStack;
    stack.ss_size = sizeof(g_altStack);
    ::sigaltstack(&stack, nullptr);
    struct sigaction action {};
    action.sa_sigaction = onSignal;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_SIGINFO | SA_ONSTACK | SA_RESETHAND;
    for (const int sig : {SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT}) {
        ::sigaction(sig, &action, nullptr);
    }
#endif
    return true;
}

QStringList pendingReports(const QString& directory) {
    QStringList out;
    const QDir dir(directory);
    for (const QString& name :
         dir.entryList({QStringLiteral("crash-*.txt")}, QDir::Files, QDir::Name)) {
        out << dir.filePath(name);
    }
    return out;
}

QString readReport(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) return {};
    return QString::fromUtf8(file.readAll());
}

bool markShown(const QString& path) {
    return QFile::rename(path, path + QStringLiteral(".shown")) || QFile::remove(path);
}

int prune(const QString& directory, int keep) {
    const QDir dir(directory);
    // Named by the time they were made: by name is oldest first.
    const QStringList shown =
        dir.entryList({QStringLiteral("crash-*.txt.shown")}, QDir::Files, QDir::Name);
    int removed = 0;
    for (qsizetype i = 0; i + std::max(keep, 0) < shown.size(); ++i) {
        const QString& name = shown[i];
        if (!QFile::remove(dir.filePath(name))) continue;
        ++removed;
        // "crash-X.txt.shown" was made beside "crash-X.dmp" (Windows).
        QFile::remove(dir.filePath(name.chopped(10) + QStringLiteral(".dmp")));
    }
    return removed;
}

void crashForTest() {
#if defined(_WIN32)
    RaiseException(EXCEPTION_ACCESS_VIOLATION, 0, 0, nullptr);
#else
    ::raise(SIGSEGV);
#endif
    std::abort();  // not reached
}

}  // namespace hz::ui::crash
