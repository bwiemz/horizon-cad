// A process that crashes on purpose, for test_CrashReport.cpp: it installs the
// crash handler as the application does, then crashes as it is told.
//
//   hz_crash_child <report directory> <log file> [raise|fault|overflow]
//
// raise: crashForTest(), as `horizon --crash-for-test` does; fault: a write to
// an address nothing is mapped at; overflow: recursion until the stack ends.

#include <QString>
#include <cstdint>
#include <cstring>

#include "horizon/ui/CrashReport.h"

#if defined(_WIN32)
#include <windows.h>
#endif

namespace {

int recurse(int depth) {
    volatile char frame[1024];
    frame[0] = static_cast<char>(depth);
    // Never reached (the stack ends long before); there so the recursion has
    // an end a compiler can see.
    if (depth > 100'000'000) return 0;
    return recurse(depth + 1) + frame[0];
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 3 && argc != 4) return 2;
#if defined(_WIN32)
    // No "stopped working" box for a crash made on purpose: it would wait for
    // someone to close it.
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
#endif
    if (!hz::ui::crash::install(QString::fromLocal8Bit(argv[1]), QString::fromLocal8Bit(argv[2]),
                                QStringLiteral("Horizon CAD test child"))) {
        return 3;
    }
    const char* how = argc == 4 ? argv[3] : "raise";
    if (std::strcmp(how, "fault") == 0) {
        // NOLINTNEXTLINE(performance-no-int-to-ptr): a pointer to nothing is the point.
        auto* const nowhere = reinterpret_cast<volatile int*>(std::uintptr_t{16});
        *nowhere = 1;
    } else if (std::strcmp(how, "overflow") == 0) {
        return recurse(0);
    }
    hz::ui::crash::crashForTest();
}
