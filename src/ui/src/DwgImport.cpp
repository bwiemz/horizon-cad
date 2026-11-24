#include "horizon/ui/DwgImport.h"

#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QStringList>
#include <QTemporaryDir>
#include <algorithm>
#include <atomic>
#include <mutex>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX  // windows.h's min and max macros would take std::max
#endif
#include <windows.h>
#endif

#include "horizon/document/Document.h"
#include "horizon/fileio/AtomicFile.h"
#include "horizon/fileio/DxfFormat.h"

namespace hz::ui {

namespace {

/// How long dwg2dxf has to start, and to stop once told to.
constexpr int kStartMs = 30000;
/// How often a reading looks to see whether it has been cancelled.
constexpr int kPollMs = 50;
/// How much of what dwg2dxf writes to its standard error is read, from its
/// end: its reason is there, and its warnings, one to an object, can run to
/// megabytes.
constexpr qint64 kMessagesTail = 64 * 1024;

/// LibreDWG's error bits (Dwg_Error in its dwg.h) that end a reading, as
/// dwg2dxf reports them: "READ ERROR 0x800".
constexpr unsigned kClassesNotFound = 1U << 7U;
constexpr unsigned kSectionNotFound = 1U << 8U;
constexpr unsigned kPageNotFound = 1U << 9U;
constexpr unsigned kInternalError = 1U << 10U;
constexpr unsigned kInvalidDwg = 1U << 11U;
constexpr unsigned kIoError = 1U << 12U;
constexpr unsigned kOutOfMemory = 1U << 13U;

std::mutex& converterMutex() {
    static std::mutex mutex;
    return mutex;
}

QString& chosenConverter() {
    static QString program;
    return program;
}

std::atomic<int>& timeout() {
    static std::atomic<int> milliseconds{5 * 60 * 1000};
    return milliseconds;
}

bool fail(std::string* error, const QString& why) {
    if (error) *error = why.toStdString();
    return false;
}

/// Why dwg2dxf could not read a file, from what it wrote to its standard
/// error: the bits of its "READ ERROR", or failing those its last line.
QString whyNotRead(const QString& messages) {
    static const QRegularExpression code(QStringLiteral("READ ERROR 0x([0-9a-fA-F]+)"));
    const QRegularExpressionMatch match = code.match(messages);
    const unsigned bits = match.hasMatch() ? match.captured(1).toUInt(nullptr, 16) : 0U;
    if ((bits & kOutOfMemory) != 0U) {
        return QCoreApplication::translate("hz::ui::DwgImport",
                                           "there was not memory enough to read it");
    }
    if ((bits & kIoError) != 0U) {
        return QCoreApplication::translate("hz::ui::DwgImport", "LibreDWG could not read the file");
    }
    if ((bits & kInvalidDwg) != 0U) {
        return QCoreApplication::translate("hz::ui::DwgImport",
                                           "it is not a DWG file, or not one LibreDWG can read");
    }
    if ((bits & (kClassesNotFound | kSectionNotFound | kPageNotFound)) != 0U) {
        return QCoreApplication::translate(
            "hz::ui::DwgImport", "parts of it are missing: it may be damaged, or cut short");
    }
    if ((bits & kInternalError) != 0U) {
        return QCoreApplication::translate("hz::ui::DwgImport",
                                           "LibreDWG met something in it that it cannot read yet");
    }
    const QStringList lines = messages.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
    const QString last = lines.isEmpty() ? QString() : lines.last().trimmed();
    return last.isEmpty()
               ? QCoreApplication::translate("hz::ui::DwgImport", "LibreDWG could not read it")
               : QCoreApplication::translate("hz::ui::DwgImport", "LibreDWG could not read it (%1)")
                     .arg(last);
}

}  // namespace

bool DwgImport::load(const std::string& path, doc::Document& doc, std::string* error,
                     io::ImportReport* report, const std::atomic<bool>* cancelled) {
    const QString program = converter();
    if (program.isEmpty()) {
        return fail(error, QCoreApplication::translate(
                               "hz::ui::DwgImport",
                               "Horizon CAD reads DWG files with LibreDWG's dwg2dxf, which this "
                               "computer does not have: install LibreDWG, or save the drawing "
                               "as DXF in the program that made it"));
    }
    if (std::string why = io::whyUnreadable(io::pathFromUtf8(path)); !why.empty()) {
        return fail(error, QString::fromStdString(why));
    }
    QTemporaryDir work;
    if (!work.isValid()) {
        return fail(error,
                    QCoreApplication::translate("hz::ui::DwgImport",
                                                "there is no temporary folder to read it in (%1)")
                        .arg(work.errorString()));
    }
    // dwg2dxf opens files by narrow names, which on Windows hold only the
    // characters of the code page: it reads a copy, by a plain name, in the
    // folder it runs in, and writes beside it the same name with ".dxf".
    const QString input = QStringLiteral("drawing.dwg");
    const QString output = QStringLiteral("drawing.dxf");
    if (!QFile::copy(QString::fromStdString(path), work.filePath(input))) {
        return fail(error,
                    QCoreApplication::translate(
                        "hz::ui::DwgImport", "the file could not be read (check its permissions)"));
    }

    QProcess process;
    process.setProgram(program);
    // The drawing and nothing else. Where there is no getopt (MSVC), LibreDWG
    // builds dwg2dxf with its own (programs/getopt.c), which starts again at
    // the first argument on every call: given any option it reads that option
    // for ever, and never reads the file (#238).
    process.setArguments({input});
    process.setWorkingDirectory(work.path());
    QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
    // Its tracing writes megabytes for a drawing.
    environment.remove(QStringLiteral("LIBREDWG_TRACE"));
    process.setProcessEnvironment(environment);
    process.setStandardInputFile(QProcess::nullDevice());
    process.setStandardOutputFile(QProcess::nullDevice());
    const QString messages = work.filePath(QStringLiteral("messages.txt"));
    process.setStandardErrorFile(messages);
#ifdef Q_OS_WIN
    // A program that cannot load (a DLL missing) or that crashes shows a
    // dialog on Windows and waits for it to be answered: no one answers it,
    // and the reading waited out its time saying nothing of why. A child
    // inherits its parent's error mode, so with this it fails at once, with
    // the status it failed with. Set only while it starts.
    const UINT errorMode =
        SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX | SEM_NOOPENFILEERRORBOX);
#endif
    process.start();
    const bool started = process.waitForStarted(kStartMs);
#ifdef Q_OS_WIN
    SetErrorMode(errorMode);
#endif
    if (!started) {
        return fail(error,
                    QCoreApplication::translate("hz::ui::DwgImport",
                                                "LibreDWG's dwg2dxf (%1) could not be started: %2")
                        .arg(QDir::toNativeSeparators(program), process.errorString()));
    }
    // Waited for a little at a time, so that a cancelled reading, or a
    // window closing, stops it at once rather than when it is done.
    const int limit = timeoutMs();
    QElapsedTimer clock;
    clock.start();
    while (!process.waitForFinished(kPollMs) && process.state() != QProcess::NotRunning) {
        const bool stop = cancelled != nullptr && cancelled->load();
        if (!stop && clock.elapsed() < limit) continue;
        process.kill();
        process.waitForFinished(kStartMs);
        if (stop) {
            return fail(error, QCoreApplication::translate("hz::ui::DwgImport",
                                                           "the reading was cancelled"));
        }
        return fail(error, QCoreApplication::translate(
                               "hz::ui::DwgImport",
                               "LibreDWG had not read it after %n second(s), and was stopped",
                               nullptr, std::max(1, limit / 1000)));
    }
    if (process.exitStatus() == QProcess::CrashExit) {
        // The status it stopped with (on Windows 0xC0000135: a DLL missing;
        // 0xC0000005: an access violation), for whoever reads the log.
        return fail(error,
                    QCoreApplication::translate(
                        "hz::ui::DwgImport",
                        "LibreDWG stopped while reading it: the file may be damaged, or "
                        "hold something LibreDWG cannot read yet") +
                        QStringLiteral(" [0x%1]").arg(static_cast<quint32>(process.exitCode()), 8,
                                                      16, QLatin1Char('0')));
    }
    if (process.exitCode() != 0) {
        QFile said(messages);
        QByteArray tail;
        if (said.open(QIODevice::ReadOnly)) {
            said.seek(std::max<qint64>(0, said.size() - kMessagesTail));
            tail = said.readAll();
        }
        return fail(error, whyNotRead(QString::fromLocal8Bit(tail)));
    }
    // Named after the drawing, except by LibreDWG's MSVC builds: their own
    // basename() finds no name in one with no folder in it, and they write
    // ".dxf" (#238). The folder holds nothing else of the kind.
    QString written = work.filePath(output);
    if (!QFileInfo::exists(written)) {
        written = work.filePath(QStringLiteral(".dxf"));
    }
    if (QFileInfo(written).size() == 0) {
        return fail(error, QCoreApplication::translate(
                               "hz::ui::DwgImport", "LibreDWG read it, but wrote nothing of it"));
    }
    std::string why;
    if (!io::DxfFormat::load(written.toStdString(), doc, &why, report)) {
        return fail(error, QCoreApplication::translate(
                               "hz::ui::DwgImport",
                               "LibreDWG read it, but what it made of it could not be read: %1")
                               .arg(QString::fromStdString(why)));
    }
    return true;
}

QString DwgImport::converter() {
    {
        const std::lock_guard<std::mutex> lock(converterMutex());
        if (!chosenConverter().isEmpty()) return chosenConverter();
    }
    const QString name = QStringLiteral("dwg2dxf");
    if (QCoreApplication::instance() != nullptr) {
        // Not const: returned, it is moved.
        QString beside =
            QStandardPaths::findExecutable(name, {QCoreApplication::applicationDirPath()});
        if (!beside.isEmpty()) return beside;
    }
    return QStandardPaths::findExecutable(name);
}

void DwgImport::setConverter(const QString& program) {
    const std::lock_guard<std::mutex> lock(converterMutex());
    chosenConverter() = program;
}

int DwgImport::timeoutMs() {
    return timeout().load();
}

void DwgImport::setTimeoutMs(int milliseconds) {
    timeout().store(std::max(1, milliseconds));
}

bool DwgImport::isDwg(const QString& fileName) {
    return fileName.endsWith(QStringLiteral(".dwg"), Qt::CaseInsensitive);
}

bool DwgImport::loadDrawing(const std::string& path, doc::Document& doc, std::string* error,
                            io::ImportReport* report, const std::atomic<bool>* cancelled) {
    if (isDwg(QString::fromStdString(path))) return load(path, doc, error, report, cancelled);
    return io::DxfFormat::load(path, doc, error, report);
}

}  // namespace hz::ui
