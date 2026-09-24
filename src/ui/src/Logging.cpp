#include "horizon/ui/Logging.h"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <QDir>
#include <QtGlobal>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace hz::ui {

namespace {

QString g_logFile;

spdlog::level::level_enum levelFor(QtMsgType type) {
    switch (type) {
        case QtDebugMsg:
            return spdlog::level::debug;
        case QtInfoMsg:
            return spdlog::level::info;
        case QtWarningMsg:
            return spdlog::level::warn;
        case QtCriticalMsg:
            return spdlog::level::err;
        case QtFatalMsg:
            return spdlog::level::critical;
    }
    return spdlog::level::info;
}

void logQtMessage(QtMsgType type, const QMessageLogContext& context, const QString& message) {
    const std::string text = message.toStdString();
    if (context.category != nullptr && std::strcmp(context.category, "default") != 0) {
        spdlog::log(levelFor(type), "[qt:{}] {}", context.category, text);
    } else {
        spdlog::log(levelFor(type), "[qt] {}", text);
    }
    // Qt aborts after a fatal message returns; make sure it is on disk first.
    if (type == QtFatalMsg) spdlog::default_logger()->flush();
}

constexpr std::size_t kMaxLogBytes = 5u * 1024u * 1024u;
constexpr std::size_t kLogFiles = 3;

}  // namespace

QString initializeLogging(const QString& logDirectory) {
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_mt>());

    QString file;
    std::string fileError;
    if (QDir().mkpath(logDirectory)) {
        file = QDir(logDirectory).filePath(QStringLiteral("horizon.log"));
        try {
            sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                file.toStdString(), kMaxLogBytes, kLogFiles));
        } catch (const spdlog::spdlog_ex& e) {
            fileError = e.what();
            file.clear();
        }
    } else {
        fileError = "cannot create " + logDirectory.toStdString();
    }

    auto logger = std::make_shared<spdlog::logger>("horizon", sinks.begin(), sinks.end());
    logger->set_level(spdlog::level::info);
    logger->flush_on(spdlog::level::warn);
    spdlog::set_default_logger(logger);
    qInstallMessageHandler(logQtMessage);

    if (!fileError.empty()) {
        spdlog::warn("Logging to stderr only; the log file could not be opened: {}", fileError);
    }
    g_logFile = file;
    return file;
}

QString logFilePath() {
    return g_logFile;
}

void shutdownLogging() {
    qInstallMessageHandler(nullptr);
    // Flush but keep the logger: spdlog::shutdown() would null the default
    // logger, and anything that logs afterwards — a destructor, the terminate
    // handler — would dereference it (spdlog's free functions do not check).
    if (auto logger = spdlog::default_logger()) logger->flush();
}

}  // namespace hz::ui
