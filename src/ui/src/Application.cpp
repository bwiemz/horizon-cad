#include "horizon/ui/Application.h"

#include <spdlog/spdlog.h>

#include <QMessageBox>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

#include "horizon/ui/Logging.h"

namespace hz::ui {

namespace {

/// A handler that throws on every repaint would otherwise raise a dialog per
/// frame; the log still records each one.
constexpr qint64 kMinMillisecondsBetweenDialogs = 10'000;

}  // namespace

Application::Application(int& argc, char** argv) : QApplication(argc, argv) {}

bool Application::notify(QObject* receiver, QEvent* event) {
    try {
        return QApplication::notify(receiver, event);
    } catch (const std::exception& e) {
        reportException(QString::fromUtf8(e.what()));
    } catch (...) {
        reportException(tr("an unknown error"));
    }
    return false;
}

void Application::reportException(const QString& what) {
    ++m_containedCount;
    spdlog::error("Contained an exception from an event handler: {}", what.toStdString());

    // One dialog at a time, and not in a storm.
    if (m_reporting) return;
    if (m_lastDialog.isValid() && m_lastDialog.elapsed() < kMinMillisecondsBetweenDialogs) return;

    m_reporting = true;
    const QString log = logFilePath();
    QString message = tr("An operation failed unexpectedly and was abandoned:\n\n%1\n\n"
                         "Your documents are still open. If anything looks wrong, save your "
                         "work under a new name and restart Horizon CAD.")
                          .arg(what);
    if (!log.isEmpty()) message += tr("\n\nDetails are in the log:\n%1").arg(log);
    QMessageBox::critical(activeWindow(), tr("Unexpected Error"), message);
    m_lastDialog.start();
    m_reporting = false;
}

void Application::installTerminateHandler() {
    std::set_terminate([] {
        std::string reason = "std::terminate called without an active exception";
        if (const std::exception_ptr current = std::current_exception()) {
            try {
                std::rethrow_exception(current);
            } catch (const std::exception& e) {
                reason = std::string("uncaught exception: ") + e.what();
            } catch (...) {
                reason = "uncaught non-standard exception";
            }
        }
        // Terminate can fire before logging starts or during teardown; the
        // default logger may not exist then, and spdlog does not check.
        if (auto logger = spdlog::default_logger()) {
            logger->critical("Terminating: {}", reason);
            logger->flush();
        } else {
            std::fprintf(stderr, "Horizon CAD terminating: %s\n", reason.c_str());
        }
        std::abort();
    });
}

}  // namespace hz::ui
