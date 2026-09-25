#include <spdlog/spdlog.h>

#include <QCommandLineParser>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QIcon>
#include <QLocale>
#include <QPalette>
#include <QSettings>
#include <QStandardPaths>
#include <QStyleFactory>
#include <QSurfaceFormat>
#include <QSysInfo>
#include <QTextStream>
#include <QTimer>

#include "horizon/Revision.h"
#include "horizon/Version.h"
#include "horizon/ui/Application.h"
#include "horizon/ui/LocaleManager.h"
#include "horizon/ui/Logging.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

// Suppress a specific Qt 6.10 qpixmap_win.cpp assertion on MSVC debug builds.
// Qt's internal bitmap mask operations trigger:
//   ASSERT: "bm.format() == QImage::Format_Mono"
// This is a Qt bug that does not affect functionality.  We install a targeted
// CRT report hook that suppresses ONLY this assertion's dialog, letting all
// other CRT error reports display normally.
#if defined(_MSC_VER) && defined(_DEBUG)
#include <crtdbg.h>

#include <cwchar>

static int __cdecl suppressQtBitmapAssert(int reportType, wchar_t* message, int* returnValue) {
    // Only suppress the specific Qt bitmap-mask assertion.
    if (reportType == _CRT_ERROR && message && wcsstr(message, L"bm.format()")) {
        if (returnValue) *returnValue = 0;  // 0 = don't break into debugger
        return 1;                           // 1 (TRUE) = handled, skip further CRT processing
    }
    return 0;  // 0 (FALSE) = not handled, continue normal CRT processing
}
#endif

static void applyDarkTheme(QApplication& app) {
    app.setStyle(QStyleFactory::create("Fusion"));

    QPalette p;
    p.setColor(QPalette::Window, QColor(45, 45, 45));
    p.setColor(QPalette::WindowText, QColor(208, 208, 208));
    p.setColor(QPalette::Base, QColor(30, 30, 30));
    p.setColor(QPalette::AlternateBase, QColor(36, 36, 36));
    p.setColor(QPalette::ToolTipBase, QColor(60, 60, 60));
    p.setColor(QPalette::ToolTipText, QColor(208, 208, 208));
    p.setColor(QPalette::Text, QColor(208, 208, 208));
    p.setColor(QPalette::Button, QColor(51, 51, 51));
    p.setColor(QPalette::ButtonText, QColor(208, 208, 208));
    p.setColor(QPalette::BrightText, QColor(255, 50, 50));
    p.setColor(QPalette::Link, QColor(74, 144, 217));
    p.setColor(QPalette::Highlight, QColor(74, 144, 217));
    p.setColor(QPalette::HighlightedText, QColor(255, 255, 255));

    p.setColor(QPalette::Disabled, QPalette::WindowText, QColor(112, 112, 112));
    p.setColor(QPalette::Disabled, QPalette::Text, QColor(112, 112, 112));
    p.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(112, 112, 112));

    app.setPalette(p);

    // Load supplementary QSS stylesheet.
    QFile qss(":/styles/dark.qss");
    if (qss.open(QIODevice::ReadOnly | QIODevice::Text)) {
        app.setStyleSheet(qss.readAll());
        qss.close();
    }
}

/// horizon --self-test: the window shown, wait for its viewport to draw a
/// frame, and report. Exit 0 once it has, 3 if it cannot (and why), 4 if it
/// has not within 20 s. Neither a recovered session nor files are
/// opened: a dialog must not keep it waiting. The packages are started this
/// way before they are shipped, since every test runs headless, where a
/// Qt with no platform plugin passed them all.
static int runSelfTest(QApplication& app, hz::ui::MainWindow& window) {
    auto* viewport = window.findChild<hz::ui::ViewportWidget*>();
    QTextStream out(stdout);
    if (viewport == nullptr) {
        out << "self-test: no viewport\n";
        return 3;
    }
    QElapsedTimer clock;
    clock.start();
    QTimer poll;
    QObject::connect(&poll, &QTimer::timeout, &app, [&] {
        if (viewport->hasDrawn()) {
            app.exit(0);
        } else if (!viewport->graphicsProblem().isEmpty()) {
            app.exit(3);  // a "Graphics Problem" box may be up: exit() ends its loop too
        } else if (clock.elapsed() > 20000) {
            app.exit(4);
        }
    });
    poll.start(100);
    const int status = app.exec();
    switch (status) {
        case 0:
            out << "self-test: the window is shown and its viewport draws ("
                << QGuiApplication::platformName() << ")\n";
            break;
        case 3:
            out << "self-test: the viewport cannot draw: " << viewport->graphicsProblem() << "\n";
            break;
        default:
            out << "self-test: the viewport drew nothing within 20 s\n";
            break;
    }
    spdlog::info("Self-test finished (status {})", status);
    return status;
}

/// Everything that needs the application object. Kept apart from main() so the
/// application — and every widget — is destroyed before logging is shut down:
/// their destructors may still log, or reach std::terminate.
static int run(int argc, char* argv[]) {
    // Contains exceptions that escape event handlers instead of terminating.
    hz::ui::Application app(argc, argv);
    app.setApplicationName("Horizon CAD");
    app.setOrganizationName("Horizon CAD Project");
    app.setApplicationVersion(QString::fromLatin1(hz::version::kString));
    app.setWindowIcon(QIcon(QStringLiteral(":/icons/horizon-cad.png")));
    // Wayland and X11 match the window to its .desktop entry by this name.
    app.setDesktopFileName(QStringLiteral("io.github.bwiemz.HorizonCAD"));

    // Logging comes first so everything after it — including Qt's own
    // warnings — lands in the log file.
    const QString logFile = hz::ui::initializeLogging(
        QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation) + "/logs");
    spdlog::info("Horizon CAD {} ({}) starting (Qt {}, {})", hz::version::kString,
                 hz::version::kRevision, qVersion(), QSysInfo::prettyProductName().toStdString());
    if (!logFile.isEmpty()) spdlog::info("Log file: {}", logFile.toStdString());

    applyDarkTheme(app);

    // Load the UI translation before any widgets are built (Phase 77).
    // An explicit choice in settings wins; otherwise follow the system locale.
    hz::ui::LocaleManager localeManager;
    const QString translationsDir =
        QDir(QApplication::applicationDirPath()).filePath("translations");
    const QString uiLanguage =
        QSettings().value("ui/language", QLocale::system().name()).toString();
    if (localeManager.apply(translationsDir, uiLanguage)) {
        spdlog::info("UI language: {}", uiLanguage.toStdString());
    }

    // horizon [file...]: files named on the command line open in tabs.
    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("Horizon CAD"));
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addPositionalArgument(QStringLiteral("files"),
                                 QCoreApplication::translate("main", "Files to open."),
                                 QStringLiteral("[files...]"));
    const QCommandLineOption selfTest(
        QStringLiteral("self-test"),
        QCoreApplication::translate("main",
                                    "Open the window, check that its viewport can draw, say "
                                    "what was found, and exit: 0 if it can."));
    parser.addOption(selfTest);
    parser.process(app);
    const QStringList files = parser.positionalArguments();

    hz::ui::MainWindow window;
    window.show();
    if (parser.isSet(selfTest)) return runSelfTest(app, window);
    // Once the window is on screen: offer back what a crashed session left,
    // then open what was asked for.
    QTimer::singleShot(0, &window, [&window, files] {
        window.offerRecovery();
        window.openFiles(files);
    });
    return app.exec();
}

int main(int argc, char* argv[]) {
#if defined(_MSC_VER) && defined(_DEBUG)
    // Install targeted hook to suppress only the Qt 6.10 bitmap-mask assertion.
    _CrtSetReportHookW2(_CRT_RPTHOOK_INSTALL, suppressQtBitmapAssert);
#endif
    hz::ui::Application::installTerminateHandler();

#if defined(__linux__)
    // A release links vcpkg's fontconfig into the application, and it looks
    // for its configuration where it was built: "Cannot load default config
    // file" on every start, and the system's font settings unread. Point it
    // at the system's, unless the user already chose one.
    if (qEnvironmentVariableIsEmpty("FONTCONFIG_FILE") &&
        QFile::exists(QStringLiteral("/etc/fonts/fonts.conf"))) {
        qputenv("FONTCONFIG_FILE", "/etc/fonts/fonts.conf");
    }
#endif

    // Request an OpenGL 3.3 Core Profile context.
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setSamples(4);
    QSurfaceFormat::setDefaultFormat(format);

    const int status = run(argc, argv);
    spdlog::info("Horizon CAD exiting (status {})", status);
    hz::ui::shutdownLogging();
    return status;
}
