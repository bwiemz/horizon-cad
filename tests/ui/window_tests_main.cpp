// Entry point for the tests that need a full QApplication (hz_ui_window_tests).

#include <gtest/gtest.h>

#include <QDir>
#include <QStandardPaths>

#include "horizon/ui/Application.h"

int main(int argc, char** argv) {
    // Headless by default; an explicit QT_QPA_PLATFORM still wins.
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
        qputenv("QT_QPA_PLATFORM", "offscreen");
    }
    // The application the product runs under, so exception containment is
    // exercised too.
    hz::ui::Application app(argc, argv);

    // Never touch the user's real data (autosave snapshots, settings), and
    // give each test process its own directory: CTest runs them in parallel,
    // and one test's "crashed session" must not be recovered by another.
    QStandardPaths::setTestModeEnabled(true);
    QCoreApplication::setOrganizationName(QStringLiteral("HorizonCadTests"));
    QCoreApplication::setApplicationName(
        QStringLiteral("hz_ui_window_tests-%1").arg(QCoreApplication::applicationPid()));

    ::testing::InitGoogleTest(&argc, argv);
    const int status = RUN_ALL_TESTS();
    QDir data(QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation));
    data.removeRecursively();
    data.cdUp();
    data.rmdir(data.absolutePath());  // the organization directory, once empty
    return status;
}
