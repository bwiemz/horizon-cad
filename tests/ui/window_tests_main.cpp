// Entry point for the tests that need a full QApplication (hz_ui_window_tests).

#include <gtest/gtest.h>

#include "horizon/ui/Application.h"

int main(int argc, char** argv) {
    // Headless by default; an explicit QT_QPA_PLATFORM still wins.
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
        qputenv("QT_QPA_PLATFORM", "offscreen");
    }
    // The application the product runs under, so exception containment is
    // exercised too.
    hz::ui::Application app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
