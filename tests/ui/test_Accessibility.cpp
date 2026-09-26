// Phase 166b: every control a screen reader reaches has a name. Qt names
// most by their text (a button's, its action's) or by the label a form gives
// them; what it cannot name must be named in code.

#include <gtest/gtest.h>

#include <QAbstractButton>
#include <QAccessible>
#include <QAccessibleInterface>
#include <QAction>
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QTabBar>
#include <QWidget>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "UiTestSupport.h"
#include "horizon/document/ConfigurationTable.h"
#include "horizon/ui/CommandPalette.h"
#include "horizon/ui/ConfigurationsDialog.h"
#include "horizon/ui/FeatureForm.h"
#include "horizon/ui/HelpWindow.h"
#include "horizon/ui/InsertBlockDialog.h"
#include "horizon/ui/LocaleManager.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/PolarArrayDialog.h"
#include "horizon/ui/Preferences.h"
#include "horizon/ui/PreferencesDialog.h"
#include "horizon/ui/RectArrayDialog.h"
#include "horizon/ui/VariablesDialog.h"
#include "horizon/ui/ViewportWidget.h"

using hz::test::accessibleName;
using hz::test::unnamedControls;
using hz::ui::MainWindow;

TEST(AccessibilityTest, EveryControlOfTheWindowHasAName) {
    MainWindow w;
    w.show();
    QApplication::processEvents();
    const auto unnamed = unnamedControls(w);
    for (const auto& control : unnamed) ADD_FAILURE() << "no accessible name: " << control;

    // A tab's close button says which tab it closes.
    auto* tabs = w.findChild<QTabBar*>(QStringLiteral("documentTabs"));
    ASSERT_NE(tabs, nullptr);
    QWidget* close = tabs->tabButton(0, QTabBar::RightSide);
    if (close == nullptr) close = tabs->tabButton(0, QTabBar::LeftSide);
    ASSERT_NE(close, nullptr);
    EXPECT_EQ(accessibleName(close), QStringLiteral("Close ") + tabs->tabText(0));
}

// A form's fields are named by their labels, as Qt names a field that a form
// layout gives a label: a screen reader says "Depth:", not "spin box".
TEST(AccessibilityTest, AFormsFieldsAreNamedByTheirLabels) {
    hz::ui::FeatureForm form(nullptr, QStringLiteral("Extrude"));
    form.length(QStringLiteral("depth"), QStringLiteral("Depth:"), 10.0, 0.0, 100.0);
    form.angle(QStringLiteral("angle"), QStringLiteral("Angle:"), 30.0, 0.0, 90.0);
    form.count(QStringLiteral("count"), QStringLiteral("Instances:"), 3, 1, 10);
    form.choice(QStringLiteral("way"), QStringLiteral("Goes:"), {QStringLiteral("Up")});
    form.text(QStringLiteral("name"), QStringLiteral("Name:"));
    form.checklist(QStringLiteral("edges"), QStringLiteral("Edges:"), {{QStringLiteral("e1"), {}}});
    form.checklist(QStringLiteral("faces"), QStringLiteral("&Faces && loops &"),
                   {{QStringLiteral("f1"), {}}});
    form.dialog().show();
    QApplication::processEvents();
    const auto unnamed = unnamedControls(form.dialog());
    for (const auto& control : unnamed) ADD_FAILURE() << "no accessible name: " << control;
    const QString depth =
        accessibleName(form.dialog().findChild<QWidget*>(QStringLiteral("depth")));
    EXPECT_TRUE(depth.startsWith(QStringLiteral("Depth"))) << depth.toStdString();
    // Qt 6.9 does not name an item view by its label; the form names it.
    EXPECT_EQ(accessibleName(form.dialog().findChild<QWidget*>(QStringLiteral("edges"))),
              QStringLiteral("Edges:"));
    EXPECT_EQ(accessibleName(form.dialog().findChild<QWidget*>(QStringLiteral("faces"))),
              QStringLiteral("Faces & loops &"))
        << "a mnemonic's & dropped, && a literal &, and a last & kept";
    form.dialog().hide();
}

// The dialogs' controls are named too: the window's walk never opens them.
// (The Bill of Materials is checked where its test opens it.)
TEST(AccessibilityTest, EveryControlOfEachDialogHasAName) {
    QAction open(QStringLiteral("Open"));
    QAction save(QStringLiteral("Save"));
    const std::map<std::string, std::string> variables{{"wall", "3 mm"}};
    std::vector<std::pair<const char*, std::unique_ptr<QWidget>>> dialogs;
    dialogs.emplace_back("Command Palette",
                         std::make_unique<hz::ui::CommandPalette>(QList<QAction*>{&open, &save}));
    dialogs.emplace_back("Variables", std::make_unique<hz::ui::VariablesDialog>(
                                          variables, hz::math::LengthUnit::Millimetre));
    dialogs.emplace_back("Configurations", std::make_unique<hz::ui::ConfigurationsDialog>(
                                               hz::doc::ConfigurationTable{}, variables));
    dialogs.emplace_back("Preferences",
                         std::make_unique<hz::ui::PreferencesDialog>(
                             hz::ui::Preferences::load(), QStringList{QStringLiteral("de")}));
    dialogs.emplace_back("Polar Array", std::make_unique<hz::ui::PolarArrayDialog>());
    dialogs.emplace_back("Rectangular Array", std::make_unique<hz::ui::RectArrayDialog>());
    dialogs.emplace_back("Insert Block", std::make_unique<hz::ui::InsertBlockDialog>(
                                             std::vector<std::string>{"bolt"}));
    dialogs.emplace_back("User Guide", std::make_unique<hz::ui::HelpWindow>());
    for (auto& [name, dialog] : dialogs) {
        dialog->show();
        QApplication::processEvents();
        for (const auto& control : unnamedControls(*dialog)) {
            ADD_FAILURE() << name << ": no accessible name: " << control;
        }
        dialog->hide();
    }
}

// A name given in code is translated with the rest of the window: a German
// screen reader says "Ansichtsfenster", not "Viewport".
TEST(AccessibilityTest, TheNamesAreTranslated) {
#ifndef HZ_SHIPPED_TRANSLATIONS_DIR
    GTEST_SKIP() << "no application target in this build";
#else
    const QString dir = QStringLiteral(HZ_SHIPPED_TRANSLATIONS_DIR);
    if (!QFile::exists(QDir(dir).filePath(QStringLiteral("horizon_de.qm")))) {
        GTEST_SKIP() << "the catalogs were not compiled (no lrelease)";
    }
    hz::ui::LocaleManager german;
    ASSERT_TRUE(german.apply(dir, QStringLiteral("de")));
    MainWindow w;
    auto* viewport = w.findChild<hz::ui::ViewportWidget*>();
    ASSERT_NE(viewport, nullptr);
    EXPECT_EQ(accessibleName(viewport), QStringLiteral("Ansichtsfenster"));

    auto* tabs = w.findChild<QTabBar*>(QStringLiteral("documentTabs"));
    ASSERT_NE(tabs, nullptr);
    QWidget* close = tabs->tabButton(0, QTabBar::RightSide);
    if (close == nullptr) close = tabs->tabButton(0, QTabBar::LeftSide);
    ASSERT_NE(close, nullptr);
    EXPECT_EQ(accessibleName(close), tabs->tabText(0) + QStringLiteral(" schließen"));
#endif
}
