// Application essentials (Phase 111).

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QSettings>
#include <QSpinBox>
#include <QTabBar>
#include <QTemporaryDir>
#include <QTimer>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "UiTestSupport.h"
#include "horizon/Revision.h"
#include "horizon/Version.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/Preferences.h"
#include "horizon/ui/RecentFiles.h"
#include "horizon/ui/ViewportWidget.h"

using hz::test::DialogResponder;
using hz::ui::MainWindow;
using hz::ui::Preferences;
using hz::ui::RecentFiles;

namespace {

/// A one-line DXF drawing at `path`.
void writeDrawing(const QString& path) {
    hz::doc::Document doc;
    doc.draftDocument().addEntity(
        std::make_shared<hz::draft::DraftLine>(hz::math::Vec2(0, 0), hz::math::Vec2(1, 0)));
    std::string error;
    ASSERT_TRUE(hz::io::DxfFormat::save(path.toStdString(), doc, &error)) << error;
}

int tabCount(MainWindow& w) {
    return w.findChild<QTabBar*>(QStringLiteral("documentTabs"))->count();
}

QMenu* recentMenu(MainWindow& w) {
    auto* menu = w.findChild<QMenu*>(QStringLiteral("recentFilesMenu"));
    EXPECT_NE(menu, nullptr);
    if (menu) emit menu->aboutToShow();  // rebuilt as it opens
    return menu;
}

QAction* item(QMenu* menu, const QString& fileName) {
    for (QAction* a : menu->actions()) {
        if (a->text().endsWith(fileName)) return a;
    }
    return nullptr;
}

}  // namespace

TEST(AppEssentialsTest, EveryShortcutRunsOneCommand) {
    // A key bound to two actions is ambiguous, and Qt runs neither: Ctrl+Z,
    // Ctrl+S and eight more were bound once in the menus and again on the
    // ribbon, so they did nothing from the keyboard.
    MainWindow w;
    std::map<std::string, std::vector<std::string>> byKey;
    for (QAction* a : w.findChildren<QAction*>()) {
        for (const QKeySequence& k : a->shortcuts()) {
            if (!k.isEmpty()) byKey[k.toString().toStdString()].push_back(a->text().toStdString());
        }
    }
    for (const auto& [key, actions] : byKey) {
        std::string names;
        for (const auto& a : actions) names += " | " + a;
        EXPECT_EQ(actions.size(), 1u) << key << ":" << names;
    }
    ASSERT_TRUE(byKey.count("Ctrl+Z"));

    // The ribbon shows the shared action by its own short name.
    auto* newAction = w.findChild<QAction*>(QStringLiteral("action_new"));
    ASSERT_NE(newAction, nullptr);
    EXPECT_EQ(newAction->iconText(), QStringLiteral("New"));
    EXPECT_EQ(newAction->text(), QStringLiteral("&New Drawing")) << "and the menu by its own";
}

TEST(AppEssentialsTest, OpenedFilesAreFirstInOpenRecent) {
    RecentFiles::clear();
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString first = dir.filePath(QStringLiteral("first.dxf"));
    const QString second = dir.filePath(QStringLiteral("second.dxf"));
    writeDrawing(first);
    writeDrawing(second);

    MainWindow w;
    const int before = tabCount(w);
    w.openFiles({first, second});
    EXPECT_EQ(tabCount(w), before + 2) << "each file in a tab of its own";
    ASSERT_EQ(RecentFiles::list().size(), 2);
    EXPECT_EQ(QFileInfo(RecentFiles::list()[0]).fileName(), QStringLiteral("second.dxf"));

    // Choosing one that is already open shows its tab and moves it first.
    QAction* entry = item(recentMenu(w), QStringLiteral("first.dxf"));
    ASSERT_NE(entry, nullptr);
    entry->trigger();
    EXPECT_EQ(tabCount(w), before + 2);
    EXPECT_EQ(QFileInfo(RecentFiles::list()[0]).fileName(), QStringLiteral("first.dxf"));
    RecentFiles::clear();
}

TEST(AppEssentialsTest, AFileThatIsGoneIsTakenOffOpenRecent) {
    RecentFiles::clear();
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString gone = dir.filePath(QStringLiteral("gone.dxf"));
    writeDrawing(gone);
    RecentFiles::add(gone);
    QFile::remove(gone);

    MainWindow w;
    QAction* entry = item(recentMenu(w), QStringLiteral("gone.dxf"));
    ASSERT_NE(entry, nullptr);
    DialogResponder error(QMessageBox::Ok);
    entry->trigger();
    EXPECT_TRUE(error.seen()) << "the user is told";
    EXPECT_TRUE(RecentFiles::list().isEmpty()) << "and it is off the list";
}

TEST(AppEssentialsTest, OpenRecentKeepsTheTenNewest) {
    RecentFiles::clear();
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    for (int i = 0; i < 12; ++i) RecentFiles::add(dir.filePath(QStringLiteral("f%1.dxf").arg(i)));
    RecentFiles::add(dir.filePath(QStringLiteral("f5.dxf")));
    const QStringList files = RecentFiles::list();
    ASSERT_EQ(files.size(), RecentFiles::kMax);
    EXPECT_EQ(QFileInfo(files[0]).fileName(), QStringLiteral("f5.dxf")) << "again, so first";
    EXPECT_EQ(QFileInfo(files[1]).fileName(), QStringLiteral("f11.dxf"));
    EXPECT_EQ(files.count(files[0]), 1) << "and only once";
    EXPECT_FALSE(files.contains(dir.filePath(QStringLiteral("f0.dxf")))) << "the oldest go";
    RecentFiles::clear();
}

TEST(AppEssentialsTest, EveryDockCanBeShownFromTheViewMenu) {
    MainWindow w;
    QMenu* view = nullptr;
    for (QAction* top : w.menuBar()->actions()) {
        if (top->menu() && top->text().remove(QLatin1Char('&')) == QStringLiteral("View")) {
            view = top->menu();
        }
    }
    ASSERT_NE(view, nullptr);
    const auto docks = w.findChildren<QDockWidget*>();
    ASSERT_FALSE(docks.isEmpty());
    for (QDockWidget* dock : docks) {
        EXPECT_TRUE(view->actions().contains(dock->toggleViewAction()))
            << dock->objectName().toStdString() << " is not in View";
    }
}

TEST(AppEssentialsTest, TheWindowLayoutIsKeptForTheNextSession) {
    {
        MainWindow w;
        auto* tree = w.findChild<QDockWidget*>(QStringLiteral("FeatureTreePanel"));
        ASSERT_NE(tree, nullptr);
        tree->hide();
        ASSERT_TRUE(w.close());
    }
    {
        MainWindow w;
        auto* tree = w.findChild<QDockWidget*>(QStringLiteral("FeatureTreePanel"));
        ASSERT_NE(tree, nullptr);
        EXPECT_TRUE(tree->isHidden()) << "hidden as it was left";
    }
    QSettings().remove(QStringLiteral("window"));
}

TEST(AppEssentialsTest, PreferencesAreKeptAndPutIntoEffect) {
    MainWindow w;
    // Answer the dialog when it opens: a 5 mm grid, a 20 px snap, inches to
    // two places, and autosave off.
    bool answered = false;
    QTimer poll;
    QElapsedTimer clock;
    clock.start();
    QObject::connect(&poll, &QTimer::timeout, [&] {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr || dialog->objectName() != QStringLiteral("preferencesDialog")) {
            if (clock.elapsed() > 5000) poll.stop();
            return;
        }
        poll.stop();
        dialog->findChild<QSpinBox*>(QStringLiteral("autosaveMinutes"))->setValue(0);
        dialog->findChild<QDoubleSpinBox*>(QStringLiteral("gridSpacing"))->setValue(5.0);
        dialog->findChild<QSpinBox*>(QStringLiteral("snapPixels"))->setValue(20);
        auto* unit = dialog->findChild<QComboBox*>(QStringLiteral("lengthUnit"));
        unit->setCurrentIndex(unit->findData(QStringLiteral("in")));
        dialog->findChild<QSpinBox*>(QStringLiteral("decimals"))->setValue(2);
        dialog->findChild<QSpinBox*>(QStringLiteral("undoLimit"))->setValue(50);
        answered = true;
        dialog->accept();
    });
    poll.start(5);
    w.findChild<QAction*>(QStringLiteral("action_preferences"))->trigger();
    ASSERT_TRUE(answered);

    const Preferences saved = Preferences::load();
    EXPECT_DOUBLE_EQ(saved.gridSpacing, 5.0);
    EXPECT_EQ(saved.snapPixels, 20);
    EXPECT_EQ(saved.lengthUnit, QStringLiteral("in"));
    EXPECT_EQ(Preferences::current().decimals, 2);
    EXPECT_EQ(saved.undoLimit, 50);
    EXPECT_EQ(w.activeDocument()->undoStack().limit(), 50u) << "the open document keeps 50 steps";

    auto* viewport = w.findChild<hz::ui::ViewportWidget*>();
    ASSERT_NE(viewport, nullptr);
    EXPECT_DOUBLE_EQ(viewport->snapEngine().gridSpacing(), 5.0);
    EXPECT_DOUBLE_EQ(viewport->snapPixels(), 20.0);
    EXPECT_FALSE(w.findChild<QTimer*>(QStringLiteral("autosaveTimer"))->isActive())
        << "autosave is off";

    Preferences{}.save();  // back to the defaults for the tests after this one
}

TEST(AppEssentialsTest, LengthsAreShownInTheDisplayUnit) {
    Preferences p;
    EXPECT_EQ(p.formatLength(12.5), QStringLiteral("12.500 mm"));
    p.lengthUnit = QStringLiteral("in");
    EXPECT_EQ(p.formatLength(25.4), QStringLiteral("1.000 in"));
    EXPECT_EQ(p.formatArea(25.4 * 25.4 * 2), QStringLiteral("2.000 in\u00B2"));
    p.lengthUnit = QStringLiteral("m");
    p.decimals = 1;
    EXPECT_EQ(p.formatLength(1500.0), QStringLiteral("1.5 m"));
}

TEST(AppEssentialsTest, FollowingTheSystemLanguageStoresNoLanguage) {
    // main.cpp reads "ui/language" with the system locale as its default: a
    // stored empty string would force the source language instead.
    Preferences p;
    p.language = QStringLiteral("de");
    p.save();
    EXPECT_EQ(QSettings().value(QStringLiteral("ui/language")).toString(), QStringLiteral("de"));
    p.language.clear();
    p.save();
    EXPECT_FALSE(QSettings().contains(QStringLiteral("ui/language")));
}

TEST(AppEssentialsTest, AboutSaysWhatIsRunning) {
    MainWindow w;
    DialogResponder about(QMessageBox::Ok, QStringLiteral("About Horizon CAD"));
    w.findChild<QAction*>(QStringLiteral("action_about"))->trigger();
    ASSERT_TRUE(about.seen());
    EXPECT_TRUE(about.text().contains(QStringLiteral("Horizon CAD"))) << about.text().toStdString();
    EXPECT_TRUE(about.text().contains(QString::fromLatin1(hz::version::kString)))
        << "the version from project(): " << about.text().toStdString();
    // The GPL's "Appropriate Legal Notices" for an interactive program: the
    // licence, that there is no warranty, and where its text is.
    const QString notices = about.informativeText();
    EXPECT_TRUE(notices.contains(QStringLiteral("GNU General Public License, version 3")))
        << notices.toStdString();
    EXPECT_TRUE(notices.contains(QStringLiteral("NO WARRANTY"))) << notices.toStdString();
    EXPECT_TRUE(notices.contains(QStringLiteral("LICENSE"))) << notices.toStdString();
}

TEST(AppEssentialsTest, TheVersionIsOneNumberEverywhere) {
    // Phase 115: project() is the only place the version is set. The
    // generated header spells out its parts and the whole the same way.
    EXPECT_EQ(std::string(hz::version::kString), std::to_string(hz::version::kMajor) + "." +
                                                     std::to_string(hz::version::kMinor) + "." +
                                                     std::to_string(hz::version::kPatch));
    EXPECT_STRNE(hz::version::kRevision, "") << "a revision, or \"unknown\" outside git";
}
