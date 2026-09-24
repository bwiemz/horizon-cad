// Application essentials (Phase 111).

#include <gtest/gtest.h>

#include <QAction>
#include <QDockWidget>
#include <QFileInfo>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QSettings>
#include <QTabBar>
#include <QTemporaryDir>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/RecentFiles.h"

using hz::test::DialogResponder;
using hz::ui::MainWindow;
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
