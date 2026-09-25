// Drawing sheets (Phase 148): a sheet made from the active part, or from a
// part chosen, shows the part's standard views on paper, seen from above,
// on its own locked layers. It saves as a .hzdwg and opens again, its title
// block and scale are set in forms, and it follows its part when the part
// changes on disk or is saved in its tab.

#include <gtest/gtest.h>

#include <QAction>
#include <QFile>
#include <QRegularExpression>
#include <QStatusBar>
#include <QTabBar>
#include <QTemporaryDir>
#include <QTimer>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <string>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

using hz::test::FilePicker;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::ui::MainWindow;

namespace {

void trigger(MainWindow& w, const char* name) {
    auto* action = w.findChild<QAction*>(QString::fromLatin1(name));
    ASSERT_NE(action, nullptr) << name;
    action->trigger();
}

bool waitUntil(const std::function<bool()>& done, int ms) {
    QElapsedTimer clock;
    clock.start();
    while (!done() && clock.elapsed() < ms) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    }
    return done();
}

QTabBar* tabBar(MainWindow& w) {
    return w.findChild<QTabBar*>(QStringLiteral("documentTabs"));
}

/// A 40 x 20 block @p depth deep, built and saved at @p path. Saved over
/// one already there, its time moves on, however coarse the clock.
void saveBlock(const QString& path, double depth) {
    const bool existed = QFile::exists(path);
    hz::doc::Document part;
    part.setType(hz::doc::DocumentType::Part);
    part.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(40, 20, depth));
    ASSERT_TRUE(part.rebuildModel());
    ASSERT_TRUE(hz::io::NativeFormat::save(path.toStdString(), part));
    if (existed) {
        const std::filesystem::path file(path.toStdString());
        std::filesystem::last_write_time(
            file, std::filesystem::last_write_time(file) + std::chrono::seconds(2));
    }
}

/// What the title block says, one line per text.
QString titleBlockText(const hz::doc::Document& sheet) {
    QStringList lines;
    for (const auto& entity : sheet.draftDocument().entities()) {
        if (entity->layer() != "TitleBlock") continue;
        if (const auto* text = dynamic_cast<const hz::draft::DraftText*>(entity.get())) {
            lines << QString::fromStdString(text->text());
        }
    }
    return lines.join(QLatin1Char('\n'));
}

/// How many entities are on @p layer.
size_t countOn(const hz::doc::Document& sheet, const std::string& layer) {
    const auto& entities = sheet.draftDocument().entities();
    return static_cast<size_t>(std::count_if(entities.begin(), entities.end(),
                                             [&](const auto& e) { return e->layer() == layer; }));
}

/// The extent of what is on @p layer, along x (0) or y (1).
double extentOn(const hz::doc::Document& sheet, const std::string& layer, int axis) {
    double low = 1e300;
    double high = -1e300;
    for (const auto& entity : sheet.draftDocument().entities()) {
        if (entity->layer() != layer) continue;
        const auto box = entity->boundingBox();
        low = std::min(low, axis == 0 ? box.min().x : box.min().y);
        high = std::max(high, axis == 0 ? box.max().x : box.max().y);
    }
    return high > low ? high - low : 0.0;
}

/// Whether the view looks straight down at the paper.
bool seenFromAbove(MainWindow& w) {
    const auto& camera = w.findChild<hz::ui::ViewportWidget*>()->camera();
    const auto d = camera.eye() - camera.target();
    return std::abs(d.x) < 1e-9 && std::abs(d.y) < 1e-9 && d.z > 0.0;
}

/// Open the part at @p path and make a drawing of it: the sheet.
hz::doc::Document* drawingOfOpenPart(MainWindow& w, const QString& path) {
    if (!w.openPath(path)) return nullptr;
    hz::doc::Document* part = w.activeDocument();
    if (!waitUntil([&] { return !w.backgroundWorkRunning(); }, 10000)) return nullptr;
    w.findChild<QAction*>(QStringLiteral("action_new_drawing_from_part"))->trigger();
    return w.activeDocument() != part ? w.activeDocument() : nullptr;
}

}  // namespace

// The active part's drawing: a new, unsaved tab of its views on paper, seen
// from above, on the sheet's own layers, locked, and titled with the part.
TEST(DrawingsTest, ANewDrawingOfTheActivePartShowsItsSheet) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    saveBlock(block, 10);
    MainWindow w;
    const int tabsBefore = tabBar(w)->count();
    hz::doc::Document* sheet = drawingOfOpenPart(w, block);
    ASSERT_NE(sheet, nullptr);
    EXPECT_EQ(tabBar(w)->count(), tabsBefore + 2) << "the part's tab, and the sheet's";
    EXPECT_TRUE(tabBar(w)->tabText(tabBar(w)->currentIndex()).startsWith("Drawing of block"));
    EXPECT_EQ(sheet->type(), hz::doc::DocumentType::Drawing);
    EXPECT_TRUE(sheet->isDirty()) << "new, and not saved";
    EXPECT_GT(countOn(*sheet, "Visible"), 0u);
    EXPECT_GT(countOn(*sheet, "Border"), 0u);
    for (const char* layer : {"Visible", "Hidden", "Border", "TitleBlock"}) {
        const auto* props = sheet->layerManager().getLayer(layer);
        ASSERT_NE(props, nullptr) << layer;
        EXPECT_TRUE(props->locked) << layer;
    }
    EXPECT_TRUE(titleBlockText(*sheet).contains(QStringLiteral("block")));
    EXPECT_TRUE(titleBlockText(*sheet).contains(QStringLiteral("SCALE: ")));
    EXPECT_TRUE(seenFromAbove(w));
    const auto& camera = w.findChild<hz::ui::ViewportWidget*>()->camera();
    EXPECT_NEAR(camera.target().x, 420.0 / 2, 1e-6) << "the A3 paper in the middle";
    EXPECT_NEAR(camera.target().y, 297.0 / 2, 1e-6);
}

// The sheet commands work on a sheet, and say so elsewhere.
TEST(DrawingsTest, TheSheetCommandsSayTheyNeedASheet) {
    MainWindow w;
    trigger(w, "action_drawing_scale");
    EXPECT_TRUE(w.statusBar()->currentMessage().contains(QStringLiteral("drawing sheet")))
        << w.statusBar()->currentMessage().toStdString();
}

// Saved as a .hzdwg, closed, and opened again: the same sheet, drawn from
// its part, not modified; opened while open, its tab is shown.
TEST(DrawingsTest, ADrawingSavesAndOpensAgain) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    saveBlock(block, 10);
    MainWindow w;
    hz::doc::Document* sheet = drawingOfOpenPart(w, block);
    ASSERT_NE(sheet, nullptr);
    const size_t drawn = countOn(*sheet, "Visible");

    // Named when first saved; the name made a .hzdwg.
    const QString file = dir.filePath(QStringLiteral("block-sheet"));
    {
        FilePicker picker(file);
        trigger(w, "action_save");
    }
    const QString saved = file + QStringLiteral(".hzdwg");
    ASSERT_TRUE(QFile::exists(saved));
    EXPECT_FALSE(sheet->isDirty());
    hz::io::DrawingDocumentSpec spec;
    ASSERT_TRUE(hz::io::DrawingDocumentIO::readSpec(saved.toStdString(), spec));
    EXPECT_TRUE(hz::doc::DocumentManager::samePath(spec.partPath, block.toStdString()));

    emit tabBar(w)->tabCloseRequested(tabBar(w)->currentIndex());
    ASSERT_NE(w.activeDocument(), sheet) << "closed without asking";
    const int tabs = tabBar(w)->count();
    ASSERT_TRUE(w.openPath(saved));
    hz::doc::Document* again = w.activeDocument();
    EXPECT_EQ(tabBar(w)->count(), tabs + 1);
    EXPECT_EQ(tabBar(w)->tabText(tabBar(w)->currentIndex()), QStringLiteral("block-sheet.hzdwg"));
    EXPECT_EQ(countOn(*again, "Visible"), drawn);
    EXPECT_FALSE(again->isDirty());
    EXPECT_TRUE(seenFromAbove(w));

    ASSERT_TRUE(w.openPath(saved));
    EXPECT_EQ(w.activeDocument(), again) << "its tab shown, not a second";
    EXPECT_EQ(tabBar(w)->count(), tabs + 1);
}

// The title block's fields and paper, and the scale, are set in forms; each
// draws the sheet again, modified.
TEST(DrawingsTest, TheTitleBlockAndScaleAreSetInForms) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    saveBlock(block, 10);
    MainWindow w;
    hz::doc::Document* sheet = drawingOfOpenPart(w, block);
    ASSERT_NE(sheet, nullptr);
    sheet->setDirty(false);

    {
        FormFiller form(QStringLiteral("Title Block"), FormAnswers()
                                                           .text("title", "Bracket")
                                                           .text("partNumber", "HZ-042")
                                                           .choose("paper", "A4")
                                                           .choose("orientation", "Portrait"));
        trigger(w, "action_drawing_title_block");
        ASSERT_TRUE(form.seen());
    }
    EXPECT_TRUE(titleBlockText(*sheet).contains(QStringLiteral("Bracket")));
    EXPECT_TRUE(titleBlockText(*sheet).contains(QStringLiteral("PART: HZ-042")));
    EXPECT_FALSE(titleBlockText(*sheet).contains(QStringLiteral("block")));
    // A4 portrait, its border 10 mm in from the edges.
    EXPECT_NEAR(extentOn(*sheet, "Border", 0), 210.0 - 20.0, 1e-6);
    EXPECT_NEAR(extentOn(*sheet, "Border", 1), 297.0 - 20.0, 1e-6);
    EXPECT_NEAR(w.findChild<hz::ui::ViewportWidget*>()->camera().target().x, 210.0 / 2, 1e-6)
        << "the new paper in view";
    EXPECT_TRUE(sheet->isDirty());

    {
        FormFiller form(QStringLiteral("Drawing Scale"), FormAnswers().choose("scale", "1:1"));
        trigger(w, "action_drawing_scale");
        ASSERT_TRUE(form.seen());
    }
    EXPECT_TRUE(titleBlockText(*sheet).contains(QStringLiteral("SCALE: 1:1")));
    const double atOne = extentOn(*sheet, "Visible", 0);
    {
        FormFiller form(QStringLiteral("Drawing Scale"), FormAnswers().choose("scale", "2:1"));
        trigger(w, "action_drawing_scale");
        ASSERT_TRUE(form.seen());
    }
    EXPECT_TRUE(titleBlockText(*sheet).contains(QStringLiteral("SCALE: 2:1")));
    EXPECT_GT(extentOn(*sheet, "Visible", 0), 1.5 * atOne) << "drawn twice as large";
}

// A sheet of a part chosen follows the part: changed on disk by another
// program, and saved from its tab with an edit.
TEST(DrawingsTest, ADrawingFollowsItsPart) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    saveBlock(block, 10);
    MainWindow w;
    const int tabs = tabBar(w)->count();
    {
        FilePicker picker(block);  // no part is active: one is chosen
        trigger(w, "action_new_drawing_from_part");
    }
    ASSERT_EQ(tabBar(w)->count(), tabs + 1);
    hz::doc::Document* sheet = w.activeDocument();
    ASSERT_GT(countOn(*sheet, "Visible"), 0u);
    {
        FormFiller form(QStringLiteral("Drawing Scale"), FormAnswers().choose("scale", "1:1"));
        trigger(w, "action_drawing_scale");
        ASSERT_TRUE(form.seen());
    }
    const double tall10 = extentOn(*sheet, "Visible", 1);

    // Changed on disk.
    w.findChild<QTimer*>(QStringLiteral("partWatchTimer"))->setInterval(10);
    saveBlock(block, 30);
    ASSERT_TRUE(waitUntil([&] { return extentOn(*sheet, "Visible", 1) > tall10 + 20.0; }, 5000))
        << "drawn from the deeper part";
    const double tall30 = extentOn(*sheet, "Visible", 1);

    // Edited in its tab and saved.
    ASSERT_TRUE(w.openPath(block));
    hz::doc::Document* part = w.activeDocument();
    ASSERT_NE(part, sheet);
    ASSERT_TRUE(waitUntil([&] { return !w.backgroundWorkRunning(); }, 10000));
    ASSERT_TRUE(part->featureTree().feature(0)->setParameter("depth", 50.0));
    ASSERT_TRUE(part->rebuildModel());
    part->setDirty(true);
    trigger(w, "action_save");
    ASSERT_FALSE(part->isDirty());
    EXPECT_GT(extentOn(*sheet, "Visible", 1), tall30 + 20.0) << "drawn from the saved part";
}

// A sheet's own file changed by another program is read again: drawn from
// what it now says, and not modified. It was read as a part, and failed.
TEST(DrawingsTest, ASheetChangedOnDiskIsReadAgain) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    saveBlock(block, 10);
    hz::io::DrawingDocumentSpec spec;
    spec.partPath = block.toStdString();
    spec.titleBlock.title = "First";
    const std::string file = dir.filePath(QStringLiteral("block.hzdwg")).toStdString();
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(file, spec));

    MainWindow w;
    ASSERT_TRUE(w.openPath(QString::fromStdString(file)));
    hz::doc::Document* sheet = w.activeDocument();
    ASSERT_TRUE(titleBlockText(*sheet).contains(QStringLiteral("First")));

    w.findChild<QTimer*>(QStringLiteral("partWatchTimer"))->setInterval(10);
    spec.titleBlock.title = "Second";
    ASSERT_TRUE(hz::io::DrawingDocumentIO::save(file, spec));
    const std::filesystem::path path(file);
    std::filesystem::last_write_time(
        path, std::filesystem::last_write_time(path) + std::chrono::seconds(2));
    ASSERT_TRUE(
        waitUntil([&] { return titleBlockText(*sheet).contains(QStringLiteral("Second")); }, 5000));
    EXPECT_EQ(w.activeDocument(), sheet) << "the same tab, drawn again";
    EXPECT_FALSE(sheet->isDirty());
}

// A sheet plots on its own paper, at 1:1, as the export form opens: it is
// drawn to size. The form opened on A4, fitted.
TEST(DrawingsTest, ASheetPlotsOnItsPaperAtFullSize) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    saveBlock(block, 10);
    MainWindow w;
    ASSERT_NE(drawingOfOpenPart(w, block), nullptr);

    const QString svg = dir.filePath(QStringLiteral("sheet.svg"));
    FormFiller form(QStringLiteral("Export SVG"), FormAnswers());  // as it opens
    {
        FilePicker picker(svg);
        trigger(w, "export_svg");
    }
    ASSERT_TRUE(form.seen());
    QFile file(svg);
    ASSERT_TRUE(file.open(QIODevice::ReadOnly));
    const QString text = QString::fromUtf8(file.readAll());
    EXPECT_TRUE(text.contains(QStringLiteral("width=\"420mm\" height=\"297mm\""))) << "A3";

    // The border, 10 mm in from the paper's edges: drawn at 1:1.
    double low = 1e300;
    double high = -1e300;
    const QRegularExpression points(QStringLiteral("points=\"([^\"]*)\""));
    for (auto it = points.globalMatch(text); it.hasNext();) {
        for (const QString& pair :
             it.next().captured(1).split(QLatin1Char(' '), Qt::SkipEmptyParts)) {
            const double x = pair.section(QLatin1Char(','), 0, 0).toDouble();
            low = std::min(low, x);
            high = std::max(high, x);
        }
    }
    EXPECT_NEAR(low, 10.0, 1e-3);
    EXPECT_NEAR(high, 410.0, 1e-3);
}
