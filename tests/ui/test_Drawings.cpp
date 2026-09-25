// Drawing sheets (Phase 148): a sheet made from the active part, or from a
// part chosen, shows the part's standard views on paper, seen from above,
// on its own locked layers. It saves as a .hzdwg and opens again, its title
// block and scale are set in forms, and it follows its part when the part
// changes on disk or is saved in its tab.
//
// Views added to it (Phase 149): a section cut through a view, a detail
// clicked on one, a view moved by clicks or removed with those taken from
// it, and all of them kept when the sheet is laid out at another scale.

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
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/Tool.h"
#include "horizon/ui/ViewportWidget.h"

using hz::test::FilePicker;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::test::ToolDriver;
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

// ---------------------------------------------------------------------------
// Views added to a sheet (Phase 149)
// ---------------------------------------------------------------------------

namespace {

/// Where the sheet of the block at @p path lays its views out: as the
/// workbench does, on an A3 sheet.
hz::model::Drawing layoutOf(const QString& path) {
    hz::doc::Document part;
    EXPECT_TRUE(hz::io::NativeFormat::load(path.toStdString(), part));
    EXPECT_TRUE(part.rebuildModel());
    return hz::model::DrawingGenerator::sheetLayout(*part.solid(), hz::model::Sheet{},
                                                    hz::model::TitleBlock{});
}

/// Save the active sheet (named @p path the first time) and read what it
/// says.
hz::io::DrawingDocumentSpec savedSpec(MainWindow& w, const QString& path) {
    if (w.activeDocument()->filePath().empty()) {
        FilePicker picker(path);
        trigger(w, "action_save");
    } else {
        trigger(w, "action_save");
    }
    hz::io::DrawingDocumentSpec spec;
    EXPECT_TRUE(hz::io::DrawingDocumentIO::readSpec(path.toStdString(), spec));
    return spec;
}

/// What is written on the ViewLabels layer.
QStringList labelTexts(const hz::doc::Document& sheet) {
    QStringList texts;
    for (const auto& entity : sheet.draftDocument().entities()) {
        if (entity->layer() != "ViewLabels") continue;
        if (const auto* text = dynamic_cast<const hz::draft::DraftText*>(entity.get())) {
            texts << QString::fromStdString(text->text());
        }
    }
    return texts;
}

bool selecting(MainWindow& w) {
    const auto* tool = w.findChild<hz::ui::ViewportWidget*>()->activeTool();
    return tool != nullptr && tool->name() == "Select";
}

/// A detail of the Front view of @p front, about its lower-left corner, 8 mm
/// across on the sheet, labelled @p label: clicked, then the form accepted.
void addDetailOfFront(MainWindow& w, const hz::model::DrawingView& front, const QString& label) {
    ToolDriver drive(w);
    trigger(w, "action_add_detail_view");
    FormFiller form(QStringLiteral("Add Detail View"), FormAnswers().text("label", label));
    const hz::math::Vec2 corner = front.placement;
    drive.click(corner);
    drive.click({corner.x + 8.0, corner.y});
    ASSERT_TRUE(waitUntil([&] { return form.seen(); }, 5000));
    ASSERT_TRUE(waitUntil([&] { return selecting(w); }, 5000)) << "back to Select";
}

}  // namespace

// A section cut through the Front view: captioned, hatched, its cut marked
// on Front, saved and opened again; a cut that misses the part is refused.
TEST(DrawingsTest, ASectionIsCutThroughAView) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    saveBlock(block, 10);
    MainWindow w;
    hz::doc::Document* sheet = drawingOfOpenPart(w, block);
    ASSERT_NE(sheet, nullptr);

    {
        FormFiller form(QStringLiteral("Add Section View"), FormAnswers()
                                                                .choose("view", "1: Front")
                                                                .choose("cut", "Vertical")
                                                                .number("offset", 0.0)
                                                                .text("label", "A"));
        trigger(w, "action_add_section_view");
        ASSERT_TRUE(form.seen());
    }
    EXPECT_TRUE(labelTexts(*sheet).contains(QStringLiteral("SECTION A-A")));
    EXPECT_EQ(labelTexts(*sheet).count(QStringLiteral("A")), 2)
        << "a letter at each end of the cut";
    EXPECT_GT(countOn(*sheet, "Section"), 0u);
    EXPECT_GT(countOn(*sheet, "Hatch"), 0u);
    EXPECT_TRUE(sheet->isDirty());

    const QString file = dir.filePath(QStringLiteral("block.hzdwg"));
    const auto spec = savedSpec(w, file);
    ASSERT_EQ(spec.views.size(), 5u) << "the four standard views, and the section";
    EXPECT_EQ(spec.views[4].role, hz::model::ViewRole::Section);
    EXPECT_EQ(spec.views[4].source, 0);
    EXPECT_EQ(spec.views[4].label, "A");

    const size_t labels = countOn(*sheet, "ViewLabels");
    {
        FormFiller form(QStringLiteral("Add Section View"),
                        FormAnswers().choose("view", "1: Front").number("offset", 500.0));
        trigger(w, "action_add_section_view");
        ASSERT_TRUE(form.seen());
    }
    EXPECT_TRUE(w.statusBar()->currentMessage().contains(QStringLiteral("misses")))
        << w.statusBar()->currentMessage().toStdString();
    EXPECT_EQ(countOn(*sheet, "ViewLabels"), labels) << "nothing added";

    emit tabBar(w)->tabCloseRequested(tabBar(w)->currentIndex());
    ASSERT_TRUE(w.openPath(file));
    EXPECT_TRUE(labelTexts(*w.activeDocument()).contains(QStringLiteral("SECTION A-A")));
}

// A detail: its centre clicked on a view (a click off every view is refused),
// then its radius; the form takes the scale, twice the view's by default.
TEST(DrawingsTest, ADetailIsClickedOnAView) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    saveBlock(block, 10);
    const hz::model::Drawing layout = layoutOf(block);
    const hz::model::DrawingView& front = layout.views[0];
    MainWindow w;
    hz::doc::Document* sheet = drawingOfOpenPart(w, block);
    ASSERT_NE(sheet, nullptr);

    {
        ToolDriver drive(w);
        trigger(w, "action_add_detail_view");
        drive.click({front.placement.x - 5.0, front.placement.y - 5.0});  // off every view
        EXPECT_FALSE(selecting(w)) << "still waiting for a centre on a view";
    }
    addDetailOfFront(w, front, QStringLiteral("B"));

    double twice = 10.0;
    for (const double s : hz::model::DrawingGenerator::standardScales()) {
        if (s >= 2.0 * front.scale - 1e-9) twice = s;
    }
    const QString caption =
        QStringLiteral("DETAIL B (%1)")
            .arg(QString::fromStdString(hz::model::DrawingGenerator::scaleName(twice)));
    EXPECT_TRUE(labelTexts(*sheet).contains(caption))
        << labelTexts(*sheet).join(", ").toStdString();
    const hz::draft::DraftCircle* circle = nullptr;
    for (const auto& entity : sheet->draftDocument().entities()) {
        if (entity->layer() != "ViewLabels") continue;
        if (const auto* c = dynamic_cast<const hz::draft::DraftCircle*>(entity.get())) circle = c;
    }
    ASSERT_NE(circle, nullptr) << "circled on Front";
    EXPECT_NEAR(circle->center().x, front.placement.x, 1.0);
    EXPECT_NEAR(circle->center().y, front.placement.y, 1.0);
    EXPECT_NEAR(circle->radius(), 8.0, 1.0);

    const auto spec = savedSpec(w, dir.filePath(QStringLiteral("block.hzdwg")));
    ASSERT_EQ(spec.views.size(), 5u);
    EXPECT_EQ(spec.views[4].role, hz::model::ViewRole::Detail);
    EXPECT_EQ(spec.views[4].source, 0);
    EXPECT_DOUBLE_EQ(spec.views[4].scale, twice);
}

// A view clicked, then where the clicked point goes: it moves by as much.
TEST(DrawingsTest, AViewIsMovedByClicks) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    saveBlock(block, 10);
    const hz::model::Drawing layout = layoutOf(block);
    const hz::model::DrawingView& iso = layout.views[3];
    MainWindow w;
    ASSERT_NE(drawingOfOpenPart(w, block), nullptr);

    ToolDriver drive(w);
    trigger(w, "action_move_view");
    const hz::math::Vec2 at{iso.placement.x + iso.sheetWidth() / 2.0,
                            iso.placement.y + iso.sheetHeight() / 2.0};
    drive.click(at);
    drive.click({at.x - 20.0, at.y - 15.0});
    ASSERT_TRUE(waitUntil([&] { return selecting(w); }, 5000)) << "back to Select";

    const auto spec = savedSpec(w, dir.filePath(QStringLiteral("block.hzdwg")));
    ASSERT_EQ(spec.views.size(), 4u);
    // A click lands on a pixel, and may snap: within a millimetre.
    EXPECT_NEAR(spec.views[3].placement.x, iso.placement.x - 20.0, 1.0);
    EXPECT_NEAR(spec.views[3].placement.y, iso.placement.y - 15.0, 1.0);
    EXPECT_NEAR(spec.views[0].placement.x, layout.views[0].placement.x, 1e-9) << "the rest stay";
}

// A view removed takes the views taken from it with it, and says so.
TEST(DrawingsTest, ARemovedViewTakesItsDetailWithIt) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    saveBlock(block, 10);
    const hz::model::Drawing layout = layoutOf(block);
    MainWindow w;
    hz::doc::Document* sheet = drawingOfOpenPart(w, block);
    ASSERT_NE(sheet, nullptr);
    addDetailOfFront(w, layout.views[0], QStringLiteral("B"));
    ASSERT_FALSE(labelTexts(*sheet).isEmpty());

    {
        FormFiller form(QStringLiteral("Remove View"), FormAnswers().choose("view", "1: Front"));
        trigger(w, "action_remove_view");
        ASSERT_TRUE(form.seen());
    }
    EXPECT_TRUE(w.statusBar()->currentMessage().contains(QStringLiteral("went with it")))
        << w.statusBar()->currentMessage().toStdString();
    EXPECT_TRUE(labelTexts(*sheet).isEmpty()) << "the detail, its caption and its circle gone";
    const auto spec = savedSpec(w, dir.filePath(QStringLiteral("block.hzdwg")));
    ASSERT_EQ(spec.views.size(), 3u);
    for (const auto& v : spec.views) EXPECT_EQ(v.role, hz::model::ViewRole::Projection);
    EXPECT_NE(spec.views[0].kind, hz::model::StandardView::Front);
}

// The Scale form lays the standard views out again and keeps the section
// and the detail: the section at the new scale, the detail as many times
// its view's (at a standard scale), each placed where there is room.
TEST(DrawingsTest, TheScaleFormKeepsSectionsAndDetails) {
    QTemporaryDir dir;
    const QString block = dir.filePath(QStringLiteral("block.hzpart"));
    saveBlock(block, 10);
    const hz::model::Drawing layout = layoutOf(block);
    MainWindow w;
    hz::doc::Document* sheet = drawingOfOpenPart(w, block);
    ASSERT_NE(sheet, nullptr);
    {
        FormFiller form(QStringLiteral("Add Section View"),
                        FormAnswers().choose("view", "1: Front").text("label", "A"));
        trigger(w, "action_add_section_view");
        ASSERT_TRUE(form.seen());
    }
    addDetailOfFront(w, layout.views[0], QStringLiteral("B"));
    const QString file = dir.filePath(QStringLiteral("block.hzdwg"));
    const auto before = savedSpec(w, file);
    ASSERT_EQ(before.views.size(), 6u);
    const double ratio = before.views[5].scale / before.views[0].scale;

    {
        FormFiller form(QStringLiteral("Drawing Scale"), FormAnswers().choose("scale", "1:1"));
        trigger(w, "action_drawing_scale");
        ASSERT_TRUE(form.seen());
    }
    const auto after = savedSpec(w, file);
    ASSERT_EQ(after.views.size(), 6u);
    EXPECT_DOUBLE_EQ(after.views[0].scale, 1.0);
    EXPECT_EQ(after.views[4].role, hz::model::ViewRole::Section);
    EXPECT_DOUBLE_EQ(after.views[4].scale, 1.0);
    EXPECT_EQ(after.views[4].source, 0);
    EXPECT_EQ(after.views[5].role, hz::model::ViewRole::Detail);
    // As many times its view's as before, or the standard scale above that.
    double atLeast = 10.0;
    for (const double s : hz::model::DrawingGenerator::standardScales()) {
        if (s >= ratio * 1.0 - 1e-9) atLeast = s;
    }
    EXPECT_DOUBLE_EQ(after.views[5].scale, atLeast);
    EXPECT_EQ(after.views[5].source, 0);
    EXPECT_TRUE(labelTexts(*sheet).contains(QStringLiteral("SECTION A-A")));
    EXPECT_FALSE(w.statusBar()->currentMessage().contains(QStringLiteral("no room")));
}
