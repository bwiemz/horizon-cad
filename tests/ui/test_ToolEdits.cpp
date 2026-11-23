// Drawing with the tools the way a user does (Phase 112): clicks through the
// viewport's own mouse handling, into the document, and back out by undo.

#include <gtest/gtest.h>

#include <QAction>
#include <QCoreApplication>
#include <QDoubleSpinBox>
#include <QKeyEvent>
#include <QLabel>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "UiTestSupport.h"
#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/Layer.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/PolygonTool.h"
#include "horizon/ui/Preferences.h"
#include "horizon/ui/Tool.h"
#include "horizon/ui/ViewportWidget.h"

using hz::math::Vec2;
using hz::test::ToolDriver;
using hz::ui::MainWindow;

namespace {

void trigger(MainWindow& w, const char* name) {
    auto* action = w.findChild<QAction*>(QString::fromLatin1(name));
    ASSERT_NE(action, nullptr) << name;
    action->trigger();
}

template <typename T>
std::vector<const T*> all(MainWindow& w) {
    std::vector<const T*> out;
    for (const auto& e : w.activeDocument()->draftDocument().entities()) {
        if (const auto* t = dynamic_cast<const T*>(e.get())) out.push_back(t);
    }
    return out;
}

bool near(const Vec2& a, const Vec2& b) {
    return (a - b).length() < 1e-9;
}

}  // namespace

TEST(ToolEditsTest, TheLineToolDrawsALineThatUndoTakesAway) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_line");
    drive.click(Vec2(2, 3));
    drive.move(Vec2(7, 3));
    drive.click(Vec2(7, 3));

    auto lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(2, 3)));
    EXPECT_TRUE(near(lines[0]->end(), Vec2(7, 3)));
    EXPECT_TRUE(w.activeDocument()->isDirty());

    trigger(w, "action_undo");
    EXPECT_TRUE(all<hz::draft::DraftLine>(w).empty());
    trigger(w, "action_redo");
    EXPECT_EQ(all<hz::draft::DraftLine>(w).size(), 1u);
}

TEST(ToolEditsTest, TheCircleAndRectangleToolsDrawWhatWasClicked) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_circle");
    drive.click(Vec2(0, 0));
    drive.click(Vec2(3, 0));
    auto circles = all<hz::draft::DraftCircle>(w);
    ASSERT_EQ(circles.size(), 1u);
    EXPECT_TRUE(near(circles[0]->center(), Vec2(0, 0)));
    EXPECT_NEAR(circles[0]->radius(), 3.0, 1e-9);

    trigger(w, "tool_rectangle");
    drive.click(Vec2(5, 1));
    drive.click(Vec2(9, 4));
    auto rects = all<hz::draft::DraftRectangle>(w);
    ASSERT_EQ(rects.size(), 1u);
    const auto c = rects[0]->corners();
    for (const Vec2& corner : {Vec2(5, 1), Vec2(9, 1), Vec2(9, 4), Vec2(5, 4)}) {
        bool found = false;
        for (const Vec2& p : c) found = found || near(p, corner);
        EXPECT_TRUE(found) << corner.x << ", " << corner.y;
    }
}

TEST(ToolEditsTest, ALineClickedAndDeletedCanBeUndone) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_line");
    drive.click(Vec2(2, 3));
    drive.click(Vec2(7, 3));
    ASSERT_EQ(all<hz::draft::DraftLine>(w).size(), 1u);

    trigger(w, "tool_select");
    drive.click(Vec2(4.5, 3));
    drive.key(Qt::Key_Delete);
    EXPECT_TRUE(all<hz::draft::DraftLine>(w).empty()) << "the clicked line is selected and deleted";

    trigger(w, "action_undo");
    EXPECT_EQ(all<hz::draft::DraftLine>(w).size(), 1u);
}

// Starting Insert Block while it is already active used to destroy the active
// tool and then deactivate it: a use after free (AddressSanitizer reports it).
// A stretch is kept once it is made. It was put back at once and shown only
// after an undo and a redo, so Stretch seemed to do nothing.
TEST(ToolEditsTest, AStretchIsKeptAndUndone) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_line");
    drive.click(Vec2(0, 0));
    drive.move(Vec2(10, 0));
    drive.click(Vec2(10, 0));
    trigger(w, "tool_stretch");
    drive.click(Vec2(8, -2));  // a window round the line's end
    drive.move(Vec2(12, 2));
    drive.click(Vec2(12, 2));
    drive.click(Vec2(10, 0));  // from the end
    drive.move(Vec2(15, 3));
    drive.click(Vec2(15, 3));  // to here

    auto lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(0, 0))) << "outside the window: stays";
    EXPECT_TRUE(near(lines[0]->end(), Vec2(15, 3))) << "inside it: moved";
    trigger(w, "action_undo");
    EXPECT_TRUE(near(all<hz::draft::DraftLine>(w)[0]->end(), Vec2(10, 0)));
    trigger(w, "action_redo");
    EXPECT_TRUE(near(all<hz::draft::DraftLine>(w)[0]->end(), Vec2(15, 3)));
}

// Another tool chosen in the middle of a drag puts back what was dragged, as
// Escape does. It was left moved, with no command to undo it: the Select
// tool's grips and Polyline Edit's vertices.
TEST(ToolEditsTest, AnotherToolMidDragPutsTheDragBack) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_line");
    drive.click(Vec2(0, 0));
    drive.move(Vec2(10, 0));
    drive.click(Vec2(10, 0));
    trigger(w, "tool_select");
    drive.click(Vec2(5, 0));  // chosen: its grips show
    drive.press(Vec2(10, 0));
    drive.dragTo(Vec2(14, 4));
    ASSERT_TRUE(near(all<hz::draft::DraftLine>(w)[0]->end(), Vec2(14, 4))) << "being dragged";
    trigger(w, "tool_circle");
    EXPECT_TRUE(near(all<hz::draft::DraftLine>(w)[0]->end(), Vec2(10, 0))) << "put back";

    auto& doc = *w.activeDocument();
    doc.undoStack().push(std::make_unique<hz::doc::AddEntityCommand>(
        doc.draftDocument(), std::make_shared<hz::draft::DraftPolyline>(
                                 std::vector<Vec2>{Vec2(20, 0), Vec2(30, 0), Vec2(30, 10)})));
    trigger(w, "tool_polyline-edit");
    drive.click(Vec2(25, 0));  // this one to edit
    drive.press(Vec2(30, 10));
    drive.dragTo(Vec2(34, 12));
    ASSERT_TRUE(near(all<hz::draft::DraftPolyline>(w)[0]->points()[2], Vec2(34, 12)));
    trigger(w, "tool_line");
    EXPECT_TRUE(near(all<hz::draft::DraftPolyline>(w)[0]->points()[2], Vec2(30, 10)));

    trigger(w, "action_undo");
    EXPECT_TRUE(all<hz::draft::DraftPolyline>(w).empty()) << "nothing else to undo first";
}

// Undo or redo in the middle of a drag puts the drag back first, as another
// tool chosen does. The grip's release recorded a step from the state before
// the undo: it put back what was undone, and threw away what could be
// redone.
TEST(ToolEditsTest, UndoMidDragPutsTheDragBackFirst) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_line");
    drive.click(Vec2(0, 0));
    drive.move(Vec2(10, 0));
    drive.click(Vec2(10, 0));
    trigger(w, "tool_line");
    drive.click(Vec2(0, 20));
    drive.move(Vec2(10, 20));
    drive.click(Vec2(10, 20));
    ASSERT_EQ(all<hz::draft::DraftLine>(w).size(), 2u);

    trigger(w, "tool_select");
    drive.click(Vec2(5, 0));
    drive.press(Vec2(10, 0));
    drive.dragTo(Vec2(14, 4));
    ASSERT_TRUE(near(all<hz::draft::DraftLine>(w)[0]->end(), Vec2(14, 4))) << "being dragged";
    trigger(w, "action_undo");
    drive.releaseAt(drive.viewport().worldToScreen(Vec2(14, 4)));

    auto lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 1u) << "the second line undone";
    EXPECT_TRUE(near(lines[0]->end(), Vec2(10, 0))) << "and the drag put back";
    trigger(w, "action_redo");
    lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 2u) << "still there to redo";
    EXPECT_TRUE(near(lines[0]->end(), Vec2(10, 0)));
}

TEST(ToolEditsTest, InsertBlockCanBeStartedAgainWhileItIsActive) {
    MainWindow w;
    auto bolt = std::make_shared<hz::draft::BlockDefinition>();
    bolt->name = "Bolt";
    bolt->entities.push_back(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0)));
    w.activeDocument()->draftDocument().blockTable().addBlock(bolt);

    for (int round = 0; round < 3; ++round) {
        hz::test::FormFiller accept(QStringLiteral("Insert Block"), hz::test::FormAnswers{});
        trigger(w, "action_block-insert");
        ASSERT_TRUE(accept.seen()) << "round " << round;
        const hz::ui::Tool* active = w.findChild<hz::ui::ViewportWidget*>()->activeTool();
        ASSERT_NE(active, nullptr);
        EXPECT_EQ(active->name(), "Insert Block");
    }
    trigger(w, "tool_select");  // deactivates the last one
}

// Deleting a box selection is one step, and undoing it puts every entity back
// where it was in the drawing order, not at the end.
TEST(ToolEditsTest, DeletingASelectionAndUndoingKeepsTheDrawingOrder) {
    MainWindow w;
    auto& drawing = w.activeDocument()->draftDocument();
    std::vector<uint64_t> order;
    for (int i = 0; i < 6; ++i) {
        const double y = 2.0 * i;
        auto line = std::make_shared<hz::draft::DraftLine>(Vec2(0, y), Vec2(3, y));
        order.push_back(line->id());
        drawing.addEntity(line);
    }
    const auto ids = [&drawing] {
        std::vector<uint64_t> out;
        for (const auto& e : drawing.entities()) out.push_back(e->id());
        return out;
    };

    ToolDriver drive(w);
    trigger(w, "tool_select");
    drive.drag(Vec2(-1, 1), Vec2(4, 7.5));  // a window around lines 1, 2 and 3
    drive.key(Qt::Key_Delete);
    EXPECT_EQ(ids(), (std::vector<uint64_t>{order[0], order[4], order[5]}));

    trigger(w, "action_undo");
    EXPECT_EQ(ids(), order) << "undo restores the drawing order";
    trigger(w, "action_redo");
    EXPECT_EQ(ids(), (std::vector<uint64_t>{order[0], order[4], order[5]}));
}

// -- Precise input (Phase 128) -----------------------------------------------

namespace {

/// Type @p text at the viewport, key by key, then Enter.
void type(ToolDriver& drive, const std::string& text) {
    for (char c : text) {
        Qt::Key key = Qt::Key_unknown;
        if (c >= '0' && c <= '9') key = static_cast<Qt::Key>(Qt::Key_0 + (c - '0'));
        if (c == ',') key = Qt::Key_Comma;
        if (c == '.') key = Qt::Key_Period;
        if (c == '-') key = Qt::Key_Minus;
        if (c == '@') key = Qt::Key_At;
        if (c == '<') key = Qt::Key_Less;
        drive.key(key);
    }
    drive.key(Qt::Key_Return);
}

hz::ui::ViewportWidget& viewportOf(MainWindow& w) {
    return *w.findChild<hz::ui::ViewportWidget*>();
}

/// The drafting aids as they were, for the next test in this process.
struct RestoreDraftingAids {
    ~RestoreDraftingAids() {
        hz::ui::Preferences prefs = hz::ui::Preferences::current();
        prefs.objectSnap = true;
        prefs.gridSnap = true;
        prefs.ortho = false;
        prefs.polarTracking = false;
        prefs.save();
    }
};

}  // namespace

// Points typed at the keyboard: absolute, relative and polar. The line tool
// chains, each line starting where the last ended, until Enter.
TEST(ToolEditsTest, TypedPointsDrawAChainOfLines) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_line");
    type(drive, "2,3");
    type(drive, "@5,0");
    type(drive, "@4<90");
    drive.key(Qt::Key_Return);  // with nothing typed: the chain ends

    auto lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(2, 3)));
    EXPECT_TRUE(near(lines[0]->end(), Vec2(7, 3)));
    EXPECT_TRUE(near(lines[1]->start(), Vec2(7, 3)));
    EXPECT_TRUE(near(lines[1]->end(), Vec2(7, 7)));

    drive.click(Vec2(20, 20));
    drive.click(Vec2(25, 20));
    lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_TRUE(near(lines[2]->start(), Vec2(20, 20))) << "a new chain, not one from (7, 7)";
}

// A length typed alone goes from the last point toward the cursor.
TEST(ToolEditsTest, ALengthGoesTowardTheCursor) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_line");
    drive.click(Vec2(1, 1));
    drive.move(Vec2(1, 9));
    type(drive, "2.5");
    const auto lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_TRUE(near(lines[0]->end(), Vec2(1, 3.5)));
}

// What is not a point is refused and said, and changes nothing; Escape drops
// what is typed before it stops the tool.
TEST(ToolEditsTest, ARefusedPointIsShownAndEscapeDropsTypingFirst) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_line");
    drive.click(Vec2(0, 0));
    type(drive, "3,,4");
    EXPECT_TRUE(all<hz::draft::DraftLine>(w).empty());
    auto* prompt = w.findChild<QLabel*>(QStringLiteral("statusPrompt"));
    ASSERT_NE(prompt, nullptr);
    EXPECT_TRUE(prompt->text().contains(QStringLiteral("'3,,4' is not a point")))
        << prompt->text().toStdString();

    auto& viewport = viewportOf(w);
    drive.key(Qt::Key_1);
    drive.key(Qt::Key_Escape);
    EXPECT_FALSE(viewport.typedPoint().typing());
    EXPECT_TRUE(viewport.activeTool()->basePoint()) << "the line being drawn is still there";
    drive.key(Qt::Key_Escape);
    EXPECT_FALSE(viewport.activeTool()->basePoint());
}

// Ortho holds a line to the axes, polar tracking to steps of 15 degrees, and
// only one of them is on at a time.
TEST(ToolEditsTest, OrthoAndPolarTrackingHoldTheDirection) {
    const RestoreDraftingAids restore;
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "action_ortho");
    trigger(w, "tool_line");
    drive.click(Vec2(0, 0));
    drive.click(Vec2(5, 1));
    drive.key(Qt::Key_Return);
    auto lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_TRUE(near(lines[0]->end(), Vec2(5, 0)));

    trigger(w, "action_grid_snap");  // off, so the cursor is where it is clicked
    trigger(w, "action_polar");
    EXPECT_FALSE(w.findChild<QAction*>(QStringLiteral("action_ortho"))->isChecked())
        << "polar tracking turns ortho off";
    drive.click(Vec2(0, 10));
    drive.click(Vec2(10, 12.5));  // 14 degrees
    lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 2u);
    const Vec2 d = lines[1]->end() - lines[1]->start();
    EXPECT_NEAR(std::atan2(d.y, d.x) * 180.0 / 3.14159265358979323846, 15.0, 1e-9);
}

// With the object snap off (F3), a click near an endpoint is not pulled onto it.
TEST(ToolEditsTest, SnapsCanBeSwitchedOff) {
    const RestoreDraftingAids restore;
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_line");
    drive.click(Vec2(0, 0));
    drive.click(Vec2(10, 0));
    drive.key(Qt::Key_Return);

    // Snapped while the snaps are on: the click reaches the endpoint.
    drive.click(Vec2(0.07, 0.05));
    drive.click(Vec2(4, 3));
    drive.key(Qt::Key_Return);
    auto lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 2u);
    ASSERT_TRUE(near(lines[1]->start(), Vec2(0, 0))) << "the click is within the snap's reach";

    trigger(w, "action_object_snap");
    trigger(w, "action_grid_snap");
    drive.click(Vec2(0.07, 0.05));
    drive.click(Vec2(4.07, 3.05));
    lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_GT(lines[2]->start().length(), 0.02) << "not snapped to the endpoint at the origin";
}

// A selected line's and circle's geometry can be typed into the property
// panel: each edit is one undo step.
TEST(ToolEditsTest, GeometryIsEditedInThePropertyPanel) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_line");
    drive.click(Vec2(1, 1));
    drive.click(Vec2(4, 5));  // length 5
    drive.key(Qt::Key_Return);
    trigger(w, "tool_select");
    drive.click(Vec2(2.5, 3));
    const auto spin = [&w](const char* name) {
        return w.findChild<QDoubleSpinBox*>(QString::fromLatin1(name));
    };
    ASSERT_NE(spin("lineLength"), nullptr);
    EXPECT_NEAR(spin("lineLength")->value(), 5.0, 1e-4) << "the panel shows the selected line";

    spin("lineLength")->setValue(10.0);
    auto lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(1, 1))) << "the start stays";
    EXPECT_TRUE(near(lines[0]->end(), Vec2(7, 9))) << "the end moves along the line";

    spin("lineEndX")->setValue(1.0);
    lines = all<hz::draft::DraftLine>(w);
    EXPECT_TRUE(near(lines[0]->end(), Vec2(1, 9)));

    trigger(w, "action_undo");
    lines = all<hz::draft::DraftLine>(w);
    EXPECT_TRUE(near(lines[0]->end(), Vec2(7, 9))) << "one edit, one undo";
    trigger(w, "action_undo");
    lines = all<hz::draft::DraftLine>(w);
    EXPECT_TRUE(near(lines[0]->end(), Vec2(4, 5)));

    trigger(w, "tool_circle");
    drive.click(Vec2(20, 0));
    drive.click(Vec2(22, 0));
    trigger(w, "tool_select");
    drive.click(Vec2(20, 2));
    ASSERT_NE(spin("circleRadius"), nullptr);
    spin("circleRadius")->setValue(6.5);
    const auto circles = all<hz::draft::DraftCircle>(w);
    ASSERT_EQ(circles.size(), 1u);
    EXPECT_NEAR(circles[0]->radius(), 6.5, 1e-12);
    EXPECT_TRUE(near(circles[0]->center(), Vec2(20, 0)));
}

// Selecting one ellipse after another only shows them. The panel's own guard
// was cleared halfway through filling it in, so setting the second ellipse's
// values into the fields pushed "edits" of it onto the undo stack.
TEST(ToolEditsTest, SelectingOnlyShowsItDoesNotEdit) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    doc.draftDocument().addEntity(
        std::make_shared<hz::draft::DraftEllipse>(Vec2(0, 0), 4.0, 2.0, 0.0));
    doc.draftDocument().addEntity(
        std::make_shared<hz::draft::DraftEllipse>(Vec2(20, 0), 6.0, 3.0, 0.5));
    const auto steps = doc.undoStack().revision();
    trigger(w, "tool_select");
    drive.click(Vec2(4, 0));
    drive.click(Vec2(26, 0));
    drive.click(Vec2(4, 0));
    EXPECT_EQ(doc.undoStack().revision(), steps) << "nothing was pushed";
    EXPECT_FALSE(doc.undoStack().canUndo());
}

// Create Block asks for the base point, offering the centre of the selection.
TEST(ToolEditsTest, CreateBlockTakesItsBasePoint) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_line");
    drive.click(Vec2(0, 0));
    drive.click(Vec2(4, 0));
    drive.key(Qt::Key_Return);
    trigger(w, "tool_select");
    drive.click(Vec2(2, 0));

    hz::draft::LayerProperties walls;
    walls.name = "Walls";
    w.activeDocument()->layerManager().addLayer(walls);
    w.activeDocument()->layerManager().setCurrentLayer("Walls");
    hz::test::FormFiller filler(QStringLiteral("Create Block"),
                                hz::test::FormAnswers()
                                    .text(QStringLiteral("blockName"), QStringLiteral("Bar"))
                                    .number(QStringLiteral("baseX"), 0.0)
                                    .number(QStringLiteral("baseY"), 0.0));
    trigger(w, "action_block-create");
    ASSERT_TRUE(filler.seen());
    EXPECT_NEAR(filler.shown(QStringLiteral("baseX")), 2.0, 1e-9) << "the selection's centre";
    const auto block = w.activeDocument()->draftDocument().blockTable().findBlock("Bar");
    ASSERT_NE(block, nullptr);
    EXPECT_TRUE(near(block->basePoint, Vec2(0, 0)));
    const auto refs = all<hz::draft::DraftBlockRef>(w);
    ASSERT_EQ(refs.size(), 1u);
    EXPECT_TRUE(near(refs[0]->insertPos(), Vec2(0, 0)));
    EXPECT_EQ(refs[0]->layer(), "Walls") << "on the current layer, as Insert Block puts it";
}

// The ellipse's last point, typed, is the one it takes. It used the cursor's
// last position, which a typed point never moves.
TEST(ToolEditsTest, AnEllipseTakesItsTypedLastPoint) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_ellipse");
    type(drive, "0,0");
    type(drive, "@6,0");
    drive.move(Vec2(1, 1));
    type(drive, "@0,2");
    const auto ellipses = all<hz::draft::DraftEllipse>(w);
    ASSERT_EQ(ellipses.size(), 1u);
    EXPECT_NEAR(ellipses[0]->semiMajor(), 6.0, 1e-9);
    EXPECT_NEAR(ellipses[0]->semiMinor(), 2.0, 1e-9) << "not the cursor's (1, 1)";
}

namespace {

/// Select @p id as a click would, so the property panel shows it.
void selectOnly(MainWindow& w, uint64_t id) {
    auto& viewport = viewportOf(w);
    viewport.selectionManager().clearSelection();
    viewport.selectionManager().select(id);
    emit viewport.selectionChanged();
}

}  // namespace

// An arc's angles typed into the panel are kept in [0, 360): -10 is 350.
TEST(ToolEditsTest, AnArcsAnglesStayInRangeWhenEdited) {
    MainWindow w;
    auto arc = std::make_shared<hz::draft::DraftArc>(Vec2(0, 0), 5.0, 0.5, 2.0);
    w.activeDocument()->draftDocument().addEntity(arc);
    selectOnly(w, arc->id());
    auto* start = w.findChild<QDoubleSpinBox*>(QStringLiteral("arcStartAngle"));
    ASSERT_NE(start, nullptr);
    start->setValue(-10.0);
    const auto arcs = all<hz::draft::DraftArc>(w);
    ASSERT_EQ(arcs.size(), 1u);
    EXPECT_NEAR(arcs[0]->startAngle(), 350.0 * 3.14159265358979323846 / 180.0, 1e-9);
}

// A line with no length (a grip dragged onto its other end) takes a length
// typed into the panel along its angle field; it used to take nothing.
TEST(ToolEditsTest, ALineWithNoLengthTakesOneFromThePanel) {
    MainWindow w;
    auto line = std::make_shared<hz::draft::DraftLine>(Vec2(3, 3), Vec2(3, 3));
    w.activeDocument()->draftDocument().addEntity(line);
    selectOnly(w, line->id());
    auto* length = w.findChild<QDoubleSpinBox*>(QStringLiteral("lineLength"));
    ASSERT_NE(length, nullptr);
    length->setValue(5.0);
    const auto lines = all<hz::draft::DraftLine>(w);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_TRUE(near(lines[0]->end(), Vec2(8, 3)));
}

// A regular polygon (Phase 168): Up and Down change how many sides, the
// centre and a corner place it, and it is one closed polyline, every corner
// as far from the centre, undone in one step. The count is kept for the
// next, between 3 and 1,024.
TEST(ToolEditsTest, APolygonIsDrawnWithTheSidesAsked) {
    MainWindow w;
    ToolDriver drive(w);
    trigger(w, "tool_polygon");
    auto& view = drive.viewport();
    ASSERT_NE(view.activeTool(), nullptr);
    ASSERT_EQ(view.activeTool()->name(), "Polygon");
    drive.key(Qt::Key_Up);
    drive.key(Qt::Key_Up);  // eight
    drive.click(Vec2(0, 0));
    drive.click(Vec2(10, 0));
    auto polygons = all<hz::draft::DraftPolyline>(w);
    ASSERT_EQ(polygons.size(), 1u);
    EXPECT_TRUE(polygons[0]->closed());
    ASSERT_EQ(polygons[0]->points().size(), 8u);
    for (const Vec2& corner : polygons[0]->points()) {
        EXPECT_NEAR(corner.length(), 10.0, 1e-9);
    }
    EXPECT_TRUE(near(polygons[0]->points().front(), Vec2(10, 0))) << "a corner where clicked";

    // Shift takes ten at a time, and no fewer than three are left.
    QKeyEvent shiftDown(QEvent::KeyPress, Qt::Key_Down, Qt::ShiftModifier);
    QCoreApplication::sendEvent(&view, &shiftDown);
    QCoreApplication::sendEvent(&view, &shiftDown);
    drive.click(Vec2(30, 0));
    drive.click(Vec2(30, 5));
    polygons = all<hz::draft::DraftPolyline>(w);
    ASSERT_EQ(polygons.size(), 2u);
    EXPECT_EQ(polygons[1]->points().size(), 3u);

    // A corner on the centre is no polygon: nothing, and the tool waits.
    drive.click(Vec2(50, 0));
    drive.click(Vec2(50, 0));
    EXPECT_EQ(all<hz::draft::DraftPolyline>(w).size(), 2u);
    drive.key(Qt::Key_Escape);

    w.activeDocument()->undoStack().undo();
    EXPECT_EQ(all<hz::draft::DraftPolyline>(w).size(), 1u) << "one step each";
}

// The polygon's corners, worked out: as many as asked within the limits, on
// the circle through the corner given, and none for a corner on the centre
// or one that is not a finite number.
TEST(ToolEditsTest, APolygonsCornersAreOnItsCircle) {
    using hz::ui::PolygonTool;
    const auto hexagon = PolygonTool::corners(Vec2(1, 2), Vec2(4, 6), 6);
    ASSERT_EQ(hexagon.size(), 6u);
    for (size_t i = 0; i < hexagon.size(); ++i) {
        EXPECT_NEAR((hexagon[i] - Vec2(1, 2)).length(), 5.0, 1e-12);
        const Vec2 side = hexagon[(i + 1) % 6] - hexagon[i];
        EXPECT_NEAR(side.length(), 5.0, 1e-9) << "a hexagon's side is its radius";
    }
    EXPECT_EQ(PolygonTool::corners(Vec2(0, 0), Vec2(1, 0), 2).size(), 3u);
    EXPECT_EQ(PolygonTool::corners(Vec2(0, 0), Vec2(1, 0), 100'000).size(),
              static_cast<size_t>(PolygonTool::kMostSides));
    EXPECT_TRUE(PolygonTool::corners(Vec2(3, 3), Vec2(3, 3), 6).empty());
    EXPECT_TRUE(
        PolygonTool::corners(Vec2(0, 0), Vec2(std::numeric_limits<double>::infinity(), 0), 6)
            .empty());
}

// The view zooms by command too (Phase 168): to what is selected, filling
// the view with it wherever it is and however small, and in and out a step
// at a time; with nothing selected, it says so and stays.
TEST(ToolEditsTest, TheViewZoomsToTheSelectionAndInAndOut) {
    MainWindow w;
    ToolDriver drive(w);
    auto& view = drive.viewport();
    auto& drawing = w.activeDocument()->draftDocument();
    auto speck = std::make_shared<hz::draft::DraftCircle>(Vec2(5000, -3000), 0.01);
    drawing.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(10, 0)));
    drawing.addEntity(speck);

    const auto eyeBefore = view.camera().eye();
    trigger(w, "action_zoom_selection");
    EXPECT_NEAR((view.camera().eye() - eyeBefore).length(), 0.0, 1e-12) << "nothing selected";

    view.selectionManager().select(speck->id());
    trigger(w, "action_zoom_selection");
    const QPointF centre = view.worldToScreen(Vec2(5000, -3000));
    EXPECT_NEAR(centre.x(), view.width() / 2.0, 2.0);
    EXPECT_NEAR(centre.y(), view.height() / 2.0, 2.0);
    const double across = std::abs(view.worldToScreen(Vec2(5000.01, -3000)).x() - centre.x());
    EXPECT_GT(across, 0.1 * view.height()) << "the speck fills much of the view";

    const double distance = (view.camera().eye() - view.camera().target()).length();
    trigger(w, "action_zoom_in");
    EXPECT_NEAR((view.camera().eye() - view.camera().target()).length(), distance / 1.25,
                1e-9 * distance);
    trigger(w, "action_zoom_out");
    trigger(w, "action_zoom_out");
    EXPECT_NEAR((view.camera().eye() - view.camera().target()).length(), distance * 1.25,
                1e-9 * distance);
}

// Edit ▸ Select All (Ctrl+A) takes everything a box round the drawing would:
// what is on a hidden or locked layer stays out, as it does from a box. Then
// Delete takes what is selected, as one step undone whole.
TEST(ToolEditsTest, SelectAllTakesWhatABoxRoundEverythingWould) {
    MainWindow w;
    ToolDriver drive(w);
    auto& view = drive.viewport();
    auto* doc = w.activeDocument();
    auto& drawing = doc->draftDocument();
    for (const char* name : {"Hidden", "Locked"}) {
        hz::draft::LayerProperties props;
        props.name = name;
        props.visible = std::string(name) != "Hidden";
        props.locked = std::string(name) == "Locked";
        doc->layerManager().addLayer(props);
    }
    std::vector<uint64_t> free;
    for (int i = 0; i < 5; ++i) {
        auto line = std::make_shared<hz::draft::DraftLine>(Vec2(i, 0), Vec2(i, 1));
        drawing.addEntity(line);
        free.push_back(line->id());
    }
    auto hidden = std::make_shared<hz::draft::DraftCircle>(Vec2(0, 5), 1.0);
    hidden->setLayer("Hidden");
    auto locked = std::make_shared<hz::draft::DraftCircle>(Vec2(5, 5), 1.0);
    locked->setLayer("Locked");
    drawing.addEntity(hidden);
    drawing.addEntity(locked);

    auto* selectAll = w.findChild<QAction*>(QStringLiteral("action_select_all"));
    ASSERT_NE(selectAll, nullptr);
    EXPECT_EQ(selectAll->shortcut(), QKeySequence(QKeySequence::SelectAll));
    selectAll->trigger();
    auto& sel = view.selectionManager();
    EXPECT_EQ(sel.count(), free.size());
    for (uint64_t id : free) EXPECT_TRUE(sel.isSelected(id));
    EXPECT_FALSE(sel.isSelected(hidden->id()));
    EXPECT_FALSE(sel.isSelected(locked->id()));

    drive.key(Qt::Key_Delete);
    EXPECT_EQ(drawing.entities().size(), 2u) << "the hidden and the locked are kept";
    trigger(w, "action_undo");
    EXPECT_EQ(drawing.entities().size(), 7u) << "one step undone whole";
}

// Match Properties: click an entity, then each that is to look like it. Each
// click is one step to undo, and gives the layer, colour, width and line type
// together; what is on a locked layer is not changed, though the first may
// be on one.
TEST(ToolEditsTest, MatchPropertiesGivesOnesStyleToOthers) {
    MainWindow w;
    ToolDriver drive(w);
    auto* doc = w.activeDocument();
    auto& drawing = doc->draftDocument();
    for (const char* name : {"Walls", "Locked"}) {
        hz::draft::LayerProperties props;
        props.name = name;
        props.locked = std::string(name) == "Locked";
        doc->layerManager().addLayer(props);
    }
    auto source = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(10, 0));
    source->setLayer("Locked");
    source->setColor(0xFFFF0000u);
    source->setLineWidth(0.7);
    source->setLineType(2);
    auto target = std::make_shared<hz::draft::DraftCircle>(Vec2(0, 20), 5.0);
    target->setLayer("Walls");
    auto held = std::make_shared<hz::draft::DraftLine>(Vec2(0, 40), Vec2(10, 40));
    held->setLayer("Locked");
    drawing.addEntity(source);
    drawing.addEntity(target);
    drawing.addEntity(held);
    const uint32_t heldColor = held->color();

    trigger(w, "tool_match-properties");
    auto& view = drive.viewport();
    drive.click(Vec2(5, 0));   // the source, on a locked layer: only read
    drive.click(Vec2(5, 20));  // a point of the circle
    EXPECT_EQ(target->layer(), "Locked");
    EXPECT_EQ(target->color(), 0xFFFF0000u);
    EXPECT_DOUBLE_EQ(target->lineWidth(), 0.7);
    EXPECT_EQ(target->lineType(), 2);

    trigger(w, "action_undo");
    EXPECT_EQ(target->layer(), "Walls") << "one step undone whole";
    EXPECT_NE(target->color(), 0xFFFF0000u);
    EXPECT_EQ(target->lineType(), 0);
    trigger(w, "action_redo");
    EXPECT_EQ(target->color(), 0xFFFF0000u);

    source->setColor(0xFF00FF00u);  // a locked one is never changed
    held->setLayer("Locked");
    drive.click(Vec2(5, 40));
    EXPECT_EQ(held->color(), heldColor);

    // Escape picks another first; with one selected as it starts, that one
    // is the first.
    drive.key(Qt::Key_Escape);
    view.selectionManager().clearSelection();
    view.selectionManager().select(target->id());
    trigger(w, "tool_select");
    trigger(w, "tool_match-properties");
    auto source2 = std::make_shared<hz::draft::DraftLine>(Vec2(30, 0), Vec2(40, 0));
    drawing.addEntity(source2);
    drive.click(Vec2(35, 0));
    EXPECT_EQ(source2->color(), 0xFFFF0000u) << "the selected circle was the first";
    EXPECT_EQ(source2->layer(), "Locked");
}

// Edit ▸ Purge Unused takes out the layers nothing is on, as one step, and
// says so; with nothing to take, it changes nothing and says that.
TEST(ToolEditsTest, PurgeUnusedTakesOutEmptyLayers) {
    MainWindow w;
    auto* doc = w.activeDocument();
    for (int i = 0; i < 50; ++i) {
        hz::draft::LayerProperties props;
        props.name = "Imported " + std::to_string(i);
        doc->layerManager().addLayer(props);
    }
    auto line = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0));
    line->setLayer("Imported 7");
    doc->draftDocument().addEntity(line);
    const size_t before = doc->layerManager().layerNames().size();

    trigger(w, "action_purge");
    EXPECT_EQ(doc->layerManager().layerNames().size(), 2u) << "0 and the one in use";
    EXPECT_NE(doc->layerManager().getLayer("Imported 7"), nullptr);
    trigger(w, "action_undo");
    EXPECT_EQ(doc->layerManager().layerNames().size(), before) << "one step undone whole";
    trigger(w, "action_redo");
    trigger(w, "action_purge");
    EXPECT_EQ(doc->layerManager().layerNames().size(), 2u) << "nothing more to purge";
    EXPECT_FALSE(doc->undoStack().canRedo());
}

// Edit ▸ Delete Duplicates takes out each copy, as one step, keeping the
// first; with a selection, only among what is selected.
TEST(ToolEditsTest, DeleteDuplicatesKeepsTheFirstOfEach) {
    MainWindow w;
    ToolDriver drive(w);
    auto& drawing = w.activeDocument()->draftDocument();
    auto first = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(10, 0));
    auto again = std::make_shared<hz::draft::DraftLine>(Vec2(10, 0), Vec2(0, 0));
    auto circle = std::make_shared<hz::draft::DraftCircle>(Vec2(5, 5), 2.0);
    auto circleAgain = std::make_shared<hz::draft::DraftCircle>(Vec2(5, 5), 2.0);
    for (const std::shared_ptr<hz::draft::DraftEntity>& e :
         std::vector<std::shared_ptr<hz::draft::DraftEntity>>{first, again, circle, circleAgain}) {
        drawing.addEntity(e);
    }

    // Only the circles selected: only their copy goes.
    auto& sel = drive.viewport().selectionManager();
    sel.select(circle->id());
    sel.select(circleAgain->id());
    trigger(w, "action_delete_duplicates");
    EXPECT_EQ(drawing.entities().size(), 3u);
    EXPECT_EQ(drawing.findEntity(circleAgain->id()), nullptr);

    // Nothing selected: the whole drawing.
    sel.clearSelection();
    trigger(w, "action_delete_duplicates");
    EXPECT_EQ(drawing.entities().size(), 2u);
    EXPECT_NE(drawing.findEntity(first->id()), nullptr) << "the first kept";
    EXPECT_EQ(drawing.findEntity(again->id()), nullptr);
    trigger(w, "action_undo");
    EXPECT_EQ(drawing.entities().size(), 3u) << "one step undone whole";
}
