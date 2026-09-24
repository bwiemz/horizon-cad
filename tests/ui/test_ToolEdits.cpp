// Drawing with the tools the way a user does (Phase 112): clicks through the
// viewport's own mouse handling, into the document, and back out by undo.

#include <gtest/gtest.h>

#include <QAction>
#include <QDoubleSpinBox>
#include <QLabel>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/ui/MainWindow.h"
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
