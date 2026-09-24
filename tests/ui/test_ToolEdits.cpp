// Drawing with the tools the way a user does (Phase 112): clicks through the
// viewport's own mouse handling, into the document, and back out by undo.

#include <gtest/gtest.h>

#include <QAction>
#include <memory>
#include <vector>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/ui/MainWindow.h"
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
