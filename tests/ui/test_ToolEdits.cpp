// Drawing with the tools the way a user does (Phase 112): clicks through the
// viewport's own mouse handling, into the document, and back out by undo.

#include <gtest/gtest.h>

#include <QAction>
#include <memory>
#include <vector>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/ui/MainWindow.h"

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
