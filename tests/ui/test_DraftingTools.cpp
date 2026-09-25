// The 2D drafting tools tested by what they draw (Phase 145): each is driven
// through the viewport the way a user drives it, the geometry it commits is
// checked exactly, and undo takes it away again. The tools that change what
// is drawn leave a locked or hidden layer alone.

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QElapsedTimer>
#include <QInputDialog>
#include <QTimer>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftAngularDimension.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftLeader.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRadialDimension.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/Constants.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

using hz::draft::DraftArc;
using hz::draft::DraftCircle;
using hz::draft::DraftLine;
using hz::draft::DraftPolyline;
using hz::math::Vec2;
using hz::test::ToolDriver;
using hz::ui::MainWindow;

namespace {

constexpr double kPi = hz::math::kPi;

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

/// A line put straight into the drawing, as a drawing opened from a file has.
std::shared_ptr<DraftLine> addLine(MainWindow& w, const Vec2& a, const Vec2& b,
                                   const std::string& layer = "0") {
    auto line = std::make_shared<DraftLine>(a, b);
    line->setLayer(layer);
    w.activeDocument()->draftDocument().addEntity(line);
    return line;
}

void addLayer(MainWindow& w, const std::string& name) {
    hz::draft::LayerProperties props;
    props.name = name;
    w.activeDocument()->layerManager().addLayer(props);
}

/// Where a click aimed at @p world lands. A mouse event carries a whole pixel,
/// so a tool that takes the cursor as it is (no snap) gets a point up to half
/// a pixel from the one aimed at.
Vec2 landed(ToolDriver& drive, const Vec2& world) {
    const QPoint at = drive.viewport().worldToScreen(world).toPoint();
    return drive.viewport().worldPositionAtCursor(at.x(), at.y());
}

/// Look straight down on the drawing, as View > Top does: the view 2D work is
/// drawn in. (A new window opens in an isometric view whose eye is so close
/// that the drawing's plane passes through its eye plane, along x + y = 14; a
/// click aimed near there lands far from where it was aimed.)
void viewFromTop(ToolDriver& drive) {
    drive.viewport().camera().setTopView();
}

/// Type @p digits at the viewport, key by key, then Enter: a number for a tool
/// that reads one (an angle, a factor, a radius).
void type(ToolDriver& drive, const std::string& digits) {
    for (char c : digits) {
        drive.key(c == '.' ? Qt::Key_Period : static_cast<Qt::Key>(Qt::Key_0 + (c - '0')));
    }
    drive.key(Qt::Key_Return);
}

/// Choose what is at each point with the Select tool: a click, then Shift
/// clicks that add to it.
void select(MainWindow& w, ToolDriver& drive, const std::vector<Vec2>& at) {
    trigger(w, "tool_select");
    bool first = true;
    for (const Vec2& p : at) {
        drive.clickAt(drive.viewport().worldToScreen(p),
                      first ? Qt::NoModifier : Qt::ShiftModifier);
        first = false;
    }
}

/// Answers the next text dialog with @p text.
class TextAnswer {
public:
    explicit TextAnswer(QString text) : m_text(std::move(text)) {
        QObject::connect(&m_timer, &QTimer::timeout, [this] { poll(); });
        m_timer.start(5);
        m_clock.start();
    }
    bool seen() const { return m_seen; }

private:
    void poll() {
        auto* dialog = qobject_cast<QInputDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr) {
            if (m_clock.elapsed() > 5000) m_timer.stop();
            return;
        }
        m_timer.stop();
        m_seen = true;
        dialog->setTextValue(m_text);
        dialog->accept();
    }

    QString m_text;
    bool m_seen = false;
    QTimer m_timer;
    QElapsedTimer m_clock;
};

}  // namespace

// -- Drawing tools -------------------------------------------------------------

// An arc is drawn counter-clockwise from its centre, through the start point's
// radius and angle, to the end point's angle.
TEST(DraftingToolsTest, TheArcToolDrawsFromCentreStartAndEnd) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    trigger(w, "tool_arc");
    drive.click(Vec2(0, 0));
    drive.click(Vec2(3, 0));
    drive.click(Vec2(0, 3));

    const auto arcs = all<DraftArc>(w);
    ASSERT_EQ(arcs.size(), 1u);
    EXPECT_TRUE(near(arcs[0]->center(), Vec2(0, 0)));
    EXPECT_NEAR(arcs[0]->radius(), 3.0, 1e-9);
    EXPECT_NEAR(arcs[0]->startAngle(), 0.0, 1e-9);
    EXPECT_NEAR(arcs[0]->endAngle(), kPi / 2, 1e-9);
    EXPECT_EQ(arcs[0]->layer(), "0");

    trigger(w, "action_undo");
    EXPECT_TRUE(all<DraftArc>(w).empty());
    trigger(w, "action_redo");
    EXPECT_EQ(all<DraftArc>(w).size(), 1u);
}

// A polyline takes every clicked point until Enter; Escape drops one being
// drawn, and a single point is no polyline.
TEST(DraftingToolsTest, ThePolylineToolDrawsItsPointsUntilEnter) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    trigger(w, "tool_polyline");
    drive.click(Vec2(0, 0));
    drive.click(Vec2(4, 0));
    drive.click(Vec2(4, 3));
    drive.key(Qt::Key_Return);

    auto polylines = all<DraftPolyline>(w);
    ASSERT_EQ(polylines.size(), 1u);
    const std::vector<Vec2> expected{Vec2(0, 0), Vec2(4, 0), Vec2(4, 3)};
    ASSERT_EQ(polylines[0]->points().size(), expected.size());
    for (size_t i = 0; i < expected.size(); ++i) {
        EXPECT_TRUE(near(polylines[0]->points()[i], expected[i])) << "point " << i;
    }
    EXPECT_FALSE(polylines[0]->closed());

    drive.click(Vec2(10, 0));
    drive.click(Vec2(12, 0));
    drive.key(Qt::Key_Escape);
    drive.click(Vec2(10, 5));
    drive.key(Qt::Key_Return);
    EXPECT_EQ(all<DraftPolyline>(w).size(), 1u) << "escaped, then one point only";

    trigger(w, "action_undo");
    EXPECT_TRUE(all<DraftPolyline>(w).empty());
    trigger(w, "action_redo");
    EXPECT_EQ(all<DraftPolyline>(w).size(), 1u);
}

// A spline's control points are the clicked points, in order; it needs four.
TEST(DraftingToolsTest, TheSplineToolTakesItsControlPointsInOrder) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    trigger(w, "tool_spline");
    const std::vector<Vec2> points{Vec2(0, 0), Vec2(2, 4), Vec2(5, 4), Vec2(7, 0)};
    for (const Vec2& p : points) drive.click(p);
    drive.key(Qt::Key_Return);

    auto splines = all<hz::draft::DraftSpline>(w);
    ASSERT_EQ(splines.size(), 1u);
    ASSERT_EQ(splines[0]->controlPoints().size(), points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        EXPECT_TRUE(near(splines[0]->controlPoints()[i], points[i])) << "point " << i;
    }
    EXPECT_FALSE(splines[0]->closed());

    drive.click(Vec2(20, 0));
    drive.click(Vec2(22, 3));
    drive.click(Vec2(24, 0));
    drive.key(Qt::Key_Return);
    EXPECT_EQ(all<hz::draft::DraftSpline>(w).size(), 1u) << "three points are too few";

    trigger(w, "action_undo");
    EXPECT_TRUE(all<hz::draft::DraftSpline>(w).empty());
    trigger(w, "action_redo");
    EXPECT_EQ(all<hz::draft::DraftSpline>(w).size(), 1u);
}

// A click on a closed shape hatches inside its outline, on the current layer;
// a click on an open line hatches nothing.
TEST(DraftingToolsTest, TheHatchToolFillsTheClickedClosedShape) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    w.activeDocument()->draftDocument().addEntity(
        std::make_shared<hz::draft::DraftRectangle>(Vec2(0, 0), Vec2(6, 4)));
    addLine(w, Vec2(10, 0), Vec2(14, 0));
    addLayer(w, "Hatching");
    w.activeDocument()->layerManager().setCurrentLayer("Hatching");

    trigger(w, "tool_hatch");
    drive.click(Vec2(12, 0));
    EXPECT_TRUE(all<hz::draft::DraftHatch>(w).empty()) << "an open line has no inside";

    drive.click(Vec2(3, 0));
    const auto hatches = all<hz::draft::DraftHatch>(w);
    ASSERT_EQ(hatches.size(), 1u);
    const std::vector<Vec2> outline{Vec2(0, 0), Vec2(6, 0), Vec2(6, 4), Vec2(0, 4)};
    ASSERT_EQ(hatches[0]->boundary().size(), outline.size());
    for (size_t i = 0; i < outline.size(); ++i) {
        EXPECT_TRUE(near(hatches[0]->boundary()[i], outline[i])) << "corner " << i;
    }
    EXPECT_EQ(hatches[0]->layer(), "Hatching");

    trigger(w, "action_undo");
    EXPECT_TRUE(all<hz::draft::DraftHatch>(w).empty());
    EXPECT_EQ(all<hz::draft::DraftRectangle>(w).size(), 1u) << "the outline stays";
    trigger(w, "action_redo");
    EXPECT_EQ(all<hz::draft::DraftHatch>(w).size(), 1u);
}

// A leader runs through its clicked points and carries the text typed for it
// at Enter; with the text refused, there is no leader.
TEST(DraftingToolsTest, TheLeaderToolTakesItsPointsAndText) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    trigger(w, "tool_leader");
    drive.click(Vec2(0, 0));
    drive.click(Vec2(3, 2));
    drive.click(Vec2(6, 2));
    {
        TextAnswer answer(QStringLiteral("Check fit"));
        drive.key(Qt::Key_Return);
        ASSERT_TRUE(answer.seen());
    }
    auto leaders = all<hz::draft::DraftLeader>(w);
    ASSERT_EQ(leaders.size(), 1u);
    const std::vector<Vec2> expected{Vec2(0, 0), Vec2(3, 2), Vec2(6, 2)};
    ASSERT_EQ(leaders[0]->points().size(), expected.size());
    for (size_t i = 0; i < expected.size(); ++i) {
        EXPECT_TRUE(near(leaders[0]->points()[i], expected[i])) << "point " << i;
    }
    EXPECT_EQ(leaders[0]->text(), "Check fit");

    drive.click(Vec2(10, 0));
    drive.click(Vec2(12, 2));
    {
        hz::test::ModalCloser refuse;
        drive.key(Qt::Key_Return);
        EXPECT_EQ(refuse.dismissed().size(), 1);
    }
    EXPECT_EQ(all<hz::draft::DraftLeader>(w).size(), 1u) << "no text, no leader";

    trigger(w, "action_undo");
    EXPECT_TRUE(all<hz::draft::DraftLeader>(w).empty());
    trigger(w, "action_redo");
    EXPECT_EQ(all<hz::draft::DraftLeader>(w).size(), 1u);
}

// -- Modifying tools -----------------------------------------------------------

// Offset copies a line parallel to it, on the side of the cursor and as far
// from it as the cursor is, and a circle through the cursor, in or out. The
// copy keeps the source's style; the source stays.
TEST(DraftingToolsTest, OffsetCopiesThroughTheCursor) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    auto line = addLine(w, Vec2(0, 0), Vec2(10, 0));
    line->setLineType(2);
    auto circle = std::make_shared<DraftCircle>(Vec2(20, 0), 3.0);
    w.activeDocument()->draftDocument().addEntity(circle);

    trigger(w, "tool_offset");
    drive.click(Vec2(5, 0));
    drive.move(Vec2(3, 2));
    drive.click(Vec2(3, 2));
    const double above = landed(drive, Vec2(3, 2)).y;
    drive.click(Vec2(5, 0));
    drive.move(Vec2(3, -2));
    drive.click(Vec2(3, -2));
    const double below = landed(drive, Vec2(3, -2)).y;

    auto lines = all<DraftLine>(w);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(0, 0)) && near(lines[0]->end(), Vec2(10, 0)))
        << "the source stays";
    EXPECT_TRUE(near(lines[1]->start(), Vec2(0, above)));
    EXPECT_TRUE(near(lines[1]->end(), Vec2(10, above)));
    EXPECT_NEAR(above, 2.0, 0.01) << "the cursor's side and distance";
    EXPECT_TRUE(near(lines[2]->start(), Vec2(0, below)));
    EXPECT_TRUE(near(lines[2]->end(), Vec2(10, below)));
    EXPECT_NEAR(below, -2.0, 0.01);
    EXPECT_EQ(lines[1]->lineType(), 2) << "still dashed";

    drive.click(Vec2(23, 0));
    drive.move(Vec2(25, 0));
    drive.click(Vec2(25, 0));
    drive.click(Vec2(23, 0));
    drive.move(Vec2(21, 0));
    drive.click(Vec2(21, 0));
    auto circles = all<DraftCircle>(w);
    ASSERT_EQ(circles.size(), 3u);
    EXPECT_TRUE(near(circles[1]->center(), Vec2(20, 0)));
    EXPECT_NEAR(circles[1]->radius(), landed(drive, Vec2(25, 0)).distanceTo(Vec2(20, 0)), 1e-9)
        << "out to the cursor";
    EXPECT_NEAR(circles[2]->radius(), landed(drive, Vec2(21, 0)).distanceTo(Vec2(20, 0)), 1e-9)
        << "in to the cursor";

    trigger(w, "action_undo");
    trigger(w, "action_undo");
    EXPECT_EQ(all<DraftCircle>(w).size(), 1u) << "one step per copy";
    trigger(w, "action_undo");
    EXPECT_EQ(all<DraftLine>(w).size(), 2u);
    trigger(w, "action_redo");
    EXPECT_EQ(all<DraftLine>(w).size(), 3u);
}

// Break splits a line where the line nearest the click crosses it; the pieces
// keep its style, and the crossing line is not touched.
TEST(DraftingToolsTest, BreakSplitsALineWhereAnotherCrossesIt) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    auto target = addLine(w, Vec2(0, 0), Vec2(10, 0));
    target->setLineType(2);
    target->setGroupId(7);
    addLine(w, Vec2(4, -3), Vec2(4, 3));

    trigger(w, "tool_break");
    drive.click(Vec2(7, 0));

    auto lines = all<DraftLine>(w);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(4, -3)) && near(lines[0]->end(), Vec2(4, 3)))
        << "the crossing line is untouched";
    EXPECT_TRUE(near(lines[1]->start(), Vec2(0, 0)));
    EXPECT_TRUE(near(lines[1]->end(), Vec2(4, 0)));
    EXPECT_TRUE(near(lines[2]->start(), Vec2(4, 0)));
    EXPECT_TRUE(near(lines[2]->end(), Vec2(10, 0)));
    for (const auto* piece : {lines[1], lines[2]}) {
        EXPECT_EQ(piece->lineType(), 2);
        EXPECT_EQ(piece->groupId(), 7u);
    }

    trigger(w, "action_undo");
    lines = all<DraftLine>(w);
    ASSERT_EQ(lines.size(), 2u) << "one step";
    EXPECT_EQ(lines[0]->id(), target->id()) << "the line itself is back";
    trigger(w, "action_redo");
    EXPECT_EQ(all<DraftLine>(w).size(), 3u);
}

// A circle crossed twice breaks into the two arcs between the crossings.
TEST(DraftingToolsTest, BreakCutsACircleIntoTwoArcs) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    w.activeDocument()->draftDocument().addEntity(std::make_shared<DraftCircle>(Vec2(0, 0), 3.0));
    addLine(w, Vec2(0, -5), Vec2(0, 5));

    trigger(w, "tool_break");
    drive.click(Vec2(3, 0));

    EXPECT_TRUE(all<DraftCircle>(w).empty());
    const auto arcs = all<DraftArc>(w);
    ASSERT_EQ(arcs.size(), 2u);
    for (const auto* arc : arcs) {
        EXPECT_TRUE(near(arc->center(), Vec2(0, 0)));
        EXPECT_NEAR(arc->radius(), 3.0, 1e-9);
    }
    EXPECT_NEAR(arcs[0]->startAngle(), kPi / 2, 1e-9) << "the left half";
    EXPECT_NEAR(arcs[0]->endAngle(), 3 * kPi / 2, 1e-9);
    EXPECT_NEAR(arcs[1]->startAngle(), 3 * kPi / 2, 1e-9) << "the right half";
    EXPECT_NEAR(arcs[1]->endAngle(), kPi / 2, 1e-9);

    trigger(w, "action_undo");
    EXPECT_EQ(all<DraftCircle>(w).size(), 1u);
    EXPECT_TRUE(all<DraftArc>(w).empty());
}

// Extend lengthens the end nearer the click to the nearest line in its way,
// not a farther one.
TEST(DraftingToolsTest, ExtendReachesTheNearestBoundaryFromTheClickedEnd) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    addLine(w, Vec2(0, 0), Vec2(6, 0));
    addLine(w, Vec2(10, -3), Vec2(10, 3));
    addLine(w, Vec2(14, -3), Vec2(14, 3));
    addLine(w, Vec2(-4, -3), Vec2(-4, 3));
    const auto extended = [&w] {
        for (const auto* line : all<DraftLine>(w)) {
            if (std::abs(line->start().y) < 1e-12 && std::abs(line->end().y) < 1e-12) return line;
        }
        return static_cast<const DraftLine*>(nullptr);
    };

    trigger(w, "tool_extend");
    drive.click(Vec2(5, 0));
    ASSERT_NE(extended(), nullptr);
    EXPECT_TRUE(near(extended()->start(), Vec2(0, 0)));
    EXPECT_TRUE(near(extended()->end(), Vec2(10, 0))) << "to x = 10, not x = 14";

    drive.click(Vec2(1, 0));
    ASSERT_NE(extended(), nullptr);
    EXPECT_TRUE(near(extended()->start(), Vec2(-4, 0)));
    EXPECT_TRUE(near(extended()->end(), Vec2(10, 0)));
    EXPECT_EQ(all<DraftLine>(w).size(), 4u);

    trigger(w, "action_undo");
    EXPECT_TRUE(near(extended()->start(), Vec2(0, 0)));
    trigger(w, "action_undo");
    EXPECT_TRUE(near(extended()->end(), Vec2(6, 0)));
    trigger(w, "action_redo");
    EXPECT_TRUE(near(extended()->end(), Vec2(10, 0)));
}

// Polyline Edit drags a vertex, adds one on the clicked segment (A), removes
// the clicked one (D) and closes the polyline (C): each is one undo step.
TEST(DraftingToolsTest, PolylineEditMovesAddsRemovesAndCloses) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    w.activeDocument()->draftDocument().addEntity(
        std::make_shared<DraftPolyline>(std::vector<Vec2>{Vec2(0, 0), Vec2(10, 0), Vec2(10, 10)}));
    const auto points = [&w] { return all<DraftPolyline>(w).at(0)->points(); };

    trigger(w, "tool_polyline-edit");
    drive.click(Vec2(5, 0));  // this one to edit
    drive.drag(Vec2(10, 10), Vec2(13, 12));
    ASSERT_EQ(points().size(), 3u);
    EXPECT_TRUE(near(points()[2], Vec2(13, 12))) << "dragged";

    drive.key(Qt::Key_A);
    drive.click(Vec2(4, 0));
    ASSERT_EQ(points().size(), 4u);
    EXPECT_TRUE(near(points()[1], Vec2(landed(drive, Vec2(4, 0)).x, 0)))
        << "on the clicked segment, where it was clicked";
    EXPECT_TRUE(near(points()[2], Vec2(10, 0)));

    drive.key(Qt::Key_D);
    drive.click(Vec2(13, 12));
    ASSERT_EQ(points().size(), 3u);
    EXPECT_TRUE(near(points()[2], Vec2(10, 0))) << "the last one is gone";

    drive.key(Qt::Key_C);
    EXPECT_TRUE(all<DraftPolyline>(w).at(0)->closed());

    trigger(w, "action_undo");
    EXPECT_FALSE(all<DraftPolyline>(w).at(0)->closed());
    trigger(w, "action_undo");
    EXPECT_EQ(points().size(), 4u);
    trigger(w, "action_undo");
    EXPECT_EQ(points().size(), 3u);
    trigger(w, "action_undo");
    EXPECT_TRUE(near(points()[2], Vec2(10, 10))) << "the drag undone";
    trigger(w, "action_redo");
    EXPECT_TRUE(near(points()[2], Vec2(13, 12)));
}

// Join (J) makes one polyline of two that meet end to end: the point they
// share is one vertex, not two, and the other polyline is gone.
TEST(DraftingToolsTest, PolylineEditJoinsTwoPolylinesThatMeet) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    auto& drawing = w.activeDocument()->draftDocument();
    auto first = std::make_shared<DraftPolyline>(std::vector<Vec2>{Vec2(0, 0), Vec2(10, 0)});
    drawing.addEntity(first);
    drawing.addEntity(
        std::make_shared<DraftPolyline>(std::vector<Vec2>{Vec2(10, 0), Vec2(10, 10), Vec2(0, 10)}));

    trigger(w, "tool_polyline-edit");
    drive.click(Vec2(5, 0));
    drive.key(Qt::Key_J);
    drive.click(Vec2(10, 5));

    auto polylines = all<DraftPolyline>(w);
    ASSERT_EQ(polylines.size(), 1u);
    EXPECT_EQ(polylines[0]->id(), first->id());
    const std::vector<Vec2> expected{Vec2(0, 0), Vec2(10, 0), Vec2(10, 10), Vec2(0, 10)};
    ASSERT_EQ(polylines[0]->points().size(), expected.size())
        << "the shared point (10, 0) is one vertex, not a segment of no length";
    for (size_t i = 0; i < expected.size(); ++i) {
        EXPECT_TRUE(near(polylines[0]->points()[i], expected[i])) << "point " << i;
    }

    trigger(w, "action_undo");
    polylines = all<DraftPolyline>(w);
    ASSERT_EQ(polylines.size(), 2u);
    EXPECT_EQ(polylines[0]->points().size(), 2u);
}

// Mirror adds a reflected copy of the selection across the clicked axis and
// selects it; the originals stay. The copy of an arc is the reflection of the
// arc, not the arc turned the other way round.
TEST(DraftingToolsTest, MirrorCopiesTheSelectionAcrossTheClickedAxis) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    auto line = addLine(w, Vec2(1, 1), Vec2(4, 2));
    auto arc = std::make_shared<DraftArc>(Vec2(3, -2), 1.0, 0.0, kPi / 2);
    w.activeDocument()->draftDocument().addEntity(arc);
    select(w, drive, {Vec2(2.5, 1.5), arc->midPoint()});
    ASSERT_EQ(drive.viewport().selectionManager().selectedIds().size(), 2u);

    trigger(w, "tool_mirror");
    drive.click(Vec2(0, -3));
    drive.click(Vec2(0, 3));  // the axis x = 0

    const auto lines = all<DraftLine>(w);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(1, 1)) && near(lines[0]->end(), Vec2(4, 2)))
        << "the original stays";
    EXPECT_TRUE(near(lines[1]->start(), Vec2(-1, 1)));
    EXPECT_TRUE(near(lines[1]->end(), Vec2(-4, 2)));
    const auto arcs = all<DraftArc>(w);
    ASSERT_EQ(arcs.size(), 2u);
    EXPECT_TRUE(near(arcs[1]->center(), Vec2(-3, -2)));
    EXPECT_NEAR(arcs[1]->radius(), 1.0, 1e-9);
    const auto reflect = [](const Vec2& p) { return Vec2(-p.x, p.y); };
    EXPECT_TRUE(near(arcs[1]->midPoint(), reflect(arc->midPoint())))
        << "the same quarter, reflected";
    const auto selected = drive.viewport().selectionManager().selectedIds();
    ASSERT_EQ(selected.size(), 2u) << "the copies are selected";
    for (uint64_t id : selected) EXPECT_NE(id, line->id());

    trigger(w, "action_undo");
    EXPECT_EQ(all<DraftLine>(w).size(), 1u);
    EXPECT_EQ(all<DraftArc>(w).size(), 1u);
    trigger(w, "action_redo");
    EXPECT_EQ(all<DraftLine>(w).size(), 2u);
}

// Rotate adds a copy of the selection turned about the clicked centre, by a
// typed angle in degrees or to the clicked direction, and selects the copy.
TEST(DraftingToolsTest, RotateCopiesTheSelectionAboutTheClickedCentre) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    addLine(w, Vec2(1, 0), Vec2(3, 0));
    select(w, drive, {Vec2(2, 0)});

    trigger(w, "tool_rotate");
    drive.click(Vec2(0, 0));
    type(drive, "90");
    auto lines = all<DraftLine>(w);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(1, 0))) << "the original stays";
    EXPECT_TRUE(near(lines[1]->start(), Vec2(0, 1)));
    EXPECT_TRUE(near(lines[1]->end(), Vec2(0, 3)));

    drive.click(Vec2(0, 0));
    drive.click(Vec2(-2, 0));  // a half turn: the copy just made, turned again
    lines = all<DraftLine>(w);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_TRUE(near(lines[2]->start(), Vec2(0, -1)));
    EXPECT_TRUE(near(lines[2]->end(), Vec2(0, -3)));

    trigger(w, "action_undo");
    EXPECT_EQ(all<DraftLine>(w).size(), 2u);
    trigger(w, "action_undo");
    EXPECT_EQ(all<DraftLine>(w).size(), 1u);
    trigger(w, "action_redo");
    EXPECT_EQ(all<DraftLine>(w).size(), 2u);
}

// Scale adds a copy of the selection scaled from the clicked base point, by a
// typed factor, or by the clicked distance over the selection's distance.
TEST(DraftingToolsTest, ScaleCopiesTheSelectionFromTheClickedBasePoint) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    w.activeDocument()->draftDocument().addEntity(std::make_shared<DraftCircle>(Vec2(2, 1), 1.0));
    select(w, drive, {Vec2(3, 1)});

    trigger(w, "tool_scale");
    drive.click(Vec2(0, 0));
    type(drive, "2");
    auto circles = all<DraftCircle>(w);
    ASSERT_EQ(circles.size(), 2u);
    EXPECT_TRUE(near(circles[0]->center(), Vec2(2, 1))) << "the original stays";
    EXPECT_TRUE(near(circles[1]->center(), Vec2(4, 2)));
    EXPECT_NEAR(circles[1]->radius(), 2.0, 1e-9);

    // The copy, centred sqrt(20) from the base: a click sqrt(80) away doubles it.
    drive.click(Vec2(0, 0));
    drive.click(Vec2(8, 4));
    circles = all<DraftCircle>(w);
    ASSERT_EQ(circles.size(), 3u);
    EXPECT_TRUE(near(circles[2]->center(), Vec2(8, 4)));
    EXPECT_NEAR(circles[2]->radius(), 4.0, 1e-9);

    trigger(w, "action_undo");
    trigger(w, "action_undo");
    EXPECT_EQ(all<DraftCircle>(w).size(), 1u);
    trigger(w, "action_redo");
    EXPECT_EQ(all<DraftCircle>(w).size(), 2u);
}

// Move drags the selection by where the drag went, in one undo step.
TEST(DraftingToolsTest, MoveDragsTheSelection) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    auto line = addLine(w, Vec2(0, 0), Vec2(4, 0));
    addLine(w, Vec2(0, 5), Vec2(4, 5));  // not selected
    select(w, drive, {Vec2(2, 0)});

    trigger(w, "tool_move");
    drive.drag(Vec2(2, 0), Vec2(5, 3));
    auto lines = all<DraftLine>(w);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_EQ(lines[0]->id(), line->id()) << "moved, not copied";
    EXPECT_TRUE(near(lines[0]->start(), Vec2(3, 3)));
    EXPECT_TRUE(near(lines[0]->end(), Vec2(7, 3)));
    EXPECT_TRUE(near(lines[1]->start(), Vec2(0, 5))) << "what was not selected stays";

    trigger(w, "action_undo");
    EXPECT_TRUE(near(all<DraftLine>(w)[0]->start(), Vec2(0, 0)));
    trigger(w, "action_redo");
    EXPECT_TRUE(near(all<DraftLine>(w)[0]->start(), Vec2(3, 3)));
}

// Copy and Paste put a copy of the selection with its centre at the clicked
// point; Duplicate puts one a step down and to the right. The originals stay.
TEST(DraftingToolsTest, CopyPasteAndDuplicatePlaceCopies) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    addLine(w, Vec2(0, 0), Vec2(4, 0));
    select(w, drive, {Vec2(2, 0)});

    trigger(w, "action_copy");
    trigger(w, "action_paste");
    drive.click(Vec2(10, 5));
    auto lines = all<DraftLine>(w);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(0, 0)));
    EXPECT_TRUE(near(lines[1]->start(), Vec2(8, 5)));
    EXPECT_TRUE(near(lines[1]->end(), Vec2(12, 5)));

    trigger(w, "action_duplicate");  // the pasted copy is what is selected now
    lines = all<DraftLine>(w);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_TRUE(near(lines[2]->start(), Vec2(9, 4)));
    EXPECT_TRUE(near(lines[2]->end(), Vec2(13, 4)));

    trigger(w, "action_undo");
    trigger(w, "action_undo");
    EXPECT_EQ(all<DraftLine>(w).size(), 1u);
    trigger(w, "action_redo");
    EXPECT_EQ(all<DraftLine>(w).size(), 2u);
}

// Fillet rounds the corner of two lines with an arc of the typed radius,
// tangent to both, and cuts the lines back to it: one undo step.
TEST(DraftingToolsTest, FilletRoundsTheCornerOfTwoLines) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    auto along = addLine(w, Vec2(0, 0), Vec2(10, 0));
    along->setLineType(2);
    addLine(w, Vec2(0, 0), Vec2(0, 10));

    trigger(w, "tool_fillet");
    type(drive, "2");
    drive.click(Vec2(5, 0));
    drive.click(Vec2(0, 5));

    auto lines = all<DraftLine>(w);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(2, 0)));
    EXPECT_TRUE(near(lines[0]->end(), Vec2(10, 0)));
    EXPECT_TRUE(near(lines[1]->start(), Vec2(0, 2)));
    EXPECT_TRUE(near(lines[1]->end(), Vec2(0, 10)));
    const auto arcs = all<DraftArc>(w);
    ASSERT_EQ(arcs.size(), 1u);
    EXPECT_TRUE(near(arcs[0]->center(), Vec2(2, 2)));
    EXPECT_NEAR(arcs[0]->radius(), 2.0, 1e-9);
    EXPECT_TRUE(near(arcs[0]->startPoint(), Vec2(0, 2))) << "the corner's quarter";
    EXPECT_TRUE(near(arcs[0]->endPoint(), Vec2(2, 0)));
    EXPECT_EQ(arcs[0]->lineType(), 2) << "styled as the first line";

    trigger(w, "action_undo");
    lines = all<DraftLine>(w);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(0, 0)));
    EXPECT_TRUE(near(lines[1]->start(), Vec2(0, 0)));
    EXPECT_TRUE(all<DraftArc>(w).empty());
    trigger(w, "action_redo");
    EXPECT_EQ(all<DraftArc>(w).size(), 1u);
}

// Chamfer cuts the corner of two lines with a line the typed distance along
// each: one undo step.
TEST(DraftingToolsTest, ChamferCutsTheCornerOfTwoLines) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    addLine(w, Vec2(0, 0), Vec2(10, 0));
    addLine(w, Vec2(0, 0), Vec2(0, 10));

    trigger(w, "tool_chamfer");
    type(drive, "3");
    drive.click(Vec2(5, 0));
    drive.click(Vec2(0, 5));

    const auto lines = all<DraftLine>(w);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(3, 0)));
    EXPECT_TRUE(near(lines[0]->end(), Vec2(10, 0)));
    EXPECT_TRUE(near(lines[1]->start(), Vec2(0, 3)));
    EXPECT_TRUE(near(lines[1]->end(), Vec2(0, 10)));
    EXPECT_TRUE(near(lines[2]->start(), Vec2(3, 0)));
    EXPECT_TRUE(near(lines[2]->end(), Vec2(0, 3))) << "the cut";

    trigger(w, "action_undo");
    ASSERT_EQ(all<DraftLine>(w).size(), 2u);
    EXPECT_TRUE(near(all<DraftLine>(w)[0]->start(), Vec2(0, 0)));
    trigger(w, "action_redo");
    EXPECT_EQ(all<DraftLine>(w).size(), 3u);
}

// -- Dimensions ----------------------------------------------------------------

// A linear dimension measures between its two clicked points, horizontally
// when its line is placed above or below them, vertically when to the side.
TEST(DraftingToolsTest, ALinearDimensionMeasuresTheClickedPoints) {
    using Orientation = hz::draft::DraftLinearDimension::Orientation;
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    trigger(w, "tool_dim-linear");
    drive.click(Vec2(0, 0));
    drive.click(Vec2(8, 1));
    drive.click(Vec2(4, 4));
    drive.click(Vec2(20, 0));
    drive.click(Vec2(21, 5));
    drive.click(Vec2(25, 2));

    const auto dims = all<hz::draft::DraftLinearDimension>(w);
    ASSERT_EQ(dims.size(), 2u);
    EXPECT_TRUE(near(dims[0]->defPoint1(), Vec2(0, 0)));
    EXPECT_TRUE(near(dims[0]->defPoint2(), Vec2(8, 1)));
    EXPECT_TRUE(near(dims[0]->dimLinePoint(), Vec2(4, 4)));
    EXPECT_EQ(dims[0]->orientation(), Orientation::Horizontal);
    EXPECT_NEAR(dims[0]->computedValue(), 8.0, 1e-9) << "the horizontal distance";
    EXPECT_EQ(dims[1]->orientation(), Orientation::Vertical);
    EXPECT_NEAR(dims[1]->computedValue(), 5.0, 1e-9) << "the vertical distance";

    trigger(w, "action_undo");
    EXPECT_EQ(all<hz::draft::DraftLinearDimension>(w).size(), 1u);
    trigger(w, "action_redo");
    EXPECT_EQ(all<hz::draft::DraftLinearDimension>(w).size(), 2u);
}

// Tab while a dimension is placed changes its kind: a linear one steps through
// horizontal, vertical and aligned, a radial one becomes a diameter.
TEST(DraftingToolsTest, TabChangesTheKindOfDimensionBeingPlaced) {
    using Orientation = hz::draft::DraftLinearDimension::Orientation;
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    trigger(w, "tool_dim-linear");
    drive.click(Vec2(0, 0));
    drive.click(Vec2(3, 4));
    drive.key(Qt::Key_Tab);  // horizontal
    drive.key(Qt::Key_Tab);  // vertical
    drive.key(Qt::Key_Tab);  // aligned
    drive.click(Vec2(-2, 3));
    const auto dims = all<hz::draft::DraftLinearDimension>(w);
    ASSERT_EQ(dims.size(), 1u);
    EXPECT_EQ(dims[0]->orientation(), Orientation::Aligned);
    EXPECT_NEAR(dims[0]->computedValue(), 5.0, 1e-9) << "along the points";

    w.activeDocument()->draftDocument().addEntity(std::make_shared<DraftCircle>(Vec2(20, 0), 3.0));
    trigger(w, "tool_dim-radial");
    drive.click(Vec2(23, 0));
    drive.key(Qt::Key_Tab);
    drive.move(Vec2(25, 4));
    drive.click(Vec2(25, 4));
    const auto radial = all<hz::draft::DraftRadialDimension>(w);
    ASSERT_EQ(radial.size(), 1u);
    EXPECT_TRUE(radial[0]->isDiameter());
    EXPECT_NEAR(radial[0]->computedValue(), 6.0, 1e-9);
}

// A radial dimension measures the clicked circle or arc, its text where the
// cursor is.
TEST(DraftingToolsTest, ARadialDimensionMeasuresTheClickedCircleOrArc) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    auto& drawing = w.activeDocument()->draftDocument();
    drawing.addEntity(std::make_shared<DraftCircle>(Vec2(0, 0), 3.0));
    drawing.addEntity(std::make_shared<DraftArc>(Vec2(10, 0), 2.0, 0.0, kPi));

    trigger(w, "tool_dim-radial");
    drive.click(Vec2(3, 0));
    drive.move(Vec2(4, 4));
    drive.click(Vec2(4, 4));
    drive.click(Vec2(10, 2));
    drive.move(Vec2(12, 3));
    drive.click(Vec2(12, 3));

    const auto dims = all<hz::draft::DraftRadialDimension>(w);
    ASSERT_EQ(dims.size(), 2u);
    EXPECT_TRUE(near(dims[0]->center(), Vec2(0, 0)));
    EXPECT_NEAR(dims[0]->radius(), 3.0, 1e-9);
    EXPECT_FALSE(dims[0]->isDiameter());
    EXPECT_NEAR(dims[0]->computedValue(), 3.0, 1e-9);
    EXPECT_TRUE(near(dims[0]->textPoint(), landed(drive, Vec2(4, 4))));
    EXPECT_TRUE(near(dims[1]->center(), Vec2(10, 0)));
    EXPECT_NEAR(dims[1]->computedValue(), 2.0, 1e-9) << "the arc's radius";

    trigger(w, "action_undo");
    EXPECT_EQ(all<hz::draft::DraftRadialDimension>(w).size(), 1u);
    trigger(w, "action_redo");
    EXPECT_EQ(all<hz::draft::DraftRadialDimension>(w).size(), 2u);
}

// An angular dimension measures the angle between the two clicked lines, at
// their crossing, with its arc through the cursor on the side it is.
TEST(DraftingToolsTest, AnAngularDimensionMeasuresBetweenTwoLines) {
    MainWindow w;
    ToolDriver drive(w);
    viewFromTop(drive);
    addLine(w, Vec2(0, 0), Vec2(10, 0));
    addLine(w, Vec2(0, 0), Vec2(5, 5));

    trigger(w, "tool_dim-angular");
    drive.click(Vec2(6, 0));
    drive.click(Vec2(3, 3));
    drive.move(Vec2(4, 1));
    drive.click(Vec2(4, 1));

    const auto dims = all<hz::draft::DraftAngularDimension>(w);
    ASSERT_EQ(dims.size(), 1u);
    EXPECT_TRUE(near(dims[0]->vertex(), Vec2(0, 0)));
    EXPECT_NEAR(dims[0]->computedValue(), 45.0, 1e-9);
    const double r = landed(drive, Vec2(4, 1)).length();
    EXPECT_NEAR(dims[0]->arcRadius(), r, 1e-9) << "through the cursor";
    EXPECT_TRUE(near(dims[0]->line1Point(), Vec2(r, 0)));
    EXPECT_TRUE(near(dims[0]->line2Point(), Vec2(r, r) * std::sqrt(0.5)));

    trigger(w, "action_undo");
    EXPECT_TRUE(all<hz::draft::DraftAngularDimension>(w).empty());
    trigger(w, "action_redo");
    EXPECT_EQ(all<hz::draft::DraftAngularDimension>(w).size(), 1u);
}

// -- Locked and hidden layers --------------------------------------------------

namespace {

/// The line (0, 0)-(10, 0) on the layer "Guarded", with lines on layer 0 for
/// the tools to work against: one meeting it at (5, 0), one across its way at
/// x = 15, and one at y = 10 chosen with it.
struct GuardedScene {
    uint64_t guarded = 0;

    explicit GuardedScene(MainWindow& w) {
        addLayer(w, "Guarded");
        guarded = addLine(w, Vec2(0, 0), Vec2(10, 0), "Guarded")->id();
        addLine(w, Vec2(5, 0), Vec2(5, 5));
        addLine(w, Vec2(15, -5), Vec2(15, 5));
        addLine(w, Vec2(0, 10), Vec2(4, 10));
    }

    /// Whether the layer holds the guarded line alone, as it was.
    bool untouched(MainWindow& w) const {
        std::vector<const hz::draft::DraftEntity*> onLayer;
        for (const auto& e : w.activeDocument()->draftDocument().entities()) {
            if (e->layer() == "Guarded") onLayer.push_back(e.get());
        }
        if (onLayer.size() != 1 || onLayer[0]->id() != guarded) return false;
        const auto* line = dynamic_cast<const DraftLine*>(onLayer[0]);
        return line != nullptr && near(line->start(), Vec2(0, 0)) && near(line->end(), Vec2(10, 0));
    }
};

const std::vector<std::string> kModifyingTools{"offset",  "break", "extend", "trim",   "fillet",
                                               "chamfer", "move",  "mirror", "rotate", "scale"};

/// What a user does with @p tool to change the guarded line, having chosen
/// it and the line at y = 10.
void useOnTheGuardedLine(MainWindow& w, ToolDriver& drive, const std::string& tool) {
    trigger(w, ("tool_" + tool).c_str());
    if (tool == "offset") {
        drive.click(Vec2(3, 0));
        drive.move(Vec2(3, -2));
        drive.click(Vec2(3, -2));
    } else if (tool == "break" || tool == "trim") {
        drive.click(Vec2(8, 0));
    } else if (tool == "extend") {
        drive.click(Vec2(9, 0));
    } else if (tool == "fillet" || tool == "chamfer") {
        drive.click(Vec2(5, 3));
        drive.click(Vec2(8, 0));
    } else if (tool == "move") {
        drive.drag(Vec2(2, 10), Vec2(2, 13));
    } else if (tool == "mirror") {
        drive.click(Vec2(-2, -3));
        drive.click(Vec2(-2, 3));
    } else if (tool == "rotate") {
        drive.click(Vec2(-3, -3));
        type(drive, "90");
    } else if (tool == "scale") {
        drive.click(Vec2(-3, -3));
        type(drive, "2");
    }
    drive.key(Qt::Key_Escape);
}

/// Each modifying tool used on the guarded line, chosen with a line on layer
/// 0, after @p guard has hidden or locked its layer: the line is untouched.
/// The same use changes it on a layer left alone, so the clicks do reach it.
void expectEveryToolLeavesItAlone(void (*guard)(hz::draft::LayerProperties&)) {
    for (const std::string& tool : kModifyingTools) {
        for (const bool guarded : {true, false}) {
            SCOPED_TRACE(tool + (guarded ? " on a guarded layer" : " on an ordinary layer"));
            MainWindow w;
            ToolDriver drive(w);
            viewFromTop(drive);
            const GuardedScene scene(w);
            select(w, drive, {Vec2(2, 0), Vec2(2, 10)});
            ASSERT_EQ(drive.viewport().selectionManager().selectedIds().size(), 2u);
            if (guarded) guard(*w.activeDocument()->layerManager().getLayer("Guarded"));

            useOnTheGuardedLine(w, drive, tool);
            EXPECT_EQ(scene.untouched(w), guarded);
        }
    }
}

}  // namespace

// Every tool that changes what is drawn leaves an entity on a locked layer as
// it is, even when it was chosen before the layer was locked.
TEST(DraftingToolsTest, ModifyingToolsLeaveALockedLayerAlone) {
    expectEveryToolLeavesItAlone([](hz::draft::LayerProperties& layer) { layer.locked = true; });
}

// And an entity on a hidden layer: what cannot be seen is not changed.
TEST(DraftingToolsTest, ModifyingToolsLeaveAHiddenLayerAlone) {
    expectEveryToolLeavesItAlone([](hz::draft::LayerProperties& layer) { layer.visible = false; });
}
