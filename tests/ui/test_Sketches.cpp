// Sketches on planes (Phase 131), through the window: a sketch is made on a
// principal plane or a face, drawn into with the ordinary tools in its own
// coordinates, finished, and extruded or revolved, holes and all.

#include <gtest/gtest.h>

#include <QAction>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QListWidget>
#include <QStatusBar>
#include <QTreeWidget>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/FeatureTreePanel.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

using hz::math::Vec2;
using hz::math::Vec3;
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

/// Run @p command, answering its form titled @p title.
void run(MainWindow& w, const char* command, const QString& title, FormAnswers answers) {
    FormFiller filler(title, std::move(answers));
    trigger(w, command);
    EXPECT_TRUE(filler.seen()) << "no \"" << title.toStdString() << "\" form";
}

double partVolume(const hz::doc::Document& doc) {
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

struct Box {
    Vec3 lo{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max()};
    Vec3 hi{-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(),
            -std::numeric_limits<double>::max()};
};

Box boundsOf(const hz::topo::Solid& solid) {
    Box b;
    for (const auto& v : solid.vertices()) {
        b.lo = Vec3(std::min(b.lo.x, v.point.x), std::min(b.lo.y, v.point.y),
                    std::min(b.lo.z, v.point.z));
        b.hi = Vec3(std::max(b.hi.x, v.point.x), std::max(b.hi.y, v.point.y),
                    std::max(b.hi.z, v.point.z));
    }
    return b;
}

bool near(const Vec3& a, const Vec3& b) {
    return (a - b).length() < 1e-6;
}

/// A circle of @p r as the kernel facets it: a polygon of @p n sides.
double facetedCircleArea(double r, int n = 32) {
    return 0.5 * n * r * r * std::sin(2.0 * hz::math::kPi / n);
}

void rectangle(ToolDriver& drive, MainWindow& w, const Vec2& a, const Vec2& b) {
    trigger(w, "tool_rectangle");
    drive.click(a);
    drive.click(b);
}

void circle(ToolDriver& drive, MainWindow& w, const Vec2& centre, double r) {
    trigger(w, "tool_circle");
    drive.click(centre);
    drive.click(centre + Vec2(r, 0));
}

}  // namespace

// A sketch on XZ: drawn in its own coordinates (the view works in them), kept
// out of the drawing, and extruded along its normal, out of the front.
TEST(SketchesTest, ASketchOnXZIsDrawnInItsOwnPlaneAndExtruded) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    trigger(w, "action_sketch_xz");
    const auto sketch = doc.editedSketch();
    ASSERT_NE(sketch, nullptr);
    EXPECT_EQ(drive.viewport().activeSketch(), sketch.get());
    EXPECT_TRUE(w.findChild<QAction*>(QStringLiteral("action_sketch_finish"))->isEnabled());

    rectangle(drive, w, Vec2(0, 0), Vec2(10, 5));
    EXPECT_EQ(sketch->entities().size(), 1u) << "drawn into the sketch";
    EXPECT_TRUE(doc.draftDocument().entities().empty()) << "not into the drawing";

    trigger(w, "action_sketch_finish");
    EXPECT_EQ(doc.editedSketch(), nullptr);
    EXPECT_EQ(drive.viewport().activeSketch(), nullptr);

    run(w, "action_extrude", QStringLiteral("Extrude"),
        FormAnswers().number(QStringLiteral("size"), 4.0));
    ASSERT_NE(doc.solid(), nullptr) << w.statusBar()->currentMessage().toStdString();
    EXPECT_NEAR(partVolume(doc), 200.0, 1e-9);
    const Box b = boundsOf(*doc.solid());
    EXPECT_TRUE(near(b.lo, Vec3(0, -4, 0))) << "the sketch's x is the world's x, its y the z";
    EXPECT_TRUE(near(b.hi, Vec3(10, 0, 5)));
}

// A plate with a hole: both drawn in one sketch, extruded as one.
TEST(SketchesTest, APlateWithAHoleIsExtrudedFromItsSketch) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    trigger(w, "action_sketch_xy");
    rectangle(drive, w, Vec2(0, 0), Vec2(20, 10));
    circle(drive, w, Vec2(5, 5), 2.0);
    // A note on the sketch is not part of the profile.
    doc.editedSketch()->addEntity(std::make_shared<hz::draft::DraftText>(Vec2(12, 5), "A", 1.0));

    run(w, "action_extrude", QStringLiteral("Extrude"),
        FormAnswers().number(QStringLiteral("size"), 3.0));
    ASSERT_NE(doc.solid(), nullptr) << w.statusBar()->currentMessage().toStdString();
    EXPECT_EQ(doc.editedSketch(), nullptr) << "Extrude finishes the sketch it takes";
    EXPECT_NEAR(partVolume(doc), (200.0 - facetedCircleArea(2.0)) * 3.0, 1e-6);
}

// The drawing itself, with a note on it: the note no longer stops Extrude.
TEST(SketchesTest, ANoteInTheDrawingDoesNotStopExtrude) {
    MainWindow w;
    auto& doc = *w.activeDocument();
    doc.draftDocument().addEntity(
        std::make_shared<hz::draft::DraftRectangle>(Vec2(0, 0), Vec2(4, 5)));
    doc.draftDocument().addEntity(std::make_shared<hz::draft::DraftText>(Vec2(1, 1), "PLATE", 0.5));
    run(w, "action_extrude", QStringLiteral("Extrude"),
        FormAnswers().number(QStringLiteral("size"), 2.0));
    EXPECT_NEAR(partVolume(doc), 40.0, 1e-9) << w.statusBar()->currentMessage().toStdString();
}

// A sketch on the top face of a box: its origin is the face's middle, and a
// boss drawn there stands on the face.
TEST(SketchesTest, ASketchOnAFaceStandsOnIt) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    run(w, "action_box", QStringLiteral("Box"),
        FormAnswers()
            .number(QStringLiteral("size0"), 10.0)
            .number(QStringLiteral("size1"), 10.0)
            .number(QStringLiteral("size2"), 10.0));
    run(w, "action_sketch_face", QStringLiteral("Sketch on a Face"),
        FormAnswers().chooseContaining(QStringLiteral("face"), QStringLiteral("facing (0, 0, 1)")));
    const auto sketch = doc.editedSketch();
    ASSERT_NE(sketch, nullptr);
    EXPECT_TRUE(near(sketch->plane().origin(), Vec3(5, 5, 10)))
        << sketch->plane().origin().x << "," << sketch->plane().origin().y << ","
        << sketch->plane().origin().z;
    EXPECT_TRUE(near(sketch->plane().normal(), Vec3(0, 0, 1)));

    circle(drive, w, Vec2(0, 0), 2.0);
    run(w, "action_extrude", QStringLiteral("Extrude"),
        FormAnswers().number(QStringLiteral("size"), 3.0).combine(hz::doc::BodyOperation::Join));
    EXPECT_NEAR(partVolume(doc), 1000.0 + facetedCircleArea(2.0) * 3.0, 1e-6)
        << w.statusBar()->currentMessage().toStdString();
    EXPECT_NEAR(boundsOf(*doc.solid()).hi.z, 13.0, 1e-9);
}

// Made taller, the box carries the sketch on its top face up with it, and
// the boss drawn there with the sketch (Phase 157); undo brings both back.
TEST(SketchesTest, ASketchOnAFaceFollowsItWhenThePartChanges) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    run(w, "action_box", QStringLiteral("Box"),
        FormAnswers()
            .number(QStringLiteral("size0"), 10.0)
            .number(QStringLiteral("size1"), 10.0)
            .number(QStringLiteral("size2"), 10.0));
    run(w, "action_sketch_face", QStringLiteral("Sketch on a Face"),
        FormAnswers().chooseContaining(QStringLiteral("face"), QStringLiteral("facing (0, 0, 1)")));
    const auto sketch = doc.editedSketch();
    ASSERT_NE(sketch, nullptr);
    const std::string top = doc.featureTree().feature(0)->featureID() + "/top";
    EXPECT_EQ(sketch->face(), top) << "the face it is on, by name";
    circle(drive, w, Vec2(0, 0), 2.0);
    run(w, "action_extrude", QStringLiteral("Extrude"),
        FormAnswers().number(QStringLiteral("size"), 3.0).combine(hz::doc::BodyOperation::Join));
    ASSERT_NEAR(boundsOf(*doc.solid()).hi.z, 13.0, 1e-9);

    auto* list = w.findChild<QListWidget*>(QStringLiteral("sketchList"));
    ASSERT_NE(list, nullptr);
    ASSERT_EQ(list->count(), 1);
    EXPECT_TRUE(list->item(0)->text().contains(QStringLiteral("on a face")))
        << list->item(0)->text().toStdString();

    {
        FormFiller edit(QStringLiteral("Edit Box"),
                        FormAnswers().number(QStringLiteral("depth"), 20.0));
        auto* panel = w.findChild<hz::ui::FeatureTreePanel*>();
        ASSERT_NE(panel, nullptr);
        auto* tree = panel->findChild<QTreeWidget*>();
        ASSERT_NE(tree, nullptr);
        tree->setCurrentItem(tree->topLevelItem(0));
        trigger(w, "editFeature");
        ASSERT_TRUE(edit.seen());
    }
    EXPECT_NEAR(boundsOf(*doc.solid()).hi.z, 23.0, 1e-9)
        << "the boss on the taller box: " << w.statusBar()->currentMessage().toStdString();
    EXPECT_TRUE(near(sketch->plane().origin(), Vec3(5, 5, 20)));

    trigger(w, "action_undo");
    EXPECT_NEAR(boundsOf(*doc.solid()).hi.z, 13.0, 1e-9);
    EXPECT_TRUE(near(sketch->plane().origin(), Vec3(5, 5, 10)));
}

// Built on a worker, from a copy: the part's own sketch is placed where the
// copy's was, so it is drawn, and edited, on the face where it now is.
TEST(SketchesTest, ASketchFollowsItsFaceWhenThePartIsBuiltOnAWorker) {
    MainWindow w;
    w.setRebuildMode(MainWindow::RebuildMode::Always);
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    const auto settle = [&w] {
        QElapsedTimer waited;
        waited.start();
        while (w.backgroundWorkRunning() && waited.elapsed() < 30'000) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        }
    };
    run(w, "action_box", QStringLiteral("Box"),
        FormAnswers()
            .number(QStringLiteral("size0"), 10.0)
            .number(QStringLiteral("size1"), 10.0)
            .number(QStringLiteral("size2"), 10.0));
    settle();
    run(w, "action_sketch_face", QStringLiteral("Sketch on a Face"),
        FormAnswers().chooseContaining(QStringLiteral("face"), QStringLiteral("facing (0, 0, 1)")));
    const auto sketch = doc.editedSketch();
    ASSERT_NE(sketch, nullptr);
    circle(drive, w, Vec2(0, 0), 2.0);
    run(w, "action_extrude", QStringLiteral("Extrude"),
        FormAnswers().number(QStringLiteral("size"), 3.0).combine(hz::doc::BodyOperation::Join));
    settle();
    {
        FormFiller edit(QStringLiteral("Edit Box"),
                        FormAnswers().number(QStringLiteral("depth"), 20.0));
        auto* tree = w.findChild<hz::ui::FeatureTreePanel*>()->findChild<QTreeWidget*>();
        tree->setCurrentItem(tree->topLevelItem(0));
        trigger(w, "editFeature");
        ASSERT_TRUE(edit.seen());
    }
    settle();
    EXPECT_FALSE(doc.needsBuild());
    EXPECT_NEAR(boundsOf(*doc.solid()).hi.z, 23.0, 1e-9);
    EXPECT_TRUE(near(sketch->plane().origin(), Vec3(5, 5, 20)))
        << "the part's own sketch, not only the worker's copy: z = " << sketch->plane().origin().z;
}

// Phase 157b: an edge of the part projected into a sketch, as construction
// geometry, is drawn again where the edge is when the part changes; the
// Construction command makes it part of the profile, and back.
TEST(SketchesTest, AnEdgeProjectedIntoASketchFollowsThePart) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    run(w, "action_box", QStringLiteral("Box"),
        FormAnswers()
            .number(QStringLiteral("size0"), 10.0)
            .number(QStringLiteral("size1"), 10.0)
            .number(QStringLiteral("size2"), 10.0));
    run(w, "action_sketch_face", QStringLiteral("Sketch on a Face"),
        FormAnswers().chooseContaining(QStringLiteral("face"), QStringLiteral("facing (0, 0, 1)")));
    const auto sketch = doc.editedSketch();
    ASSERT_NE(sketch, nullptr);
    // The top's edge at x = 10, listed from either end.
    run(w, "action_project_edges", QStringLiteral("Project Edges"),
        FormAnswers()
            .check(QStringLiteral("edges"), {QStringLiteral("(10, 0, 10) – (10, 10, 10)"),
                                             QStringLiteral("(10, 10, 10) – (10, 0, 10)")})
            .choose(QStringLiteral("kind"), QStringLiteral("Construction geometry")));
    ASSERT_EQ(sketch->entities().size(), 1u) << w.statusBar()->currentMessage().toStdString();
    const auto projected = sketch->entities().front();
    EXPECT_TRUE(projected->construction());
    EXPECT_FALSE(projected->sourceEdge().empty());
    const auto* line = dynamic_cast<const hz::draft::DraftLine*>(projected.get());
    ASSERT_NE(line, nullptr);
    EXPECT_NEAR(line->start().x, 5.0, 1e-9) << "x = 10 is 5 from the face's middle";
    const uint64_t id = projected->id();

    // Construction, and back: each one undo step.
    drive.viewport().selectionManager().clearSelection();
    drive.viewport().selectionManager().select(id);
    trigger(w, "action_construction");
    EXPECT_FALSE(sketch->drawing().sharedEntity(id)->construction());
    trigger(w, "action_undo");
    EXPECT_TRUE(sketch->drawing().sharedEntity(id)->construction());
    drive.viewport().selectionManager().clearSelection();

    circle(drive, w, Vec2(0, 0), 2.0);
    run(w, "action_extrude", QStringLiteral("Extrude"),
        FormAnswers().number(QStringLiteral("size"), 3.0).combine(hz::doc::BodyOperation::Join));
    ASSERT_NEAR(boundsOf(*doc.solid()).hi.z, 13.0, 1e-9)
        << "the guide is not part of the profile: "
        << w.statusBar()->currentMessage().toStdString();
    {
        FormFiller edit(QStringLiteral("Edit Box"),
                        FormAnswers().number(QStringLiteral("width"), 30.0));
        auto* tree = w.findChild<hz::ui::FeatureTreePanel*>()->findChild<QTreeWidget*>();
        tree->setCurrentItem(tree->topLevelItem(0));
        trigger(w, "editFeature");
        ASSERT_TRUE(edit.seen());
    }
    const auto now = sketch->drawing().sharedEntity(id);
    const auto* moved = dynamic_cast<const hz::draft::DraftLine*>(now.get());
    ASSERT_NE(moved, nullptr);
    EXPECT_NEAR(moved->start().x, 25.0, 1e-9) << "the edge at x = 30 now";
}

// Undoing the new sketch takes it away, and the window leaves it.
TEST(SketchesTest, UndoingANewSketchLeavesIt) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    trigger(w, "action_sketch_xy");
    ASSERT_NE(doc.editedSketch(), nullptr);
    trigger(w, "action_undo");
    EXPECT_TRUE(doc.sketches().empty());
    EXPECT_EQ(doc.editedSketch(), nullptr);
    EXPECT_EQ(drive.viewport().activeSketch(), nullptr);
    EXPECT_FALSE(w.findChild<QAction*>(QStringLiteral("action_sketch_finish"))->isEnabled());
}

// A line begun in a sketch is not finished in the drawing when undo takes the
// sketch away: its first point was in the sketch's frame. The tool started
// again only for Edit and Finish Sketch, so the next click made a line from a
// point in one frame to a point in the other.
TEST(SketchesTest, UndoingASketchMidLineStartsTheLineAgain) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    trigger(w, "action_sketch_xz");
    trigger(w, "tool_line");
    drive.click(Vec2(3, 4));
    trigger(w, "action_undo");
    ASSERT_EQ(doc.editedSketch(), nullptr);
    drive.click(Vec2(10, 10));
    EXPECT_TRUE(doc.draftDocument().entities().empty()) << "that click begins a line";
    drive.click(Vec2(12, 10));
    ASSERT_EQ(doc.draftDocument().entities().size(), 1u);
    const auto* line =
        dynamic_cast<const hz::draft::DraftLine*>(doc.draftDocument().entities()[0].get());
    ASSERT_NE(line, nullptr);
    EXPECT_NEAR(line->start().x, 10.0, 1e-9);
    EXPECT_NEAR(line->start().y, 10.0, 1e-9);
}

// A finished sketch is edited again from the sketch list, where what is drawn
// goes into it, and undo takes it back out of it.
TEST(SketchesTest, ASketchIsEditedAgainFromTheList) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    trigger(w, "action_sketch_xy");
    rectangle(drive, w, Vec2(0, 0), Vec2(4, 4));
    trigger(w, "action_sketch_finish");
    const auto sketch = doc.sketches().front();

    auto* list = w.findChild<QListWidget*>(QStringLiteral("sketchList"));
    ASSERT_NE(list, nullptr);
    ASSERT_EQ(list->count(), 1);
    EXPECT_TRUE(list->item(0)->text().startsWith("Sketch 1"));
    emit list->itemDoubleClicked(list->item(0));
    EXPECT_EQ(doc.editedSketch(), sketch);
    EXPECT_TRUE(list->item(0)->text().contains("editing"));

    circle(drive, w, Vec2(10, 10), 1.0);
    EXPECT_EQ(sketch->entities().size(), 2u);
    trigger(w, "action_undo");
    EXPECT_EQ(sketch->entities().size(), 1u) << "undone in the sketch";
    EXPECT_EQ(doc.editedSketch(), sketch) << "still editing it";
}

// A section on XZ, revolved about the sketch's own vertical axis (the world's
// z): a ring, by Pappus.
TEST(SketchesTest, ASketchIsRevolvedAboutItsOwnAxis) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    trigger(w, "action_sketch_xz");
    rectangle(drive, w, Vec2(10, 0), Vec2(20, 10));
    run(w, "action_revolve", QStringLiteral("Revolve"),
        FormAnswers().number(QStringLiteral("size"), 360.0));
    ASSERT_NE(doc.solid(), nullptr) << w.statusBar()->currentMessage().toStdString();
    const double pappus = 2.0 * hz::math::kPi * 15.0 * 100.0;
    EXPECT_NEAR(partVolume(doc), pappus, pappus * 0.01);
    const Box b = boundsOf(*doc.solid());
    EXPECT_NEAR(b.hi.z, 10.0, 1e-9) << "turned about z, standing 10 high";
}
