// Picking in 3D (Phase 132), through the window: a click on a solid chooses
// the face or edge under it, as the Select tool's clicks in the drawing do,
// and the commands that work on faces and edges take what was chosen.

#include <gtest/gtest.h>

#include <QAction>
#include <QStatusBar>
#include <cmath>
#include <optional>
#include <string>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

using hz::math::Vec3;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::test::ToolDriver;
using hz::ui::MainWindow;
using Pick = hz::ui::ViewportWidget::ModelPick;

namespace {

void trigger(MainWindow& w, const char* name) {
    auto* action = w.findChild<QAction*>(QString::fromLatin1(name));
    ASSERT_NE(action, nullptr) << name;
    action->trigger();
}

void run(MainWindow& w, const char* command, const QString& title, FormAnswers answers) {
    FormFiller filler(title, std::move(answers));
    trigger(w, command);
    EXPECT_TRUE(filler.seen()) << "no \"" << title.toStdString() << "\" form";
}

/// A 10 x 10 x 10 box from the origin, seen from the front, right and above,
/// with the Select tool.
void boxInView(MainWindow& w, ToolDriver& drive) {
    run(w, "action_box", QStringLiteral("Box"),
        FormAnswers()
            .number(QStringLiteral("size0"), 10.0)
            .number(QStringLiteral("size1"), 10.0)
            .number(QStringLiteral("size2"), 10.0));
    drive.viewport().camera().lookAt(Vec3(35, -30, 40), Vec3(5, 5, 5), Vec3(0, 0, 1));
    trigger(w, "tool_select");
}

/// The name of the box's face whose points all have coordinate @p axis equal
/// to @p value (0 x, 1 y, 2 z).
std::string faceWhere(const hz::topo::Solid& solid, int axis, double value) {
    for (const auto& face : solid.faces()) {
        const auto* start = face.outerLoop ? face.outerLoop->halfEdge : nullptr;
        bool all = start != nullptr;
        for (const auto* he = start; he && all;) {
            const Vec3& p = he->origin->point;
            all = std::abs((axis == 0 ? p.x : axis == 1 ? p.y : p.z) - value) < 1e-9;
            he = he->next == start ? nullptr : he->next;
        }
        if (all) return face.topoId.tag();
    }
    return {};
}

/// The name of the box's edge from @p a to @p b, either way round.
std::string edgeBetween(const hz::topo::Solid& solid, const Vec3& a, const Vec3& b) {
    for (const auto& edge : solid.edges()) {
        const auto* he = edge.halfEdge;
        if (!he || !he->origin || !he->next || !he->next->origin) continue;
        const Vec3& p = he->origin->point;
        const Vec3& q = he->next->origin->point;
        if (((p - a).length() < 1e-9 && (q - b).length() < 1e-9) ||
            ((p - b).length() < 1e-9 && (q - a).length() < 1e-9)) {
            return edge.topoId.tag();
        }
    }
    return {};
}

}  // namespace

// A click on the top face chooses it; Shift and a click on the front adds
// that; Shift and the top again takes it away; a click on nothing clears.
TEST(PickingTest, ClicksChooseFaces) {
    MainWindow w;
    ToolDriver drive(w);
    boxInView(w, drive);
    const auto& solid = *w.activeDocument()->solid();
    const std::string top = faceWhere(solid, 2, 10.0);
    const std::string front = faceWhere(solid, 1, 0.0);
    ASSERT_FALSE(top.empty());
    auto& view = drive.viewport();

    drive.clickAt(view.projectToScreen(Vec3(4, 6, 10)));
    ASSERT_EQ(view.modelSelection().size(), 1u);
    EXPECT_EQ(view.modelSelection()[0], (Pick{0, top, false}));

    drive.clickAt(view.projectToScreen(Vec3(4, 0, 6)), Qt::ShiftModifier);
    ASSERT_EQ(view.modelSelection().size(), 2u);
    EXPECT_EQ(view.modelSelection()[1], (Pick{0, front, false}));

    drive.clickAt(view.projectToScreen(Vec3(4, 6, 10)), Qt::ShiftModifier);
    ASSERT_EQ(view.modelSelection().size(), 1u);
    EXPECT_EQ(view.modelSelection()[0].tag, front);

    drive.clickAt(view.projectToScreen(Vec3(60, 60, -40)));
    EXPECT_TRUE(view.modelSelection().empty()) << "a click on nothing";
}

// Near an edge the click takes the edge; a hidden edge is not taken through
// the face in front of it.
TEST(PickingTest, AClickNearAnEdgeTakesItUnlessItIsHidden) {
    MainWindow w;
    ToolDriver drive(w);
    boxInView(w, drive);
    const auto& solid = *w.activeDocument()->solid();
    auto& view = drive.viewport();
    const std::string topFront = edgeBetween(solid, Vec3(0, 0, 10), Vec3(10, 0, 10));
    ASSERT_FALSE(topFront.empty());

    const QPointF onEdge = view.projectToScreen(Vec3(5, 0, 10));
    drive.clickAt(onEdge + QPointF(1.5, 1.0));
    ASSERT_EQ(view.modelSelection().size(), 1u);
    EXPECT_EQ(view.modelSelection()[0], (Pick{0, topFront, true}));

    // The back vertical edge at (0, 10) is behind the box from here.
    const std::string hidden = edgeBetween(solid, Vec3(0, 10, 0), Vec3(0, 10, 10));
    drive.clickAt(view.projectToScreen(Vec3(0, 10, 5)));
    ASSERT_EQ(view.modelSelection().size(), 1u);
    EXPECT_NE(view.modelSelection()[0].tag, hidden);
    EXPECT_FALSE(view.modelSelection()[0].edge) << "the face in front of it";
}

// Moving over the part shows what a click would choose.
TEST(PickingTest, TheCursorShowsWhatItIsOver) {
    MainWindow w;
    ToolDriver drive(w);
    boxInView(w, drive);
    auto& view = drive.viewport();
    drive.moveAt(view.projectToScreen(Vec3(10, 5, 5)));
    ASSERT_TRUE(view.modelHover().has_value());
    EXPECT_EQ(view.modelHover()->tag, faceWhere(*w.activeDocument()->solid(), 0, 10.0));
    EXPECT_TRUE(view.modelSelection().empty()) << "only shown, not chosen";
}

// Shell opens the face clicked: the list comes with it checked.
TEST(PickingTest, ShellOpensTheClickedFace) {
    MainWindow w;
    ToolDriver drive(w);
    boxInView(w, drive);
    drive.clickAt(drive.viewport().projectToScreen(Vec3(5, 5, 10)));
    run(w, "action_shell", QStringLiteral("Shell"),
        FormAnswers().number(QStringLiteral("thickness"), 1.0));
    auto& doc = *w.activeDocument();
    ASSERT_EQ(doc.featureTree().featureCount(), 2u)
        << w.statusBar()->currentMessage().toStdString();
    const auto* shell = dynamic_cast<const hz::doc::ShellFeature*>(doc.featureTree().feature(1));
    ASSERT_NE(shell, nullptr);
    EXPECT_EQ(shell->removedFaceIds().size(), 1u);
}

// Fillet rounds the edges clicked, two of them with Shift.
TEST(PickingTest, FilletRoundsTheClickedEdges) {
    MainWindow w;
    ToolDriver drive(w);
    boxInView(w, drive);
    auto& view = drive.viewport();
    drive.clickAt(view.projectToScreen(Vec3(5, 0, 10)) + QPointF(1.0, 1.0));
    drive.clickAt(view.projectToScreen(Vec3(10, 5, 10)) + QPointF(1.0, 1.0), Qt::ShiftModifier);
    ASSERT_EQ(view.modelSelection().size(), 2u);
    run(w, "action_fillet-3d", QStringLiteral("Fillet"),
        FormAnswers().number(QStringLiteral("size"), 1.0));
    auto& doc = *w.activeDocument();
    ASSERT_EQ(doc.featureTree().featureCount(), 2u)
        << w.statusBar()->currentMessage().toStdString();
    const auto* fillet = dynamic_cast<const hz::doc::FilletFeature*>(doc.featureTree().feature(1));
    ASSERT_NE(fillet, nullptr);
    EXPECT_EQ(fillet->edgeIds().size(), 2u);
    EXPECT_TRUE(view.modelSelection().empty()) << "the model was rebuilt: what was clicked is gone";
}

// A sketch on the clicked face, without asking which.
TEST(PickingTest, ASketchGoesOnTheClickedFace) {
    MainWindow w;
    ToolDriver drive(w);
    boxInView(w, drive);
    drive.clickAt(drive.viewport().projectToScreen(Vec3(10, 5, 5)));
    trigger(w, "action_sketch_face");
    const auto sketch = w.activeDocument()->editedSketch();
    ASSERT_NE(sketch, nullptr);
    EXPECT_NEAR((sketch->plane().origin() - Vec3(10, 5, 5)).length(), 0.0, 1e-9);
    EXPECT_NEAR((sketch->plane().normal() - Vec3(1, 0, 0)).length(), 0.0, 1e-9);
    EXPECT_TRUE(drive.viewport().modelSelection().empty()) << "sketching clears it";
}
