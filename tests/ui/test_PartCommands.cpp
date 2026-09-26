// The modelling commands that had none (Phase 133): Loft, Sweep and datums
// from the Model menu; a feature's definition edited with its angles in
// degrees and its directions chosen; rolling the part back from the tree.

#include <gtest/gtest.h>

#include <QAction>
#include <QStatusBar>
#include <QTreeWidget>
#include <cmath>
#include <string>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/FeatureTreePanel.h"
#include "horizon/ui/MainWindow.h"

using hz::math::Vec2;
using hz::math::Vec3;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::test::ToolDriver;
using hz::ui::MainWindow;

namespace {

void trigger(QObject& owner, const char* name) {
    auto* action = owner.findChild<QAction*>(QString::fromLatin1(name));
    ASSERT_NE(action, nullptr) << name;
    action->trigger();
}

void run(MainWindow& w, const char* command, const QString& title, FormAnswers answers) {
    FormFiller filler(title, std::move(answers));
    trigger(w, command);
    EXPECT_TRUE(filler.seen()) << "no \"" << title.toStdString() << "\" form";
}

double partVolume(const hz::doc::Document& doc) {
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

void rectangle(ToolDriver& drive, MainWindow& w, const Vec2& a, const Vec2& b) {
    trigger(w, "tool_rectangle");
    drive.click(a);
    drive.click(b);
}

/// Select feature row @p row in the feature tree.
void selectFeature(MainWindow& w, int row) {
    auto* tree = w.findChild<hz::ui::FeatureTreePanel*>()->findChild<QTreeWidget*>();
    tree->setCurrentItem(tree->topLevelItem(row));
}

}  // namespace

// A revolve's angle is edited in degrees; it used to be shown and typed in
// radians (a full turn was 6.2832).
TEST(PartCommandsTest, AnAngleIsEditedInDegrees) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    trigger(w, "action_sketch_xy");
    rectangle(drive, w, Vec2(10, 0), Vec2(20, 10));
    run(w, "action_revolve", QStringLiteral("Revolve"),
        FormAnswers().number(QStringLiteral("size"), 360.0));
    ASSERT_EQ(doc.featureTree().featureCount(), 1u)
        << w.statusBar()->currentMessage().toStdString();

    selectFeature(w, 0);
    FormFiller filler(QStringLiteral("Edit Revolve"),
                      FormAnswers().number(QStringLiteral("angle"), 90.0));
    trigger(*w.findChild<hz::ui::FeatureTreePanel*>(), "editFeature");
    ASSERT_TRUE(filler.seen());
    EXPECT_NEAR(filler.shown(QStringLiteral("angle")), 360.0, 1e-9) << "shown in degrees";
    EXPECT_NEAR(doc.featureTree().feature(0)->parameters().at("angle"), hz::math::kPi / 2, 1e-12);
}

// An extrusion's direction is chosen in its edit form: here, reversed.
TEST(PartCommandsTest, AnExtrusionsDirectionIsChosen) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    trigger(w, "action_sketch_xy");
    rectangle(drive, w, Vec2(0, 0), Vec2(4, 5));
    run(w, "action_extrude", QStringLiteral("Extrude"),
        FormAnswers().number(QStringLiteral("size"), 2.0));
    selectFeature(w, 0);
    FormFiller filler(QStringLiteral("Edit Extrude"),
                      FormAnswers().choose(QStringLiteral("direction"), QStringLiteral("-Z")));
    trigger(*w.findChild<hz::ui::FeatureTreePanel*>(), "editFeature");
    ASSERT_TRUE(filler.seen());
    ASSERT_NE(doc.solid(), nullptr);
    double lowest = 0.0;
    for (const auto& v : doc.solid()->vertices()) lowest = std::min(lowest, v.point.z);
    EXPECT_NEAR(lowest, -2.0, 1e-9);
    EXPECT_NEAR(partVolume(doc), 40.0, 1e-9);
}

// A datum plane 10 above XY, a sketch on it, and a loft from a square on XY
// to the smaller square on the datum: a frustum.
TEST(PartCommandsTest, ALoftJoinsSketchesOnADatumPlane) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    run(w, "action_datum_plane", QStringLiteral("Datum Plane"),
        FormAnswers().number(QStringLiteral("offset"), 10.0));
    ASSERT_EQ(doc.featureTree().featureCount(), 1u)
        << w.statusBar()->currentMessage().toStdString();

    trigger(w, "action_sketch_xy");
    rectangle(drive, w, Vec2(-2, -2), Vec2(2, 2));
    trigger(w, "action_sketch_finish");
    run(w, "action_sketch_datum", QStringLiteral("Sketch on a Datum Plane"), FormAnswers());
    ASSERT_NE(doc.editedSketch(), nullptr);
    EXPECT_NEAR(doc.editedSketch()->plane().origin().z, 10.0, 1e-9);
    rectangle(drive, w, Vec2(-1, -1), Vec2(1, 1));
    trigger(w, "action_sketch_finish");

    run(w, "action_loft", QStringLiteral("Loft"),
        FormAnswers().check(QStringLiteral("sections"), {QStringLiteral("Sketch")}));
    ASSERT_EQ(doc.featureTree().featureCount(), 2u)
        << w.statusBar()->currentMessage().toStdString();
    // A frustum of square sections 4 and 2 wide, 10 high: h/3 (A1 + A2 + sqrt(A1 A2)).
    EXPECT_NEAR(partVolume(doc), 10.0 / 3.0 * (16.0 + 4.0 + 8.0), 1e-6);
}

// A square on XZ swept along a line on XY: a bar.
TEST(PartCommandsTest, ASweepCarriesAProfileAlongAPath) {
    MainWindow w;
    ToolDriver drive(w);
    auto& doc = *w.activeDocument();
    trigger(w, "action_sketch_xz");
    rectangle(drive, w, Vec2(-1, 0), Vec2(1, 2));
    trigger(w, "action_sketch_finish");
    trigger(w, "action_sketch_xy");
    trigger(w, "tool_line");
    drive.click(Vec2(0, 0));
    drive.click(Vec2(0, 10));
    drive.key(Qt::Key_Return);
    trigger(w, "action_sketch_finish");

    run(w, "action_sweep", QStringLiteral("Sweep"),
        FormAnswers()
            .choose(QStringLiteral("profile"), QStringLiteral("Sketch 1"))
            .choose(QStringLiteral("path"), QStringLiteral("Sketch 2")));
    ASSERT_EQ(doc.featureTree().featureCount(), 1u)
        << w.statusBar()->currentMessage().toStdString();
    EXPECT_NEAR(partVolume(doc), 2.0 * 2.0 * 10.0, 1e-6);
}

// Rolled back from the tree's menu, the later feature is left out, and undo
// brings it back.
TEST(PartCommandsTest, ThePartIsRolledBackFromTheTree) {
    MainWindow w;
    auto& doc = *w.activeDocument();
    run(w, "action_box", QStringLiteral("Box"),
        FormAnswers()
            .number(QStringLiteral("size0"), 10.0)
            .number(QStringLiteral("size1"), 10.0)
            .number(QStringLiteral("size2"), 10.0));
    run(w, "action_shell", QStringLiteral("Shell"),
        FormAnswers()
            .number(QStringLiteral("thickness"), 1.0)
            .check(QStringLiteral("faces"), {QStringLiteral("facing (0, 0, 1)")}));
    ASSERT_EQ(doc.featureTree().featureCount(), 2u);
    ASSERT_NEAR(partVolume(doc), 1000.0 - 8.0 * 8.0 * 9.0, 1e-6);

    selectFeature(w, 0);
    trigger(*w.findChild<hz::ui::FeatureTreePanel*>(), "rollbackHere");
    EXPECT_EQ(doc.featureTree().rollbackIndex(), 0);
    EXPECT_NEAR(partVolume(doc), 1000.0, 1e-6) << "the shell left out";

    trigger(w, "action_undo");
    EXPECT_EQ(doc.featureTree().rollbackIndex(), -1);
    EXPECT_NEAR(partVolume(doc), 1000.0 - 8.0 * 8.0 * 9.0, 1e-6);
}

// -- Extrude and pattern options (Phase 134) ----------------------------------

// A hole cut through all, reversed down into a box from a sketch on its top,
// then repeated by a pattern of that cut alone.
TEST(PartCommandsTest, AHoleThroughAllIsRepeatedByAPattern) {
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
    // The face's middle is the sketch's origin: a 2 x 2 square near a corner.
    rectangle(drive, w, Vec2(-4, -4), Vec2(-2, -2));
    run(w, "action_extrude", QStringLiteral("Extrude"),
        FormAnswers()
            .number(QStringLiteral("size"), 1.0)
            .choose(QStringLiteral("extent"), QStringLiteral("Through all"))
            .choose(QStringLiteral("way"), QStringLiteral("Reversed"))
            .combine(hz::doc::BodyOperation::Cut));
    ASSERT_EQ(doc.featureTree().featureCount(), 2u)
        << w.statusBar()->currentMessage().toStdString();
    EXPECT_NEAR(partVolume(doc), 1000.0 - 40.0, 1e-6) << "through the whole box, not 1 deep";

    run(w, "action_pattern-linear", QStringLiteral("Linear Pattern"),
        FormAnswers()
            .choose(QStringLiteral("direction"), QStringLiteral("+X"))
            .number(QStringLiteral("spacing"), 3.0)
            .number(QStringLiteral("count"), 3)
            .check(QStringLiteral("features"), {QStringLiteral("Extrude")}));
    ASSERT_EQ(doc.featureTree().featureCount(), 3u)
        << w.statusBar()->currentMessage().toStdString();
    EXPECT_NEAR(partVolume(doc), 1000.0 - 3 * 40.0, 1e-6) << "three holes, one box";
}

// A cylinder stood on its side at a point, from the Cylinder form.
TEST(PartCommandsTest, APrimitiveIsPlacedFromItsForm) {
    MainWindow w;
    auto& doc = *w.activeDocument();
    run(w, "action_cylinder", QStringLiteral("Cylinder"),
        FormAnswers()
            .number(QStringLiteral("size0"), 1.0)
            .number(QStringLiteral("size1"), 6.0)
            .number(QStringLiteral("atX"), 5.0)
            .choose(QStringLiteral("axis"), QStringLiteral("+Y")));
    ASSERT_NE(doc.solid(), nullptr) << w.statusBar()->currentMessage().toStdString();
    double hiY = -1e9;
    double loX = 1e9;
    for (const auto& v : doc.solid()->vertices()) {
        hiY = std::max(hiY, v.point.y);
        loX = std::min(loX, v.point.x);
    }
    EXPECT_NEAR(hiY, 6.0, 1e-9) << "its length along y";
    EXPECT_NEAR(loX, 4.0, 1e-9) << "its axis through x = 5";
}

// Phase 162: Model ▸ Mirror, in the box's face facing +X, makes one solid
// twice its size, hole and all, as one undo step. (A feature mirrored alone
// is checked in the document's tests.)
TEST(PartCommandsTest, AMirrorInAFaceDoublesThePart) {
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
    rectangle(drive, w, Vec2(-4, -4), Vec2(-2, -2));
    run(w, "action_extrude", QStringLiteral("Extrude"),
        FormAnswers()
            .number(QStringLiteral("size"), 1.0)
            .choose(QStringLiteral("extent"), QStringLiteral("Through all"))
            .choose(QStringLiteral("way"), QStringLiteral("Reversed"))
            .combine(hz::doc::BodyOperation::Cut));
    ASSERT_NEAR(partVolume(doc), 1000.0 - 40.0, 1e-6);

    // The box's side at x = 10, not the hole's wall at x = 1, which faces +X
    // too.
    run(w, "action_mirror-3d", QStringLiteral("Mirror"),
        FormAnswers().chooseContaining(QStringLiteral("plane"),
                                       QStringLiteral("facing (1, 0, 0) at (10,")));
    ASSERT_EQ(doc.featureTree().featureCount(), 3u)
        << w.statusBar()->currentMessage().toStdString();
    EXPECT_NEAR(partVolume(doc), 2 * (1000.0 - 40.0), 1e-6) << "the part and its image";
    ASSERT_NE(doc.solid(), nullptr);
    EXPECT_EQ(doc.solid()->shellCount(), 1u) << "joined at the face";

    trigger(w, "action_undo");
    EXPECT_EQ(doc.featureTree().featureCount(), 2u);
    EXPECT_NEAR(partVolume(doc), 1000.0 - 40.0, 1e-6);
}
