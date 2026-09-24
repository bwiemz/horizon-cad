// The 3D ribbon commands add features to the part — each one undoable,
// saved with the document and rebuilt with it — where most of them used to
// drop a fixed demo mesh into the scene outside the document.

#include <gtest/gtest.h>

#include <QAction>
#include <QStatusBar>
#include <QTreeWidget>
#include <cmath>
#include <memory>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/FeatureTreePanel.h"
#include "horizon/ui/MainWindow.h"

using hz::doc::BodyOperation;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::ui::MainWindow;

namespace {

QAction* action(MainWindow& w, const char* name) {
    auto* found = w.findChild<QAction*>(QString::fromLatin1(name));
    EXPECT_NE(found, nullptr) << name;
    return found;
}

/// Trigger a ribbon command whose form is titled `title`, answering it.
void run(MainWindow& w, const char* command, const QString& title, FormAnswers answers) {
    FormFiller filler(title, std::move(answers));
    action(w, command)->trigger();
    EXPECT_TRUE(filler.seen()) << "no \"" << title.toStdString() << "\" form";
}

double partVolume(const hz::doc::Document& doc) {
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

size_t bodies(const hz::doc::Document& doc) {
    return doc.solid() ? doc.solid()->shellCount() : 0;
}

/// A 10 x 10 x 10 box made with the Box command.
void box(MainWindow& w, double x = 10, double y = 10, double z = 10,
         std::optional<BodyOperation> operation = std::nullopt) {
    run(w, "action_box", QStringLiteral("Box"),
        FormAnswers()
            .number(QStringLiteral("size0"), x)
            .number(QStringLiteral("size1"), y)
            .number(QStringLiteral("size2"), z)
            .combine(operation));
}

}  // namespace

TEST(SolidCommandsTest, APrimitiveIsAFeatureOfThePart) {
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    {
        FormFiller filler(QStringLiteral("Box"), FormAnswers()
                                                     .number(QStringLiteral("size0"), 4.0)
                                                     .number(QStringLiteral("size1"), 5.0)
                                                     .number(QStringLiteral("size2"), 6.0));
        action(w, "action_box")->trigger();
        ASSERT_TRUE(filler.seen());
        EXPECT_EQ(filler.proposed(), BodyOperation::NewBody) << "the first body starts one";
    }
    ASSERT_EQ(doc.featureTree().featureCount(), 1u);
    EXPECT_NEAR(partVolume(doc), 120.0, 1e-9);
    EXPECT_TRUE(doc.isDirty());

    action(w, "action_undo")->trigger();
    EXPECT_EQ(doc.featureTree().featureCount(), 0u);
    EXPECT_EQ(doc.solid(), nullptr) << "no demo mesh outside the document";
}

TEST(SolidCommandsTest, PrimitivesJoinOrCutThePart) {
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    box(w);
    // A cylinder of radius 2 cut through it: the inscribed prism of the facet
    // count the feature uses.
    run(w, "action_cylinder", QStringLiteral("Cylinder"),
        FormAnswers()
            .number(QStringLiteral("size0"), 2.0)
            .number(QStringLiteral("size1"), 10.0)
            .combine(BodyOperation::Cut));
    ASSERT_EQ(doc.featureTree().featureCount(), 2u);
    const auto* cylinder = doc.featureTree().feature(1);
    const int n = static_cast<int>(cylinder->parameters().at("segments"));
    // Centred on the origin, so a quarter of it lies inside the box.
    const double prism = 0.5 * n * 4.0 * std::sin(2.0 * std::acos(-1.0) / n) * 10.0;
    EXPECT_NEAR(partVolume(doc), 1000.0 - prism / 4.0, 1e-6);

    // Sphere, cone and torus forms exist and add features.
    run(w, "action_sphere", QStringLiteral("Sphere"),
        FormAnswers().number(QStringLiteral("size0"), 1.0).combine(BodyOperation::NewBody));
    run(w, "action_cone", QStringLiteral("Cone"), FormAnswers().combine(BodyOperation::NewBody));
    run(w, "action_torus", QStringLiteral("Torus"), FormAnswers().combine(BodyOperation::NewBody));
    EXPECT_EQ(doc.featureTree().featureCount(), 5u);
    EXPECT_EQ(doc.failedFeatureIndex(), -1) << doc.lastBuildMessage();
}

TEST(SolidCommandsTest, CombineJoinsTheBodies) {
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    box(w);

    // One body: nothing to combine, so nothing is added.
    action(w, "action_boolean-union")->trigger();
    EXPECT_EQ(doc.featureTree().featureCount(), 1u);
    EXPECT_TRUE(w.statusBar()->currentMessage().contains("fewer than two"))
        << w.statusBar()->currentMessage().toStdString();

    // A second box as its own body, overlapping the first by half.
    box(w, 5, 10, 10, BodyOperation::NewBody);
    ASSERT_EQ(bodies(doc), 2u);
    action(w, "action_boolean-subtract")->trigger();
    EXPECT_EQ(doc.featureTree().featureCount(), 3u);
    EXPECT_EQ(bodies(doc), 1u);
    EXPECT_NEAR(partVolume(doc), 500.0, 1e-6) << "the first body minus the second";
}

TEST(SolidCommandsTest, FilletAndChamferTheChosenEdges) {
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    box(w);
    {
        // The box's vertical edge at the origin.
        FormFiller filler(
            QStringLiteral("Fillet"),
            FormAnswers()
                .number(QStringLiteral("size"), 2.0)
                .check(QStringLiteral("edges"), {QStringLiteral("(0, 0, 0) – (0, 0, 10)"),
                                                 QStringLiteral("(0, 0, 10) – (0, 0, 0)")}));
        action(w, "action_fillet-3d")->trigger();
        ASSERT_TRUE(filler.seen());
        EXPECT_EQ(filler.offered(QStringLiteral("edges")).size(), 12)
            << "a box has twelve edges to choose from";
    }
    ASSERT_EQ(doc.featureTree().featureCount(), 2u)
        << w.statusBar()->currentMessage().toStdString();
    const double filleted = partVolume(doc);
    // The corner loses (1 - pi/4) r^2 per unit length. Faceted, a little more:
    // the chords run inside the arc.
    const double exact = 1000.0 - (1.0 - std::acos(-1.0) / 4.0) * 4.0 * 10.0;
    EXPECT_LT(filleted, exact);
    EXPECT_GT(filleted, exact - 0.5);

    // Nothing chosen: not added.
    run(w, "action_chamfer-3d", QStringLiteral("Chamfer"), FormAnswers());
    EXPECT_EQ(doc.featureTree().featureCount(), 2u);
    EXPECT_TRUE(w.statusBar()->currentMessage().contains("no edges"))
        << w.statusBar()->currentMessage().toStdString();

    // A chamfer on the opposite vertical edge takes a right triangle of side 1.
    run(w, "action_chamfer-3d", QStringLiteral("Chamfer"),
        FormAnswers()
            .number(QStringLiteral("size"), 1.0)
            .check(QStringLiteral("edges"), {QStringLiteral("(10, 10, 0) – (10, 10, 10)"),
                                             QStringLiteral("(10, 10, 10) – (10, 10, 0)")}));
    ASSERT_EQ(doc.featureTree().featureCount(), 3u)
        << w.statusBar()->currentMessage().toStdString();
    EXPECT_NEAR(partVolume(doc), filleted - 0.5 * 10.0, 1e-6);
}

TEST(SolidCommandsTest, ShellOpensTheChosenFace) {
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    box(w);
    run(w, "action_shell", QStringLiteral("Shell"),
        FormAnswers()
            .number(QStringLiteral("thickness"), 1.0)
            .check(QStringLiteral("faces"), {QStringLiteral("facing (0, 0, 1)")}));
    ASSERT_EQ(doc.featureTree().featureCount(), 2u)
        << w.statusBar()->currentMessage().toStdString();
    // An open-topped cup: the 8 x 8 x 9 inside is hollow.
    EXPECT_NEAR(partVolume(doc), 1000.0 - 8.0 * 8.0 * 9.0, 1e-6);
    // Open at the top: the inner rim is at z = 10. The list named each face
    // by the way its loop wound, and a box's loops wind inwards, so "facing
    // (0, 0, 1)" was the bottom (the same volume, open underneath).
    bool rimAtTop = false;
    for (const auto& v : doc.solid()->vertices()) {
        if (std::abs(v.point.x - 1.0) < 1e-9 && std::abs(v.point.y - 1.0) < 1e-9 &&
            std::abs(v.point.z - 10.0) < 1e-9) {
            rimAtTop = true;
        }
    }
    EXPECT_TRUE(rimAtTop) << "the face that faces up is the one opened";
}

TEST(SolidCommandsTest, DraftTapersTheSides) {
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    box(w);
    run(w, "action_draft", QStringLiteral("Draft"),
        FormAnswers().number(QStringLiteral("angle"), 5.0));
    ASSERT_EQ(doc.featureTree().featureCount(), 2u);
    EXPECT_EQ(doc.failedFeatureIndex(), -1) << doc.lastBuildMessage();
    // Each side leans 5 degrees out from the neutral plane at the base, so the
    // part widens toward the pull — as it must to leave a cavity it is pulled
    // out of: the integral of (10 + 2 z tan 5deg)^2 over the height.
    const double t = std::tan(5.0 * std::acos(-1.0) / 180.0);
    EXPECT_NEAR(partVolume(doc), (std::pow(10.0 + 20.0 * t, 3) - 1000.0) / (6.0 * t), 1e-6);
}

TEST(SolidCommandsTest, PatternsRepeatThePart) {
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    box(w);
    run(w, "action_pattern-linear", QStringLiteral("Linear Pattern"),
        FormAnswers()
            .number(QStringLiteral("spacing"), 20.0)
            .number(QStringLiteral("count"), 3)
            .choose(QStringLiteral("direction"), QStringLiteral("+Y")));
    EXPECT_EQ(bodies(doc), 3u);
    EXPECT_NEAR(partVolume(doc), 3000.0, 1e-6);
    action(w, "action_undo")->trigger();

    // Four quarter turns about Z: the copies meet at the axis and merge into
    // one 20 x 20 block.
    run(w, "action_pattern-circular", QStringLiteral("Circular Pattern"),
        FormAnswers().number(QStringLiteral("count"), 4));
    EXPECT_EQ(bodies(doc), 1u);
    EXPECT_NEAR(partVolume(doc), 4000.0, 1e-6);
}

TEST(SolidCommandsTest, CommandsThatNeedABodySaySo) {
    MainWindow w;
    for (const char* command :
         {"action_fillet-3d", "action_chamfer-3d", "action_shell", "action_draft",
          "action_pattern-linear", "action_pattern-circular"}) {
        action(w, command)->trigger();
        EXPECT_TRUE(w.statusBar()->currentMessage().contains("make one first")) << command;
    }
    EXPECT_EQ(w.activeDocument()->featureTree().featureCount(), 0u);
}

TEST(SolidCommandsTest, AHollowPartIsOneBody) {
    // A ball with a closed cavity has two shells but one body; combining it
    // used to fill the cavity (Union) or keep only the core (Intersect).
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    run(w, "action_sphere", QStringLiteral("Sphere"),
        FormAnswers().number(QStringLiteral("size0"), 5.0));
    run(w, "action_sphere", QStringLiteral("Sphere"),
        FormAnswers().number(QStringLiteral("size0"), 2.0).combine(BodyOperation::Cut));
    ASSERT_EQ(doc.failedFeatureIndex(), -1) << doc.lastBuildMessage();
    ASSERT_EQ(bodies(doc), 2u) << "the outside and the cavity";
    const double hollow = partVolume(doc);

    action(w, "action_boolean-union")->trigger();
    EXPECT_EQ(doc.featureTree().featureCount(), 2u) << "nothing to combine";
    EXPECT_TRUE(w.statusBar()->currentMessage().contains("fewer than two"))
        << w.statusBar()->currentMessage().toStdString();
    EXPECT_NEAR(partVolume(doc), hollow, 1e-9);
}

TEST(SolidCommandsTest, EditingKeepsAZeroAndRefusesWhatAFeatureCannotUse) {
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    // A pointed cone: its top radius is 0, which the edit form used to show
    // (and could save) as 0.001.
    run(w, "action_cone", QStringLiteral("Cone"), FormAnswers());
    ASSERT_EQ(doc.featureTree().featureCount(), 1u);
    ASSERT_DOUBLE_EQ(doc.featureTree().feature(0)->parameters().at("topRadius"), 0.0);
    doc.setDirty(false);

    auto* panel = w.findChild<hz::ui::FeatureTreePanel*>();
    ASSERT_NE(panel, nullptr);
    auto* tree = panel->findChild<QTreeWidget*>();
    tree->setCurrentItem(tree->topLevelItem(0));
    auto* edit = panel->findChild<QAction*>(QStringLiteral("editFeature"));
    ASSERT_NE(edit, nullptr);
    {
        FormFiller filler(QStringLiteral("Edit Cone"), FormAnswers());
        edit->trigger();
        ASSERT_TRUE(filler.seen());
        EXPECT_EQ(filler.shown(QStringLiteral("topRadius")), 0.0) << "not clamped to 0.001";
    }
    EXPECT_DOUBLE_EQ(doc.featureTree().feature(0)->parameters().at("topRadius"), 0.0);
    EXPECT_FALSE(doc.isDirty());

    // Two facets are refused, and said to be; the height edit alongside goes in.
    const auto history = doc.undoStack().revision();
    {
        FormFiller filler(QStringLiteral("Edit Cone"), FormAnswers()
                                                           .number(QStringLiteral("segments"), 2)
                                                           .number(QStringLiteral("height"), 4));
        edit->trigger();
        ASSERT_TRUE(filler.seen());
    }
    EXPECT_TRUE(w.statusBar()->currentMessage().contains("Segments per turn"))
        << w.statusBar()->currentMessage().toStdString();
    EXPECT_DOUBLE_EQ(doc.featureTree().feature(0)->parameters().at("height"), 4.0);
    EXPECT_NE(doc.featureTree().feature(0)->parameters().at("segments"), 2.0);
    EXPECT_NE(doc.undoStack().revision(), history);
}
