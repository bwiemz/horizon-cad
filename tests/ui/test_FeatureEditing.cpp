// Editing a part's history from the window — edit, suppress, delete, and the
// commands that add features — is undoable, and the document's modified
// state follows. So are assembly edits.

#include <gtest/gtest.h>

#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QLineEdit>
#include <QTemporaryDir>
#include <QTimer>
#include <QTreeWidget>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "UiTestSupport.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/FeatureTreePanel.h"
#include "horizon/ui/MainWindow.h"

using hz::doc::BodyOperation;
using hz::math::Vec2;
using hz::ui::MainWindow;

namespace {

using hz::test::FilePicker;
using hz::test::FormAnswers;
using hz::test::FormFiller;

QAction* action(QObject& owner, const char* name) {
    auto* found = owner.findChild<QAction*>(QString::fromLatin1(name));
    EXPECT_NE(found, nullptr) << name;
    return found;
}

QAction* menuAction(MainWindow& w, const QString& text) {
    for (QAction* a : w.findChildren<QAction*>()) {
        if (a->text() == text) return a;
    }
    ADD_FAILURE() << "no menu item " << text.toStdString();
    return nullptr;
}

void drawRectangle(hz::doc::Document& doc, double x0, double y0, double x1, double y1) {
    auto& d = doc.draftDocument();
    d.clear();
    d.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(x0, y0), Vec2(x1, y0)));
    d.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(x1, y0), Vec2(x1, y1)));
    d.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(x1, y1), Vec2(x0, y1)));
    d.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(x0, y1), Vec2(x0, y0)));
}

void extrude(MainWindow& w, double distance, std::optional<BodyOperation> operation) {
    FormFiller filler(QStringLiteral("Extrude"),
                      FormAnswers().number({}, distance).combine(operation));
    action(w, "action_extrude")->trigger();
    ASSERT_TRUE(filler.seen());
}

double partVolume(const hz::doc::Document& doc) {
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

struct Panel {
    hz::ui::FeatureTreePanel* panel;
    QTreeWidget* tree;
    explicit Panel(MainWindow& w)
        : panel(w.findChild<hz::ui::FeatureTreePanel*>()),
          tree(panel ? panel->findChild<QTreeWidget*>() : nullptr) {}
    void select(int row) const { tree->setCurrentItem(tree->topLevelItem(row)); }
    QString status(int row) const { return tree->topLevelItem(row)->text(1); }
};

/// A 10 x 10 x 2 plate with a 2 x 2 pocket through it (192), made with the
/// Extrude command, and saved.
void makePlate(MainWindow& w) {
    hz::doc::Document& doc = *w.activeDocument();
    drawRectangle(doc, 0, 0, 10, 10);
    extrude(w, 2.0, std::nullopt);
    drawRectangle(doc, 4, 4, 6, 6);
    extrude(w, 2.0, BodyOperation::Cut);
    ASSERT_NEAR(partVolume(doc), 192.0, 1e-6);
    doc.setDirty(false);
}

}  // namespace

TEST(FeatureEditingTest, AnEditIsOneUndoableChange) {
    MainWindow w;
    makePlate(w);
    hz::doc::Document& doc = *w.activeDocument();
    Panel panel(w);
    ASSERT_NE(panel.tree, nullptr);

    // Make the plate 5 thick. The pocket still cuts 2 deep.
    panel.select(0);
    {
        FormFiller filler(QStringLiteral("Edit Extrude"),
                          FormAnswers().number(QStringLiteral("distance"), 5.0));
        action(*panel.panel, "editFeature")->trigger();
        ASSERT_TRUE(filler.seen());
    }
    EXPECT_NEAR(partVolume(doc), 500.0 - 8.0, 1e-6);
    EXPECT_TRUE(doc.isDirty()) << "an edit used to leave the document unmodified";
    EXPECT_TRUE(w.isWindowModified());

    action(w, "action_undo")->trigger();
    EXPECT_NEAR(partVolume(doc), 192.0, 1e-6) << "undo rebuilds the model";
    EXPECT_FALSE(doc.isDirty());
    action(w, "action_redo")->trigger();
    EXPECT_NEAR(partVolume(doc), 492.0, 1e-6);

    // Cancelling changes nothing and adds no undo step.
    const auto history = doc.undoStack().revision();
    {
        FormFiller filler(QStringLiteral("Edit Extrude"), FormAnswers().reject());
        action(*panel.panel, "editFeature")->trigger();
        ASSERT_TRUE(filler.seen());
    }
    EXPECT_EQ(doc.undoStack().revision(), history);
}

TEST(FeatureEditingTest, TheEditDialogChangesHowABodyCombines) {
    MainWindow w;
    makePlate(w);
    hz::doc::Document& doc = *w.activeDocument();
    Panel panel(w);
    panel.select(1);
    {
        // The pocket becomes a 2 x 2 boss, 4 high: 2 of it stands above the
        // plate.
        FormFiller filler(
            QStringLiteral("Edit Extrude"),
            FormAnswers().number(QStringLiteral("distance"), 4.0).combine(BodyOperation::Join));
        action(*panel.panel, "editFeature")->trigger();
        ASSERT_TRUE(filler.seen());
    }
    EXPECT_NEAR(partVolume(doc), 200.0 + 8.0, 1e-6);
    action(w, "action_undo")->trigger();
    EXPECT_NEAR(partVolume(doc), 192.0, 1e-6) << "one step undoes both changes";
}

TEST(FeatureEditingTest, SuppressAndDeleteFromThePanel) {
    MainWindow w;
    makePlate(w);
    hz::doc::Document& doc = *w.activeDocument();
    Panel panel(w);

    panel.select(1);
    action(*panel.panel, "suppressFeature")->trigger();
    EXPECT_NEAR(partVolume(doc), 200.0, 1e-6);
    EXPECT_EQ(panel.status(1), QStringLiteral("Suppressed"));
    EXPECT_EQ(action(*panel.panel, "suppressFeature")->text(), QStringLiteral("Unsuppress"))
        << "the selection stays on the feature, and the menu offers the way back";
    action(*panel.panel, "suppressFeature")->trigger();
    EXPECT_NEAR(partVolume(doc), 192.0, 1e-6);
    action(w, "action_undo")->trigger();
    action(w, "action_undo")->trigger();
    EXPECT_NEAR(partVolume(doc), 192.0, 1e-6);
    EXPECT_FALSE(doc.isDirty());

    panel.select(1);
    action(*panel.panel, "deleteFeature")->trigger();
    EXPECT_EQ(doc.featureTree().featureCount(), 1u);
    EXPECT_NEAR(partVolume(doc), 200.0, 1e-6);
    EXPECT_TRUE(doc.isDirty());
    action(w, "action_undo")->trigger();
    EXPECT_EQ(doc.featureTree().featureCount(), 2u);
    EXPECT_NEAR(partVolume(doc), 192.0, 1e-6);
}

TEST(FeatureEditingTest, UndoingAnExtrudeTakesAwayItsBodyAndSketch) {
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    const size_t sketches = doc.sketches().size();
    drawRectangle(doc, 0, 0, 10, 10);
    extrude(w, 2.0, std::nullopt);
    ASSERT_NE(doc.solid(), nullptr);
    EXPECT_EQ(doc.sketches().size(), sketches + 1);

    action(w, "action_undo")->trigger();
    EXPECT_EQ(doc.featureTree().featureCount(), 0u);
    EXPECT_EQ(doc.solid(), nullptr);
    EXPECT_EQ(doc.sketches().size(), sketches);
    action(w, "action_redo")->trigger();
    EXPECT_NEAR(partVolume(doc), 200.0, 1e-6);
}

TEST(FeatureEditingTest, InsertingAComponentIsUndoable) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString partPath = dir.filePath(QStringLiteral("block.hzpart"));
    {
        hz::doc::Document part;
        part.setType(hz::doc::DocumentType::Part);
        part.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(1, 1, 1));
        ASSERT_TRUE(part.rebuildModel());
        ASSERT_TRUE(hz::io::NativeFormat::save(partPath.toStdString(), part));
    }

    MainWindow w;
    menuAction(w, QStringLiteral("New Asse&mbly"))->trigger();
    hz::doc::AssemblyDocument* assembly = w.activeAssembly();
    ASSERT_NE(assembly, nullptr);
    EXPECT_FALSE(w.isWindowModified());

    {
        FilePicker picker(partPath);
        menuAction(w, QStringLiteral("&Insert Component..."))->trigger();
    }
    ASSERT_EQ(assembly->components().size(), 1u);
    EXPECT_TRUE(w.isWindowModified());

    action(w, "action_undo")->trigger();
    EXPECT_TRUE(assembly->components().empty());
    EXPECT_FALSE(w.isWindowModified()) << "back at the state it was created in";
    action(w, "action_redo")->trigger();
    EXPECT_EQ(assembly->components().size(), 1u);
    EXPECT_TRUE(w.isWindowModified());
}

TEST(FeatureEditingTest, OkWithoutChangesIsNotAnEdit) {
    // A spin box rounds what it shows to its decimals: a 360-degree revolve's
    // 2pi shows as 6.2832. Comparing that with the stored value made OK on an
    // untouched dialog an edit — one that changed the angle.
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    drawRectangle(doc, 2, 0, 4, 5);
    {
        FormFiller filler(QStringLiteral("Revolve"), FormAnswers().number({}, 360.0));
        action(w, "action_revolve")->trigger();
        ASSERT_TRUE(filler.seen());
    }
    ASSERT_EQ(doc.featureTree().featureCount(), 1u);
    const double angle = doc.featureTree().feature(0)->parameters().at("angle");
    doc.setDirty(false);
    const auto history = doc.undoStack().revision();

    Panel panel(w);
    panel.select(0);
    {
        FormFiller filler(QStringLiteral("Edit Revolve"), FormAnswers());
        action(*panel.panel, "editFeature")->trigger();
        ASSERT_TRUE(filler.seen());
    }
    EXPECT_EQ(doc.undoStack().revision(), history);
    EXPECT_FALSE(doc.isDirty());
    EXPECT_EQ(doc.featureTree().feature(0)->parameters().at("angle"), angle);
}

TEST(FeatureEditingTest, WhereADraggedFeatureLands) {
    using hz::ui::FeatureTreePanel;
    // Three rows. Dropped on the upper half of a row: before it; lower half:
    // after it; below every row: last. The result is the index after the move.
    EXPECT_EQ(FeatureTreePanel::dropDestination(0, 2, true, 3), 2);
    EXPECT_EQ(FeatureTreePanel::dropDestination(0, 2, false, 3), 1);
    EXPECT_EQ(FeatureTreePanel::dropDestination(0, 1, false, 3), 0) << "back where it was";
    EXPECT_EQ(FeatureTreePanel::dropDestination(0, 0, true, 3), 0) << "onto itself";
    EXPECT_EQ(FeatureTreePanel::dropDestination(2, 0, false, 3), 0);
    EXPECT_EQ(FeatureTreePanel::dropDestination(2, 0, true, 3), 1);
    EXPECT_EQ(FeatureTreePanel::dropDestination(1, -1, false, 3), 2);
    EXPECT_EQ(FeatureTreePanel::dropDestination(-1, 0, false, 3), -1) << "nothing dragged";
    EXPECT_EQ(FeatureTreePanel::dropDestination(0, -1, false, 0), 0) << "empty list";
}

TEST(FeatureEditingTest, AReorderFromThePanelIsUndoable) {
    MainWindow w;
    makePlate(w);
    hz::doc::Document& doc = *w.activeDocument();
    Panel panel(w);
    const hz::doc::Feature* pocket = doc.featureTree().feature(1);

    emit panel.panel->featureReordered(1, 0);
    EXPECT_EQ(doc.featureTree().feature(0), pocket);
    EXPECT_EQ(doc.failedFeatureIndex(), 0) << "a cut before the plate has nothing to cut";
    EXPECT_EQ(panel.status(0), QStringLiteral("FAILED"));

    action(w, "action_undo")->trigger();
    EXPECT_EQ(doc.featureTree().feature(1), pocket);
    EXPECT_NEAR(partVolume(doc), 192.0, 1e-6);
    EXPECT_FALSE(doc.isDirty());
}

TEST(FeatureEditingTest, MilestoneTwoAPlateWithAHoleFilletedUndoneSavedAndReopened) {
    // "A user can model a plate with a hole, fillet an edge, undo it, save and
    // reopen it — through the UI."
    MainWindow w;
    hz::doc::Document& doc = *w.activeDocument();
    drawRectangle(doc, 0, 0, 10, 10);
    extrude(w, 5.0, std::nullopt);
    doc.draftDocument().clear();
    doc.draftDocument().addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(5, 5), 2.0));
    extrude(w, 5.0, BodyOperation::Cut);
    ASSERT_EQ(doc.featureTree().featureCount(), 2u);
    ASSERT_EQ(doc.solid()->genus(), 1);
    const double plate = partVolume(doc);

    {
        FormFiller filler(
            QStringLiteral("Fillet"),
            FormAnswers()
                .number(QStringLiteral("size"), 1.0)
                .check(QStringLiteral("edges"), {QStringLiteral("(10, 0, 0) – (10, 0, 5)"),
                                                 QStringLiteral("(10, 0, 5) – (10, 0, 0)")}));
        action(w, "action_fillet-3d")->trigger();
        ASSERT_TRUE(filler.seen());
    }
    ASSERT_EQ(doc.featureTree().featureCount(), 3u) << "the fillet was refused";
    const double filleted = partVolume(doc);
    EXPECT_LT(filleted, plate);

    action(w, "action_undo")->trigger();
    EXPECT_EQ(doc.featureTree().featureCount(), 2u);
    EXPECT_NEAR(partVolume(doc), plate, 1e-9);
    action(w, "action_redo")->trigger();
    EXPECT_NEAR(partVolume(doc), filleted, 1e-9);

    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const std::string path = dir.filePath(QStringLiteral("plate.hzpart")).toStdString();
    std::string error;
    ASSERT_TRUE(hz::io::NativeFormat::save(path, doc, &error)) << error;
    hz::doc::Document reopened;
    ASSERT_TRUE(hz::io::NativeFormat::load(path, reopened, &error)) << error;
    ASSERT_TRUE(reopened.rebuildModel()) << reopened.lastBuildMessage();
    EXPECT_NEAR(partVolume(reopened), filleted, 1e-9) << "the same part, fillet and all";
}
