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

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/ui/FeatureTreePanel.h"
#include "horizon/ui/MainWindow.h"

using hz::doc::BodyOperation;
using hz::math::Vec2;
using hz::ui::MainWindow;

namespace {

/// Answers the next modal dialog titled `title`: sets the spin boxes named in
/// `values`, picks `operation` in the "bodyOperation" combo when given, and
/// accepts. With `cancel`, rejects it instead.
class FormFiller {
public:
    FormFiller(QString title, std::map<QString, double> values,
               std::optional<BodyOperation> operation = std::nullopt, bool cancel = false)
        : m_title(std::move(title)),
          m_values(std::move(values)),
          m_operation(operation),
          m_cancel(cancel) {
        QObject::connect(&m_timer, &QTimer::timeout, [this] { poll(); });
        m_timer.start(5);
        m_clock.start();
    }
    bool seen() const { return m_seen; }

private:
    void poll() {
        auto* dialog = qobject_cast<QDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr || dialog->windowTitle() != m_title) {
            if (m_clock.elapsed() > 5000) m_timer.stop();
            return;
        }
        m_timer.stop();
        m_seen = true;
        if (m_cancel) {
            dialog->reject();
            return;
        }
        for (const auto& [name, value] : m_values) {
            auto* spin = dialog->findChild<QDoubleSpinBox*>(name);
            if (spin == nullptr) {
                ADD_FAILURE() << "no field " << name.toStdString();
                dialog->reject();
                return;
            }
            spin->setValue(value);
        }
        if (m_operation) {
            auto* combo = dialog->findChild<QComboBox*>(QStringLiteral("bodyOperation"));
            if (combo == nullptr) {
                ADD_FAILURE() << "no body operation choice";
                dialog->reject();
                return;
            }
            combo->setCurrentIndex(combo->findData(static_cast<int>(*m_operation)));
        }
        dialog->accept();
    }

    QString m_title;
    std::map<QString, double> m_values;
    std::optional<BodyOperation> m_operation;
    bool m_cancel;
    bool m_seen = false;
    QTimer m_timer;
    QElapsedTimer m_clock;
};

/// Picks `path` in the next file dialog by typing it into the file name box.
/// (QFileDialog::selectFile() leaves that box alone while it has focus, which
/// a shown dialog gives it — the accept then finds no file and does nothing.)
/// Gives up — rejecting the dialog — rather than hang the test.
class FilePicker {
public:
    explicit FilePicker(QString path) : m_path(std::move(path)) {
        QObject::connect(&m_timer, &QTimer::timeout, [this] { poll(); });
        m_timer.start(20);
        m_clock.start();
    }

private:
    void poll() {
        auto* dialog = qobject_cast<QFileDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr) {
            if (m_seen || m_clock.elapsed() > 10'000) m_timer.stop();
            return;
        }
        m_seen = true;
        if (m_clock.elapsed() > 10'000) {
            ADD_FAILURE() << "the file dialog would not take " << m_path.toStdString();
            dialog->reject();
            m_timer.stop();
            return;
        }
        auto* name = dialog->findChild<QLineEdit*>(QStringLiteral("fileNameEdit"));
        if (name == nullptr) {
            ADD_FAILURE() << "the file dialog has no file name box";
            dialog->reject();
            m_timer.stop();
            return;
        }
        name->setText(m_path);
        static_cast<QDialog*>(dialog)->accept();  // QFileDialog's own accept() is protected
    }

    QString m_path;
    bool m_seen = false;
    QTimer m_timer;
    QElapsedTimer m_clock;
};

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
    FormFiller filler(QStringLiteral("Extrude"), {{QString(), distance}}, operation);
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
        FormFiller filler(QStringLiteral("Edit Extrude"), {{QStringLiteral("distance"), 5.0}});
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
        FormFiller filler(QStringLiteral("Edit Extrude"), {}, std::nullopt, /*cancel=*/true);
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
        FormFiller filler(QStringLiteral("Edit Extrude"), {{QStringLiteral("distance"), 4.0}},
                          BodyOperation::Join);
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
        FormFiller filler(QStringLiteral("Revolve"), {{QString(), 360.0}});
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
        FormFiller filler(QStringLiteral("Edit Revolve"), {});
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
