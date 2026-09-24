// The GUI thread stays free (Phase 137): a feature added is built once, on a
// worker when builds are slow, and one that fails itself is withdrawn; a part
// opened is built on a worker; the model is tessellated once for each build.

#include <gtest/gtest.h>

#include <QAction>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QStatusBar>
#include <QTemporaryDir>
#include <memory>
#include <string>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/ui/MainWindow.h"

using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::ui::MainWindow;

namespace {

void trigger(MainWindow& w, const char* name) {
    auto* action = w.findChild<QAction*>(QString::fromLatin1(name));
    ASSERT_NE(action, nullptr) << name;
    action->trigger();
}

/// A 10 mm box at @p x along X, combined with the part as @p result says
/// (the form's own choice when empty).
void box(MainWindow& w, double x = 0.0, const QString& result = QString()) {
    FormAnswers answers = FormAnswers()
                              .number(QStringLiteral("size0"), 10.0)
                              .number(QStringLiteral("size1"), 10.0)
                              .number(QStringLiteral("size2"), 10.0)
                              .number(QStringLiteral("atX"), x);
    if (!result.isEmpty()) answers.choose(QStringLiteral("bodyOperation"), result);
    FormFiller filler(QStringLiteral("Box"), std::move(answers));
    trigger(w, "action_box");
    EXPECT_TRUE(filler.seen());
}

double volume(const hz::doc::Document& doc) {
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

/// Process events until @p done, for at most @p ms.
template <typename Done>
bool waitFor(Done done, int ms = 20000) {
    QElapsedTimer clock;
    clock.start();
    while (!done() && clock.elapsed() < ms) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    }
    return done();
}

}  // namespace

// A feature added is built once. It was built twice: first to try it, always
// on the GUI thread, then again to show it.
TEST(GuiThreadFreeTest, AFeatureAddedIsBuiltOnce) {
    MainWindow w;
    box(w);
    auto& doc = *w.activeDocument();
    const auto builds = doc.builds();
    box(w, 5.0);  // joins the first
    EXPECT_EQ(doc.builds(), builds + 1);
    EXPECT_EQ(doc.featureTree().featureCount(), 2u);
    EXPECT_NEAR(volume(doc), 1500.0, 1e-6);
}

// A feature that fails itself is withdrawn: the part and the undo history are
// as they were, with nothing to redo, and the reason is given.
TEST(GuiThreadFreeTest, AFeatureThatFailsItselfIsWithdrawn) {
    MainWindow w;
    box(w);
    auto& doc = *w.activeDocument();
    box(w, 100.0, QStringLiteral("Keep the intersection"));  // touches nothing
    EXPECT_EQ(doc.featureTree().featureCount(), 1u);
    EXPECT_NEAR(volume(doc), 1000.0, 1e-6) << "the part as it was";
    EXPECT_FALSE(doc.undoStack().canRedo()) << "withdrawn, not waiting to be redone";
    EXPECT_TRUE(w.statusBar()->currentMessage().contains(QStringLiteral("not added")))
        << w.statusBar()->currentMessage().toStdString();
    trigger(w, "action_undo");
    EXPECT_EQ(doc.featureTree().featureCount(), 0u) << "the first box was the step before";
}

// On a worker too: the build comes back, and the feature that fails itself
// is withdrawn then.
TEST(GuiThreadFreeTest, OnAWorkerAFeatureThatFailsItselfIsWithdrawnToo) {
    MainWindow w;
    w.setRebuildMode(MainWindow::RebuildMode::Always);
    box(w);
    ASSERT_TRUE(waitFor([&] { return !w.rebuildRunning(); }));
    auto& doc = *w.activeDocument();
    const auto builds = doc.builds();
    box(w, 100.0, QStringLiteral("Keep the intersection"));
    EXPECT_TRUE(w.rebuildRunning());
    EXPECT_EQ(doc.builds(), builds) << "nothing built here, to try it";
    EXPECT_EQ(doc.featureTree().featureCount(), 2u) << "in, while its build runs";
    ASSERT_TRUE(waitFor([&] { return !w.rebuildRunning(); }));
    EXPECT_EQ(doc.featureTree().featureCount(), 1u);
    EXPECT_NEAR(volume(doc), 1000.0, 1e-6);
    EXPECT_FALSE(doc.undoStack().canRedo());
}

// A part opened is built on a worker: how long its model takes to build is
// not known, and a large one froze the window as it opened. Asked never to,
// it is built at once.
TEST(GuiThreadFreeTest, APartOpenedIsBuiltOnAWorker) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("part.hcad"));
    {
        MainWindow w;
        box(w);
        ASSERT_TRUE(hz::io::NativeFormat::save(path.toStdString(), *w.activeDocument()));
    }

    MainWindow w;
    ASSERT_TRUE(w.openPath(path));
    EXPECT_TRUE(w.rebuildRunning());
    EXPECT_EQ(w.activeDocument()->builds(), 0u) << "not built here";
    ASSERT_TRUE(waitFor([&] { return !w.rebuildRunning(); }));
    EXPECT_NEAR(volume(*w.activeDocument()), 1000.0, 1e-6);

    MainWindow here;
    here.setRebuildMode(MainWindow::RebuildMode::Never);
    ASSERT_TRUE(here.openPath(path));
    EXPECT_FALSE(here.rebuildRunning());
    EXPECT_NEAR(volume(*here.activeDocument()), 1000.0, 1e-6);
}

// The model is tessellated once for each build: the scene is rebuilt for
// more than a new model (a sketch opened and closed), and tessellated it
// every time.
TEST(GuiThreadFreeTest, TheModelIsTessellatedOnceForEachBuild) {
    MainWindow w;
    box(w);
    const auto tessellations = w.tessellations();
    trigger(w, "action_sketch_xy");
    trigger(w, "action_sketch_finish");
    EXPECT_EQ(w.tessellations(), tessellations) << "the same model";
    box(w, 5.0);
    EXPECT_EQ(w.tessellations(), tessellations + 1) << "a new one";
}

// A large file is read on a worker (here, any: Always), and its tab comes
// when it has been read; a part's model is then built on a worker too.
TEST(GuiThreadFreeTest, ALargeFileIsReadOnAWorker) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString part = dir.filePath(QStringLiteral("part.hcad"));
    const QString drawing = dir.filePath(QStringLiteral("plan.dxf"));
    {
        MainWindow w;
        box(w);
        ASSERT_TRUE(hz::io::NativeFormat::save(part.toStdString(), *w.activeDocument()));
        hz::doc::Document plan;
        plan.draftDocument().addEntity(
            std::make_shared<hz::draft::DraftLine>(hz::math::Vec2(0, 0), hz::math::Vec2(10, 0)));
        ASSERT_TRUE(hz::io::DxfFormat::save(drawing.toStdString(), plan));
    }

    MainWindow w;
    w.setRebuildMode(MainWindow::RebuildMode::Always);
    const auto* first = w.activeDocument();
    ASSERT_TRUE(w.openPath(part));
    EXPECT_TRUE(w.backgroundWorkRunning()) << "being read";
    EXPECT_EQ(w.activeDocument(), first) << "no tab until it is read";
    EXPECT_FALSE(w.openPath(drawing)) << "one file at a time";
    ASSERT_TRUE(waitFor([&] { return !w.backgroundWorkRunning(); }));
    ASSERT_NE(w.activeDocument(), first);
    EXPECT_NEAR(volume(*w.activeDocument()), 1000.0, 1e-6) << "read, then built";

    ASSERT_TRUE(w.openPath(drawing));
    ASSERT_TRUE(waitFor([&] { return !w.backgroundWorkRunning(); }));
    EXPECT_EQ(w.activeDocument()->draftDocument().entities().size(), 1u);
    EXPECT_EQ(QString::fromStdString(w.activeDocument()->filePath()), drawing);

    // Open again: the tab it is in is shown, not another.
    const auto* shown = w.activeDocument();
    ASSERT_TRUE(w.openPath(part));
    ASSERT_TRUE(waitFor([&] { return !w.backgroundWorkRunning(); }));
    EXPECT_NE(w.activeDocument(), shown);
    ASSERT_TRUE(w.openPath(drawing));
    EXPECT_EQ(w.activeDocument(), shown) << "already open: its tab";
}
