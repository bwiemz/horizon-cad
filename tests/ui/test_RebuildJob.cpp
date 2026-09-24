// A long model rebuild runs on a worker thread and does not freeze the window
// (Phase 114): the document is snapshotted, rebuilt apart, and the result
// applied only if nothing changed meanwhile.

#include <gtest/gtest.h>

#include <QAction>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QProgressBar>
#include <QTemporaryDir>
#include <memory>

#include "UiTestSupport.h"
#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/RebuildJob.h"

using hz::doc::Document;
using hz::doc::PrimitiveFeature;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::ui::MainWindow;
using hz::ui::RebuildJob;

namespace {

double volumeOf(const Document& doc) {
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

TEST(RebuildJobTest, AJobRebuildsASnapshotOnAWorker) {
    Document doc;
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(2, 3, 4));
    RebuildJob job(doc);
    EXPECT_EQ(job.stamp(), RebuildJob::stampOf(doc));
    job.start();
    ASSERT_TRUE(waitFor([&] { return job.finished(); }));
    auto result = job.takeResult();
    ASSERT_FALSE(result.cancelled);
    ASSERT_NE(result.solid, nullptr);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*result.solid).volume, 24.0, 1e-9);
    EXPECT_EQ(doc.solid(), nullptr) << "the document itself is not touched";

    EXPECT_TRUE(doc.applyBuild(std::move(result)));
    EXPECT_NEAR(volumeOf(doc), 24.0, 1e-9);
}

TEST(RebuildJobTest, ACancelledJobGivesNoModel) {
    Document doc;
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(2, 3, 4));
    RebuildJob job(doc);
    job.cancel();
    job.start();
    ASSERT_TRUE(waitFor([&] { return job.finished(); }));
    EXPECT_TRUE(job.takeResult().cancelled);
}

TEST(RebuildJobTest, AChangedDocumentNoLongerMatchesItsSnapshot) {
    Document doc;
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(2, 3, 4));
    RebuildJob job(doc);
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(1, 1, 1));
    EXPECT_FALSE(job.stamp() == RebuildJob::stampOf(doc));
}

TEST(RebuildJobTest, TheWindowAppliesAWorkersRebuild) {
    MainWindow w;
    w.setRebuildMode(MainWindow::RebuildMode::Always);
    FormFiller filler(QStringLiteral("Box"), FormAnswers()
                                                 .number(QStringLiteral("size0"), 2.0)
                                                 .number(QStringLiteral("size1"), 3.0)
                                                 .number(QStringLiteral("size2"), 4.0));
    w.findChild<QAction*>(QStringLiteral("action_box"))->trigger();
    ASSERT_TRUE(filler.seen());
    EXPECT_TRUE(w.rebuildRunning()) << "the rebuild went to a worker";
    EXPECT_TRUE(w.findChild<QProgressBar*>(QStringLiteral("rebuildProgress"))->isVisibleTo(&w));

    ASSERT_TRUE(waitFor([&] { return !w.rebuildRunning(); }));
    EXPECT_NEAR(volumeOf(*w.activeDocument()), 24.0, 1e-9);
    EXPECT_FALSE(w.findChild<QProgressBar*>(QStringLiteral("rebuildProgress"))->isVisibleTo(&w));
}

TEST(RebuildJobTest, AnEditWhileTheWorkerRunsIsBuiltToo) {
    MainWindow w;
    w.setRebuildMode(MainWindow::RebuildMode::Always);
    FormFiller filler(QStringLiteral("Box"), FormAnswers()
                                                 .number(QStringLiteral("size0"), 2.0)
                                                 .number(QStringLiteral("size1"), 3.0)
                                                 .number(QStringLiteral("size2"), 4.0));
    w.findChild<QAction*>(QStringLiteral("action_box"))->trigger();
    ASSERT_TRUE(filler.seen());
    // Undo before the worker is done: its result is out of date, and what is
    // built in the end is the part as it is now — empty.
    w.findChild<QAction*>(QStringLiteral("action_undo"))->trigger();
    ASSERT_TRUE(waitFor([&] { return !w.rebuildRunning(); }));
    EXPECT_EQ(w.activeDocument()->featureTree().featureCount(), 0u);
    EXPECT_EQ(w.activeDocument()->solid(), nullptr);
}

TEST(RebuildJobTest, AStepImportOnAWorkerMakesThePart) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    const QString path = dir.filePath(QStringLiteral("bracket.step"));
    auto box = hz::model::PrimitiveFactory::makeBox(3, 4, 5);
    ASSERT_TRUE(hz::io::StepFormat::save(path.toStdString(), {box.get()}));

    MainWindow w;
    w.setRebuildMode(MainWindow::RebuildMode::Always);
    {
        hz::test::FilePicker picker(path);
        w.findChild<QAction*>(QStringLiteral("import_step"))->trigger();
    }
    ASSERT_TRUE(waitFor([&] { return !w.backgroundWorkRunning(); }));
    EXPECT_EQ(w.activeDocument()->type(), hz::doc::DocumentType::Part);
    EXPECT_NEAR(volumeOf(*w.activeDocument()), 60.0, 1e-9);
}
