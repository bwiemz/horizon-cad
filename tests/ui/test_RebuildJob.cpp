// A long model rebuild runs on a worker thread and does not freeze the window
// (Phase 114): the document is snapshotted, rebuilt apart, and the result
// applied only if nothing changed meanwhile.

#include <gtest/gtest.h>

#include <QAction>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QProgressBar>
#include <QTabBar>
#include <QTemporaryDir>
#include <QToolButton>
#include <algorithm>
#include <cmath>
#include <memory>
#include <system_error>
#include <thread>

#ifdef __linux__
#include <sys/resource.h>
#endif

#include "UiTestSupport.h"
#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/ui/BackgroundTask.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/RebuildJob.h"
#include "horizon/ui/WorkerThread.h"

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

TEST(RebuildJobTest, ACancelledRebuildIsDoneWhenTheTabIsShownAgain) {
    MainWindow w;
    w.setRebuildMode(MainWindow::RebuildMode::Always);
    for (QAction* a : w.findChildren<QAction*>()) {
        if (a->text().remove(QLatin1Char('&')) == QStringLiteral("New Part")) a->trigger();
    }
    Document& part = *w.activeDocument();
    ASSERT_EQ(part.type(), hz::doc::DocumentType::Part);
    // A pattern first, slow enough that Cancel is seen before the build ends.
    part.featureTree().addFeature(PrimitiveFeature::makeBox(2, 3, 4));
    part.featureTree().addFeature(
        hz::doc::PatternFeature::makeLinear(hz::math::Vec3(1, 0, 0), 5.0, 12));
    part.undoStack().push(std::make_unique<hz::doc::AddFeatureCommand>(
        part, PrimitiveFeature::makeBox(1, 1, 1), nullptr));
    ASSERT_TRUE(waitFor([&] { return !w.rebuildRunning(); }));

    // Undo, and cancel the rebuild it starts: the model stays behind its
    // features. It used to stay so for good, even after the tab was shown again.
    w.findChild<QAction*>(QStringLiteral("action_undo"))->trigger();
    ASSERT_TRUE(w.rebuildRunning());
    w.findChild<QToolButton*>(QStringLiteral("cancelRebuild"))->click();
    ASSERT_TRUE(waitFor([&] { return !w.rebuildRunning(); }));
    EXPECT_EQ(part.solid(), nullptr) << "nothing was built";

    auto* tabs = w.findChild<QTabBar*>(QStringLiteral("documentTabs"));
    const int partTab = tabs->currentIndex();
    w.findChild<QAction*>(QStringLiteral("action_new"))->trigger();  // another tab
    tabs->setCurrentIndex(partTab);
    EXPECT_TRUE(w.rebuildRunning()) << "showing the tab again catches the model up";
    ASSERT_TRUE(waitFor([&] { return !w.rebuildRunning(); }));
    ASSERT_NE(part.solid(), nullptr);
    EXPECT_NEAR(volumeOf(part), 24.0 * 12, 1e-6);
}

// Saving while a rebuild runs on a worker caches the part as it is now. The
// document still held the part from before the edit, and Save wrote its mesh
// as the file's tessellation cache, which lightweight assembly loads show.
TEST(RebuildJobTest, SavingDuringARebuildCachesThePartAsItIsNow) {
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    MainWindow w;
    w.setRebuildMode(MainWindow::RebuildMode::Always);
    const auto addBox = [&w](double x, double y, double z) {
        FormFiller filler(QStringLiteral("Box"), FormAnswers()
                                                     .number(QStringLiteral("size0"), x)
                                                     .number(QStringLiteral("size1"), y)
                                                     .number(QStringLiteral("size2"), z));
        w.findChild<QAction*>(QStringLiteral("action_box"))->trigger();
        return filler.seen();
    };
    ASSERT_TRUE(addBox(2, 3, 4));
    ASSERT_TRUE(waitFor([&] { return !w.rebuildRunning(); }));
    Document& part = *w.activeDocument();
    ASSERT_NE(part.solid(), nullptr);
    const std::string path = dir.filePath(QStringLiteral("part.hcad")).toStdString();
    part.setFilePath(path);

    ASSERT_TRUE(addBox(10, 1, 1));
    ASSERT_TRUE(waitFor([&] { return !w.rebuildRunning(); }));

    // Undo the long box. The rebuild goes to a worker; until it is applied,
    // the document's solid still has the long box in it.
    w.findChild<QAction*>(QStringLiteral("action_undo"))->trigger();
    ASSERT_TRUE(w.rebuildRunning());
    w.findChild<QAction*>(QStringLiteral("action_save"))->trigger();

    const auto mesh = hz::io::NativeFormat::loadPartMesh(path);
    ASSERT_NE(mesh, nullptr);
    float maxExtent = 0.0f;
    for (float c : mesh->positions) maxExtent = std::max(maxExtent, std::abs(c));
    EXPECT_LT(maxExtent, 4.5f) << "the cache still held the undone 10-long box";
    ASSERT_TRUE(waitFor([&] { return !w.rebuildRunning(); }));
}

#ifdef __linux__
// When no thread can be started, background work runs on the caller's
// thread. std::thread's exception used to escape start(): the job was never
// finished, so it was waited for forever and every later rebuild queued
// behind it. RLIMIT_NPROC at 0 makes the system refuse new threads.
TEST(RebuildJobTest, WithoutThreadsTheWorkIsDoneHere) {
    rlimit saved{};
    ASSERT_EQ(getrlimit(RLIMIT_NPROC, &saved), 0);
    rlimit none = saved;
    none.rlim_cur = 0;
    ASSERT_EQ(setrlimit(RLIMIT_NPROC, &none), 0);
    bool denied = false;
    try {
        std::thread probe([] {});
        probe.join();
    } catch (const std::system_error&) {
        denied = true;
    }

    std::thread::id ranOn;
    bool taskFinished = false;
    int taskResult = 0;
    bool jobFinished = false;
    Document doc;
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(2, 3, 4));
    if (denied) {
        std::thread holder;
        hz::ui::startWorker(holder, [&ranOn] { ranOn = std::this_thread::get_id(); });
        EXPECT_FALSE(holder.joinable());

        hz::ui::BackgroundTask<int> task([](const std::atomic<bool>&) { return 7; });
        task.start();
        taskFinished = task.finished();
        taskResult = task.take();

        RebuildJob job(doc);
        job.start();
        jobFinished = job.finished();
    }
    setrlimit(RLIMIT_NPROC, &saved);
    if (!denied) GTEST_SKIP() << "this process may start threads whatever the limit (root)";

    EXPECT_EQ(ranOn, std::this_thread::get_id());
    EXPECT_TRUE(taskFinished);
    EXPECT_EQ(taskResult, 7);
    EXPECT_TRUE(jobFinished);
}
#endif
