// Work on worker threads, without a window: a model rebuild from its
// snapshot (Phase 114), background tasks, and starting a worker when the
// system has no thread to give (Phase 125). These run in the ThreadSanitizer
// job; the window tests do not, since Qt itself is not instrumented.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <system_error>
#include <thread>

#ifdef __linux__
#include <sys/resource.h>
#endif

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/ui/BackgroundTask.h"
#include "horizon/ui/RebuildJob.h"
#include "horizon/ui/WorkerThread.h"

using hz::doc::Document;
using hz::doc::PrimitiveFeature;
using hz::ui::BackgroundTask;
using hz::ui::RebuildJob;

namespace {

/// Wait until @p done, for at most @p seconds.
template <typename Done>
bool waitFor(Done done, int seconds = 60) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    while (!done() && std::chrono::steady_clock::now() < until) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return done();
}

double volumeOf(const hz::topo::Solid& solid) {
    return hz::model::MassPropertiesCalculator::compute(solid).volume;
}

}  // namespace

TEST(WorkerTest, AJobRebuildsASnapshotOnAWorker) {
    Document doc;
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(2, 3, 4));
    RebuildJob job(doc);
    EXPECT_EQ(job.stamp(), RebuildJob::stampOf(doc));
    job.start();
    ASSERT_TRUE(waitFor([&] { return job.finished(); }));
    auto result = job.takeResult();
    ASSERT_FALSE(result.cancelled);
    ASSERT_NE(result.solid, nullptr);
    EXPECT_NEAR(volumeOf(*result.solid), 24.0, 1e-9);
    EXPECT_EQ(doc.solid(), nullptr) << "the document itself is not touched";

    EXPECT_TRUE(doc.applyBuild(std::move(result)));
    ASSERT_NE(doc.solid(), nullptr);
    EXPECT_NEAR(volumeOf(*doc.solid()), 24.0, 1e-9);
}

TEST(WorkerTest, ACancelledJobGivesNoModel) {
    Document doc;
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(2, 3, 4));
    RebuildJob job(doc);
    job.cancel();
    job.start();
    ASSERT_TRUE(waitFor([&] { return job.finished(); }));
    EXPECT_TRUE(job.takeResult().cancelled);
}

TEST(WorkerTest, AChangedDocumentNoLongerMatchesItsSnapshot) {
    Document doc;
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(2, 3, 4));
    RebuildJob job(doc);
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(1, 1, 1));
    EXPECT_FALSE(job.stamp() == RebuildJob::stampOf(doc));
}

// The worker builds from its own copy, so the document can be edited while
// the rebuild runs: here features are added on this thread throughout. The
// features, entities and IDs made on both threads at once are where a shared
// structure would show under ThreadSanitizer.
TEST(WorkerTest, TheDocumentCanBeEditedWhileItsRebuildRuns) {
    Document doc;
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(2, 3, 4));
    doc.featureTree().addFeature(
        hz::doc::PatternFeature::makeLinear(hz::math::Vec3(1, 0, 0), 5.0, 12));
    RebuildJob job(doc);
    job.start();
    // At least a few edits, however fast the rebuild; more while it runs.
    int edits = 0;
    while (edits < 20 || (!job.finished() && edits < 200)) {
        doc.featureTree().addFeature(PrimitiveFeature::makeBox(1, 1, 1));
        ++edits;
    }
    ASSERT_TRUE(waitFor([&] { return job.finished(); }));
    const auto result = job.takeResult();
    ASSERT_FALSE(result.cancelled);
    ASSERT_NE(result.solid, nullptr);
    EXPECT_NEAR(volumeOf(*result.solid), 24.0 * 12, 1e-6) << "built as the snapshot was";
    EXPECT_FALSE(job.stamp() == RebuildJob::stampOf(doc)) << edits << " edits";
}

// A background task's work sees its Cancel, and its result comes back.
TEST(WorkerTest, ABackgroundTaskSeesItsCancelAndReturnsItsResult) {
    std::atomic<bool> started{false};
    BackgroundTask<int> task([&started](const std::atomic<bool>& cancelled) {
        started = true;
        while (!cancelled) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return 42;
    });
    std::atomic<bool> finishedCalled{false};
    task.start([&finishedCalled] { finishedCalled = true; });
    ASSERT_TRUE(waitFor([&] { return started.load(); }));
    EXPECT_FALSE(task.finished());
    task.cancel();
    ASSERT_TRUE(waitFor([&] { return task.finished(); }));
    EXPECT_TRUE(task.cancelled());
    EXPECT_TRUE(waitFor([&] { return finishedCalled.load(); }));
    EXPECT_EQ(task.take(), 42);
    EXPECT_TRUE(task.error().empty());
}

#ifdef __linux__
// When no thread can be started, background work runs on the caller's
// thread. std::thread's exception used to escape start(): the job was never
// finished, so it was waited for forever and every later rebuild queued
// behind it. RLIMIT_NPROC at 0 makes the system refuse new threads.
TEST(WorkerTest, WithoutThreadsTheWorkIsDoneHere) {
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

        BackgroundTask<int> task([](const std::atomic<bool>&) { return 7; });
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
