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
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "UiTestSupport.h"
#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/geometry/Frame.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/RebuildJob.h"

using hz::doc::Document;
using hz::doc::ImportedBodyFeature;
using hz::doc::PrimitiveFeature;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::ui::MainWindow;

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

/// A part of one imported body, @p body, from @p source; with @p note, what its read said of it.
std::unique_ptr<Document> partOf(std::unique_ptr<hz::topo::Solid> body, const std::string& source,
                                 const std::string& note = {}) {
    auto part = std::make_unique<Document>();
    part->setType(hz::doc::DocumentType::Part);
    auto imported = std::make_unique<ImportedBodyFeature>(
        std::shared_ptr<const hz::topo::Solid>(std::move(body)), source);
    imported->setNote(note);
    part->featureTree().addFeature(std::move(imported));
    return part;
}

/// A cylinder on a tilted axis, from a file drawn in inches, read as File ▸ Import ▸ STEP reads
/// it. Kept in a part file and read back, it is defined a last bit otherwise (its frames, scaled
/// into millimetres, are renormalised): what a worker's copy read again from its STEP held.
std::unique_ptr<hz::topo::Solid> importedCylinderInInches() {
    const hz::math::Vec3 axis = hz::math::Vec3(1.0, 2.0, 3.0).normalized();
    const auto cylinder =
        hz::model::exact::makeCylinder(hz::geo::Frame::fromAxis({0.3, 0.7, -1.1}, axis), 1.3, 2.9);
    std::string text = hz::io::StepFormat::toString({cylinder.get()});
    const std::string millimetres = "SI_UNIT(.MILLI.,.METRE.)";
    const auto unit = text.find(millimetres);
    const auto data = text.find("ENDSEC;", text.find("DATA;"));
    EXPECT_NE(unit, std::string::npos);
    EXPECT_NE(data, std::string::npos);
    if (unit == std::string::npos || data == std::string::npos) return nullptr;
    text.insert(data,
                "#9998 = (LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.));\n"
                "#9999 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#9998);\n");
    text.replace(unit, millimetres.size(), "CONVERSION_BASED_UNIT('INCH',#9999)");
    auto read = hz::io::StepFormat::fromString(text);
    EXPECT_EQ(read.size(), 1u) << hz::io::StepFormat::lastError();
    return read.empty() ? nullptr : std::move(read.front());
}

/// @p job's result, once its worker is done with it.
hz::doc::BuildResult ranOnItsWorker(hz::ui::RebuildJob& job) {
    job.start();
    for (int i = 0; i < 12000 && !job.finished(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_TRUE(job.finished());
    return job.takeResult();
}

}  // namespace

// A worker's copy of a part shares the document's imported bodies (I4), so a counterpart made on
// either thread is the other's: a worker's build after the window's reuses the window's, and the
// window's after a worker's reuses the worker's. Neither meshes the bodies again. (A copy that
// read its bodies again from their STEP held this cylinder a last bit otherwise, and each switch
// between the threads meshed it again.) Reused, the counterpart is the one made before it, its
// part and its mesh the very ones. (Re-baselined for Task 8: a counterpart reused whole says it
// meshed no body, ExactCounterpart::bodiesMeshed 0, so it is a new object sharing the one made
// before it, its part, its mesh and its bodies, where it was that object itself.)
TEST(RebuildJobTest, AWorkerAndTheWindowReuseEachOthersExactCounterpart) {
    {
        SCOPED_TRACE("the window's build first");
        const auto part = partOf(importedCylinderInInches(), "cylinder.step");
        ASSERT_TRUE(part->rebuildModel()) << part->lastBuildMessage();
        ASSERT_NE(part->exactSolid(), nullptr) << part->exactNote();
        const hz::topo::Solid* const shown = part->exactSolid();
        const auto mesh = part->exactMesh();
        hz::ui::RebuildJob job(*part);
        hz::doc::BuildResult result = ranOnItsWorker(job);
        EXPECT_EQ(result.failedFeatureIndex, -1) << result.failureMessage;
        ASSERT_TRUE(part->applyBuild(std::move(result))) << part->lastBuildMessage();
        EXPECT_EQ(part->exactSolid(), shown) << "the window's counterpart, reused";
        EXPECT_EQ(part->exactMesh(), mesh);
    }
    {
        SCOPED_TRACE("a worker's build first");
        const auto part = partOf(importedCylinderInInches(), "cylinder.step");
        hz::ui::RebuildJob job(*part);
        ASSERT_TRUE(part->applyBuild(ranOnItsWorker(job))) << part->lastBuildMessage();
        ASSERT_NE(part->exactSolid(), nullptr) << part->exactNote();
        const hz::topo::Solid* const shown = part->exactSolid();
        const auto mesh = part->exactMesh();
        ASSERT_TRUE(part->rebuildModel()) << part->lastBuildMessage();
        EXPECT_EQ(part->exactSolid(), shown) << "the worker's counterpart, reused";
        EXPECT_EQ(part->exactMesh(), mesh);
    }
}

// A worker's rebuild of a part made of imports reads no STEP (I4): its copy's bodies are the
// document's own. A body kept without its exact form, its carriers and a note saying why (as a
// part file saved before stage 2 may keep one), is the body the worker builds, and the counterpart
// the window made of it, which says why it is shown from facets, is the worker's too. Its STEP read
// again would have come in exact: a box's faces are planes.
TEST(RebuildJobTest, AWorkersRebuildOfImportsReadsNoStep) {
    const std::string note =
        "imported body \"old.step\": not exact (as it was saved); kept as it was";
    const auto part = partOf(hz::model::PrimitiveFactory::makeBox(2, 3, 4), "old.step", note);
    ASSERT_TRUE(part->rebuildModel()) << part->lastBuildMessage();
    ASSERT_EQ(part->exactSolid(), nullptr);
    EXPECT_EQ(part->exactNoteKind(), hz::doc::ExactNoteKind::ShownFromFacets);
    EXPECT_EQ(part->exactNote(), note);
    const auto made = part->exactCounterpart();
    ASSERT_NE(made, nullptr);

    hz::ui::RebuildJob job(*part);
    hz::doc::BuildResult result = ranOnItsWorker(job);
    EXPECT_EQ(result.exact, made) << "the window's counterpart, made of the same body";
    ASSERT_TRUE(part->applyBuild(std::move(result))) << part->lastBuildMessage();
    EXPECT_EQ(part->exactSolid(), nullptr) << "a box read again from its STEP comes in exact";
    EXPECT_EQ(part->exactNoteKind(), hz::doc::ExactNoteKind::ShownFromFacets);
    EXPECT_EQ(part->exactNote(), note);
    EXPECT_NEAR(volumeOf(*part), 24.0, 1e-9);
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
    // Under load (ctest --parallel) the worker can finish before Cancel is
    // clicked, and its build is applied: then there is nothing to test.
    if (part.solid() != nullptr) GTEST_SKIP() << "the rebuild ended before Cancel was clicked";

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
