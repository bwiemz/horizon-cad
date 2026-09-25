#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"

using namespace hz::doc;
using hz::math::Vec2;
using hz::math::Vec3;

namespace fs = std::filesystem;

namespace {

// Helper: build a sketch with a rectangle profile.
std::shared_ptr<Sketch> makeRectSketch(double w, double h) {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(w, 0)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(w, 0), Vec2(w, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(w, h), Vec2(0, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, h), Vec2(0, 0)));
    return sketch;
}

// A part loader that populates the document with a 10x5x3 box part.
bool fakeBoxPartLoader(const std::string& /*path*/, Document& doc) {
    auto sketch = makeRectSketch(10.0, 5.0);
    doc.addSketch(sketch);
    doc.featureTree().addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 3.0));
    doc.setType(DocumentType::Part);
    return true;
}

// Unique temp file helper (the file must exist for canonical-path dedup
// and mtime watching to engage).
std::string makeTempFile(const std::string& name) {
    fs::path path = fs::temp_directory_path() / name;
    std::ofstream out(path);
    out << "{}";
    return path.string();
}

}  // namespace

// ---------------------------------------------------------------------------
// NewDocumentSetsType
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, NewDocumentSetsType) {
    DocumentManager mgr;
    auto drawing = mgr.newDocument(DocumentType::Drawing);
    auto part = mgr.newDocument(DocumentType::Part);
    auto assembly = mgr.newAssembly();

    ASSERT_NE(drawing, nullptr);
    ASSERT_NE(part, nullptr);
    ASSERT_NE(assembly, nullptr);
    EXPECT_EQ(drawing->type(), DocumentType::Drawing);
    EXPECT_EQ(part->type(), DocumentType::Part);
    EXPECT_EQ(mgr.documents().size(), 2u);
    EXPECT_EQ(mgr.assemblies().size(), 1u);
}

// ---------------------------------------------------------------------------
// OpenPartRequiresLoader
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, OpenPartRequiresLoader) {
    DocumentManager mgr;
    EXPECT_EQ(mgr.openPart("/nonexistent/part.hzpart"), nullptr);
}

// ---------------------------------------------------------------------------
// OpenPartDeduplicatesByPath
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, OpenPartDeduplicatesByPath) {
    DocumentManager mgr;
    mgr.setPartLoader(fakeBoxPartLoader);

    std::string path = makeTempFile("hz_test_dedupe.hzpart");

    auto first = mgr.openPart(path);
    ASSERT_NE(first, nullptr);
    EXPECT_EQ(first->type(), DocumentType::Part);

    // Same file again — same instance, no duplicate registration.
    auto second = mgr.openPart(path);
    EXPECT_EQ(first, second);
    EXPECT_EQ(mgr.documents().size(), 1u);

    // Also via a non-canonical spelling of the same path.
    fs::path indirect = fs::path(path).parent_path() / "." / fs::path(path).filename();
    auto third = mgr.openPart(indirect.string());
    EXPECT_EQ(first, third);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// FindByPath
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, FindByPath) {
    DocumentManager mgr;
    mgr.setPartLoader(fakeBoxPartLoader);

    std::string path = makeTempFile("hz_test_find.hzpart");
    auto doc = mgr.openPart(path);
    ASSERT_NE(doc, nullptr);

    EXPECT_EQ(mgr.findByPath(path), doc);
    EXPECT_EQ(mgr.findByPath("/does/not/exist.hzpart"), nullptr);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// CloseDocumentUnregisters
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, CloseDocumentUnregisters) {
    DocumentManager mgr;
    mgr.setPartLoader(fakeBoxPartLoader);

    std::string path = makeTempFile("hz_test_close.hzpart");
    auto doc = mgr.openPart(path);
    ASSERT_NE(doc, nullptr);

    EXPECT_TRUE(mgr.closeDocument(doc));
    EXPECT_TRUE(mgr.documents().empty());
    EXPECT_EQ(mgr.findByPath(path), nullptr);
    EXPECT_FALSE(mgr.closeDocument(doc));

    // Re-opening after close creates a fresh instance.
    auto reopened = mgr.openPart(path);
    ASSERT_NE(reopened, nullptr);
    EXPECT_NE(reopened, doc);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// PollExternalChangesDetectsMtimeChange
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, PollExternalChangesDetectsMtimeChange) {
    DocumentManager mgr;
    mgr.setPartLoader(fakeBoxPartLoader);

    std::string path = makeTempFile("hz_test_extchange.hzpart");
    auto doc = mgr.openPart(path);
    ASSERT_NE(doc, nullptr);

    // Nothing changed yet.
    EXPECT_TRUE(mgr.pollExternalChanges().empty());

    // Simulate an external writer: bump the file's mtime.
    auto newTime = fs::last_write_time(path) + std::chrono::seconds(2);
    fs::last_write_time(path, newTime);

    std::string notifiedPath;
    mgr.setExternalChangeCallback([&](const std::string& p) { notifiedPath = p; });

    auto changed = mgr.pollExternalChanges();
    ASSERT_EQ(changed.size(), 1u);
    EXPECT_FALSE(notifiedPath.empty());

    // A change reports exactly once.
    EXPECT_TRUE(mgr.pollExternalChanges().empty());

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// NoteSavedSuppressesSelfChange
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, NoteSavedSuppressesSelfChange) {
    DocumentManager mgr;
    auto doc = mgr.newDocument(DocumentType::Part);

    std::string path = makeTempFile("hz_test_notesaved.hzpart");
    doc->setFilePath(path);
    mgr.noteSaved(doc);

    // The manager now dedups by the saved path...
    EXPECT_EQ(mgr.findByPath(path), doc);
    // ...and the save itself is not reported as an external change.
    EXPECT_TRUE(mgr.pollExternalChanges().empty());

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// ResolveComponentLightweightUsesLoadedMeshOnly
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, ResolveComponentLightweightUsesMeshLoader) {
    DocumentManager mgr;

    bool meshLoaderCalled = false;
    mgr.setMeshLoader([&](const std::string&) {
        meshLoaderCalled = true;
        auto mesh = std::make_shared<hz::geo::MeshData>();
        mesh->positions = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        mesh->normals = {0, 0, 1, 0, 0, 1, 0, 0, 1};
        mesh->indices = {0, 1, 2};
        return mesh;
    });

    ComponentInstance comp;
    comp.partPath = "widget.hzpart";

    EXPECT_TRUE(mgr.resolveComponent(comp, ComponentState::Lightweight, "/tmp"));
    EXPECT_TRUE(meshLoaderCalled);
    ASSERT_NE(comp.cachedMesh, nullptr);
    EXPECT_EQ(comp.cachedMesh->indices.size(), 3u);
    EXPECT_EQ(comp.state, ComponentState::Lightweight);
    // Lightweight resolution never loads the feature tree.
    EXPECT_EQ(comp.resolvedPart, nullptr);
}

// ---------------------------------------------------------------------------
// ResolveComponentLightweightFallsBackToFullLoad
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, ResolveComponentLightweightFallsBackToFullLoad) {
    DocumentManager mgr;
    // No mesh loader — only the part loader is available, so lightweight
    // resolution must fall back to a temporary full load + tessellation.
    mgr.setPartLoader(fakeBoxPartLoader);

    ComponentInstance comp;
    comp.partPath = "box.hzpart";

    EXPECT_TRUE(mgr.resolveComponent(comp, ComponentState::Lightweight, "/tmp"));
    ASSERT_NE(comp.cachedMesh, nullptr);
    EXPECT_FALSE(comp.cachedMesh->positions.empty());
    EXPECT_FALSE(comp.cachedMesh->indices.empty());
    // The temporary document is not registered as open.
    EXPECT_TRUE(mgr.documents().empty());
}

// ---------------------------------------------------------------------------
// ResolveComponentResolvedLoadsFeatureTree
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, ResolveComponentResolvedLoadsFeatureTree) {
    DocumentManager mgr;
    mgr.setPartLoader(fakeBoxPartLoader);

    std::string path = makeTempFile("hz_test_resolved.hzpart");

    ComponentInstance comp;
    comp.partPath = path;  // absolute

    EXPECT_TRUE(mgr.resolveComponent(comp, ComponentState::Resolved));
    ASSERT_NE(comp.resolvedPart, nullptr);
    EXPECT_EQ(comp.state, ComponentState::Resolved);
    EXPECT_GT(comp.resolvedPart->featureTree().featureCount(), 0u);
    // The model was rebuilt and tessellated for display.
    EXPECT_NE(comp.resolvedPart->solid(), nullptr);
    ASSERT_NE(comp.cachedMesh, nullptr);
    EXPECT_FALSE(comp.cachedMesh->positions.empty());

    // The resolved part is found by its path (a shared instance), and held
    // by its components alone: it is not a document kept open (Phase 138).
    EXPECT_TRUE(mgr.documents().empty());
    EXPECT_EQ(mgr.findByPath(path), comp.resolvedPart);

    // A second instance of the same part shares the document, and the mesh.
    ComponentInstance comp2;
    comp2.partPath = path;
    EXPECT_TRUE(mgr.resolveComponent(comp2, ComponentState::Resolved));
    EXPECT_EQ(comp2.resolvedPart, comp.resolvedPart);
    EXPECT_EQ(comp2.cachedMesh, comp.cachedMesh) << "tessellated once, for both";

    // Opened in a tab too, it is kept; let go of by the components alone, it
    // goes.
    EXPECT_EQ(mgr.openPart(path), comp.resolvedPart);
    EXPECT_EQ(mgr.documents().size(), 1u);
    ASSERT_TRUE(mgr.closeDocument(comp.resolvedPart));
    comp.resolvedPart.reset();
    comp2.resolvedPart.reset();
    EXPECT_EQ(mgr.findByPath(path), nullptr) << "released";

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// ResolveComponentFailsWithoutLoaders
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, ResolveComponentFailsWithoutLoaders) {
    DocumentManager mgr;

    ComponentInstance comp;
    comp.partPath = "missing.hzpart";

    EXPECT_FALSE(mgr.resolveComponent(comp, ComponentState::Lightweight, "/tmp"));
    EXPECT_FALSE(mgr.resolveComponent(comp, ComponentState::Resolved, "/tmp"));
    EXPECT_EQ(comp.cachedMesh, nullptr);
    EXPECT_EQ(comp.resolvedPart, nullptr);
}

// ---------------------------------------------------------------------------
// NoteSavedUnregistersPreviousPath
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, NoteSavedUnregistersPreviousPath) {
    DocumentManager mgr;
    mgr.setPartLoader(fakeBoxPartLoader);

    std::string pathA = makeTempFile("hz_test_saveas_a.hzpart");
    std::string pathB = makeTempFile("hz_test_saveas_b.hzpart");

    auto doc = mgr.openPart(pathA);
    ASSERT_NE(doc, nullptr);

    // Save As: the UI mutates the file path, then notifies the manager.
    doc->setFilePath(pathB);
    mgr.noteSaved(doc);

    // The old path no longer resolves to the renamed document...
    EXPECT_EQ(mgr.findByPath(pathA), nullptr);
    // ...and reopening it loads a FRESH document instead of aliasing.
    auto reopened = mgr.openPart(pathA);
    ASSERT_NE(reopened, nullptr);
    EXPECT_NE(reopened, doc);
    // The new path resolves to the renamed document.
    EXPECT_EQ(mgr.findByPath(pathB), doc);

    std::remove(pathA.c_str());
    std::remove(pathB.c_str());
}

// ---------------------------------------------------------------------------
// CloseDocumentKeepsUnrelatedRegistrations
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, CloseDocumentKeepsUnrelatedRegistrations) {
    DocumentManager mgr;
    mgr.setPartLoader(fakeBoxPartLoader);

    std::string pathA = makeTempFile("hz_test_close_a.hzpart");
    std::string pathB = makeTempFile("hz_test_close_b.hzpart");

    auto docA = mgr.openPart(pathA);
    auto docB = mgr.openPart(pathB);
    ASSERT_NE(docA, nullptr);
    ASSERT_NE(docB, nullptr);

    // docA was renamed onto B's path (Save As over an open file), then the
    // registry must not lose docB's registration when docA closes.
    docA->setFilePath(pathB);
    ASSERT_TRUE(mgr.closeDocument(docA));

    EXPECT_EQ(mgr.findByPath(pathB), docB);

    std::remove(pathA.c_str());
    std::remove(pathB.c_str());
}

// ---------------------------------------------------------------------------
// ResolvedThenLightweightReleasesFeatureTree
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, ResolvedThenLightweightReleasesFeatureTree) {
    DocumentManager mgr;
    mgr.setPartLoader(fakeBoxPartLoader);

    std::string path = makeTempFile("hz_test_demote.hzpart");

    ComponentInstance comp;
    comp.partPath = path;

    ASSERT_TRUE(mgr.resolveComponent(comp, ComponentState::Resolved));
    ASSERT_NE(comp.resolvedPart, nullptr);

    // Demoting to Lightweight must release the full part document
    // (Lightweight = cached tessellation + transform only).
    ASSERT_TRUE(mgr.resolveComponent(comp, ComponentState::Lightweight));
    EXPECT_EQ(comp.resolvedPart, nullptr);
    EXPECT_NE(comp.cachedMesh, nullptr);
    EXPECT_EQ(comp.state, ComponentState::Lightweight);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// Living assemblies (Phase 144): a part changed on disk is read again.
// ---------------------------------------------------------------------------

TEST(DocumentManagerTest, SamePathSeesThroughSpelling) {
    const std::string path = makeTempFile("hz_test_samepath.hzpart");
    const fs::path file(path);
    const std::string roundabout =
        (file.parent_path() / "nowhere" / ".." / file.filename()).string();
    EXPECT_TRUE(DocumentManager::samePath(path, roundabout));
    EXPECT_FALSE(DocumentManager::samePath(path, path + ".other"));
    EXPECT_FALSE(DocumentManager::samePath(path, ""));
    std::remove(path.c_str());
}

TEST(DocumentManagerTest, AReleasedPartIsReadAgain) {
    DocumentManager mgr;
    int reads = 0;
    mgr.setPartLoader([&reads](const std::string& path, Document& doc) {
        ++reads;
        return fakeBoxPartLoader(path, doc);
    });
    const std::string path = makeTempFile("hz_test_release.hzpart");

    ComponentInstance first;
    first.partPath = path;
    ASSERT_TRUE(mgr.resolveComponent(first, ComponentState::Resolved));
    ComponentInstance second;
    second.partPath = path;
    ASSERT_TRUE(mgr.resolveComponent(second, ComponentState::Resolved));
    EXPECT_EQ(reads, 1) << "one read while a component holds it";

    // Changed on disk: released, the next component reads the file again.
    mgr.releasePart(path);
    EXPECT_EQ(mgr.findByPath(path), nullptr);
    ComponentInstance third;
    third.partPath = path;
    ASSERT_TRUE(mgr.resolveComponent(third, ComponentState::Resolved));
    EXPECT_EQ(reads, 2);
    EXPECT_NE(third.resolvedPart, first.resolvedPart);
    EXPECT_EQ(first.resolvedPart, second.resolvedPart) << "what they had, they keep";

    std::remove(path.c_str());
}

TEST(DocumentManagerTest, APartOpenInATabIsNotReleased) {
    DocumentManager mgr;
    mgr.setPartLoader(fakeBoxPartLoader);
    const std::string path = makeTempFile("hz_test_release_tab.hzpart");

    const auto tab = mgr.openPart(path);
    ASSERT_NE(tab, nullptr);
    mgr.releasePart(path);
    EXPECT_EQ(mgr.findByPath(path), tab) << "the tab's document is the part";

    std::remove(path.c_str());
}

TEST(DocumentManagerTest, ALightweightComponentsFileIsWatched) {
    DocumentManager mgr;
    mgr.setMeshLoader([](const std::string&) {
        auto mesh = std::make_shared<hz::geo::MeshData>();
        mesh->positions = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        mesh->normals = {0, 0, 1, 0, 0, 1, 0, 0, 1};
        mesh->indices = {0, 1, 2};
        return mesh;
    });
    const std::string path = makeTempFile("hz_test_watch_light.hzpart");

    ComponentInstance comp;
    comp.partPath = path;
    ASSERT_TRUE(mgr.resolveComponent(comp, ComponentState::Lightweight));
    EXPECT_TRUE(mgr.pollExternalChanges().empty());

    fs::last_write_time(path, fs::last_write_time(path) + std::chrono::seconds(2));
    const auto changed = mgr.pollExternalChanges();
    ASSERT_EQ(changed.size(), 1u);
    EXPECT_TRUE(DocumentManager::samePath(changed.front(), path));

    std::remove(path.c_str());
}

TEST(DocumentManagerTest, APlacedPartStaysWatchedWhenItsTabCloses) {
    DocumentManager mgr;
    mgr.setPartLoader(fakeBoxPartLoader);
    const std::string path = makeTempFile("hz_test_watch_placed.hzpart");

    ComponentInstance comp;
    comp.partPath = path;
    ASSERT_TRUE(mgr.resolveComponent(comp, ComponentState::Resolved));
    const auto tab = mgr.openPart(path);
    ASSERT_EQ(tab, comp.resolvedPart) << "the component's part, opened in a tab";
    ASSERT_TRUE(mgr.closeDocument(tab));

    fs::last_write_time(path, fs::last_write_time(path) + std::chrono::seconds(2));
    EXPECT_EQ(mgr.pollExternalChanges().size(), 1u) << "the assembly still places it";

    std::remove(path.c_str());
}

// A part read again after a change is tessellated again, even while an
// older mesh of it is still held (an undo step holds one) and the new read
// has been built as many times as the old one had.
TEST(DocumentManagerTest, APartReadAgainIsNotShownWithItsOldMesh) {
    DocumentManager mgr;
    double depth = 3.0;
    mgr.setPartLoader([&depth](const std::string&, Document& doc) {
        auto sketch = makeRectSketch(10.0, 5.0);
        doc.addSketch(sketch);
        doc.featureTree().addFeature(
            std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), depth));
        doc.setType(DocumentType::Part);
        return true;
    });
    const std::string path = makeTempFile("hz_test_reread_mesh.hzpart");
    const auto height = [](const ComponentInstance& c) {
        double high = 0.0;
        for (size_t i = 2; c.cachedMesh && i < c.cachedMesh->positions.size(); i += 3) {
            high = std::max(high, static_cast<double>(c.cachedMesh->positions[i]));
        }
        return high;
    };

    ComponentInstance before;
    before.partPath = path;
    ASSERT_TRUE(mgr.resolveComponent(before, ComponentState::Resolved));
    EXPECT_NEAR(height(before), 3.0, 1e-9);

    depth = 7.0;  // the file changed
    mgr.releasePart(path);
    ComponentInstance after;
    after.partPath = path;
    ASSERT_TRUE(mgr.resolveComponent(after, ComponentState::Resolved));
    ASSERT_NE(after.resolvedPart, before.resolvedPart);
    ASSERT_EQ(after.resolvedPart->builds(), before.resolvedPart->builds());
    EXPECT_NEAR(height(after), 7.0, 1e-9) << "not the mesh of the part as it was";

    std::remove(path.c_str());
}
