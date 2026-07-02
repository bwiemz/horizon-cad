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
        auto mesh = std::make_shared<hz::render::MeshData>();
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

    // The resolved part is a registered open document (shared instance).
    EXPECT_EQ(mgr.documents().size(), 1u);
    EXPECT_EQ(mgr.findByPath(path), comp.resolvedPart);

    // A second instance of the same part shares the document.
    ComponentInstance comp2;
    comp2.partPath = path;
    EXPECT_TRUE(mgr.resolveComponent(comp2, ComponentState::Resolved));
    EXPECT_EQ(comp2.resolvedPart, comp.resolvedPart);
    EXPECT_EQ(mgr.documents().size(), 1u);

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
