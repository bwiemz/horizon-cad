// Integration test for Phase 41: the parts → assembly workflow.
//
// Builds a part, saves it as .hzpart (with tessellation cache), references
// it twice from an assembly saved as .hzasm, then reloads everything through
// a DocumentManager wired to NativeFormat — exactly how the application
// wires it — exercising both lightweight and resolved component resolution.

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>

#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/NativeFormat.h"

using namespace hz::doc;
using hz::io::NativeFormat;
using hz::math::Mat4;
using hz::math::Vec2;
using hz::math::Vec3;

namespace fs = std::filesystem;

namespace {

std::shared_ptr<Sketch> makeRectSketch(double w, double h) {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(w, 0)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(w, 0), Vec2(w, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(w, h), Vec2(0, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, h), Vec2(0, 0)));
    return sketch;
}

// Wire a DocumentManager to the real file format, as the application does.
void wireToNativeFormat(DocumentManager& mgr) {
    mgr.setPartLoader(
        [](const std::string& path, Document& doc) { return NativeFormat::load(path, doc); });
    mgr.setMeshLoader([](const std::string& path) { return NativeFormat::loadPartMesh(path); });
    mgr.setAssemblyLoader([](const std::string& path, AssemblyDocument& doc) {
        return NativeFormat::loadAssembly(path, doc);
    });
}

}  // namespace

TEST(MultiDocumentIntegrationTest, PartToAssemblyWorkflow) {
    fs::path dir = fs::temp_directory_path() / "hz_test_multidoc";
    fs::create_directories(dir);

    const std::string partPath = (dir / "plate.hzpart").string();
    const std::string asmPath = (dir / "stack.hzasm").string();

    // --- Author a part and save it (with tessellation cache) ---
    {
        Document part;
        part.setType(DocumentType::Part);
        auto sketch = makeRectSketch(20.0, 10.0);
        sketch->setName("PlateProfile");
        part.addSketch(sketch);
        part.featureTree().addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 2.0));
        ASSERT_TRUE(part.rebuildModel());
        ASSERT_NE(part.solid(), nullptr);
        ASSERT_TRUE(NativeFormat::save(partPath, part));
    }

    // --- Author an assembly referencing the part twice ---
    {
        AssemblyDocument asmDoc;

        ComponentInstance bottom;
        bottom.name = "plate-bottom";
        bottom.partPath = partPath;
        asmDoc.addComponent(bottom);

        ComponentInstance top;
        top.name = "plate-top";
        top.partPath = partPath;
        top.transform = Mat4::translation(Vec3(0, 0, 2.0));
        asmDoc.addComponent(top);

        ASSERT_TRUE(NativeFormat::saveAssembly(asmPath, asmDoc));
    }

    // --- Reload through a DocumentManager (fresh session) ---
    DocumentManager mgr;
    wireToNativeFormat(mgr);

    auto asmDoc = mgr.openAssembly(asmPath);
    ASSERT_NE(asmDoc, nullptr);
    ASSERT_EQ(asmDoc->components().size(), 2u);

    const std::string asmDir = fs::path(asmPath).parent_path().string();

    // Lightweight resolution: display meshes only, no feature trees loaded.
    for (auto& comp : asmDoc->components()) {
        ASSERT_TRUE(mgr.resolveComponent(comp, ComponentState::Lightweight, asmDir)) << comp.name;
        ASSERT_NE(comp.cachedMesh, nullptr);
        EXPECT_FALSE(comp.cachedMesh->positions.empty());
        EXPECT_FALSE(comp.cachedMesh->indices.empty());
        EXPECT_EQ(comp.resolvedPart, nullptr);
    }
    EXPECT_TRUE(mgr.documents().empty()) << "lightweight resolution must not open part documents";

    // Resolved mode ("loaded on edit"): full feature tree, shared instance.
    auto& first = asmDoc->components()[0];
    auto& second = asmDoc->components()[1];
    ASSERT_TRUE(mgr.resolveComponent(first, ComponentState::Resolved, asmDir));
    ASSERT_TRUE(mgr.resolveComponent(second, ComponentState::Resolved, asmDir));

    ASSERT_NE(first.resolvedPart, nullptr);
    EXPECT_EQ(first.resolvedPart, second.resolvedPart)
        << "both instances must share one open part document";
    EXPECT_EQ(mgr.documents().size(), 1u);
    EXPECT_EQ(first.resolvedPart->type(), DocumentType::Part);
    EXPECT_EQ(first.resolvedPart->featureTree().featureCount(), 1u);
    EXPECT_NE(first.resolvedPart->solid(), nullptr);

    // Editing the shared part: change the extrude distance and rebuild.
    auto* feature = first.resolvedPart->featureTree().feature(0);
    ASSERT_TRUE(feature->setParameter("distance", 4.0));
    EXPECT_TRUE(first.resolvedPart->rebuildModel());
    EXPECT_NE(first.resolvedPart->solid(), nullptr);

    // External change notification: another process rewrites the part file.
    {
        Document part;
        ASSERT_TRUE(NativeFormat::load(partPath, part));
        auto newTime = fs::last_write_time(partPath) + std::chrono::seconds(2);
        fs::last_write_time(partPath, newTime);
    }
    auto changed = mgr.pollExternalChanges();
    ASSERT_EQ(changed.size(), 1u);
    EXPECT_NE(changed[0].find("plate.hzpart"), std::string::npos);

    fs::remove_all(dir);
}
