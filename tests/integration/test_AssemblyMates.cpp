// Integration test for Phase 42: the full mate workflow.
//
// Authors a plate part, builds an assembly with two instances mated
// face-to-face, saves and reloads the assembly, then solves the mates via
// frames extracted from the parts' B-Rep — verifying the stack height
// analytically.

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/AssemblyMates.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/AssemblySolver.h"
#include "horizon/modeling/MateGeometry.h"

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

void wireToNativeFormat(DocumentManager& mgr) {
    mgr.setPartLoader(
        [](const std::string& path, Document& doc) { return NativeFormat::load(path, doc); });
    mgr.setMeshLoader([](const std::string& path) { return NativeFormat::loadPartMesh(path); });
    mgr.setAssemblyLoader([](const std::string& path, AssemblyDocument& doc) {
        return NativeFormat::loadAssembly(path, doc);
    });
}

}  // namespace

TEST(AssemblyMatesIntegrationTest, MatedStackSolvesAfterReload) {
    fs::path dir = fs::temp_directory_path() / "hz_test_asm_mates";
    fs::create_directories(dir);

    const std::string partPath = (dir / "plate.hzpart").string();
    const std::string asmPath = (dir / "stack.hzasm").string();

    // --- Author a 20x10x2 plate part -----------------------------------
    std::string featureTag;
    {
        Document part;
        part.setType(DocumentType::Part);
        auto sketch = makeRectSketch(20.0, 10.0);
        sketch->setName("PlateProfile");
        part.addSketch(sketch);
        auto feature = std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 2.0);
        featureTag = feature->featureID();
        part.featureTree().addFeature(std::move(feature));
        ASSERT_TRUE(part.rebuildModel());
        ASSERT_TRUE(NativeFormat::save(partPath, part));
    }

    // --- Author an assembly: two plates, top of A mated to bottom of B --
    {
        AssemblyDocument asmDoc;

        ComponentInstance bottom;
        bottom.name = "plate-bottom";
        bottom.partPath = partPath;
        uint64_t bottomId = asmDoc.addComponent(bottom);

        ComponentInstance top;
        top.name = "plate-top";
        top.partPath = partPath;
        // Deliberately misplaced: the mate solve must stack it.
        top.transform = Mat4::translation(Vec3(7, -3, 40)) * Mat4::rotationX(0.25);
        uint64_t topId = asmDoc.addComponent(top);

        Mate ground;
        ground.type = MateType::Fixed;
        ground.a.componentId = bottomId;
        asmDoc.addMate(ground);

        Mate stack;
        stack.type = MateType::Coincident;
        stack.a = {bottomId, hz::topo::TopologyID::make(featureTag, "cap_top")};
        stack.b = {topId, hz::topo::TopologyID::make(featureTag, "cap_bottom")};
        asmDoc.addMate(stack);

        ASSERT_TRUE(NativeFormat::saveAssembly(asmPath, asmDoc));
    }

    // --- Reload and solve (mirrors the application flow) -----------------
    DocumentManager mgr;
    wireToNativeFormat(mgr);

    auto asmDoc = mgr.openAssembly(asmPath);
    ASSERT_NE(asmDoc, nullptr);
    ASSERT_EQ(asmDoc->mates().size(), 2u);

    const std::string asmDir = fs::path(asmPath).parent_path().string();

    for (auto& comp : asmDoc->components()) {
        ASSERT_TRUE(mgr.resolveComponent(comp, ComponentState::Resolved, asmDir));
    }
    // The solve the application runs (Phase 158: one for everyone).
    std::string why;
    const auto mates = hz::doc::AssemblyMates::gather(*asmDoc, &why);
    if (!mates) FAIL() << why;
    const auto result = mates->solve();
    ASSERT_EQ(result.status, hz::model::AssemblySolveStatus::Success)
        << "iterations=" << result.iterations << " residual=" << result.residualNorm
        << " message=" << result.message;
    // Grounding came from the Fixed mate, not from convention.
    EXPECT_TRUE(result.message.empty());

    // The top plate's bottom face must land on z = 2 (the bottom plate's
    // top face) with a -Z normal.
    const auto& topComp = asmDoc->components()[1];
    const auto* face = hz::model::MateGeometry::findFace(
        *topComp.resolvedPart->solid(), hz::topo::TopologyID::make(featureTag, "cap_bottom"));
    ASSERT_NE(face, nullptr);
    auto localFrame = hz::model::MateGeometry::frameForFace(*face);
    if (!localFrame) FAIL() << "no frame for the top plate's bottom";
    auto placed = localFrame->transformed(result.transforms.at(topComp.id));
    EXPECT_NEAR(placed.origin.z, 2.0, 1e-6);
    EXPECT_NEAR(std::abs(placed.direction.z), 1.0, 1e-6);

    fs::remove_all(dir);
}

// One way to place an assembly by its mates, for the Assembly menu and the AI
// assistant both. Only Fixed mates leave nothing to solve: that is placed as
// it is, not a failure (the Add Mate form refused a first, Fixed mate, as
// "did not converge").
TEST(AssemblyMatesIntegrationTest, PlaceByMatesHoldsAFixedMateAlone) {
    fs::path dir = fs::temp_directory_path() / "hz_test_asm_place_by_mates";
    fs::create_directories(dir);
    const std::string partPath = (dir / "plate.hzpart").string();
    std::string featureTag;
    {
        Document part;
        part.setType(DocumentType::Part);
        auto sketch = makeRectSketch(20.0, 10.0);
        part.addSketch(sketch);
        auto feature = std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 2.0);
        featureTag = feature->featureID();
        part.featureTree().addFeature(std::move(feature));
        ASSERT_TRUE(part.rebuildModel());
        ASSERT_TRUE(NativeFormat::save(partPath, part));
    }
    DocumentManager mgr;
    wireToNativeFormat(mgr);
    AssemblyDocument asmDoc;
    asmDoc.setFilePath((dir / "rig.hzasm").string());
    ComponentInstance first;
    first.partPath = "plate.hzpart";  // from the assembly's folder
    first.transform = Mat4::translation(Vec3(5, 6, 7));
    const uint64_t a = asmDoc.addComponent(first);
    ComponentInstance second;
    second.partPath = partPath;
    second.transform = Mat4::translation(Vec3(0, 0, 40));
    const uint64_t b = asmDoc.addComponent(second);

    Mate ground;
    ground.type = MateType::Fixed;
    ground.a.componentId = a;
    asmDoc.addMate(ground);
    MatePlacement placement = placeByMates(asmDoc, mgr);
    EXPECT_TRUE(placement.placed) << placement.why;
    EXPECT_NE(asmDoc.component(a)->resolvedPart, nullptr);  // its part read whole
    EXPECT_DOUBLE_EQ(asmDoc.component(a)->transform.at(2, 3), 7.0);

    // A mate that holds moves b; one naming a face not there is said.
    Mate stack;
    stack.type = MateType::Coincident;
    stack.a = {a, hz::topo::TopologyID::make(featureTag, "cap_top")};
    stack.b = {b, hz::topo::TopologyID::make(featureTag, "cap_bottom")};
    asmDoc.addMate(stack);
    placement = placeByMates(asmDoc, mgr);
    EXPECT_TRUE(placement.placed) << placement.why;
    EXPECT_NEAR(asmDoc.component(b)->transform.at(2, 3), 9.0, 1e-6);

    Mate lost;
    lost.type = MateType::Coincident;
    lost.a = {a, hz::topo::TopologyID::make(featureTag, "no_such_face")};
    lost.b = {b, hz::topo::TopologyID::make(featureTag, "cap_bottom")};
    asmDoc.addMate(lost);
    placement = placeByMates(asmDoc, mgr);
    EXPECT_FALSE(placement.placed);
    EXPECT_FALSE(placement.gathered);
    EXPECT_NE(placement.why.find("could not be resolved"), std::string::npos) << placement.why;

    fs::remove_all(dir);
}
