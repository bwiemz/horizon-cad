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

    std::vector<hz::model::SolverComponent> solverComponents;
    for (auto& comp : asmDoc->components()) {
        ASSERT_TRUE(mgr.resolveComponent(comp, ComponentState::Resolved, asmDir));
        hz::model::SolverComponent sc;
        sc.id = comp.id;
        sc.transform = comp.transform;
        solverComponents.push_back(sc);
    }

    std::vector<hz::model::SolverMate> solverMates;
    for (const auto& mate : asmDoc->mates()) {
        hz::model::SolverMate sm;
        sm.type = mate.type;
        sm.componentA = mate.a.componentId;
        sm.componentB = mate.b.componentId;
        sm.value = mate.value;
        if (mate.type != MateType::Fixed) {
            for (auto [ref, frame] :
                 {std::pair{&mate.a, &sm.frameA}, std::pair{&mate.b, &sm.frameB}}) {
                const auto* comp = asmDoc->component(ref->componentId);
                ASSERT_NE(comp, nullptr);
                ASSERT_NE(comp->resolvedPart, nullptr);
                const auto* face =
                    hz::model::MateGeometry::findFace(*comp->resolvedPart->solid(), ref->faceId);
                ASSERT_NE(face, nullptr) << "face " << ref->faceId.tag() << " not found";
                auto extracted = hz::model::MateGeometry::frameForFace(*face);
                ASSERT_TRUE(extracted.has_value());
                *frame = *extracted;
            }
        }
        solverMates.push_back(sm);
    }

    hz::model::AssemblySolver solver;
    auto result = solver.solve(solverComponents, solverMates);
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
    ASSERT_TRUE(localFrame.has_value());
    auto placed = localFrame->transformed(result.transforms.at(topComp.id));
    EXPECT_NEAR(placed.origin.z, 2.0, 1e-6);
    EXPECT_NEAR(std::abs(placed.direction.z), 1.0, 1e-6);

    fs::remove_all(dir);
}
