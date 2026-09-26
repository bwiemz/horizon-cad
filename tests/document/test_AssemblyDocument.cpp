#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/AssemblyMates.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/math/Mat4.h"
#include "horizon/topology/Solid.h"

using namespace hz::doc;
using hz::math::Mat4;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// AddComponentAssignsUniqueIds
// ---------------------------------------------------------------------------

TEST(AssemblyDocumentTest, AddComponentAssignsUniqueIds) {
    AssemblyDocument asmDoc;

    ComponentInstance a;
    a.name = "bolt-1";
    a.partPath = "bolt.hzpart";
    uint64_t idA = asmDoc.addComponent(a);

    ComponentInstance b;
    b.name = "bolt-2";
    b.partPath = "bolt.hzpart";
    uint64_t idB = asmDoc.addComponent(b);

    EXPECT_NE(idA, 0u);
    EXPECT_NE(idB, 0u);
    EXPECT_NE(idA, idB);
    EXPECT_EQ(asmDoc.components().size(), 2u);
}

// ---------------------------------------------------------------------------
// ExplicitIdIsKeptAndCounterAdvances
// ---------------------------------------------------------------------------

TEST(AssemblyDocumentTest, ExplicitIdIsKeptAndCounterAdvances) {
    AssemblyDocument asmDoc;

    ComponentInstance a;
    a.id = 42;
    EXPECT_EQ(asmDoc.addComponent(a), 42u);

    // The next auto-assigned id must not collide with the explicit one.
    ComponentInstance b;
    uint64_t idB = asmDoc.addComponent(b);
    EXPECT_GT(idB, 42u);
}

// ---------------------------------------------------------------------------
// FindAndRemoveComponent
// ---------------------------------------------------------------------------

TEST(AssemblyDocumentTest, FindAndRemoveComponent) {
    AssemblyDocument asmDoc;

    ComponentInstance a;
    a.name = "bracket";
    uint64_t id = asmDoc.addComponent(a);

    ComponentInstance* found = asmDoc.component(id);
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->name, "bracket");

    EXPECT_TRUE(asmDoc.removeComponent(id));
    EXPECT_EQ(asmDoc.component(id), nullptr);
    EXPECT_FALSE(asmDoc.removeComponent(id));
}

// ---------------------------------------------------------------------------
// TransformIsStored
// ---------------------------------------------------------------------------

TEST(AssemblyDocumentTest, TransformIsStored) {
    AssemblyDocument asmDoc;

    ComponentInstance a;
    a.transform = Mat4::translation(Vec3(10, 20, 30));
    uint64_t id = asmDoc.addComponent(a);

    const ComponentInstance* found = asmDoc.component(id);
    ASSERT_NE(found, nullptr);
    EXPECT_DOUBLE_EQ(found->transform.at(0, 3), 10.0);
    EXPECT_DOUBLE_EQ(found->transform.at(1, 3), 20.0);
    EXPECT_DOUBLE_EQ(found->transform.at(2, 3), 30.0);
}

// ---------------------------------------------------------------------------
// DefaultStateIsLightweight
// ---------------------------------------------------------------------------

TEST(AssemblyDocumentTest, DefaultStateIsLightweight) {
    ComponentInstance a;
    EXPECT_EQ(a.state, ComponentState::Lightweight);
    EXPECT_EQ(a.cachedMesh, nullptr);
    EXPECT_EQ(a.resolvedPart, nullptr);
    EXPECT_FALSE(a.suppressed);
}

// ---------------------------------------------------------------------------
// DirtyTracking
// ---------------------------------------------------------------------------

TEST(AssemblyDocumentTest, DirtyTracking) {
    AssemblyDocument asmDoc;
    EXPECT_FALSE(asmDoc.isDirty());

    ComponentInstance a;
    uint64_t id = asmDoc.addComponent(a);
    EXPECT_TRUE(asmDoc.isDirty());

    asmDoc.setDirty(false);
    asmDoc.removeComponent(id);
    EXPECT_TRUE(asmDoc.isDirty());
}

// ---------------------------------------------------------------------------
// ClearResetsEverything
// ---------------------------------------------------------------------------

TEST(AssemblyDocumentTest, ClearResetsEverything) {
    AssemblyDocument asmDoc;
    asmDoc.setFilePath("/tmp/a.hzasm");

    ComponentInstance a;
    asmDoc.addComponent(a);
    asmDoc.clear();

    EXPECT_TRUE(asmDoc.components().empty());
    EXPECT_FALSE(asmDoc.isDirty());
    EXPECT_TRUE(asmDoc.filePath().empty());

    // Ids restart from 1 after clear.
    ComponentInstance b;
    EXPECT_EQ(asmDoc.addComponent(b), 1u);
}

// ---------------------------------------------------------------------------
// MateManagement
// ---------------------------------------------------------------------------

TEST(AssemblyDocumentTest, MateManagement) {
    AssemblyDocument asmDoc;

    ComponentInstance base;
    uint64_t baseId = asmDoc.addComponent(base);
    ComponentInstance lid;
    uint64_t lidId = asmDoc.addComponent(lid);

    Mate m;
    m.type = MateType::Coincident;
    m.a = {baseId, hz::topo::TopologyID::make("extrude_1", "cap_top")};
    m.b = {lidId, hz::topo::TopologyID::make("extrude_2", "cap_bottom")};
    uint64_t mateId = asmDoc.addMate(m);
    EXPECT_NE(mateId, 0u);

    Mate* found = asmDoc.mate(mateId);
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->type, MateType::Coincident);
    EXPECT_EQ(found->a.componentId, baseId);
    EXPECT_EQ(found->b.faceId.tag(), "extrude_2/cap_bottom");

    // Explicit ids advance the counter.
    Mate m2;
    m2.id = 50;
    EXPECT_EQ(asmDoc.addMate(m2), 50u);
    Mate m3;
    EXPECT_GT(asmDoc.addMate(m3), 50u);

    EXPECT_TRUE(asmDoc.removeMate(mateId));
    EXPECT_EQ(asmDoc.mate(mateId), nullptr);
    EXPECT_FALSE(asmDoc.removeMate(mateId));

    asmDoc.clear();
    EXPECT_TRUE(asmDoc.mates().empty());
}

// A component removed takes its mates with it (Phase 143): one left behind
// referred to nothing, and every later solve failed. The others stay.
TEST(AssemblyDocumentTest, RemovingAComponentRemovesItsMates) {
    AssemblyDocument asmDoc;
    const uint64_t base = asmDoc.addComponent(ComponentInstance{});
    const uint64_t lid = asmDoc.addComponent(ComponentInstance{});
    const uint64_t pin = asmDoc.addComponent(ComponentInstance{});
    Mate onBase;
    onBase.a = {base, hz::topo::TopologyID::make("p", "top")};
    onBase.b = {lid, hz::topo::TopologyID::make("p", "bottom")};
    Mate onPin;
    onPin.a = {lid, hz::topo::TopologyID::make("p", "top")};
    onPin.b = {pin, hz::topo::TopologyID::make("p", "bottom")};
    asmDoc.addMate(onBase);
    const uint64_t kept = asmDoc.addMate(onPin);

    const auto before = asmDoc.snapshot();
    EXPECT_TRUE(asmDoc.removeComponent(base));
    ASSERT_EQ(asmDoc.mates().size(), 1u);
    EXPECT_EQ(asmDoc.mates().front().id, kept);

    asmDoc.restore(before);
    EXPECT_EQ(asmDoc.components().size(), 3u);
    EXPECT_EQ(asmDoc.mates().size(), 2u) << "put back, mates and all";
}

// An undo puts back where components were, not the geometry they had then:
// a part changed since shows changed (Phase 144).
TEST(AssemblyDocumentTest, RestoringKeepsTheGeometryLoadedNow) {
    AssemblyDocument asmDoc;
    ComponentInstance c;
    c.partPath = "bolt.hzpart";
    c.cachedMesh = std::make_shared<hz::geo::MeshData>();
    const uint64_t id = asmDoc.addComponent(c);
    const auto before = asmDoc.snapshot();

    auto* now = asmDoc.component(id);
    ASSERT_NE(now, nullptr);
    now->transform = Mat4::translation(Vec3(5, 0, 0));
    const auto newer = std::make_shared<hz::geo::MeshData>();
    now->cachedMesh = newer;
    now->state = ComponentState::Resolved;

    asmDoc.restore(before);
    const auto* back = asmDoc.component(id);
    ASSERT_NE(back, nullptr);
    EXPECT_EQ(back->transform.transformPoint(Vec3(0, 0, 0)).x, 0.0) << "the placement goes back";
    EXPECT_EQ(back->cachedMesh, newer) << "the geometry stays as it is now";
    EXPECT_EQ(back->state, ComponentState::Resolved);

    // A component the undo brings back has the geometry it had.
    const auto kept = before.components.front().cachedMesh;
    asmDoc.removeComponent(id);
    asmDoc.restore(before);
    EXPECT_EQ(asmDoc.component(id)->cachedMesh, kept);
}

// ---------------------------------------------------------------------------
// Interference (Phase 96).  The checker existed in the modeling layer (Phase
// 48) and nothing in the document or the UI called it.
// ---------------------------------------------------------------------------

namespace {

std::shared_ptr<Document> boxPart(double w, double h, double d) {
    auto part = std::make_shared<Document>();
    part->setType(DocumentType::Part);
    part->featureTree().addFeature(PrimitiveFeature::makeBox(w, h, d));
    part->rebuildModel();
    return part;
}

uint64_t place(AssemblyDocument& asmDoc, const std::shared_ptr<Document>& part,
               const hz::math::Vec3& at) {
    ComponentInstance c;
    c.resolvedPart = part;
    c.state = ComponentState::Resolved;
    c.transform = hz::math::Mat4::translation(at);
    return asmDoc.addComponent(c);
}

}  // namespace

TEST(AssemblyDocumentTest, FindInterferenceMeasuresPlacedComponents) {
    AssemblyDocument asmDoc;
    auto block = boxPart(10, 10, 10);
    const uint64_t a = place(asmDoc, block, hz::math::Vec3(0, 0, 0));
    const uint64_t b = place(asmDoc, block, hz::math::Vec3(8, 0, 0));   // 2 into a
    const uint64_t c = place(asmDoc, block, hz::math::Vec3(18, 0, 0));  // touches b
    (void)c;

    const auto report = asmDoc.findInterference();
    EXPECT_TRUE(report.unchecked.empty());
    ASSERT_EQ(report.pairs.size(), 1u) << "face contact is not interference";
    const auto& pair = report.pairs.front();
    EXPECT_TRUE((pair.componentA == a && pair.componentB == b) ||
                (pair.componentA == b && pair.componentB == a));
    EXPECT_TRUE(pair.volumeResolved);
    EXPECT_NEAR(pair.volume, 200.0, 1e-9);
}

TEST(AssemblyDocumentTest, FindInterferenceSkipsSuppressedAndListsUnresolved) {
    AssemblyDocument asmDoc;
    auto block = boxPart(10, 10, 10);
    place(asmDoc, block, hz::math::Vec3(0, 0, 0));
    const uint64_t hidden = place(asmDoc, block, hz::math::Vec3(5, 0, 0));
    asmDoc.component(hidden)->suppressed = true;

    ComponentInstance lightweight;  // no resolved part
    const uint64_t light = asmDoc.addComponent(lightweight);

    const auto report = asmDoc.findInterference();
    EXPECT_TRUE(report.pairs.empty()) << "a suppressed component takes no part";
    ASSERT_EQ(report.unchecked.size(), 1u);
    EXPECT_EQ(report.unchecked.front(), light);
}

TEST(AssemblyDocumentTest, InterferenceIsMeasuredFromCopiesTheAssemblyCanChangeUnder) {
    // Phase 114: the measuring runs on a worker, so it reads copies of the
    // placed solids taken beforehand, not the assembly itself.
    AssemblyDocument asmDoc;
    auto block = boxPart(10, 10, 10);
    place(asmDoc, block, hz::math::Vec3(0, 0, 0));
    const uint64_t moved = place(asmDoc, block, hz::math::Vec3(8, 0, 0));
    const auto input = asmDoc.interferenceInput();
    EXPECT_EQ(input.faceCount(), 12u);

    // Moved clear after the input was taken: the measurement is of the input.
    asmDoc.component(moved)->transform = hz::math::Mat4::translation(hz::math::Vec3(50, 0, 0));
    const auto report = AssemblyDocument::measureInterference(input);
    ASSERT_EQ(report.pairs.size(), 1u);
    EXPECT_NEAR(report.pairs.front().volume, 200.0, 1e-9);
    EXPECT_TRUE(asmDoc.findInterference().pairs.empty()) << "and the assembly as it is now";
}

// An assembly drawn is its components as one solid: each part placed, its
// names its own (two instances of one part told apart), a suppressed one
// left out, and one with no solid listed as missing.
TEST(AssemblyDocumentTest, TheDrawingSolidIsTheComponentsEachNamedApart) {
    AssemblyDocument asmDoc;
    auto block = boxPart(10, 10, 10);
    const uint64_t a = place(asmDoc, block, hz::math::Vec3(0, 0, 0));
    const uint64_t b = place(asmDoc, block, hz::math::Vec3(30, 0, 0));
    const uint64_t hidden = place(asmDoc, block, hz::math::Vec3(60, 0, 0));
    asmDoc.component(hidden)->suppressed = true;
    ComponentInstance lightweight;
    const uint64_t light = asmDoc.addComponent(lightweight);

    std::vector<uint64_t> missing;
    const auto solid = asmDoc.drawingSolid(
        [](const ComponentInstance& c) {
            return c.resolvedPart ? c.resolvedPart->solid() : nullptr;
        },
        &missing);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faces().size(), 2 * block->solid()->faces().size());
    EXPECT_EQ(missing, std::vector<uint64_t>{light});

    int ofA = 0;
    int ofB = 0;
    for (const auto& e : solid->edges()) {
        const std::string& tag = e.topoId.tag();
        ofA += tag.rfind(AssemblyDocument::namePrefix(a), 0) == 0 ? 1 : 0;
        ofB += tag.rfind(AssemblyDocument::namePrefix(b), 0) == 0 ? 1 : 0;
    }
    EXPECT_EQ(ofA, static_cast<int>(block->solid()->edges().size()));
    EXPECT_EQ(ofB, ofA) << "each instance's edges named for it";
    double maxX = -1e300;
    for (const auto& v : solid->vertices()) maxX = std::max(maxX, v.point.x);
    EXPECT_NEAR(maxX, 40.0, 1e-9) << "placed where its component is";

    AssemblyDocument empty;
    EXPECT_EQ(empty.drawingSolid([](const ComponentInstance&) { return nullptr; }), nullptr);
}

// Phase 158: an assembly's mates gathered once and solved as a drag moves: a
// component held where the drag puts it; what grounds the assembly without a
// hold (here its first component) grounds it still.
TEST(AssemblyMatesTest, AComponentIsHeldWhereADragPutsIt) {
    AssemblyDocument asmDoc;
    auto block = boxPart(10, 10, 10);
    const uint64_t base = place(asmDoc, block, Vec3(0, 0, 0));
    const uint64_t lid = place(asmDoc, block, Vec3(12, 0, 10));
    const std::string feature = block->featureTree().feature(0)->featureID();
    Mate on;
    on.type = MateType::Coincident;
    on.a = {base, hz::topo::TopologyID::fromTag(feature + "/top")};
    on.b = {lid, hz::topo::TopologyID::fromTag(feature + "/bottom")};
    asmDoc.addMate(on);

    std::string why;
    auto mates = AssemblyMates::gather(asmDoc, &why);
    if (!mates) FAIL() << why;
    EXPECT_EQ(mates->solve().status, hz::model::AssemblySolveStatus::Success);

    // Along the base's top: held there, and the base where it is.
    const Mat4 along = Mat4::translation(Vec3(30, 5, 10));
    auto result = mates->solve({AssemblyMates::Hold{lid, along}, false});
    ASSERT_EQ(result.status, hz::model::AssemblySolveStatus::Success) << result.message;
    EXPECT_NEAR(result.transforms.at(lid).transformPoint(Vec3()).x, 30.0, 1e-9);
    EXPECT_NEAR((result.transforms.at(base).transformPoint(Vec3())).length(), 0.0, 1e-9);

    // Up off it: the mate cannot be met with the lid held there; the base is
    // not lifted to meet it.
    const Mat4 up = Mat4::translation(Vec3(30, 5, 25));
    result = mates->solve({AssemblyMates::Hold{lid, up}, false});
    EXPECT_NE(result.status, hz::model::AssemblySolveStatus::Success);

    // Let go from there, it comes back down onto the top.
    mates->place({{lid, up}});
    result = mates->solve({std::nullopt, false});
    ASSERT_EQ(result.status, hz::model::AssemblySolveStatus::Success) << result.message;
    EXPECT_NEAR(result.transforms.at(lid).transformPoint(Vec3()).z, 10.0, 1e-6);
    EXPECT_NEAR(result.transforms.at(lid).transformPoint(Vec3()).x, 30.0, 1e-6);
}

TEST(AssemblyMatesTest, AMateOnAFaceThatIsGoneIsSaidSo) {
    AssemblyDocument asmDoc;
    auto block = boxPart(10, 10, 10);
    const uint64_t base = place(asmDoc, block, Vec3(0, 0, 0));
    const uint64_t lid = place(asmDoc, block, Vec3(12, 0, 10));
    Mate on;
    on.type = MateType::Coincident;
    on.a = {base, hz::topo::TopologyID::fromTag("nosuch/top")};
    on.b = {lid, hz::topo::TopologyID::fromTag("nosuch/bottom")};
    const uint64_t id = asmDoc.addMate(on);
    std::string why;
    EXPECT_FALSE(AssemblyMates::gather(asmDoc, &why).has_value());
    EXPECT_EQ(why,
              "mate " + std::to_string(id) + " references geometry that could not be resolved");
}

namespace {

/// A one-triangle mesh in the XY plane at @p z, with its normals (+z) or
/// without.
std::shared_ptr<hz::geo::MeshData> triangleAt(float z, bool withNormals) {
    auto mesh = std::make_shared<hz::geo::MeshData>();
    mesh->positions = {0, 0, z, 1, 0, z, 0, 1, z};
    if (withNormals) mesh->normals = {0, 0, 1, 0, 0, 1, 0, 0, 1};
    mesh->indices = {0, 1, 2};
    return mesh;
}

}  // namespace

// Phase 159: a subassembly's mesh is its components' merged. One whose mesh
// has no normals (a part's cached tessellation may have none) gets them
// from its triangles, so every vertex after it keeps its own.
TEST(AssemblyDocumentTest, TheDrawingMeshKeepsANormalForEveryVertex) {
    AssemblyDocument asmDoc;
    for (const bool normals : {true, false, true}) {
        ComponentInstance c;
        c.cachedMesh = triangleAt(0.0f, normals);
        c.transform = Mat4::rotationX(std::numbers::pi);  // upside down: normals -z
        asmDoc.addComponent(c);
    }
    const auto merged = asmDoc.drawingMesh();
    ASSERT_NE(merged, nullptr);
    ASSERT_EQ(merged->normals.size(), merged->positions.size());
    for (size_t i = 0; i + 2 < merged->normals.size(); i += 3) {
        EXPECT_NEAR(merged->normals[i + 2], -1.0f, 1e-6f) << "vertex " << i / 3;
    }
    EXPECT_EQ(merged->indices.size(), 9u);
    EXPECT_EQ(merged->indices[8], 8u) << "the third triangle's own vertices";
}

// An undo keeps what a subassembly component has resolved since, as it
// keeps a part's.
TEST(AssemblyDocumentTest, RestoringKeepsASubassemblysResolvedGeometry) {
    AssemblyDocument asmDoc;
    ComponentInstance c;
    c.partPath = "pair.hzasm";
    const uint64_t id = asmDoc.addComponent(c);
    const AssemblyState before = asmDoc.snapshot();
    auto* comp = asmDoc.component(id);
    comp->resolvedAssembly = std::make_shared<AssemblyDocument>();
    comp->assemblySolid = std::make_shared<hz::topo::Solid>();
    comp->state = ComponentState::Resolved;
    asmDoc.restore(before);
    comp = asmDoc.component(id);
    ASSERT_NE(comp, nullptr);
    EXPECT_NE(comp->resolvedAssembly, nullptr);
    EXPECT_NE(comp->assemblySolid, nullptr);
    EXPECT_EQ(comp->state, ComponentState::Resolved);
}
