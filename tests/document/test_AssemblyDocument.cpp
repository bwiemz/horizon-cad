#include <gtest/gtest.h>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/math/Mat4.h"

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
