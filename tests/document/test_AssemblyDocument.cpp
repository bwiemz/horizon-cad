#include <gtest/gtest.h>

#include "horizon/document/AssemblyDocument.h"
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
