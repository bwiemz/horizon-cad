#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/BoundingBox.h"

using hz::doc::ChangeTextHeightCommand;
using hz::doc::MoveEntityCommand;
using hz::doc::RemoveEntitiesCommand;
using hz::doc::RemoveEntityCommand;
using hz::draft::DraftDocument;
using hz::draft::DraftLine;
using hz::draft::DraftText;
using hz::math::BoundingBox;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

std::vector<uint64_t> ids(const DraftDocument& d) {
    std::vector<uint64_t> out;
    for (const auto& e : d.entities()) out.push_back(e->id());
    return out;
}

std::vector<uint64_t> fill(DraftDocument& d, int n) {
    std::vector<uint64_t> out;
    for (int i = 0; i < n; ++i) {
        auto line = std::make_shared<DraftLine>(Vec2(3.0 * i, 0), Vec2(3.0 * i + 1, 0));
        out.push_back(line->id());
        d.addEntity(line);
    }
    return out;
}

bool indexedAt(const DraftDocument& d, uint64_t id, double x, double y) {
    const BoundingBox box(Vec3(x - 0.05, y - 0.05, -1), Vec3(x + 0.05, y + 0.05, 1));
    for (uint64_t hit : d.spatialIndex().query(box)) {
        if (hit == id) return true;
    }
    return false;
}

}  // namespace

// Undoing a removal puts the entity back where it was drawn, not at the end.
TEST(EntityCommandsTest, UndoingARemovalRestoresTheDrawingOrder) {
    DraftDocument d;
    const auto all = fill(d, 4);
    RemoveEntityCommand remove(d, all[1]);
    remove.execute();
    EXPECT_EQ(ids(d), (std::vector<uint64_t>{all[0], all[2], all[3]}));
    remove.undo();
    EXPECT_EQ(ids(d), all);
    remove.execute();  // redo
    EXPECT_EQ(ids(d), (std::vector<uint64_t>{all[0], all[2], all[3]}));
}

TEST(EntityCommandsTest, RemovingManyIsOneStepThatUndoesInPlace) {
    DraftDocument d;
    const auto all = fill(d, 8);
    RemoveEntitiesCommand remove(d, {all[7], all[0], all[5]});
    remove.execute();
    EXPECT_EQ(ids(d), (std::vector<uint64_t>{all[1], all[2], all[3], all[4], all[6]}));
    remove.undo();
    EXPECT_EQ(ids(d), all);
    for (uint64_t id : all) EXPECT_NE(d.findEntity(id), nullptr);
    remove.execute();
    remove.undo();
    EXPECT_EQ(ids(d), all) << "repeatable";
}

// A move re-indexes what it moved, and undo re-indexes it back, without
// rebuilding the index for the whole drawing.
TEST(EntityCommandsTest, MovingReindexesWhatMoved) {
    DraftDocument d;
    const auto all = fill(d, 3);
    hz::cstr::ConstraintSystem constraints;
    MoveEntityCommand move(d, {all[1]}, Vec2(0, 40), constraints);
    move.execute();
    EXPECT_FALSE(indexedAt(d, all[1], 3.5, 0));
    EXPECT_TRUE(indexedAt(d, all[1], 3.5, 40));
    move.undo();
    EXPECT_TRUE(indexedAt(d, all[1], 3.5, 0));
    EXPECT_FALSE(indexedAt(d, all[1], 3.5, 40));
}

// Changing a text's height changes its bounds; picking and box selection
// used to keep seeing the old, smaller box.
TEST(EntityCommandsTest, ChangingTextHeightReindexesIt) {
    DraftDocument d;
    auto text = std::make_shared<DraftText>(Vec2(0, 0), "HELLO", 1.0);
    d.addEntity(text);
    const BoundingBox small = text->boundingBox();
    ChangeTextHeightCommand grow(d, text->id(), 20.0);
    grow.execute();
    const BoundingBox big = text->boundingBox();
    ASSERT_GT(big.max().y, small.max().y + 5.0);
    EXPECT_TRUE(indexedAt(d, text->id(), big.center().x, big.max().y - 0.5));
    grow.undo();
    EXPECT_FALSE(indexedAt(d, text->id(), big.center().x, big.max().y - 0.5));
}

// Renaming a layer carries everything on it, in the drawing and in block
// definitions, and the current layer; undo takes it all back.
TEST(EntityCommandsTest, RenamingALayerCarriesItsEntities) {
    DraftDocument d;
    hz::draft::LayerManager layers;
    hz::draft::LayerProperties walls;
    walls.name = "Walls";
    walls.lineWidth = 2.0;
    layers.addLayer(walls);
    layers.setCurrentLayer("Walls");
    const auto all = fill(d, 3);
    d.findEntity(all[0])->setLayer("Walls");
    d.findEntity(all[2])->setLayer("Walls");
    auto block = std::make_shared<hz::draft::BlockDefinition>();
    block->name = "Door";
    block->entities.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(1, 0)));
    block->entities.back()->setLayer("Walls");
    d.blockTable().addBlock(block);

    hz::doc::RenameLayerCommand rename(layers, d, "Walls", "Exterior");
    rename.execute();
    ASSERT_TRUE(rename.applied());
    EXPECT_EQ(layers.getLayer("Walls"), nullptr);
    ASSERT_NE(layers.getLayer("Exterior"), nullptr);
    EXPECT_DOUBLE_EQ(layers.getLayer("Exterior")->lineWidth, 2.0) << "its properties go with it";
    EXPECT_EQ(layers.currentLayer(), "Exterior");
    EXPECT_EQ(d.findEntity(all[0])->layer(), "Exterior");
    EXPECT_EQ(d.findEntity(all[1])->layer(), "0");
    EXPECT_EQ(d.findEntity(all[2])->layer(), "Exterior");
    EXPECT_EQ(block->entities.front()->layer(), "Exterior") << "inside a block too";

    rename.undo();
    ASSERT_NE(layers.getLayer("Walls"), nullptr);
    EXPECT_EQ(layers.getLayer("Exterior"), nullptr);
    EXPECT_EQ(layers.currentLayer(), "Walls");
    EXPECT_EQ(d.findEntity(all[0])->layer(), "Walls");
    EXPECT_EQ(block->entities.front()->layer(), "Walls");
}

TEST(EntityCommandsTest, ARenameTheLayersRefuseChangesNothing) {
    DraftDocument d;
    hz::draft::LayerManager layers;
    hz::draft::LayerProperties a;
    a.name = "A";
    layers.addLayer(a);
    a.name = "B";
    layers.addLayer(a);
    const std::pair<const char*, const char*> refused[] = {
        {"0", "Base"},
        {"A", "B"},
        {"A", ""},
        {"Missing", "C"},
    };
    for (const auto& [from, to] : refused) {
        hz::doc::RenameLayerCommand rename(layers, d, from, to);
        rename.execute();
        EXPECT_FALSE(rename.applied()) << from << " -> " << to;
        rename.undo();
    }
    EXPECT_NE(layers.getLayer("0"), nullptr);
    EXPECT_NE(layers.getLayer("A"), nullptr);
    EXPECT_NE(layers.getLayer("B"), nullptr);
}

// A block made from entities is inserted at the base point given; undo puts
// the entities back where they were in the drawing order, and redo brings
// back the same reference, under the same ID.
TEST(EntityCommandsTest, CreatingABlockKeepsOrderAndIdentity) {
    DraftDocument d;
    const auto all = fill(d, 5);
    hz::doc::CreateBlockCommand create(d, "Pair", {all[1], all[3]}, Vec2(10, 10));
    create.execute();
    const uint64_t ref = create.blockRefId();
    const auto block = d.blockTable().findBlock("Pair");
    ASSERT_NE(block, nullptr);
    EXPECT_EQ(block->entities.size(), 2u);
    EXPECT_DOUBLE_EQ(block->basePoint.x, 10.0);
    const auto* inserted = dynamic_cast<const hz::draft::DraftBlockRef*>(d.findEntity(ref));
    ASSERT_NE(inserted, nullptr);
    EXPECT_DOUBLE_EQ(inserted->insertPos().y, 10.0);
    EXPECT_EQ(ids(d), (std::vector<uint64_t>{all[0], all[2], all[4], ref}));

    create.undo();
    EXPECT_EQ(ids(d), all) << "the originals back in their places";
    EXPECT_EQ(d.blockTable().findBlock("Pair"), nullptr);

    create.execute();  // redo
    EXPECT_EQ(create.blockRefId(), ref);
    EXPECT_NE(d.findEntity(ref), nullptr);
    EXPECT_EQ(d.blockTable().findBlock("Pair"), block);
}

// Exploding a mirrored or half-turned block reference leaves its pieces where
// it drew them. The mirror was ignored, and arcs kept their angles under the
// half turn.
TEST(EntityCommandsTest, ExplodingPutsThePiecesWhereTheBlockDrewThem) {
    auto door = std::make_shared<hz::draft::BlockDefinition>();
    door->name = "Door";
    door->entities.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(0, 10)));
    door->entities.push_back(
        std::make_shared<hz::draft::DraftArc>(Vec2(0, 0), 10.0, 0.0, 1.5707963267948966));
    door->entities.push_back(std::make_shared<DraftText>(Vec2(2, 1), "D", 1.0));

    for (const bool mirrored : {true, false}) {
        SCOPED_TRACE(mirrored ? "mirrored" : "scaled by -1");
        DraftDocument d;
        d.blockTable().addBlock(door);
        auto ref = std::make_shared<hz::draft::DraftBlockRef>(door, Vec2(5, 5), 0.0,
                                                              mirrored ? 1.0 : -1.0);
        if (mirrored) ref->mirror(Vec2(5, 0), Vec2(5, 1));
        d.addEntity(ref);
        const hz::draft::DraftBlockRef placed = *ref;
        hz::doc::ExplodeBlockCommand explode(d, ref->id());
        explode.execute();
        ASSERT_EQ(d.entities().size(), 3u);

        const auto* line = dynamic_cast<const DraftLine*>(d.entities()[0].get());
        const auto* arc = dynamic_cast<const hz::draft::DraftArc*>(d.entities()[1].get());
        const auto* text = dynamic_cast<const DraftText*>(d.entities()[2].get());
        ASSERT_TRUE(line && arc && text);
        const auto near = [](const Vec2& a, const Vec2& b) { return (a - b).length() < 1e-9; };
        EXPECT_TRUE(near(line->end(), placed.transformPoint(Vec2(0, 10))));
        // The arc's ends, whichever way round it now runs.
        const Vec2 from = placed.transformPoint(Vec2(10, 0));
        const Vec2 to = placed.transformPoint(Vec2(0, 10));
        EXPECT_TRUE((near(arc->startPoint(), from) && near(arc->endPoint(), to)) ||
                    (near(arc->startPoint(), to) && near(arc->endPoint(), from)));
        EXPECT_TRUE(near(arc->midPoint(),
                         placed.transformPoint(Vec2(7.0710678118654755, 7.0710678118654755))))
            << "the swing on the side the block drew it";
        EXPECT_TRUE(near(text->position(), placed.transformPoint(Vec2(2, 1))));
    }
}
