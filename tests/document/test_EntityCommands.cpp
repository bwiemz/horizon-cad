#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
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
