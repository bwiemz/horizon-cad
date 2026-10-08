#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/document/Commands.h"
#include "horizon/document/ConstraintSolveHelper.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Constants.h"

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

// A text given more lines grows downwards, and can be found there.
TEST(EntityCommandsTest, TextGivenMoreLinesIsIndexedDownThePage) {
    DraftDocument d;
    auto text = std::make_shared<DraftText>(Vec2(0, 0), "HELLO", 2.0);
    d.addEntity(text);
    const Vec2 third = text->lineBaseline(2) + Vec2(1.0, 0.5);
    EXPECT_FALSE(indexedAt(d, text->id(), third.x, third.y));
    hz::doc::ChangeTextContentCommand lines(d, text->id(), "HELLO\nTWO\nTHREE");
    lines.execute();
    EXPECT_TRUE(indexedAt(d, text->id(), third.x, third.y));
    lines.undo();
    EXPECT_FALSE(indexedAt(d, text->id(), third.x, third.y));
}

// The dimension style is one undoable step, and undo puts every setting back.
TEST(EntityCommandsTest, ChangingTheDimensionStyleUndoes) {
    DraftDocument d;
    hz::doc::UndoStack stack;
    const hz::draft::DimensionStyle before = d.dimensionStyle();
    hz::draft::DimensionStyle inches = before;
    inches.unit = "in";
    inches.showUnits = true;
    inches.precision = 3;
    inches.textHeight = 3.5;
    stack.push(std::make_unique<hz::doc::ChangeDimensionStyleCommand>(d, inches));
    EXPECT_EQ(d.dimensionStyle().unit, "in");
    EXPECT_EQ(d.dimensionStyle().precision, 3);
    EXPECT_DOUBLE_EQ(d.dimensionStyle().textHeight, 3.5);

    stack.undo();
    EXPECT_EQ(d.dimensionStyle().unit, before.unit);
    EXPECT_EQ(d.dimensionStyle().showUnits, before.showUnits);
    EXPECT_EQ(d.dimensionStyle().precision, before.precision);
    EXPECT_DOUBLE_EQ(d.dimensionStyle().textHeight, before.textHeight);
    stack.redo();
    EXPECT_EQ(d.dimensionStyle().unit, "in");
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

// Phase 157: a copy of an edge projected into a sketch is not that edge's
// projection. Duplicate and Mirror make copies that follow no edge (the next
// build would draw them back onto the original), and keep being
// construction; a clone, which stands in for the entity itself (a grip's
// undo), keeps both.
TEST(EntityCommandsTest, ACopyOfAProjectedEdgeFollowsNoEdge) {
    DraftDocument doc;
    auto projected = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 0));
    projected->setSourceEdge("extrude_1/edge:cap_top|side:e2");
    projected->setConstruction(true);
    doc.addEntity(projected);

    const auto clone = projected->clone();
    EXPECT_EQ(clone->sourceEdge(), projected->sourceEdge()) << "the entity itself, restored";
    EXPECT_TRUE(clone->construction());

    hz::doc::UndoStack stack;
    stack.push(std::make_unique<hz::doc::DuplicateEntityCommand>(
        doc, std::vector<uint64_t>{projected->id()}, Vec2(0, 5)));
    stack.push(std::make_unique<hz::doc::MirrorEntityCommand>(
        doc, std::vector<uint64_t>{projected->id()}, Vec2(0, -1), Vec2(10, -1)));
    ASSERT_EQ(doc.entities().size(), 3u);
    for (const auto& entity : doc.entities()) {
        if (entity == projected) {
            EXPECT_FALSE(entity->sourceEdge().empty()) << "the original still follows its edge";
            continue;
        }
        EXPECT_TRUE(entity->sourceEdge().empty()) << "a copy follows no edge";
        EXPECT_TRUE(entity->construction()) << "and is a guide still";
    }
}

namespace {

/// Two lines joined end to start, as drawn and constrained by hand.
struct JoinedLines {
    std::shared_ptr<DraftLine> first;
    std::shared_ptr<DraftLine> second;
    uint64_t joint = 0;
};

JoinedLines joinLines(DraftDocument& drawing, hz::cstr::ConstraintSystem& constraints) {
    JoinedLines lines;
    lines.first = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 0));
    lines.second = std::make_shared<DraftLine>(Vec2(10, 0), Vec2(10, 10));
    drawing.addEntity(lines.first);
    drawing.addEntity(lines.second);
    lines.joint = constraints.addConstraint(std::make_shared<hz::cstr::CoincidentConstraint>(
        hz::cstr::GeometryRef{lines.first->id(), hz::cstr::FeatureType::Point, 1},
        hz::cstr::GeometryRef{lines.second->id(), hz::cstr::FeatureType::Point, 0}));
    return lines;
}

}  // namespace

// Trim, Cut, Break and the rest take an entity away with no thought for its
// constraints. They go with it, in the same step, and come back on undo: left
// behind, the next solve stopped at the entity that was gone, and a Move
// after it moved with no step to undo it.
TEST(EntityCommandsTest, AnEntityTakenAwayTakesItsConstraints) {
    hz::doc::Document doc;
    auto& stack = doc.undoStack();
    auto& drawing = doc.draftDocument();
    auto& constraints = doc.constraintSystem();
    const JoinedLines lines = joinLines(drawing, constraints);

    // As Trim does: the line out, its piece in.
    auto trim = std::make_unique<hz::doc::CompositeCommand>("Trim");
    trim->addCommand(std::make_unique<RemoveEntityCommand>(drawing, lines.first->id()));
    trim->addCommand(std::make_unique<hz::doc::AddEntityCommand>(
        drawing, std::make_shared<DraftLine>(Vec2(0, 0), Vec2(5, 0))));
    stack.push(std::move(trim));
    EXPECT_TRUE(constraints.empty()) << "the joint went with the line";
    EXPECT_EQ(stack.undoCount(), 1u) << "in one step";

    stack.undo();
    EXPECT_NE(drawing.findEntity(lines.first->id()), nullptr);
    EXPECT_NE(constraints.getConstraint(lines.joint), nullptr) << "and came back with it";
    stack.redo();
    EXPECT_TRUE(constraints.empty());
    stack.undo();

    // As Cut does, and a Move after it.
    stack.push(
        std::make_unique<RemoveEntitiesCommand>(drawing, std::vector<uint64_t>{lines.first->id()}));
    EXPECT_TRUE(constraints.empty());
    const auto before = stack.undoCount();
    EXPECT_NO_THROW(stack.push(std::make_unique<MoveEntityCommand>(
        drawing, std::vector<uint64_t>{lines.second->id()}, Vec2(0, 5), constraints)));
    EXPECT_EQ(stack.undoCount(), before + 1) << "the move is a step";
    EXPECT_DOUBLE_EQ(lines.second->start().y, 5.0);
    stack.undo();
    EXPECT_DOUBLE_EQ(lines.second->start().y, 0.0);
}

// In a sketch as in the drawing: its constraints are its own.
TEST(EntityCommandsTest, AnEntityTakenFromASketchTakesItsConstraints) {
    hz::doc::Document doc;
    auto sketch = std::make_shared<hz::doc::Sketch>();
    doc.addSketch(sketch);
    const JoinedLines lines = joinLines(sketch->drawing(), sketch->constraintSystem());
    doc.undoStack().push(std::make_unique<hz::doc::CreateBlockCommand>(
        sketch->drawing(), "Corner", std::vector<uint64_t>{lines.second->id()}));
    EXPECT_TRUE(sketch->constraintSystem().empty());
    doc.undoStack().undo();
    EXPECT_NE(sketch->constraintSystem().getConstraint(lines.joint), nullptr);
}

// A constraint naming an entity that is not there (an older file's, or one
// read in part) is left out of the solve, never the end of it: the rest are
// still solved.
TEST(EntityCommandsTest, AConstraintOnAMissingEntityIsLeftOutOfTheSolve) {
    DraftDocument drawing;
    hz::cstr::ConstraintSystem constraints;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 3));
    drawing.addEntity(line);
    constraints.addConstraint(std::make_shared<hz::cstr::CoincidentConstraint>(
        hz::cstr::GeometryRef{line->id(), hz::cstr::FeatureType::Point, 1},
        hz::cstr::GeometryRef{line->id() + 1000, hz::cstr::FeatureType::Point, 0}));
    constraints.addConstraint(std::make_shared<hz::cstr::HorizontalConstraint>(
        hz::cstr::GeometryRef{line->id(), hz::cstr::FeatureType::Point, 0},
        hz::cstr::GeometryRef{line->id(), hz::cstr::FeatureType::Point, 1}));

    hz::doc::ConstraintSolveHelper::SolveAndApplyResult result;
    ASSERT_NO_THROW(result = hz::doc::ConstraintSolveHelper::solveAndApply(drawing, constraints));
    EXPECT_TRUE(result.success);
    EXPECT_NEAR(line->start().y, line->end().y, 1e-9) << "the horizontal is solved";
    EXPECT_EQ(constraints.constraints().size(), 2u) << "and the other kept, for what it names";
}

// Entities edited together, as Stretch edits them, are solved once, after
// all of them: undo puts each back as it was, and redo solves as it did. A
// solve after each one moved the others, and undo left the second line
// stretched.
TEST(EntityCommandsTest, EntitiesGripEditedTogetherAreSolvedOnce) {
    hz::doc::Document doc;
    auto& drawing = doc.draftDocument();
    auto& constraints = doc.constraintSystem();
    const JoinedLines lines = joinLines(drawing, constraints);
    const hz::cstr::GeometryRef start{lines.first->id(), hz::cstr::FeatureType::Point, 0};
    const hz::cstr::GeometryRef end{lines.first->id(), hz::cstr::FeatureType::Point, 1};
    constraints.addConstraint(std::make_shared<hz::cstr::DistanceConstraint>(start, end, 10.0));
    constraints.addConstraint(std::make_shared<hz::cstr::FixedConstraint>(start, Vec2(0, 0)));

    // The corner stretched to (13, 4), as the Stretch tool leaves it.
    std::vector<hz::doc::GripMoveCommand::Edit> edits;
    for (const auto& line : {lines.first, lines.second}) {
        edits.push_back({line->id(), line->clone(), nullptr});
    }
    lines.first->setEnd(Vec2(13, 4));
    lines.second->setStart(Vec2(13, 4));
    for (auto& edit : edits) edit.afterState = drawing.findEntity(edit.entityId)->clone();
    doc.undoStack().push(
        std::make_unique<hz::doc::GripMoveCommand>(drawing, std::move(edits), constraints));

    const auto line = [&drawing](const std::shared_ptr<DraftLine>& l) {
        return dynamic_cast<const DraftLine*>(drawing.findEntity(l->id()));
    };
    const auto holds = [&] {
        return (line(lines.first)->end() - line(lines.second)->start()).length() < 1e-6 &&
               std::abs(line(lines.first)->end().length() - 10.0) < 1e-6;
    };
    EXPECT_TRUE(holds());
    const Vec2 corner = line(lines.second)->start();
    doc.undoStack().undo();
    EXPECT_EQ((line(lines.first)->end() - Vec2(10, 0)).length(), 0.0);
    EXPECT_EQ((line(lines.second)->start() - Vec2(10, 0)).length(), 0.0) << "both back";
    doc.undoStack().redo();
    EXPECT_TRUE(holds());
    EXPECT_LT((line(lines.second)->start() - corner).length(), 1e-12);
}

// A grip edit puts a new object in the entity's place. Undoing a layer's
// rename or removal after one found the entity by the object it had moved,
// no longer in the drawing: the line stayed on a layer that was gone, and
// could not be picked.
TEST(EntityCommandsTest, UndoingALayerChangeFindsEntitiesAGripEditReplaced) {
    for (const bool rename : {true, false}) {
        SCOPED_TRACE(rename ? "renamed" : "removed");
        hz::doc::Document doc;
        auto& layers = doc.layerManager();
        hz::draft::LayerProperties walls;
        walls.name = "Walls";
        layers.addLayer(walls);
        auto& drawing = doc.draftDocument();
        auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 0));
        line->setLayer("Walls");
        drawing.addEntity(line);
        const uint64_t id = line->id();

        auto& stack = doc.undoStack();
        if (rename) {
            stack.push(
                std::make_unique<hz::doc::RenameLayerCommand>(layers, drawing, "Walls", "Outer"));
        } else {
            stack.push(std::make_unique<hz::doc::RemoveLayerCommand>(layers, drawing, "Walls"));
        }
        // A grip drag: the line's end moved, then the step that records it.
        auto before = drawing.findEntity(id)->clone();
        line->setEnd(Vec2(12, 3));
        stack.push(std::make_unique<hz::doc::GripMoveCommand>(
            drawing, id, std::move(before), line->clone(), doc.constraintSystem()));
        stack.undo();
        ASSERT_NE(drawing.sharedEntity(id), line) << "the grip's undo put another object there";
        stack.undo();
        EXPECT_EQ(drawing.findEntity(id)->layer(), "Walls");
        stack.redo();
        EXPECT_EQ(drawing.findEntity(id)->layer(), rename ? "Outer" : "0");
    }
}

// Explode undone and redone gives back the same pieces, so the steps after it
// still find them (a Move's redo moved nothing, its pieces had new IDs);
// undone, the reference is back where it was drawn; and two references of one
// block exploded are two groups, not one.
TEST(EntityCommandsTest, ExplodingIsTheSamePiecesEachTime) {
    auto pair = std::make_shared<hz::draft::BlockDefinition>();
    pair->name = "Pair";
    for (const double y : {0.0, 1.0}) {
        pair->entities.push_back(std::make_shared<DraftLine>(Vec2(0, y), Vec2(4, y)));
        pair->entities.back()->setGroupId(7);  // a group within the block
    }
    DraftDocument d;
    d.blockTable().addBlock(pair);
    fill(d, 1);
    auto ref = std::make_shared<hz::draft::DraftBlockRef>(pair, Vec2(10, 10));
    auto other = std::make_shared<hz::draft::DraftBlockRef>(pair, Vec2(20, 10));
    d.addEntity(ref);
    d.addEntity(other);
    fill(d, 1);
    const auto drawn = ids(d);

    hz::doc::UndoStack stack;
    hz::cstr::ConstraintSystem constraints;
    auto explode = std::make_unique<hz::doc::ExplodeBlockCommand>(d, ref->id());
    auto* exploded = explode.get();
    stack.push(std::move(explode));
    const auto pieces = exploded->explodedIds();
    ASSERT_EQ(pieces.size(), 2u);
    stack.push(std::make_unique<MoveEntityCommand>(d, std::vector<uint64_t>{pieces[0]}, Vec2(0, 5),
                                                   constraints));
    stack.undo();
    stack.undo();
    EXPECT_EQ(ids(d), drawn) << "the reference back in its place";
    stack.redo();
    EXPECT_EQ(exploded->explodedIds(), pieces) << "the same pieces";
    stack.redo();
    const auto* moved = dynamic_cast<const DraftLine*>(d.findEntity(pieces[0]));
    ASSERT_NE(moved, nullptr);
    EXPECT_DOUBLE_EQ(moved->start().y, 15.0) << "and the move after it redone on them";

    hz::doc::ExplodeBlockCommand second(d, other->id());
    second.execute();
    const uint64_t group = d.findEntity(pieces[0])->groupId();
    EXPECT_NE(group, 0u) << "a group still";
    EXPECT_EQ(d.findEntity(pieces[1])->groupId(), group);
    for (const uint64_t id : second.explodedIds()) {
        EXPECT_NE(d.findEntity(id)->groupId(), group) << "each reference's pieces their own group";
    }
}

// A rectangle's sides run along the axes. Turned by other than a quarter turn,
// or mirrored in a slanted line, its copy is a closed polyline through the
// four corners, the same size: taking two opposite corners as a new box made
// a 4 x 2 rectangle turned 45 degrees a 1.41 x 4.24 one. A quarter turn, or a
// mirror in an axis or a diagonal, leaves a rectangle.
TEST(EntityCommandsTest, ARectangleTurnedOrMirroredKeepsItsShape) {
    using hz::draft::DraftPolyline;
    using hz::draft::DraftRectangle;
    const auto near = [](const Vec2& a, const Vec2& b) { return (a - b).length() < 1e-9; };
    const auto turned = [](const Vec2& p, const Vec2& c, double a) {
        const Vec2 v = p - c;
        return Vec2(c.x + v.x * std::cos(a) - v.y * std::sin(a),
                    c.y + v.x * std::sin(a) + v.y * std::cos(a));
    };

    DraftDocument d;
    auto rect = std::make_shared<DraftRectangle>(Vec2(0, 0), Vec2(4, 2));
    rect->setLineType(2);
    d.addEntity(rect);
    const auto corners = rect->corners();
    hz::doc::UndoStack stack;

    const double eighth = hz::math::kPi / 4.0;
    stack.push(std::make_unique<hz::doc::RotateEntityCommand>(d, std::vector<uint64_t>{rect->id()},
                                                              Vec2(2, 1), eighth));
    ASSERT_EQ(d.entities().size(), 2u);
    const auto* poly = dynamic_cast<const DraftPolyline*>(d.entities()[1].get());
    ASSERT_NE(poly, nullptr) << "a rectangle turned 45 degrees is a polyline";
    EXPECT_TRUE(poly->closed());
    ASSERT_EQ(poly->points().size(), 4u);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_TRUE(near(poly->points()[i], turned(corners[i], Vec2(2, 1), eighth))) << i;
    }
    EXPECT_NEAR(poly->points()[0].distanceTo(poly->points()[1]), 4.0, 1e-9) << "still 4 long";
    EXPECT_NEAR(poly->points()[1].distanceTo(poly->points()[2]), 2.0, 1e-9) << "and 2 wide";
    EXPECT_EQ(poly->lineType(), 2) << "in the rectangle's style";

    stack.push(std::make_unique<hz::doc::RotateEntityCommand>(d, std::vector<uint64_t>{rect->id()},
                                                              Vec2(2, 1), hz::math::kHalfPi));
    ASSERT_EQ(d.entities().size(), 3u);
    const auto* quarter = dynamic_cast<const DraftRectangle*>(d.entities()[2].get());
    ASSERT_NE(quarter, nullptr) << "a quarter turn leaves a rectangle";
    EXPECT_TRUE(near(quarter->corners()[0], Vec2(1, -1)));
    EXPECT_TRUE(near(quarter->corners()[2], Vec2(3, 3)));

    // Mirrored in y = 2x: (x, y) -> ((-3x + 4y) / 5, (4x + 3y) / 5).
    stack.push(std::make_unique<hz::doc::MirrorEntityCommand>(d, std::vector<uint64_t>{rect->id()},
                                                              Vec2(0, 0), Vec2(1, 2)));
    ASSERT_EQ(d.entities().size(), 4u);
    const auto* image = dynamic_cast<const DraftPolyline*>(d.entities()[3].get());
    ASSERT_NE(image, nullptr) << "mirrored in a slanted line, a polyline";
    ASSERT_EQ(image->points().size(), 4u);
    for (size_t i = 0; i < 4; ++i) {
        const Vec2& p = corners[i];
        EXPECT_TRUE(
            near(image->points()[i], Vec2((-3 * p.x + 4 * p.y) / 5.0, (4 * p.x + 3 * p.y) / 5.0)))
            << i;
    }

    stack.push(std::make_unique<hz::doc::MirrorEntityCommand>(d, std::vector<uint64_t>{rect->id()},
                                                              Vec2(0, 0), Vec2(1, 1)));
    ASSERT_EQ(d.entities().size(), 5u);
    const auto* diagonal = dynamic_cast<const DraftRectangle*>(d.entities()[4].get());
    ASSERT_NE(diagonal, nullptr) << "mirrored in a diagonal, a rectangle";
    EXPECT_TRUE(near(diagonal->corners()[0], Vec2(0, 0)));
    EXPECT_TRUE(near(diagonal->corners()[2], Vec2(2, 4)));

    stack.undo();
    stack.undo();
    stack.undo();
    stack.undo();
    EXPECT_EQ(d.entities().size(), 1u);
}

// A rectangle in a block placed at an angle explodes into the polyline the
// block drew, not a box across two of its corners.
TEST(EntityCommandsTest, ARectangleInATurnedBlockExplodesAsDrawn) {
    auto plate = std::make_shared<hz::draft::BlockDefinition>();
    plate->name = "Plate";
    plate->entities.push_back(std::make_shared<hz::draft::DraftRectangle>(Vec2(0, 0), Vec2(4, 2)));
    DraftDocument d;
    d.blockTable().addBlock(plate);
    auto ref = std::make_shared<hz::draft::DraftBlockRef>(plate, Vec2(10, 10), 0.5, 2.0);
    d.addEntity(ref);
    const hz::draft::DraftBlockRef placed = *ref;

    hz::doc::ExplodeBlockCommand explode(d, ref->id());
    explode.execute();
    ASSERT_EQ(d.entities().size(), 1u);
    const auto* poly = dynamic_cast<const hz::draft::DraftPolyline*>(d.entities()[0].get());
    ASSERT_NE(poly, nullptr);
    ASSERT_EQ(poly->points().size(), 4u);
    const auto corners = hz::draft::DraftRectangle(Vec2(0, 0), Vec2(4, 2)).corners();
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_LT(poly->points()[i].distanceTo(placed.transformPoint(corners[i])), 1e-9) << i;
    }
}
