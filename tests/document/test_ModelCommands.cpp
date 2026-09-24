// Edits to a part's history and to an assembly are undo-stack commands: undo
// puts back exactly what was there, redo the same feature object, and the
// document's modified state follows.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/modeling/MassProperties.h"

using hz::doc::AddFeatureCommand;
using hz::doc::BodyOperation;
using hz::doc::Document;
using hz::doc::ExtrudeFeature;
using hz::doc::Feature;
using hz::doc::Sketch;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

std::shared_ptr<Sketch> rectangle(double x0, double y0, double x1, double y1) {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(x0, y0), Vec2(x1, y0)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(x1, y0), Vec2(x1, y1)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(x1, y1), Vec2(x0, y1)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(x0, y1), Vec2(x0, y0)));
    return sketch;
}

std::unique_ptr<ExtrudeFeature> extrude(std::shared_ptr<Sketch> profile, double height,
                                        BodyOperation operation) {
    auto feature = std::make_unique<ExtrudeFeature>(std::move(profile), Vec3(0, 0, 1), height);
    feature->setOperation(operation);
    return feature;
}

/// Push an add and return the feature it added.
const Feature* add(Document& doc, std::unique_ptr<Feature> feature,
                   std::shared_ptr<Sketch> sketch = nullptr) {
    auto command = std::make_unique<AddFeatureCommand>(doc, std::move(feature), std::move(sketch));
    const Feature* added = command->feature();
    doc.undoStack().push(std::move(command));
    return added;
}

double volume(Document& doc) {
    EXPECT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

/// A 10 x 10 x 2 plate (200) with a 2 x 2 pocket cut through it (192).
struct Plate {
    Document doc;
    const Feature* plate = nullptr;
    const Feature* pocket = nullptr;
    Plate() {
        plate = add(doc, extrude(rectangle(0, 0, 10, 10), 2.0, BodyOperation::NewBody));
        pocket = add(doc, extrude(rectangle(4, 4, 6, 6), 2.0, BodyOperation::Cut));
        doc.setDirty(false);  // saved
    }
};

}  // namespace

TEST(ModelCommandsTest, AddUndoRedoKeepsTheSameFeature) {
    Plate p;
    EXPECT_NEAR(volume(p.doc), 192.0, 1e-6);

    p.doc.undoStack().undo();
    EXPECT_EQ(p.doc.featureTree().featureCount(), 1u);
    EXPECT_NEAR(volume(p.doc), 200.0, 1e-6);
    EXPECT_TRUE(p.doc.isDirty());

    p.doc.undoStack().redo();
    ASSERT_EQ(p.doc.featureTree().featureCount(), 2u);
    EXPECT_EQ(p.doc.featureTree().feature(1), p.pocket) << "the same object, not a copy";
    EXPECT_NEAR(volume(p.doc), 192.0, 1e-6);
    EXPECT_FALSE(p.doc.isDirty()) << "back at the saved state";
}

TEST(ModelCommandsTest, AnAddTakesItsNewSketchAlongButLeavesAnExistingOne) {
    Document doc;
    const size_t before = doc.sketches().size();
    auto fresh = rectangle(0, 0, 1, 1);
    add(doc, extrude(fresh, 1.0, BodyOperation::NewBody), fresh);
    EXPECT_EQ(doc.sketches().size(), before + 1);
    doc.undoStack().undo();
    EXPECT_EQ(doc.sketches().size(), before) << "the profile sketch made for it goes too";
    doc.undoStack().redo();
    EXPECT_EQ(doc.sketches().size(), before + 1);

    // A sketch that was already in the document is not the add's to remove.
    auto existing = rectangle(0, 0, 1, 1);
    doc.addSketch(existing);
    const size_t withExisting = doc.sketches().size();
    add(doc, extrude(existing, 1.0, BodyOperation::Join), existing);
    doc.undoStack().undo();
    EXPECT_EQ(doc.sketches().size(), withExisting);
}

TEST(ModelCommandsTest, DeleteUndoPutsTheFeatureBackWhereItWas) {
    Plate p;
    auto third = extrude(rectangle(0, 0, 1, 1), 1.0, BodyOperation::Cut);
    const Feature* corner = third.get();
    add(p.doc, std::move(third));

    p.doc.undoStack().push(std::make_unique<hz::doc::RemoveFeatureCommand>(p.doc, p.pocket));
    ASSERT_EQ(p.doc.featureTree().featureCount(), 2u);
    EXPECT_EQ(p.doc.featureTree().feature(1), corner);

    p.doc.undoStack().undo();
    ASSERT_EQ(p.doc.featureTree().featureCount(), 3u);
    EXPECT_EQ(p.doc.featureTree().feature(1), p.pocket);
    EXPECT_EQ(p.doc.featureTree().feature(2), corner);
}

TEST(ModelCommandsTest, ReorderUndoes) {
    Plate p;
    p.doc.undoStack().push(std::make_unique<hz::doc::MoveFeatureCommand>(p.doc, p.pocket, 0));
    EXPECT_EQ(p.doc.featureTree().feature(0), p.pocket);
    EXPECT_FALSE(p.doc.rebuildModel()) << "a cut with nothing to cut from fails";

    p.doc.undoStack().undo();
    EXPECT_EQ(p.doc.featureTree().feature(0), p.plate);
    EXPECT_NEAR(volume(p.doc), 192.0, 1e-6);
}

TEST(ModelCommandsTest, EditUndoRestoresParametersAndOperation) {
    Plate p;
    // Make the pocket a 4-tall boss instead.
    p.doc.undoStack().push(std::make_unique<hz::doc::EditFeatureCommand>(
        p.doc, p.pocket, std::map<std::string, double>{{"distance", 4.0}}, BodyOperation::Join));
    EXPECT_NEAR(volume(p.doc), 200.0 + 2.0 * 2.0 * 2.0, 1e-6) << "the boss rises 2 above";
    EXPECT_TRUE(p.doc.isDirty()) << "an edit is a change to save";

    p.doc.undoStack().undo();
    EXPECT_EQ(p.pocket->operation(), BodyOperation::Cut);
    EXPECT_DOUBLE_EQ(p.pocket->parameters().at("distance"), 2.0);
    EXPECT_NEAR(volume(p.doc), 192.0, 1e-6);
    EXPECT_FALSE(p.doc.isDirty());
}

TEST(ModelCommandsTest, ASuppressedCutLeavesThePlateWhole) {
    Plate p;
    p.doc.undoStack().push(
        std::make_unique<hz::doc::SetFeatureSuppressedCommand>(p.doc, p.pocket, true));
    EXPECT_TRUE(p.pocket->isSuppressed());
    EXPECT_NEAR(volume(p.doc), 200.0, 1e-6);
    EXPECT_EQ(p.doc.failedFeatureIndex(), -1);

    // Every build path agrees.
    const auto built = p.doc.featureTree().build();
    ASSERT_NE(built, nullptr);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*built).volume, 200.0, 1e-6);
    const auto bodies = p.doc.featureTree().buildBodies();
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*bodies[0]).volume, 200.0, 1e-6);

    p.doc.undoStack().undo();
    EXPECT_FALSE(p.pocket->isSuppressed());
    EXPECT_NEAR(volume(p.doc), 192.0, 1e-6);
}

TEST(ModelCommandsTest, UndoActsOnItsOwnFeatureAfterAnEditOutsideTheStack) {
    Plate p;
    // A script (say) appends a feature without the undo stack.
    auto outside = extrude(rectangle(20, 0, 21, 1), 1.0, BodyOperation::NewBody);
    const Feature* scripted = outside.get();
    p.doc.featureTree().addFeature(std::move(outside));

    p.doc.undoStack().undo();  // undoes the pocket's add
    ASSERT_EQ(p.doc.featureTree().featureCount(), 2u);
    EXPECT_EQ(p.doc.featureTree().feature(0), p.plate);
    EXPECT_EQ(p.doc.featureTree().feature(1), scripted) << "not the last feature: its own";
}

TEST(ModelCommandsTest, TheRevisionMovesWithEveryChangeToTheBuild) {
    Plate p;
    auto& tree = p.doc.featureTree();
    auto last = tree.revision();
    const auto moved = [&] {
        const bool changed = tree.revision() != last;
        last = tree.revision();
        return changed;
    };
    tree.setRollbackIndex(0);
    EXPECT_TRUE(moved());
    tree.setRollbackIndex(0);
    EXPECT_FALSE(moved()) << "setting the same index is not a change";
    tree.setRollbackIndex(-1);
    EXPECT_TRUE(moved());
    auto taken = tree.takeFeature(1);
    EXPECT_TRUE(moved());
    tree.insertFeature(1, std::move(taken));
    EXPECT_TRUE(moved());
    tree.moveFeature(0, 1);
    EXPECT_TRUE(moved());
    p.doc.undoStack().push(std::make_unique<hz::doc::EditFeatureCommand>(
        p.doc, p.plate, std::map<std::string, double>{{"distance", 3.0}}));
    EXPECT_TRUE(moved()) << "an in-place edit marks the tree changed";
    p.doc.undoStack().undo();
    EXPECT_TRUE(moved());
}

TEST(ModelCommandsTest, TheRollbackIndexFollowsInsertsAndRemovals) {
    Document doc;
    auto& tree = doc.featureTree();
    for (int i = 0; i < 4; ++i) tree.addFeature(hz::doc::PrimitiveFeature::makeBox(1, 1, 1));
    const Feature* second = tree.feature(1);
    tree.setRollbackIndex(1);  // features 0 and 1 active

    tree.insertFeature(0, hz::doc::PrimitiveFeature::makeBox(1, 1, 1));
    EXPECT_EQ(tree.rollbackIndex(), 2);
    EXPECT_EQ(tree.feature(2), second) << "still the last active feature";

    tree.takeFeature(0);
    EXPECT_EQ(tree.rollbackIndex(), 1);
    EXPECT_EQ(tree.feature(1), second);

    tree.takeFeature(3);  // after the rollback: no effect on it
    EXPECT_EQ(tree.rollbackIndex(), 1);

    // Delete-then-undo restores the index exactly.
    doc.undoStack().push(std::make_unique<hz::doc::RemoveFeatureCommand>(doc, second));
    EXPECT_EQ(tree.rollbackIndex(), 0);
    doc.undoStack().undo();
    EXPECT_EQ(tree.rollbackIndex(), 1);
    EXPECT_EQ(tree.feature(1), second);

    while (tree.featureCount() > 0) tree.takeFeature(0);
    EXPECT_EQ(tree.rollbackIndex(), -1) << "an empty tree has nothing rolled back";
}

TEST(ModelCommandsTest, AnAddedFeatureIsActiveAndUndoRestoresTheRollback) {
    Plate p;
    p.doc.featureTree().setRollbackIndex(0);
    add(p.doc, extrude(rectangle(0, 0, 1, 1), 1.0, BodyOperation::Cut));
    EXPECT_EQ(p.doc.featureTree().rollbackIndex(), -1);
    p.doc.undoStack().undo();
    EXPECT_EQ(p.doc.featureTree().rollbackIndex(), 0);
}

TEST(ModelCommandsTest, AnAssemblyEditIsOneUndoStep) {
    Document backing;
    hz::doc::AssemblyDocument assembly;
    hz::doc::ComponentInstance first;
    first.name = "first";
    assembly.addComponent(first);
    assembly.setDirty(false);

    // Insert a component and move it (as a mate solve would), then record.
    const hz::doc::AssemblyState before = assembly.snapshot();
    hz::doc::ComponentInstance second;
    second.name = "second";
    const uint64_t secondId = assembly.addComponent(second);
    assembly.component(secondId)->transform = hz::math::Mat4::translation(Vec3(5, 0, 0));
    backing.undoStack().push(std::make_unique<hz::doc::AssemblyEditCommand>(
        assembly, before, assembly.snapshot(), "Insert Component"));
    EXPECT_EQ(assembly.components().size(), 2u) << "pushing leaves the edit in place";

    backing.undoStack().undo();
    EXPECT_EQ(assembly.components().size(), 1u);
    backing.undoStack().redo();
    ASSERT_NE(assembly.component(secondId), nullptr);
    EXPECT_NEAR(assembly.component(secondId)->transform.transformPoint(Vec3(0, 0, 0)).x, 5.0,
                1e-12);

    // Ids given out before an undo are not given out again.
    backing.undoStack().undo();
    EXPECT_NE(assembly.addComponent(hz::doc::ComponentInstance{}), secondId);
}

TEST(ModelCommandsTest, AMoveAcrossTheRollbackBarLeavesTheOthersWhereTheyWere) {
    Document doc;
    auto& tree = doc.featureTree();
    tree.addFeature(hz::doc::PrimitiveFeature::makeBox(1, 1, 1));
    tree.addFeature(hz::doc::PrimitiveFeature::makeBox(2, 2, 2));
    const Feature* first = tree.feature(0);
    const Feature* second = tree.feature(1);
    tree.setRollbackIndex(0);  // only the first is active

    // Dragging the rolled-back feature above the bar makes it active, and the
    // one that was active stays so — the index used to stay at 0, swapping
    // which of the two was built.
    doc.undoStack().push(std::make_unique<hz::doc::MoveFeatureCommand>(doc, second, 0));
    EXPECT_EQ(tree.feature(0), second);
    EXPECT_EQ(tree.feature(1), first);
    EXPECT_EQ(tree.rollbackIndex(), 1) << "both are above the bar";

    doc.undoStack().undo();
    EXPECT_EQ(tree.feature(0), first);
    EXPECT_EQ(tree.rollbackIndex(), 0) << "undo restores the bar exactly";

    // Moving the active feature below a rolled-back one rolls it back.
    tree.addFeature(hz::doc::PrimitiveFeature::makeBox(3, 3, 3));  // [first, second, third]
    tree.setRollbackIndex(1);                                      // first, second active
    tree.moveFeature(1, 2);                                        // [first, third, second]
    EXPECT_EQ(tree.rollbackIndex(), 0) << "only first stays active";
}

TEST(ModelCommandsTest, UndoingACountEditRestoresTheChordTolerance) {
    // Setting a facet count returns a feature to count mode, clearing its
    // chord tolerance. Undo must put the tolerance back, not only the count.
    Document doc;
    auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(5.0, 10.0);
    ASSERT_TRUE(cylinder->setParameter("chordTolerance", 0.01));
    const Feature* feature = cylinder.get();
    doc.featureTree().addFeature(std::move(cylinder));
    const auto before = feature->parameters();

    doc.undoStack().push(std::make_unique<hz::doc::EditFeatureCommand>(
        doc, feature, std::map<std::string, double>{{"segments", 12.0}}));
    EXPECT_DOUBLE_EQ(feature->parameters().at("chordTolerance"), 0.0);
    EXPECT_DOUBLE_EQ(feature->parameters().at("segments"), 12.0);

    doc.undoStack().undo();
    EXPECT_EQ(feature->parameters(), before);
}

TEST(ModelCommandsTest, RefusedParametersLeaveTheFeatureAlone) {
    auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(5.0, 10.0);
    ASSERT_TRUE(cylinder->setParameter("chordTolerance", 0.01));
    const auto before = cylinder->parameters();

    // Two facets bound no volume; the radius and height are fine.
    const auto refused = hz::doc::refusedParameters(
        *cylinder, {{"segments", 2.0}, {"height", 4.0}, {"radius", 3.0}});
    EXPECT_EQ(refused, std::vector<std::string>{"segments"});
    EXPECT_EQ(cylinder->parameters(), before) << "trying does not change it";
}
