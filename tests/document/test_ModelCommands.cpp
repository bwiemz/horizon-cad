// Edits to a part's history and to an assembly are undo-stack commands: undo
// puts back exactly what was there, redo the same feature object, and the
// document's modified state follows.

#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/FacePlane.h"
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

// -- Editable definitions and rollback (Phase 133) ----------------------------

TEST(ModelCommandsTest, ParametersSayWhatKindTheyAre) {
    using Kind = Feature::ParameterKind;
    const auto revolve = std::make_unique<hz::doc::RevolveFeature>(
        rectangle(1, 0, 2, 1), Vec3(0, 0, 0), Vec3(0, 1, 0), 3.14159);
    EXPECT_EQ(revolve->parameterKind("angle"), Kind::Angle);
    EXPECT_EQ(revolve->parameterKind("segments"), Kind::Count);
    EXPECT_EQ(revolve->parameterKind("chordTolerance"), Kind::Length);
    const auto boolean = std::make_unique<hz::doc::BooleanFeature>(hz::model::BooleanType::Union);
    EXPECT_EQ(boolean->parameterKind("operation"), Kind::Choice);
    const auto circular =
        hz::doc::PatternFeature::makeCircular(Vec3(0, 0, 0), Vec3(0, 0, 1), 0.5, 4);
    EXPECT_EQ(circular->parameterKind("spacing"), Kind::Angle) << "the angle between copies";
    const auto linear = hz::doc::PatternFeature::makeLinear(Vec3(1, 0, 0), 5.0, 3);
    EXPECT_EQ(linear->parameterKind("spacing"), Kind::Length);
}

// A direction can be set, but not one of no length or along the sketch.
TEST(ModelCommandsTest, AnExtrusionsDirectionIsEditableWithinReason) {
    auto feature = extrude(rectangle(0, 0, 2, 2), 3.0, BodyOperation::NewBody);
    EXPECT_FALSE(feature->setVector("direction", Vec3(0, 0, 0)));
    EXPECT_FALSE(feature->setVector("direction", Vec3(1, 0, 0))) << "along its own sketch";
    EXPECT_TRUE(feature->setVector("direction", Vec3(0, 0, -5)));
    EXPECT_NEAR(feature->vectors().at("direction").z, -1.0, 1e-12) << "kept as a unit vector";
    EXPECT_TRUE(Feature::isPoint("axisPoint"));
    EXPECT_FALSE(Feature::isPoint("axisDirection"));
}

// The edit command takes directions too, and undo puts them back.
TEST(ModelCommandsTest, EditingADirectionUndoes) {
    Document doc;
    const Feature* plate = add(doc, extrude(rectangle(0, 0, 10, 10), 2.0, BodyOperation::NewBody));
    ASSERT_TRUE(doc.rebuildModel());
    doc.undoStack().push(std::make_unique<hz::doc::EditFeatureCommand>(
        doc, plate, std::map<std::string, double>{}, std::nullopt,
        std::map<std::string, Vec3>{{"direction", Vec3(0, 0, -1)}}));
    ASSERT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    double lowest = 0.0;
    for (const auto& v : doc.solid()->vertices()) lowest = std::min(lowest, v.point.z);
    EXPECT_NEAR(lowest, -2.0, 1e-9) << "extruded downwards";
    doc.undoStack().undo();
    EXPECT_NEAR(plate->vectors().at("direction").z, 1.0, 1e-12);
}

// Rolling back is one undo step, and leaves the later features out of the
// build until rolled forward.
TEST(ModelCommandsTest, RollingBackIsAnUndoableStep) {
    Plate p;
    EXPECT_NEAR(volume(p.doc), 192.0, 1e-9);
    p.doc.undoStack().push(std::make_unique<hz::doc::SetRollbackCommand>(p.doc, 0));
    EXPECT_EQ(p.doc.featureTree().rollbackIndex(), 0);
    EXPECT_NEAR(volume(p.doc), 200.0, 1e-9) << "the pocket left out";
    p.doc.undoStack().undo();
    EXPECT_EQ(p.doc.featureTree().rollbackIndex(), -1);
    EXPECT_NEAR(volume(p.doc), 192.0, 1e-9);
    p.doc.undoStack().push(std::make_unique<hz::doc::SetRollbackCommand>(p.doc, 1));
    EXPECT_EQ(p.doc.featureTree().rollbackIndex(), -1) << "back to the last feature is no rollback";
}

// Loft and Sweep say why they fail: they returned nothing, and the part said
// only that it could not be built.
TEST(ModelCommandsTest, LoftAndSweepSayWhyTheyFail) {
    auto square = rectangle(0, 0, 2, 2);
    auto triangle = std::make_shared<Sketch>(
        hz::draft::SketchPlane(Vec3(0, 0, 5), Vec3(0, 0, 1), Vec3(1, 0, 0)));
    triangle->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(2, 0)));
    triangle->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(2, 0), Vec2(1, 2)));
    triangle->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(1, 2), Vec2(0, 0)));
    hz::doc::LoftFeature loft({square, triangle});
    std::string why;
    EXPECT_EQ(loft.execute(nullptr, &why), nullptr);
    EXPECT_NE(why.find("corners"), std::string::npos) << why;

    // A path in the profile's own plane sweeps nothing.
    auto path = std::make_shared<Sketch>();
    path->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(10, 0)));
    hz::doc::SweepFeature sweep(rectangle(0, 0, 1, 1), path);
    why.clear();
    EXPECT_EQ(sweep.execute(nullptr, &why), nullptr);
    EXPECT_NE(why.find("sweeps no volume"), std::string::npos) << why;
}

// -- Extrude extents, patterns of features, placed primitives (Phase 134) -----

namespace {

/// A 10 x 10 x 10 box, and the part's volume and height range after the
/// features given.
struct BoxPart {
    Document doc;
    BoxPart() { add(doc, hz::doc::PrimitiveFeature::makeBox(10, 10, 10)); }
    std::pair<double, double> zRange() {
        EXPECT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
        double lo = 1e9;
        double hi = -1e9;
        for (const auto& v : doc.solid()->vertices()) {
            lo = std::min(lo, v.point.z);
            hi = std::max(hi, v.point.z);
        }
        return {lo, hi};
    }
};

/// A square sketch @p size wide at (@p x, @p y) on the plane z = @p z.
std::shared_ptr<Sketch> squareAt(double x, double y, double size, double z) {
    auto sketch = rectangle(x, y, x + size, y + size);
    sketch->setPlane(hz::draft::SketchPlane(Vec3(0, 0, z), Vec3(0, 0, 1), Vec3(1, 0, 0)));
    return sketch;
}

}  // namespace

TEST(ModelCommandsTest, ASymmetricExtrusionGoesHalfEachWay) {
    Document doc;
    auto feature = extrude(rectangle(0, 0, 10, 10), 4.0, BodyOperation::NewBody);
    feature->setExtent(hz::doc::ExtrudeFeature::Extent::Symmetric);
    add(doc, std::move(feature));
    EXPECT_NEAR(volume(doc), 400.0, 1e-9);
    double lo = 1e9;
    for (const auto& v : doc.solid()->vertices()) lo = std::min(lo, v.point.z);
    EXPECT_NEAR(lo, -2.0, 1e-9);
}

// A cut through all goes through the part whatever its distance says; both
// ways from a sketch in the middle.
TEST(ModelCommandsTest, ACutThroughAllGoesThroughThePart) {
    BoxPart part;
    auto cut = std::make_unique<ExtrudeFeature>(squareAt(4, 4, 2, 10), Vec3(0, 0, -1), 1.0);
    cut->setExtent(hz::doc::ExtrudeFeature::Extent::ThroughAll);
    cut->setOperation(BodyOperation::Cut);
    add(part.doc, std::move(cut));
    EXPECT_NEAR(volume(part.doc), 1000.0 - 40.0, 1e-6);

    BoxPart middle;
    auto both = std::make_unique<ExtrudeFeature>(squareAt(4, 4, 2, 5), Vec3(0, 0, 1), 1.0);
    both->setExtent(hz::doc::ExtrudeFeature::Extent::ThroughAllBoth);
    both->setOperation(BodyOperation::Cut);
    add(middle.doc, std::move(both));
    EXPECT_NEAR(volume(middle.doc), 1000.0 - 40.0, 1e-6);

    // Nothing to go through.
    Document empty;
    auto alone = extrude(rectangle(0, 0, 1, 1), 1.0, BodyOperation::NewBody);
    alone->setExtent(hz::doc::ExtrudeFeature::Extent::ThroughAll);
    add(empty, std::move(alone));
    EXPECT_FALSE(empty.rebuildModel());
    EXPECT_NE(empty.lastBuildMessage().find("no part before it"), std::string::npos)
        << empty.lastBuildMessage();
}

// A pattern of a hole repeats the hole, not the part: three holes.
TEST(ModelCommandsTest, APatternOfAFeatureRepeatsOnlyIt) {
    BoxPart part;
    auto hole = std::make_unique<ExtrudeFeature>(squareAt(1, 1, 2, 10), Vec3(0, 0, -1), 1.0);
    hole->setExtent(hz::doc::ExtrudeFeature::Extent::ThroughAll);
    hole->setOperation(BodyOperation::Cut);
    const Feature* cut = add(part.doc, std::move(hole));
    auto pattern = hz::doc::PatternFeature::makeLinear(Vec3(1, 0, 0), 3.0, 3);
    pattern->setTargets({cut->featureID()});
    add(part.doc, std::move(pattern));
    EXPECT_NEAR(volume(part.doc), 1000.0 - 3 * 40.0, 1e-6);

    // Round the part's middle: four holes, a quarter turn apart.
    BoxPart round;
    auto corner = std::make_unique<ExtrudeFeature>(squareAt(1, 1, 2, 10), Vec3(0, 0, -1), 1.0);
    corner->setExtent(hz::doc::ExtrudeFeature::Extent::ThroughAll);
    corner->setOperation(BodyOperation::Cut);
    const Feature* first = add(round.doc, std::move(corner));
    auto circular =
        hz::doc::PatternFeature::makeCircular(Vec3(5, 5, 0), Vec3(0, 0, 1), hz::math::kPi / 2, 4);
    circular->setTargets({first->featureID()});
    add(round.doc, std::move(circular));
    EXPECT_NEAR(volume(round.doc), 1000.0 - 4 * 40.0, 1e-6);

    // A target that is not there says so.
    BoxPart missing;
    auto lost = hz::doc::PatternFeature::makeLinear(Vec3(1, 0, 0), 3.0, 3);
    lost->setTargets({"extrude_999"});
    add(missing.doc, std::move(lost));
    EXPECT_FALSE(missing.doc.rebuildModel());
    EXPECT_NE(missing.doc.lastBuildMessage().find("extrude_999"), std::string::npos);
}

// Two patterns of one separate body name their copies apart. Named alike, a
// fillet on one copy's edge could find the other copy's after a rebuild.
TEST(ModelCommandsTest, TwoPatternsOfOneBodyNameTheirCopiesApart) {
    BoxPart part;
    auto post = std::make_unique<ExtrudeFeature>(squareAt(20, 0, 2, 0), Vec3(0, 0, 1), 4.0);
    post->setOperation(BodyOperation::NewBody);
    const Feature* body = add(part.doc, std::move(post));
    auto along = hz::doc::PatternFeature::makeLinear(Vec3(1, 0, 0), 5.0, 2);
    along->setTargets({body->featureID()});
    add(part.doc, std::move(along));
    auto across = hz::doc::PatternFeature::makeLinear(Vec3(0, 1, 0), 5.0, 2);
    across->setTargets({body->featureID(), body->featureID()});  // repeated once, not twice
    EXPECT_EQ(across->targets().size(), 1u);
    add(part.doc, std::move(across));
    EXPECT_NEAR(volume(part.doc), 1000.0 + 3 * 16.0, 1e-6) << "the box, the post and two copies";

    std::set<std::string> names;
    for (const auto& face : part.doc.solid()->faces()) {
        EXPECT_TRUE(names.insert(face.topoId.tag()).second) << "two faces " << face.topoId.tag();
    }
    names.clear();
    for (const auto& edge : part.doc.solid()->edges()) {
        EXPECT_TRUE(names.insert(edge.topoId.tag()).second) << "two edges " << edge.topoId.tag();
    }
}

// A primitive stands where it is put: a box on its side, along +X from (10, 0, 0).
TEST(ModelCommandsTest, APrimitiveStandsWhereItIsPut) {
    Document doc;
    auto box = hz::doc::PrimitiveFeature::makeBox(2, 3, 4);
    EXPECT_TRUE(box->setVector("basePoint", Vec3(10, 0, 0)));
    EXPECT_TRUE(box->setVector("axisDirection", Vec3(1, 0, 0)));
    EXPECT_TRUE(box->isPlaced());
    add(doc, std::move(box));
    EXPECT_NEAR(volume(doc), 24.0, 1e-9);
    double loX = 1e9;
    double hiX = -1e9;
    for (const auto& v : doc.solid()->vertices()) {
        loX = std::min(loX, v.point.x);
        hiX = std::max(hiX, v.point.x);
    }
    EXPECT_NEAR(loX, 10.0, 1e-9);
    EXPECT_NEAR(hiX, 14.0, 1e-9) << "its height, 4, now along x";

    // Upside down: along -Z from the origin.
    Document down;
    auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(1.0, 5.0);
    ASSERT_TRUE(cylinder->setVector("axisDirection", Vec3(0, 0, -1)));
    add(down, std::move(cylinder));
    ASSERT_TRUE(down.rebuildModel());
    double loZ = 1e9;
    for (const auto& v : down.solid()->vertices()) loZ = std::min(loZ, v.point.z);
    EXPECT_NEAR(loZ, -5.0, 1e-9);
}

namespace {

/// The whole name of @p doc's face facing +X.
std::string faceFacingX(Document& doc) {
    EXPECT_TRUE(doc.rebuildModel()) << doc.lastBuildMessage();
    const double outward = hz::model::outwardSign(*doc.solid());
    for (const auto& face : doc.solid()->faces()) {
        const auto plane = hz::model::planeOf(face, outward);
        if (plane && plane->normal.x > 0.99) return hz::model::wholeFaceName(face.topoId.tag());
    }
    ADD_FAILURE() << "no face facing +X";
    return {};
}

}  // namespace

// Phase 162: the whole part mirrored in one of its own faces is one solid,
// twice its size; the face is followed, so the part made wider is still
// joined to its image. In a plane clear of it, the image is a second body.
TEST(ModelCommandsTest, AMirrorOfThePartJoinsItsImage) {
    BoxPart part;
    auto mirror = hz::doc::MirrorFeature::make(Vec3(), Vec3(1, 0, 0));
    mirror->setReference("planeFace", faceFacingX(part.doc));
    add(part.doc, std::move(mirror));
    EXPECT_NEAR(volume(part.doc), 2000.0, 1e-6);
    EXPECT_EQ(part.doc.solid()->shellCount(), 1u) << "one solid where they meet";
    ASSERT_TRUE(part.doc.featureTree().feature(0)->setParameter("width", 15.0));
    EXPECT_NEAR(volume(part.doc), 3000.0, 1e-6) << "the face followed";
    EXPECT_EQ(part.doc.solid()->shellCount(), 1u);

    BoxPart apart;
    add(apart.doc, hz::doc::MirrorFeature::make(Vec3(30, 0, 0), Vec3(1, 0, 0)));
    EXPECT_NEAR(volume(apart.doc), 2000.0, 1e-6);
    EXPECT_EQ(apart.doc.solid()->shellCount(), 2u) << "two bodies";
}

// A mirror of a feature mirrors only what it adds or cuts: a hole by one
// side, its image by the other, named apart. A face that is gone says so.
TEST(ModelCommandsTest, AMirrorOfAFeatureMirrorsOnlyIt) {
    BoxPart part;
    auto hole = std::make_unique<ExtrudeFeature>(squareAt(1, 1, 2, 10), Vec3(0, 0, -1), 1.0);
    hole->setExtent(hz::doc::ExtrudeFeature::Extent::ThroughAll);
    hole->setOperation(BodyOperation::Cut);
    const Feature* cut = add(part.doc, std::move(hole));
    auto mirror = hz::doc::MirrorFeature::make(Vec3(5, 0, 0), Vec3(1, 0, 0));
    mirror->setTargets({cut->featureID(), cut->featureID()});  // mirrored once, not twice
    EXPECT_EQ(mirror->targets().size(), 1u);
    add(part.doc, std::move(mirror));
    EXPECT_NEAR(volume(part.doc), 1000.0 - 2 * 40.0, 1e-6);
    std::set<std::string> names;
    for (const auto& face : part.doc.solid()->faces()) {
        EXPECT_TRUE(names.insert(face.topoId.tag()).second) << "two faces " << face.topoId.tag();
    }
    // The image is on the other side: nothing is cut at x = 2, something at x = 8.
    bool mirrored = false;
    for (const auto& v : part.doc.solid()->vertices()) {
        mirrored = mirrored || (std::abs(v.point.x - 7.0) < 1e-9 && v.point.y > 0.5);
    }
    EXPECT_TRUE(mirrored) << "a corner of the image's hole at x = 7";

    BoxPart lost;
    auto gone = hz::doc::MirrorFeature::make(Vec3(), Vec3(1, 0, 0));
    gone->setReference("planeFace", "primitive_999/right");
    add(lost.doc, std::move(gone));
    EXPECT_FALSE(lost.doc.rebuildModel());
    EXPECT_NE(lost.doc.lastBuildMessage().find("the plane to mirror in"), std::string::npos)
        << lost.doc.lastBuildMessage();
}
