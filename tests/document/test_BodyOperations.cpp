// How a feature's body combines with the part: New body, Join, Cut,
// Intersect. Volumes are closed-form, as elsewhere in the kernel tests.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::doc::BodyOperation;
using hz::doc::Document;
using hz::doc::ExtrudeFeature;
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

std::shared_ptr<Sketch> circle(double cx, double cy, double r) {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(cx, cy), r));
    return sketch;
}

std::unique_ptr<ExtrudeFeature> extrude(std::shared_ptr<Sketch> profile, double height,
                                        BodyOperation operation) {
    auto feature = std::make_unique<ExtrudeFeature>(std::move(profile), Vec3(0, 0, 1), height);
    feature->setOperation(operation);
    return feature;
}

double volumeOf(const hz::topo::Solid& solid) {
    return hz::model::MassPropertiesCalculator::compute(solid).volume;
}

/// Rebuild through the product path and require success.
const hz::topo::Solid& rebuilt(Document& doc) {
    const bool ok = doc.rebuildModel();
    EXPECT_TRUE(ok) << doc.lastBuildMessage();
    static const hz::topo::Solid empty;
    return doc.solid() ? *doc.solid() : empty;
}

}  // namespace

TEST(BodyOperationsTest, APlateWithAHole) {
    // The case the single-body rebuild could not model at all: the second
    // extrude used to replace the first.
    Document doc;
    doc.featureTree().addFeature(extrude(rectangle(0, 0, 10, 10), 2.0, BodyOperation::NewBody));
    auto hole = extrude(circle(5, 5, 2), 2.0, BodyOperation::Cut);
    const int n = hole->segments();
    doc.featureTree().addFeature(std::move(hole));

    const hz::topo::Solid& part = rebuilt(doc);
    // The hole is the inscribed n-gon prism.
    const double holeArea = 0.5 * n * 4.0 * std::sin(2.0 * hz::math::kPi / n);
    EXPECT_NEAR(volumeOf(part), 200.0 - 2.0 * holeArea, 1e-6);
    EXPECT_EQ(part.shells().size(), 1u);
}

TEST(BodyOperationsTest, JoinUnitesOverlappingBodies) {
    Document doc;
    doc.featureTree().addFeature(extrude(rectangle(0, 0, 10, 10), 10.0, BodyOperation::NewBody));
    doc.featureTree().addFeature(extrude(rectangle(5, 0, 15, 10), 10.0, BodyOperation::Join));
    const hz::topo::Solid& part = rebuilt(doc);
    EXPECT_NEAR(volumeOf(part), 1500.0, 1e-6) << "shared material counted once";
    EXPECT_EQ(part.shells().size(), 1u);
}

TEST(BodyOperationsTest, IntersectKeepsWhatBothShare) {
    Document doc;
    doc.featureTree().addFeature(extrude(rectangle(0, 0, 10, 10), 10.0, BodyOperation::NewBody));
    doc.featureTree().addFeature(extrude(rectangle(5, 0, 15, 10), 10.0, BodyOperation::Intersect));
    EXPECT_NEAR(volumeOf(rebuilt(doc)), 500.0, 1e-6);
}

TEST(BodyOperationsTest, NewBodyKeepsBothBodies) {
    Document doc;
    doc.featureTree().addFeature(extrude(rectangle(0, 0, 10, 10), 10.0, BodyOperation::NewBody));
    doc.featureTree().addFeature(extrude(rectangle(20, 0, 30, 10), 10.0, BodyOperation::NewBody));
    const hz::topo::Solid& part = rebuilt(doc);
    EXPECT_NEAR(volumeOf(part), 2000.0, 1e-6) << "the first body is no longer discarded";
    EXPECT_EQ(part.shells().size(), 2u);
}

TEST(BodyOperationsTest, TheFirstBodyJoinsNothing) {
    // A lone Join (a new part's first feature, made with the default) is the
    // body itself.
    Document doc;
    doc.featureTree().addFeature(extrude(rectangle(0, 0, 10, 10), 3.0, BodyOperation::Join));
    EXPECT_NEAR(volumeOf(rebuilt(doc)), 300.0, 1e-6);
}

TEST(BodyOperationsTest, CuttingWithNoBodyFailsWithAReason) {
    Document doc;
    doc.featureTree().addFeature(extrude(circle(0, 0, 1), 1.0, BodyOperation::Cut));
    EXPECT_FALSE(doc.rebuildModel());
    EXPECT_EQ(doc.failedFeatureIndex(), 0);
    EXPECT_NE(doc.lastBuildMessage().find("no body to cut"), std::string::npos)
        << doc.lastBuildMessage();
}

TEST(BodyOperationsTest, ACutThatRemovesEverythingFailsWithAReason) {
    Document doc;
    doc.featureTree().addFeature(extrude(rectangle(0, 0, 10, 10), 10.0, BodyOperation::NewBody));
    doc.featureTree().addFeature(extrude(rectangle(-1, -1, 11, 11), 12.0, BodyOperation::Cut));
    EXPECT_FALSE(doc.rebuildModel());
    EXPECT_EQ(doc.failedFeatureIndex(), 1);
    EXPECT_NE(doc.lastBuildMessage().find("removes the whole body"), std::string::npos)
        << doc.lastBuildMessage();
}

TEST(BodyOperationsTest, EveryBuildPathAgrees) {
    Document doc;
    doc.featureTree().addFeature(extrude(rectangle(0, 0, 10, 10), 10.0, BodyOperation::NewBody));
    doc.featureTree().addFeature(extrude(rectangle(5, 0, 15, 10), 10.0, BodyOperation::Join));
    doc.featureTree().addFeature(extrude(circle(7.5, 5, 2), 10.0, BodyOperation::Cut));

    const auto built = doc.featureTree().build();
    ASSERT_NE(built, nullptr);
    const auto bodies = doc.featureTree().buildBodies();
    ASSERT_EQ(bodies.size(), 1u);
    const double product = volumeOf(rebuilt(doc));
    EXPECT_NEAR(volumeOf(*built), product, 1e-9);
    EXPECT_NEAR(volumeOf(*bodies[0]), product, 1e-9);
}

TEST(BodyOperationsTest, NamesRoundTrip) {
    for (const BodyOperation op : {BodyOperation::NewBody, BodyOperation::Join, BodyOperation::Cut,
                                   BodyOperation::Intersect}) {
        EXPECT_EQ(hz::doc::bodyOperationFromName(hz::doc::bodyOperationName(op)), op);
    }
    EXPECT_FALSE(hz::doc::bodyOperationFromName("union").has_value());
}

// ---------------------------------------------------------------------------
// Combining bodies (Phase 104b): the Boolean feature on the product path
// ---------------------------------------------------------------------------

TEST(BodyOperationsTest, CombiningBodiesFoldsThemInOrder) {
    // Two New-body blocks overlapping by half: 1000 each, 500 shared.
    struct Case {
        hz::model::BooleanType type;
        double volume;
    };
    for (const Case c : {Case{hz::model::BooleanType::Union, 1500.0},
                         Case{hz::model::BooleanType::Subtract, 500.0},
                         Case{hz::model::BooleanType::Intersect, 500.0}}) {
        Document doc;
        doc.featureTree().addFeature(
            extrude(rectangle(0, 0, 10, 10), 10.0, BodyOperation::NewBody));
        doc.featureTree().addFeature(
            extrude(rectangle(5, 0, 15, 10), 10.0, BodyOperation::NewBody));
        ASSERT_EQ(rebuilt(doc).shells().size(), 2u);

        doc.featureTree().addFeature(std::make_unique<hz::doc::BooleanFeature>(c.type));
        const hz::topo::Solid& part = rebuilt(doc);
        EXPECT_EQ(part.shells().size(), 1u);
        EXPECT_NEAR(volumeOf(part), c.volume, 1e-6);

        // The other build paths agree.
        const auto built = doc.featureTree().build();
        ASSERT_NE(built, nullptr);
        EXPECT_NEAR(volumeOf(*built), c.volume, 1e-6);
        const auto bodies = doc.featureTree().buildBodies();
        ASSERT_EQ(bodies.size(), 1u);
        EXPECT_NEAR(volumeOf(*bodies[0]), c.volume, 1e-6);
    }
}

TEST(BodyOperationsTest, CombiningOneBodyLeavesIt) {
    Document doc;
    doc.featureTree().addFeature(extrude(rectangle(0, 0, 10, 10), 3.0, BodyOperation::NewBody));
    doc.featureTree().addFeature(
        std::make_unique<hz::doc::BooleanFeature>(hz::model::BooleanType::Subtract));
    EXPECT_NEAR(volumeOf(rebuilt(doc)), 300.0, 1e-6);
}

TEST(BodyOperationsTest, CombiningWhatDoesNotOverlapSaysSo) {
    Document doc;
    doc.featureTree().addFeature(extrude(rectangle(0, 0, 1, 1), 1.0, BodyOperation::NewBody));
    doc.featureTree().addFeature(extrude(rectangle(5, 0, 6, 1), 1.0, BodyOperation::NewBody));
    doc.featureTree().addFeature(
        std::make_unique<hz::doc::BooleanFeature>(hz::model::BooleanType::Intersect));
    EXPECT_FALSE(doc.rebuildModel());
    EXPECT_EQ(doc.failedFeatureIndex(), 2);
    EXPECT_NE(doc.lastBuildMessage().find("do not overlap"), std::string::npos)
        << doc.lastBuildMessage();
}

// Two bodies that overlap, then a cut and a pocket through the overlap. The
// CSG read the part as one skin, so the second body's faces inside the first
// were taken for the part's outside: the cut lost its walls and was refused,
// and a pocket was lost. Each body is cut on its own now.
TEST(BodyOperationsTest, ACutThroughOverlappingBodiesCutsEach) {
    Document doc;
    doc.featureTree().addFeature(extrude(rectangle(0, 0, 10, 10), 10.0, BodyOperation::NewBody));
    doc.featureTree().addFeature(extrude(rectangle(5, 0, 15, 10), 10.0, BodyOperation::NewBody));
    doc.featureTree().addFeature(extrude(rectangle(1, -1, 3, 11), 10.0, BodyOperation::Cut));
    const auto& part = rebuilt(doc);
    EXPECT_EQ(part.shellCount(), 3u) << "the slot cuts the first body in two";
    // The first body less a 2 x 10 slot through it; the second untouched.
    EXPECT_NEAR(volumeOf(part), (1000.0 - 200.0) + 1000.0, 1e-6);

    // A pocket inside the overlap is taken from both.
    Document pocketed;
    pocketed.featureTree().addFeature(
        extrude(rectangle(0, 0, 10, 10), 10.0, BodyOperation::NewBody));
    pocketed.featureTree().addFeature(
        extrude(rectangle(5, 0, 15, 10), 10.0, BodyOperation::NewBody));
    pocketed.featureTree().addFeature(extrude(rectangle(6, 2, 8, 4), 5.0, BodyOperation::Cut));
    EXPECT_NEAR(volumeOf(rebuilt(pocketed)), 2000.0 - 2 * 20.0, 1e-6);
}

TEST(BodyOperationsTest, JoinAndIntersectWithOverlappingBodies) {
    Document joined;
    joined.featureTree().addFeature(extrude(rectangle(0, 0, 10, 10), 10.0, BodyOperation::NewBody));
    joined.featureTree().addFeature(extrude(rectangle(5, 0, 15, 10), 10.0, BodyOperation::NewBody));
    joined.featureTree().addFeature(
        extrude(rectangle(30, 0, 40, 10), 10.0, BodyOperation::NewBody));  // apart
    joined.featureTree().addFeature(extrude(rectangle(2, 2, 12, 4), 20.0, BodyOperation::Join));
    const auto& part = rebuilt(joined);
    EXPECT_EQ(part.shellCount(), 2u) << "the two it met, joined; the one apart, kept";
    EXPECT_NEAR(volumeOf(part), 1500.0 + 1000.0 + 20.0 * 10.0, 1e-6);

    Document cut;
    cut.featureTree().addFeature(extrude(rectangle(0, 0, 10, 10), 10.0, BodyOperation::NewBody));
    cut.featureTree().addFeature(extrude(rectangle(5, 0, 15, 10), 10.0, BodyOperation::NewBody));
    cut.featureTree().addFeature(extrude(rectangle(8, 0, 9, 10), 10.0, BodyOperation::Intersect));
    const auto& common = rebuilt(cut);
    EXPECT_EQ(common.shellCount(), 2u) << "each body's share, a body each";
    EXPECT_NEAR(volumeOf(common), 2 * 100.0, 1e-6);
}

// A box primitive and a cylinder primitive, as two bodies: the box's loops
// are wound in and the cylinder's out, and gathered as they were the part
// measured as the box less the cylinder.
TEST(BodyOperationsTest, BodiesWoundEitherWayMeasureAsBoth) {
    Document doc;
    doc.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(10, 10, 10));
    auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(2, 10);
    cylinder->setVector("basePoint", Vec3(30, 0, 0));
    doc.featureTree().addFeature(std::move(cylinder));
    const auto& part = rebuilt(doc);
    EXPECT_EQ(part.shellCount(), 2u);
    const double cylinderVolume = volumeOf(*hz::model::PrimitiveFactory::makeCylinder(2, 10, 32));
    EXPECT_NEAR(volumeOf(part), 1000.0 + cylinderVolume, 1e-6);
}
