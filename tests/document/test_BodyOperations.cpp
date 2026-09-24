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
    EXPECT_NE(doc.lastBuildMessage().find("nothing"), std::string::npos) << doc.lastBuildMessage();
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
