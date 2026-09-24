// A build that says how far it has got and stops when asked (Phase 114): what
// a rebuild on a worker thread needs from the feature tree.

#include <gtest/gtest.h>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/modeling/MassProperties.h"

using hz::doc::BuildControl;
using hz::doc::Document;
using hz::doc::PrimitiveFeature;

namespace {

double volumeOf(const Document& doc) {
    return doc.solid() ? hz::model::MassPropertiesCalculator::compute(*doc.solid()).volume : 0.0;
}

}  // namespace

TEST(BuildControlTest, ABuildCountsItsFeatures) {
    Document doc;
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(2, 3, 4));
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(1, 1, 1));
    BuildControl control;
    const auto result = doc.featureTree().buildWithDiagnostics(&control);
    EXPECT_FALSE(result.cancelled);
    EXPECT_NE(result.solid, nullptr);
    EXPECT_EQ(control.total.load(), 2);
    EXPECT_EQ(control.done.load(), 2);
}

TEST(BuildControlTest, ACancelledBuildStopsAndLeavesTheModelAsItWas) {
    Document doc;
    doc.featureTree().addFeature(PrimitiveFeature::makeBox(2, 3, 4));
    ASSERT_TRUE(doc.rebuildModel());
    ASSERT_NEAR(volumeOf(doc), 24.0, 1e-9);

    doc.featureTree().addFeature(PrimitiveFeature::makeBox(10, 10, 10));
    BuildControl control;
    control.cancel = true;
    auto result = doc.featureTree().buildWithDiagnostics(&control);
    EXPECT_TRUE(result.cancelled);
    EXPECT_EQ(result.solid, nullptr);
    EXPECT_EQ(control.done.load(), 0) << "it stopped before the first feature";

    EXPECT_TRUE(doc.applyBuild(std::move(result)));
    EXPECT_NEAR(volumeOf(doc), 24.0, 1e-9) << "the last completed model stays";
}
