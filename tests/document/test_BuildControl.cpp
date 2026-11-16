// A build that says how far it has got and stops when asked (Phase 114): what
// a rebuild on a worker thread needs from the feature tree.

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/modeling/MassProperties.h"

using hz::doc::BuildControl;
using hz::doc::Document;
using hz::doc::PatternFeature;
using hz::doc::PrimitiveFeature;
using hz::math::Vec3;

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

// A build cancelled in a long feature stops in it, not at its end: a pattern
// of 40 overlapping cylinders of 1,024 facets, joined for tens of seconds,
// stops within moments of the cancel, where the window waited it out.
TEST(BuildControlTest, ABuildCancelledInALongFeatureStopsInIt) {
    Document doc;
    auto cylinder = PrimitiveFeature::makeCylinder(5.0, 10.0);
    ASSERT_TRUE(cylinder->setParameter("segments", 1024));
    doc.featureTree().addFeature(std::move(cylinder));
    auto grid = PatternFeature::makeLinear(Vec3(1, 0, 0), 7.0, 8);
    ASSERT_TRUE(grid->setSecondDirection(Vec3(0, 1, 0), 7.0, 5));
    doc.featureTree().addFeature(std::move(grid));

    BuildControl control;
    hz::doc::BuildResult result;
    std::thread worker([&] { result = doc.featureTree().buildWithDiagnostics(&control); });
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    const auto asked = std::chrono::steady_clock::now();
    control.cancel = true;
    worker.join();
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - asked).count();
    EXPECT_TRUE(result.cancelled);
    EXPECT_EQ(result.solid, nullptr);
    // The pattern alone takes over 20 s optimized; stopped, a moment.
    EXPECT_LT(seconds, 5.0);
}
