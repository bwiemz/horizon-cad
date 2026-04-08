#include "horizon/document/FeatureTree.h"

#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/topology/Solid.h"

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

using namespace hz::doc;
using namespace hz::draft;
using hz::math::Vec2;
using hz::math::Vec3;

static constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

// Helper: build a sketch with a rectangle profile.
static std::shared_ptr<Sketch> makeRectSketch(double w, double h) {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(w, 0)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(w, 0), Vec2(w, h)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(w, h), Vec2(0, h)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(0, h), Vec2(0, 0)));
    return sketch;
}

// Helper: build a sketch with a rectangle offset from Y axis (for revolve).
static std::shared_ptr<Sketch> makeOffsetRectSketch() {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(5, 0), Vec2(10, 0)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(10, 0), Vec2(10, 5)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(10, 5), Vec2(5, 5)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(5, 5), Vec2(5, 0)));
    return sketch;
}

// ---------------------------------------------------------------------------
// EmptyTreeReturnsNull
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, EmptyTreeReturnsNull) {
    FeatureTree tree;
    EXPECT_EQ(tree.build(), nullptr);
}

// ---------------------------------------------------------------------------
// FeatureCount
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, FeatureCount) {
    FeatureTree tree;
    EXPECT_EQ(tree.featureCount(), 0u);

    auto sketch = makeRectSketch(10.0, 5.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 3.0));
    EXPECT_EQ(tree.featureCount(), 1u);

    tree.clear();
    EXPECT_EQ(tree.featureCount(), 0u);
}

// ---------------------------------------------------------------------------
// AddAndReplayExtrude
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, AddAndReplayExtrude) {
    FeatureTree tree;
    auto sketch = makeRectSketch(10.0, 5.0);

    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 3.0));

    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

// ---------------------------------------------------------------------------
// ReplayProducesConsistentSolid
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, ReplayProducesConsistentSolid) {
    FeatureTree tree;
    auto sketch = makeRectSketch(4.0, 3.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 2.0));

    auto solid1 = tree.build();
    auto solid2 = tree.build();

    ASSERT_NE(solid1, nullptr);
    ASSERT_NE(solid2, nullptr);
    EXPECT_EQ(solid1->vertexCount(), solid2->vertexCount());
    EXPECT_EQ(solid1->edgeCount(), solid2->edgeCount());
    EXPECT_EQ(solid1->faceCount(), solid2->faceCount());
}

// ---------------------------------------------------------------------------
// AddAndReplayRevolve
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, AddAndReplayRevolve) {
    FeatureTree tree;
    auto sketch = makeOffsetRectSketch();

    tree.addFeature(std::make_unique<RevolveFeature>(
        sketch, Vec3::Zero, Vec3::UnitY, kTwoPi));

    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

// ---------------------------------------------------------------------------
// RemoveFeature
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, RemoveFeature) {
    FeatureTree tree;
    auto sketch = makeRectSketch(5.0, 5.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 1.0));
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 2.0));
    EXPECT_EQ(tree.featureCount(), 2u);

    tree.removeFeature(0);
    EXPECT_EQ(tree.featureCount(), 1u);

    // Remaining feature should still build fine.
    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid());
}

// ---------------------------------------------------------------------------
// FeatureAccess
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, FeatureAccess) {
    FeatureTree tree;
    auto sketch = makeRectSketch(5.0, 5.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 1.0));

    const Feature* f = tree.feature(0);
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->name(), "Extrude");
    EXPECT_FALSE(f->featureID().empty());
}
