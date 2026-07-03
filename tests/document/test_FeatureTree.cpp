#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/topology/Solid.h"

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

    tree.addFeature(std::make_unique<RevolveFeature>(sketch, Vec3::Zero, Vec3::UnitY, kTwoPi));

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

// ---------------------------------------------------------------------------
// LoftFeatureBuilds
// ---------------------------------------------------------------------------

static std::shared_ptr<Sketch> makeSquareSketchOnPlane(double s, double z) {
    const double h = s * 0.5;
    auto sketch = std::make_shared<Sketch>(
        hz::draft::SketchPlane(Vec3(0, 0, z), Vec3(0, 0, 1), Vec3(1, 0, 0)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(-h, -h), Vec2(h, -h)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(h, -h), Vec2(h, h)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(h, h), Vec2(-h, h)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(-h, h), Vec2(-h, -h)));
    return sketch;
}

TEST(FeatureTreeTest, LoftFeatureBuilds) {
    FeatureTree tree;
    std::vector<std::shared_ptr<Sketch>> sections = {
        makeSquareSketchOnPlane(6.0, 0.0),
        makeSquareSketchOnPlane(3.0, 10.0),
    };
    tree.addFeature(std::make_unique<LoftFeature>(sections));

    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid());
    EXPECT_EQ(solid->faceCount(), 6u);

    const Feature* f = tree.feature(0);
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->name(), "Loft");
    EXPECT_FALSE(f->featureID().empty());
}

// ---------------------------------------------------------------------------
// SweepFeatureBuilds
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, SweepFeatureBuilds) {
    FeatureTree tree;
    auto profile = makeSquareSketchOnPlane(4.0, 0.0);
    // Path sketch: a vertical line, drawn on the XZ plane so it rises in Z.
    auto path = std::make_shared<Sketch>(
        hz::draft::SketchPlane(Vec3(0, 0, 0), Vec3(0, 1, 0), Vec3(1, 0, 0)));
    path->addEntity(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(0, 10)));

    tree.addFeature(std::make_unique<SweepFeature>(profile, path));

    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid());
    EXPECT_EQ(solid->faceCount(), 6u);

    const Feature* f = tree.feature(0);
    ASSERT_NE(f, nullptr);
    EXPECT_EQ(f->name(), "Sweep");
}

// ---------------------------------------------------------------------------
// DraftAndShellFeaturesChain
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, DraftAndShellFeaturesChain) {
    // Extrude a 10x8 rectangle 5 tall, then shell it (remove the top cap).
    FeatureTree tree;
    auto sketch = makeRectSketch(10.0, 8.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 5.0));
    // The extrude cap face id is "<featureID>/cap_top".
    std::string extId = tree.feature(0)->featureID();
    tree.addFeature(std::make_unique<ShellFeature>(
        1.0, std::vector<hz::topo::TopologyID>{hz::topo::TopologyID::make(extId, "cap_top")}));

    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid());
    EXPECT_EQ(solid->faceCount(), 14u);  // cup

    EXPECT_EQ(tree.feature(1)->name(), "Shell");
}

TEST(FeatureTreeTest, DraftFeatureTapers) {
    FeatureTree tree;
    auto sketch = makeRectSketch(10.0, 10.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 5.0));
    tree.addFeature(std::make_unique<DraftFeature>(Vec3(0, 0, 1), Vec3(0, 0, 0), std::atan(0.1)));

    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid());
    EXPECT_EQ(tree.feature(1)->name(), "Draft");
}

// ---------------------------------------------------------------------------
// PatternFeatureReplicates
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, PatternFeatureReplicates) {
    // Extrude a small box, then linear-pattern it 3x.
    FeatureTree tree;
    auto sketch = makeRectSketch(2.0, 2.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 2.0));
    tree.addFeature(PatternFeature::makeLinear(Vec3(1, 0, 0), 5.0, 3));

    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->shellCount(), 3u);
    EXPECT_EQ(solid->faceCount(), 18u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_EQ(tree.feature(1)->name(), "LinearPattern");
}

TEST(FeatureTreeTest, CircularPatternFeature) {
    FeatureTree tree;
    auto sketch = makeRectSketch(1.0, 1.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 1.0));
    tree.addFeature(PatternFeature::makeCircular(Vec3(0, 0, 0), Vec3(0, 0, 1), kTwoPi / 6.0, 6));

    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->shellCount(), 6u);
    EXPECT_EQ(tree.feature(1)->name(), "CircularPattern");
}

// ---------------------------------------------------------------------------
// Datum (reference geometry) features are non-geometric and pass the body
// through unchanged — even when they lead the tree.
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, LeadingDatumDoesNotBreakBuild) {
    FeatureTree tree;
    // A datum plane before any solid; then extrude. The datum must not make
    // the build fail even though there is no input solid when it is reached.
    tree.addFeature(DatumFeature::makePlane(
        hz::model::DatumPlane{Vec3(0, 0, 5), Vec3(0, 0, 1), Vec3(1, 0, 0)}));
    auto sketch = makeRectSketch(4.0, 3.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 2.0));

    EXPECT_TRUE(tree.feature(0)->isConstruction());
    EXPECT_EQ(tree.feature(0)->name(), "DatumPlane");

    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 6u);  // just the box; datum contributes nothing
    EXPECT_TRUE(solid->checkEulerFormula());
}

TEST(FeatureTreeTest, DatumBetweenFeaturesIsTransparent) {
    FeatureTree tree;
    auto sketch = makeRectSketch(2.0, 2.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 2.0));
    // Insert a datum axis in the middle, then pattern — the datum is skipped.
    tree.addFeature(DatumFeature::makeAxis(hz::model::DatumAxis{Vec3::Zero, Vec3(0, 0, 1)}));
    tree.addFeature(PatternFeature::makeLinear(Vec3(1, 0, 0), 5.0, 3));

    auto result = tree.buildWithDiagnostics();
    ASSERT_NE(result.solid, nullptr);
    EXPECT_EQ(result.solid->shellCount(), 3u);
    EXPECT_EQ(result.failedFeatureIndex, -1);
    EXPECT_TRUE(tree.feature(1)->isConstruction());
}

TEST(FeatureTreeTest, DatumAccessorsReconstructGeometry) {
    auto planeFeat =
        DatumFeature::makePlane(hz::model::DatumPlane{Vec3(1, 2, 3), Vec3(0, 0, 1), Vec3(1, 0, 0)});
    EXPECT_EQ(planeFeat->datumKind(), DatumFeature::DatumKind::Plane);
    EXPECT_NEAR(planeFeat->asPlane().origin.z, 3.0, 1e-12);

    auto axisFeat = DatumFeature::makeAxis(hz::model::DatumAxis{Vec3(4, 5, 6), Vec3(0, 1, 0)});
    EXPECT_EQ(axisFeat->datumKind(), DatumFeature::DatumKind::Axis);
    EXPECT_EQ(axisFeat->name(), "DatumAxis");
    EXPECT_NEAR(axisFeat->asAxis().direction.y, 1.0, 1e-12);

    auto pointFeat = DatumFeature::makePoint(hz::model::DatumPoint{Vec3(7, 8, 9)});
    EXPECT_EQ(pointFeat->datumKind(), DatumFeature::DatumKind::Point);
    EXPECT_EQ(pointFeat->name(), "DatumPoint");
    EXPECT_NEAR(pointFeat->asPoint().position.x, 7.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Primitive features — parametric box/cylinder/sphere/cone/torus base features
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, PrimitiveBoxBuilds) {
    FeatureTree tree;
    tree.addFeature(PrimitiveFeature::makeBox(2.0, 3.0, 4.0));
    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_EQ(tree.feature(0)->name(), "Box");
}

TEST(FeatureTreeTest, PrimitiveAllKindsBuildValidSolids) {
    struct Case {
        std::unique_ptr<Feature> feat;
        const char* name;
    };
    std::vector<Case> cases;
    cases.push_back({PrimitiveFeature::makeCylinder(5.0, 10.0), "Cylinder"});
    cases.push_back({PrimitiveFeature::makeSphere(4.0), "Sphere"});
    cases.push_back({PrimitiveFeature::makeCone(4.0, 2.0, 6.0), "Cone"});
    cases.push_back({PrimitiveFeature::makeTorus(8.0, 2.0), "Torus"});
    for (auto& c : cases) {
        const std::string expected = c.name;
        FeatureTree tree;
        EXPECT_EQ(c.feat->name(), expected);
        tree.addFeature(std::move(c.feat));
        auto solid = tree.build();
        ASSERT_NE(solid, nullptr) << expected;
        EXPECT_TRUE(solid->isValid()) << expected;
        EXPECT_TRUE(solid->checkEulerFormula()) << expected;
    }
}

TEST(FeatureTreeTest, PrimitiveParametricEdit) {
    auto box = PrimitiveFeature::makeBox(2.0, 2.0, 2.0);
    EXPECT_DOUBLE_EQ(box->parameters().at("width"), 2.0);
    EXPECT_TRUE(box->setParameter("width", 8.0));
    EXPECT_FALSE(box->setParameter("radius", 5.0));  // not a box parameter

    FeatureTree tree;
    tree.addFeature(std::move(box));
    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    double minX = 1e9, maxX = -1e9;
    for (const auto& v : solid->vertices()) {
        minX = std::min(minX, v.point.x);
        maxX = std::max(maxX, v.point.x);
    }
    EXPECT_NEAR(maxX - minX, 8.0, 1e-9);  // edited width took effect on rebuild
}

// ---------------------------------------------------------------------------
// FilletFeature — parametric edge rounding on the running solid
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, FilletFeatureRoundsEdge) {
    FeatureTree tree;
    tree.addFeature(PrimitiveFeature::makeBox(10.0, 10.0, 10.0));
    auto box = tree.build();
    ASSERT_NE(box, nullptr);
    ASSERT_FALSE(box->edges().empty());
    const auto edgeId = box->edges().front().topoId;
    const size_t boxFaces = box->faceCount();

    tree.addFeature(
        std::make_unique<FilletFeature>(std::vector<hz::topo::TopologyID>{edgeId}, 1.0));
    auto filleted = tree.build();
    ASSERT_NE(filleted, nullptr);
    EXPECT_TRUE(filleted->isValid());
    EXPECT_GT(filleted->faceCount(), boxFaces);  // filleting an edge adds a face
    EXPECT_EQ(tree.feature(1)->name(), "Fillet");
    EXPECT_DOUBLE_EQ(tree.feature(1)->parameters().at("radius"), 1.0);
}

TEST(FeatureTreeTest, ChamferFeatureBevelsEdge) {
    FeatureTree tree;
    tree.addFeature(PrimitiveFeature::makeBox(10.0, 10.0, 10.0));
    auto box = tree.build();
    ASSERT_NE(box, nullptr);
    ASSERT_FALSE(box->edges().empty());
    const auto edgeId = box->edges().front().topoId;
    const size_t boxFaces = box->faceCount();

    tree.addFeature(
        std::make_unique<ChamferFeature>(std::vector<hz::topo::TopologyID>{edgeId}, 1.0));
    auto chamfered = tree.build();
    ASSERT_NE(chamfered, nullptr);
    EXPECT_TRUE(chamfered->isValid());
    EXPECT_GT(chamfered->faceCount(), boxFaces);  // a chamfer face was added
    EXPECT_EQ(tree.feature(1)->name(), "Chamfer");
    EXPECT_DOUBLE_EQ(tree.feature(1)->parameters().at("distance"), 1.0);
}
