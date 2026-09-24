#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/MassProperties.h"
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

    // A full revolution of a profile clear of the axis is a torus: manifold,
    // closed, and genus 1.
    tree.addFeature(std::make_unique<RevolveFeature>(sketch, Vec3::Zero, Vec3::UnitY, kTwoPi));

    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_EQ(solid->genus(), 1);

    // A partial revolution is capped at both ends and so is a genus-0 solid.
    FeatureTree partial;
    partial.addFeature(
        std::make_unique<RevolveFeature>(sketch, Vec3::Zero, Vec3::UnitY, kTwoPi * 0.25));
    auto quarter = partial.build();
    ASSERT_NE(quarter, nullptr);
    EXPECT_TRUE(quarter->isValid()) << quarter->validationReport();
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

// A path with a quarter arc in it: straight up 10, then bending over a quarter
// circle of radius 10.  The arc is entered from its end point, so its samples
// are walked backwards.
static std::shared_ptr<Sketch> makeBentPathSketch() {
    // Normal -Y with X across gives a local Y axis of +Z.
    auto path = std::make_shared<Sketch>(
        hz::draft::SketchPlane(Vec3(0, 0, 0), Vec3(0, -1, 0), Vec3(1, 0, 0)));
    path->addEntity(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(0, 10)));
    path->addEntity(std::make_shared<hz::draft::DraftArc>(Vec2(10, 10), 10.0, hz::math::kPi * 0.5,
                                                          hz::math::kPi));
    return path;
}

TEST(FeatureTreeTest, SweepFollowsAnArcInThePath) {
    auto profile = makeSquareSketchOnPlane(2.0, 0.0);
    SweepFeature feature(profile, makeBentPathSketch());
    ASSERT_TRUE(feature.parameters().count("segments"));
    EXPECT_EQ(feature.segments(), hz::model::Sweep::kDefaultArcSegments);

    // The arc used to be swept across its chord in one straight segment.  It
    // is now a chain of mitered chords at `segments` steps per turn, so the
    // volume is exactly area x (polyline length) and approaches area x (arc
    // length) from below.
    auto sweptVolume = [&](int segments) {
        EXPECT_TRUE(feature.setParameter("segments", static_cast<double>(segments)));
        auto solid = feature.execute(nullptr);
        EXPECT_NE(solid, nullptr);
        if (!solid) return 0.0;
        const int steps = static_cast<int>(std::ceil(segments / 4.0 - 1e-9));
        const double chainLength =
            10.0 + steps * 2.0 * 10.0 * std::sin(hz::math::kPi / 4.0 / steps);
        EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*solid).volume, 4.0 * chainLength,
                    1e-8);
        EXPECT_EQ(solid->faceCount(), static_cast<size_t>(4 * (1 + steps) + 2));
        return hz::model::MassPropertiesCalculator::compute(*solid).volume;
    };

    const double exact = 4.0 * (10.0 + hz::math::kPi * 5.0);
    const double coarse = sweptVolume(16);
    const double fine = sweptVolume(128);
    EXPECT_LT(coarse, fine);
    EXPECT_LT(fine, exact) << "a chord chain is shorter than its arc";
    EXPECT_LT((exact - fine) / exact, 1e-4);

    EXPECT_FALSE(feature.setParameter("segments", 2.0));
    EXPECT_EQ(feature.segments(), 128) << "a refused edit must not change the feature";
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
    // The unit square has its corner on the axis and spans 90 degrees, so its
    // copies 60 degrees apart overlap: they merge into one body (Phase 93)
    // instead of six interpenetrating shells that counted shared material
    // several times over.
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkManifold());
    const double volume = hz::model::MassPropertiesCalculator::compute(*solid).volume;
    EXPECT_GT(volume, 1.0);
    EXPECT_LT(volume, 6.0 - 1e-6) << "overlaps are counted once";
    EXPECT_EQ(tree.feature(1)->name(), "CircularPattern");
}

TEST(FeatureTreeTest, CircularPatternOfSeparateInstancesKeepsSeparateBodies) {
    FeatureTree tree;
    // x in [5, 10], y in [0, 5]: a quarter turn apart, the copies never meet.
    tree.addFeature(std::make_unique<ExtrudeFeature>(makeOffsetRectSketch(), Vec3(0, 0, 1), 1.0));
    tree.addFeature(PatternFeature::makeCircular(Vec3(0, 0, 0), Vec3(0, 0, 1), kTwoPi / 4.0, 4));
    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->shellCount(), 4u);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*solid).volume, 100.0, 1e-9);
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
    // Every primitive is a valid solid; the torus has one handle.
    struct Expect {
        int genus;
    };
    std::vector<Expect> expects;
    cases.push_back({PrimitiveFeature::makeCylinder(5.0, 10.0), "Cylinder"});
    expects.push_back({0});
    cases.push_back({PrimitiveFeature::makeSphere(4.0), "Sphere"});
    expects.push_back({0});
    cases.push_back({PrimitiveFeature::makeCone(4.0, 2.0, 6.0), "Cone"});
    expects.push_back({0});
    cases.push_back({PrimitiveFeature::makeTorus(8.0, 2.0), "Torus"});
    expects.push_back({1});

    for (size_t i = 0; i < cases.size(); ++i) {
        auto& c = cases[i];
        const std::string expected = c.name;
        FeatureTree tree;
        EXPECT_EQ(c.feat->name(), expected);
        tree.addFeature(std::move(c.feat));
        auto solid = tree.build();
        ASSERT_NE(solid, nullptr) << expected;
        EXPECT_TRUE(solid->checkManifold()) << expected;
        EXPECT_TRUE(solid->checkEulerFormula()) << expected;
        EXPECT_TRUE(solid->isValid()) << expected << solid->validationReport();
        EXPECT_EQ(solid->genus(), expects[i].genus) << expected;
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

// ---------------------------------------------------------------------------
// buildBodies() — multi-body trees: each create feature starts a new body,
// transforms modify the active (most-recently-created) body.
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, BuildBodiesEmptyTreeReturnsNoBodies) {
    FeatureTree tree;
    EXPECT_TRUE(tree.buildBodies().empty());
}

TEST(FeatureTreeTest, BuildBodiesSinglePrimitiveIsOneBody) {
    FeatureTree tree;
    tree.addFeature(PrimitiveFeature::makeBox(2.0, 3.0, 4.0));
    auto bodies = tree.buildBodies();
    ASSERT_EQ(bodies.size(), 1u);
    ASSERT_NE(bodies[0], nullptr);
    EXPECT_EQ(bodies[0]->faceCount(), 6u);
    EXPECT_TRUE(bodies[0]->checkEulerFormula());
}

TEST(FeatureTreeTest, BuildBodiesTwoPrimitivesAreTwoBodies) {
    FeatureTree tree;
    tree.addFeature(PrimitiveFeature::makeBox(2.0, 2.0, 2.0));
    tree.addFeature(PrimitiveFeature::makeSphere(3.0));
    auto bodies = tree.buildBodies();
    ASSERT_EQ(bodies.size(), 2u);
    ASSERT_NE(bodies[0], nullptr);
    ASSERT_NE(bodies[1], nullptr);
    EXPECT_EQ(bodies[0]->faceCount(), 6u);  // box preserved as its own body
    EXPECT_TRUE(bodies[0]->isValid());
    EXPECT_TRUE(bodies[1]->isValid());
}

TEST(FeatureTreeTest, BuildBodiesTransformStaysOnActiveBody) {
    // Box, then a second box, then a fillet: the fillet modifies the active
    // (second) body, leaving two bodies total.
    FeatureTree tree;
    tree.addFeature(PrimitiveFeature::makeBox(10.0, 10.0, 10.0));
    tree.addFeature(PrimitiveFeature::makeBox(6.0, 6.0, 6.0));

    // Snapshot the second box's first edge to fillet it.
    auto twoBoxes = tree.buildBodies();
    ASSERT_EQ(twoBoxes.size(), 2u);
    const size_t activeFaces = twoBoxes[1]->faceCount();
    ASSERT_FALSE(twoBoxes[1]->edges().empty());
    const auto edgeId = twoBoxes[1]->edges().front().topoId;

    tree.addFeature(
        std::make_unique<FilletFeature>(std::vector<hz::topo::TopologyID>{edgeId}, 1.0));
    auto bodies = tree.buildBodies();
    ASSERT_EQ(bodies.size(), 2u);
    ASSERT_NE(bodies[0], nullptr);
    ASSERT_NE(bodies[1], nullptr);
    EXPECT_EQ(bodies[0]->faceCount(), 6u);           // first box untouched
    EXPECT_GT(bodies[1]->faceCount(), activeFaces);  // fillet landed on active body
    EXPECT_TRUE(bodies[1]->isValid());
}

TEST(FeatureTreeTest, BuildBodiesSkipsLeadingConstructionFeature) {
    FeatureTree tree;
    tree.addFeature(DatumFeature::makePlane(
        hz::model::DatumPlane{Vec3(0, 0, 0), Vec3(0, 0, 1), Vec3(1, 0, 0)}));
    tree.addFeature(PrimitiveFeature::makeBox(2.0, 2.0, 2.0));
    auto bodies = tree.buildBodies();
    ASSERT_EQ(bodies.size(), 1u);  // datum contributes no body
    ASSERT_NE(bodies[0], nullptr);
    EXPECT_EQ(bodies[0]->faceCount(), 6u);
}

TEST(FeatureTreeTest, BuildBodiesPrimitiveThenFilletIsOneBody) {
    FeatureTree tree;
    tree.addFeature(PrimitiveFeature::makeBox(10.0, 10.0, 10.0));
    auto box = tree.buildBodies();
    ASSERT_EQ(box.size(), 1u);
    ASSERT_FALSE(box[0]->edges().empty());
    const auto edgeId = box[0]->edges().front().topoId;

    tree.addFeature(
        std::make_unique<FilletFeature>(std::vector<hz::topo::TopologyID>{edgeId}, 1.0));
    auto bodies = tree.buildBodies();
    ASSERT_EQ(bodies.size(), 1u);  // fillet transforms, does not add a body
    ASSERT_NE(bodies[0], nullptr);
    EXPECT_TRUE(bodies[0]->isValid());
}

// ---------------------------------------------------------------------------
// BooleanFeature — combines the multi-body list into one via a Boolean op.
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, BooleanFeatureNameAndParameter) {
    BooleanFeature f(hz::model::BooleanType::Subtract);
    EXPECT_EQ(f.name(), "Boolean Subtract");
    EXPECT_EQ(f.booleanType(), hz::model::BooleanType::Subtract);
    EXPECT_TRUE(f.consumesAllBodies());

    EXPECT_TRUE(f.setParameter("operation", 0));  // -> Union
    EXPECT_EQ(f.booleanType(), hz::model::BooleanType::Union);
    EXPECT_FALSE(f.setParameter("operation", 9));  // out of range
    EXPECT_FALSE(f.setParameter("radius", 1.0));   // not a Boolean parameter
    EXPECT_DOUBLE_EQ(f.parameters().at("operation"), 0.0);
}

TEST(FeatureTreeTest, BuildBodiesBooleanCombinesTwoBodiesIntoOne) {
    // Two overlapping primitives are two bodies until a Boolean folds them.
    for (auto op : {hz::model::BooleanType::Union, hz::model::BooleanType::Subtract,
                    hz::model::BooleanType::Intersect}) {
        FeatureTree tree;
        tree.addFeature(PrimitiveFeature::makeBox(10.0, 10.0, 10.0));
        tree.addFeature(PrimitiveFeature::makeBox(6.0, 6.0, 6.0));
        EXPECT_EQ(tree.buildBodies().size(), 2u);  // independent bodies

        tree.addFeature(std::make_unique<BooleanFeature>(op));
        auto bodies = tree.buildBodies();
        ASSERT_EQ(bodies.size(), 1u);  // the Boolean folded them into one body
        EXPECT_NE(bodies[0], nullptr);
    }
}

TEST(FeatureTreeTest, BuildBodiesBooleanWithSingleBodyIsNoOp) {
    FeatureTree tree;
    tree.addFeature(PrimitiveFeature::makeBox(4.0, 4.0, 4.0));
    tree.addFeature(std::make_unique<BooleanFeature>(hz::model::BooleanType::Subtract));
    auto bodies = tree.buildBodies();
    ASSERT_EQ(bodies.size(), 1u);  // fewer than two operands -> Boolean is a no-op
    ASSERT_NE(bodies[0], nullptr);
    EXPECT_EQ(bodies[0]->faceCount(), 6u);
}

TEST(FeatureTreeTest, BuildIgnoresBooleanFeature) {
    // The single-solid build() path has no second operand, so a Boolean feature
    // passes the running solid through unchanged.
    FeatureTree tree;
    tree.addFeature(PrimitiveFeature::makeBox(4.0, 4.0, 4.0));
    tree.addFeature(std::make_unique<BooleanFeature>(hz::model::BooleanType::Union));
    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 6u);  // Boolean is a no-op in single-solid build
}

// ---------------------------------------------------------------------------
// Faceting resolution as a feature parameter (Phase 88)
//
// Phases 84-87 made resolution the knob that decides how close a faceted
// solid's volume gets to the exact one. These tests pin that the knob is
// reachable through the generic parametric interface — which is what the UI
// editor and the save format both go through — and that turning it actually
// moves the geometry.
// ---------------------------------------------------------------------------

static double volumeOf(const hz::topo::Solid& solid) {
    return hz::model::MassPropertiesCalculator::compute(solid).volume;
}

TEST(FeatureTreeTest, CurvedPrimitivesReportSegmentsAndABoxDoesNot) {
    auto cylinder = PrimitiveFeature::makeCylinder(5.0, 10.0);
    const auto cylParams = cylinder->parameters();
    ASSERT_TRUE(cylParams.count("segments"));
    EXPECT_EQ(static_cast<int>(cylParams.at("segments")), cylinder->segments());

    // A box is exact, so it has no resolution to report and refuses one.
    auto box = PrimitiveFeature::makeBox(1.0, 2.0, 3.0);
    EXPECT_EQ(box->parameters().count("segments"), 0u);
    EXPECT_FALSE(box->setParameter("segments", 64.0));
}

TEST(FeatureTreeTest, RaisingPrimitiveSegmentsMovesVolumeTowardTheAnalyticValue) {
    const double exact = hz::math::kPi * 25.0 * 10.0;

    auto coarse = PrimitiveFeature::makeCylinder(5.0, 10.0);
    ASSERT_TRUE(coarse->setParameter("segments", 16.0));
    auto fine = PrimitiveFeature::makeCylinder(5.0, 10.0);
    ASSERT_TRUE(fine->setParameter("segments", 128.0));

    auto coarseSolid = coarse->execute(nullptr);
    auto fineSolid = fine->execute(nullptr);
    ASSERT_NE(coarseSolid, nullptr);
    ASSERT_NE(fineSolid, nullptr);

    const double coarseError = exact - volumeOf(*coarseSolid);
    const double fineError = exact - volumeOf(*fineSolid);
    EXPECT_GT(coarseError, 0.0) << "an inscribed prism can only undershoot";
    EXPECT_GT(fineError, 0.0);
    EXPECT_LT(fineError, coarseError * 0.05);
}

TEST(FeatureTreeTest, PrimitiveSegmentsBelowThreeAreRefused) {
    auto cylinder = PrimitiveFeature::makeCylinder(5.0, 10.0);
    const int before = cylinder->segments();
    EXPECT_FALSE(cylinder->setParameter("segments", 2.0));
    EXPECT_FALSE(cylinder->setParameter("segments", 0.0));
    EXPECT_EQ(cylinder->segments(), before) << "a refused edit must not change the feature";
}

TEST(FeatureTreeTest, RevolveSegmentsAreAParameterAndChangeTheSolid) {
    auto sketch = makeOffsetRectSketch();
    RevolveFeature feature(sketch, Vec3::Zero, Vec3::UnitY, kTwoPi * 0.5);

    const auto params = feature.parameters();
    ASSERT_TRUE(params.count("segments"));
    ASSERT_TRUE(params.count("angle"));

    auto coarse = feature.execute(nullptr);
    ASSERT_NE(coarse, nullptr);
    ASSERT_TRUE(feature.setParameter("segments", 128.0));
    auto fine = feature.execute(nullptr);
    ASSERT_NE(fine, nullptr);

    EXPECT_GT(fine->faceCount(), coarse->faceCount());
    // Pappus for the half turn of the 5x5 rectangle centred at radius 7.5.
    const double exact = kTwoPi * 0.5 * 7.5 * 25.0;
    EXPECT_LT(exact - volumeOf(*fine), exact - volumeOf(*coarse));

    EXPECT_FALSE(feature.setParameter("segments", 2.0));
}

TEST(FeatureTreeTest, FilletArcSegmentsAreAParameterAndChangeTheMaterialRemoved) {
    FeatureTree tree;
    tree.addFeature(PrimitiveFeature::makeBox(10.0, 10.0, 10.0));
    auto box = tree.build();
    ASSERT_NE(box, nullptr);
    const auto edgeId = box->edges().front().topoId;

    auto blendedVolume = [&](int arcSegments) {
        FilletFeature feature(std::vector<hz::topo::TopologyID>{edgeId}, 1.0);
        EXPECT_TRUE(feature.setParameter("arcSegments", static_cast<double>(arcSegments)));
        auto input = tree.build();
        auto out = feature.execute(std::move(input));
        return out ? volumeOf(*out) : 0.0;
    };

    // One chord is the degenerate blend: it removes a chamfer's worth of
    // material. More chords converge on the exact fillet from below.
    const double exact = 1000.0 - (1.0 - hz::math::kPi / 4.0) * 10.0;
    const double chord = blendedVolume(1);
    const double faceted = blendedVolume(16);
    ASSERT_GT(chord, 0.0);
    ASSERT_GT(faceted, 0.0);
    EXPECT_LT(chord, faceted) << "the chord blend cuts away more than the arc does";
    EXPECT_NEAR(faceted, exact, 0.05);

    FilletFeature feature(std::vector<hz::topo::TopologyID>{edgeId}, 1.0);
    ASSERT_TRUE(feature.parameters().count("arcSegments"));
    EXPECT_FALSE(feature.setParameter("arcSegments", 0.0));
}

// ---------------------------------------------------------------------------
// Chord tolerance (Phase 90): accuracy as a distance, not a count.  A fixed
// facet count sags r(1 - cos(pi/n)), so the same count is ten times less
// accurate on a cylinder ten times the size.
// ---------------------------------------------------------------------------

static double sagitta(double radius, int segmentsPerTurn) {
    return radius * (1.0 - std::cos(hz::math::kPi / segmentsPerTurn));
}

TEST(FeatureTreeTest, ChordToleranceHoldsAtEveryRadius) {
    const double tolerance = 0.01;
    int previous = 0;
    for (double radius : {1.0, 10.0, 100.0}) {
        auto cylinder = PrimitiveFeature::makeCylinder(radius, 5.0);
        // The fixed default count's error grows with the radius...
        const double fixedSag = sagitta(radius, cylinder->segments());
        ASSERT_TRUE(cylinder->setParameter("chordTolerance", tolerance));
        EXPECT_DOUBLE_EQ(cylinder->chordTolerance(), tolerance);
        // ...the tolerance's does not.
        const int n = cylinder->segments();
        EXPECT_LE(sagitta(radius, n), tolerance * 1.0000001) << "radius " << radius;
        EXPECT_GT(n, previous) << "a wider circle needs more facets for the same sag";
        previous = n;
        if (radius >= 10.0) {
            EXPECT_GT(fixedSag, tolerance);
        }

        auto solid = cylinder->execute(nullptr);
        ASSERT_NE(solid, nullptr);
        EXPECT_EQ(solid->faceCount(), static_cast<size_t>(n + 2));
    }
}

TEST(FeatureTreeTest, ChordToleranceFollowsARadiusEdit) {
    auto sphere = PrimitiveFeature::makeSphere(2.0);
    ASSERT_TRUE(sphere->setParameter("chordTolerance", 0.01));
    const int small = sphere->segments();
    ASSERT_TRUE(sphere->setParameter("radius", 20.0));
    EXPECT_GT(sphere->segments(), small) << "the count is re-derived on every rebuild";
    EXPECT_EQ(sphere->parameters().at("segments"), static_cast<double>(sphere->segments()));

    // The governing radius is the widest circle: the outer rim of a torus.
    auto torus = PrimitiveFeature::makeTorus(10.0, 3.0);
    ASSERT_TRUE(torus->setParameter("chordTolerance", 0.01));
    EXPECT_EQ(torus->segments(), hz::model::PrimitiveFactory::segmentsForTolerance(13.0, 0.01));
}

TEST(FeatureTreeTest, SettingTheCountLeavesToleranceMode) {
    auto cylinder = PrimitiveFeature::makeCylinder(5.0, 10.0);
    ASSERT_TRUE(cylinder->setParameter("chordTolerance", 0.001));
    ASSERT_TRUE(cylinder->setParameter("segments", 12.0));
    EXPECT_EQ(cylinder->chordTolerance(), 0.0);
    EXPECT_EQ(cylinder->segments(), 12);

    // Zero switches tolerance mode off explicitly; negative and NaN are refused.
    ASSERT_TRUE(cylinder->setParameter("chordTolerance", 0.001));
    ASSERT_TRUE(cylinder->setParameter("chordTolerance", 0.0));
    EXPECT_EQ(cylinder->segments(), 12);
    EXPECT_FALSE(cylinder->setParameter("chordTolerance", -1.0));
    EXPECT_FALSE(cylinder->setParameter("chordTolerance", std::nan("")));

    // A box is exact: there is nothing for a tolerance to govern.
    auto box = PrimitiveFeature::makeBox(1.0, 2.0, 3.0);
    EXPECT_EQ(box->parameters().count("chordTolerance"), 0u);
    EXPECT_FALSE(box->setParameter("chordTolerance", 0.01));
}

TEST(FeatureTreeTest, RevolveChordToleranceUsesTheWidestProfileRadius) {
    auto sketch = makeOffsetRectSketch();  // spans radius 5..10 about Y
    RevolveFeature feature(sketch, Vec3::Zero, Vec3::UnitY, hz::math::kTwoPi);
    ASSERT_TRUE(feature.setParameter("chordTolerance", 0.005));
    const int n = feature.segments();
    EXPECT_EQ(n, hz::model::Revolve::segmentsForTolerance(10.0, 0.005));
    EXPECT_LE(sagitta(10.0, n), 0.005 * 1.0000001);
    auto solid = feature.execute(nullptr);
    ASSERT_NE(solid, nullptr);

    // A tighter budget is a closer solid, from below.
    const double exact = hz::math::kTwoPi * 7.5 * 25.0;
    ASSERT_TRUE(feature.setParameter("chordTolerance", 0.0005));
    auto finer = feature.execute(nullptr);
    ASSERT_NE(finer, nullptr);
    const double coarseErr = exact - hz::model::MassPropertiesCalculator::compute(*solid).volume;
    const double fineErr = exact - hz::model::MassPropertiesCalculator::compute(*finer).volume;
    EXPECT_GT(fineErr, 0.0);
    EXPECT_LT(fineErr, coarseErr);
}

TEST(FeatureTreeTest, FilletChordToleranceDerivesTheArcSegments) {
    FilletFeature feature(std::vector<hz::topo::TopologyID>{}, 2.0);
    ASSERT_TRUE(feature.setParameter("chordTolerance", 0.001));
    EXPECT_EQ(feature.arcSegments(), hz::model::FilletOp::arcSegmentsForTolerance(2.0, 0.001));
    // Doubling the radius at the same budget needs more chords.
    const int before = feature.arcSegments();
    ASSERT_TRUE(feature.setParameter("radius", 8.0));
    EXPECT_GT(feature.arcSegments(), before);
    ASSERT_TRUE(feature.setParameter("arcSegments", 3.0));
    EXPECT_EQ(feature.chordTolerance(), 0.0);
    EXPECT_EQ(feature.arcSegments(), 3);
}

TEST(FeatureTreeTest, SweepChordToleranceSamplesEachArcAtItsOwnRadius) {
    auto profile = makeSquareSketchOnPlane(2.0, 0.0);
    SweepFeature feature(profile, makeBentPathSketch());  // one arc, radius 10
    ASSERT_TRUE(feature.setParameter("chordTolerance", 0.001));
    auto solid = feature.execute(nullptr);
    ASSERT_NE(solid, nullptr);
    const int perTurn = hz::model::PrimitiveFactory::segmentsForTolerance(10.0, 0.001);
    const int steps = static_cast<int>(std::ceil(perTurn / 4.0 - 1e-9));
    EXPECT_EQ(solid->faceCount(), static_cast<size_t>(4 * (1 + steps) + 2));
}

// ---------------------------------------------------------------------------
// Extrude resolution (Phase 92): reported only when there is an arc to facet.
// ---------------------------------------------------------------------------

TEST(FeatureTreeTest, ExtrudeResolutionIsAParameterOfCurvedProfilesOnly) {
    auto rect = makeOffsetRectSketch();
    ExtrudeFeature square(rect, Vec3::UnitZ, 5.0);
    EXPECT_EQ(square.parameters().count("segments"), 0u);
    EXPECT_FALSE(square.setParameter("segments", 64.0)) << "a polygon extrudes exactly";
    EXPECT_FALSE(square.setParameter("chordTolerance", 0.01));

    auto disc = std::make_shared<Sketch>();
    disc->addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(0, 0), 5.0));
    ExtrudeFeature cylinder(disc, Vec3::UnitZ, 10.0);
    ASSERT_TRUE(cylinder.parameters().count("segments"));
    ASSERT_TRUE(cylinder.parameters().count("chordTolerance"));

    const double exact = hz::math::kPi * 25.0 * 10.0;
    auto coarse = cylinder.execute(nullptr);
    ASSERT_TRUE(cylinder.setParameter("segments", 128.0));
    auto fine = cylinder.execute(nullptr);
    ASSERT_NE(coarse, nullptr);
    ASSERT_NE(fine, nullptr);
    const double coarseErr = exact - hz::model::MassPropertiesCalculator::compute(*coarse).volume;
    const double fineErr = exact - hz::model::MassPropertiesCalculator::compute(*fine).volume;
    EXPECT_GT(fineErr, 0.0);
    EXPECT_LT(fineErr, coarseErr * 0.1);
}

// ---------------------------------------------------------------------------
// Exceptions from the kernel become feature failures
// ---------------------------------------------------------------------------

namespace {

/// A feature whose kernel call throws, as the NURBS constructors do on
/// invalid input.
class ThrowingFeature : public Feature {
public:
    std::string name() const override { return "Broken"; }
    std::string featureID() const override { return "broken_1"; }
    std::unique_ptr<hz::topo::Solid> execute(std::unique_ptr<hz::topo::Solid>,
                                             std::string* /*reason*/) const override {
        throw std::invalid_argument("degree must be at least 1");
    }
};

}  // namespace

TEST(FeatureTreeTest, ThrowingFeatureIsAFailureWithItsReason) {
    FeatureTree tree;
    tree.addFeature(std::make_unique<ExtrudeFeature>(makeOffsetRectSketch(), Vec3(0, 0, 1), 1.0));
    tree.addFeature(std::make_unique<ThrowingFeature>());

    BuildResult result;
    ASSERT_NO_THROW(result = tree.buildWithDiagnostics());
    EXPECT_EQ(result.failedFeatureIndex, 1);
    EXPECT_EQ(result.lastSuccessfulFeature, 0);
    EXPECT_NE(result.failureMessage.find("Broken"), std::string::npos) << result.failureMessage;
    EXPECT_NE(result.failureMessage.find("degree must be at least 1"), std::string::npos)
        << result.failureMessage;
}

TEST(FeatureTreeTest, ThrowingFeatureFailsEveryBuildPathWithoutEscaping) {
    FeatureTree tree;
    tree.addFeature(std::make_unique<ThrowingFeature>());
    std::unique_ptr<hz::topo::Solid> solid;
    ASSERT_NO_THROW(solid = tree.build());
    EXPECT_EQ(solid, nullptr);
    std::vector<std::unique_ptr<hz::topo::Solid>> bodies;
    ASSERT_NO_THROW(bodies = tree.buildBodies());
    EXPECT_TRUE(bodies.empty());
}
