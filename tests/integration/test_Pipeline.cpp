// Phase 40: Integration test suite — 55 tests covering full pipeline.
//
// Groups: Extrude (10), Revolve (5), Boolean (10), Fillet/Chamfer (8),
//         FeatureTree (7), Primitives (5), Fuzz (5), TNP Regression (3),
//         Performance (2).

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/ChamferOp.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz::model;
using namespace hz::topo;
using namespace hz::draft;
using namespace hz::doc;
using hz::math::Vec2;
using hz::math::Vec3;

static constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

// ===========================================================================
// Helpers
// ===========================================================================

static std::vector<std::shared_ptr<DraftEntity>> makeRectProfile(double w, double h) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(w, 0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(w, 0), Vec2(w, h)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(w, h), Vec2(0, h)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, h), Vec2(0, 0)));
    return profile;
}

static std::vector<std::shared_ptr<DraftEntity>> makeTriangleProfile() {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(3, 0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(3, 0), Vec2(1.5, 2.6)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(1.5, 2.6), Vec2(0, 0)));
    return profile;
}

static std::vector<std::shared_ptr<DraftEntity>> makeCircleProfile(double r) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftCircle>(Vec2(0, 0), r));
    return profile;
}

static std::vector<std::shared_ptr<DraftEntity>> makeOffsetRectProfile(double xMin,
                                                                        double xMax,
                                                                        double yMin,
                                                                        double yMax) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(xMin, yMin), Vec2(xMax, yMin)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(xMax, yMin), Vec2(xMax, yMax)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(xMax, yMax), Vec2(xMin, yMax)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(xMin, yMax), Vec2(xMin, yMin)));
    return profile;
}

static std::shared_ptr<Sketch> makeRectSketch(double w, double h) {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(w, 0)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(w, 0), Vec2(w, h)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(w, h), Vec2(0, h)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(0, h), Vec2(0, 0)));
    return sketch;
}

static std::shared_ptr<Sketch> makeOffsetRectSketch() {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(5, 0), Vec2(10, 0)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(10, 0), Vec2(10, 5)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(10, 5), Vec2(5, 5)));
    sketch->addEntity(std::make_shared<DraftLine>(Vec2(5, 5), Vec2(5, 0)));
    return sketch;
}

/// Offset all vertices AND surface control points of a solid by a translation.
static void offsetSolid(Solid& solid, const Vec3& offset) {
    for (auto& v : const_cast<std::deque<Vertex>&>(solid.vertices())) {
        v.point = v.point + offset;
    }
    std::set<hz::geo::NurbsSurface*> translated;
    for (auto& face : const_cast<std::deque<Face>&>(solid.faces())) {
        if (!face.surface) continue;
        auto* ptr = face.surface.get();
        if (translated.count(ptr)) continue;
        translated.insert(ptr);
        auto oldCPs = face.surface->controlPoints();
        for (auto& row : oldCPs) {
            for (auto& cp : row) {
                cp = cp + offset;
            }
        }
        face.surface = std::make_shared<hz::geo::NurbsSurface>(
            std::move(oldCPs), face.surface->weights(), face.surface->knotsU(),
            face.surface->knotsV(), face.surface->degreeU(), face.surface->degreeV());
    }
}

// ===========================================================================
// Extrude variations (10 tests)
// ===========================================================================

TEST(PipelineTest, ExtrudeRectangle) {
    auto profile = makeRectProfile(5.0, 3.0);
    SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3::UnitZ, 4.0, "ext_rect");
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

TEST(PipelineTest, ExtrudeTriangle) {
    auto profile = makeTriangleProfile();
    SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3::UnitZ, 5.0, "ext_tri");
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->vertexCount(), 6u);
    EXPECT_EQ(solid->edgeCount(), 9u);
    EXPECT_EQ(solid->faceCount(), 5u);
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

TEST(PipelineTest, ExtrudeCircle) {
    auto profile = makeCircleProfile(5.0);
    SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3::UnitZ, 10.0, "ext_cir");
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

TEST(PipelineTest, ExtrudeSmallProfile) {
    auto profile = makeRectProfile(0.001, 0.001);
    SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3::UnitZ, 0.001, "ext_small");
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

TEST(PipelineTest, ExtrudeLargeProfile) {
    auto profile = makeRectProfile(1000.0, 1000.0);
    SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3::UnitZ, 1000.0, "ext_large");
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

TEST(PipelineTest, ExtrudeZeroDistanceProducesDegenerate) {
    auto profile = makeRectProfile(5.0, 3.0);
    SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3::UnitZ, 0.0, "ext_zero");
    // Zero distance may produce a degenerate solid (all vertices coplanar)
    // but the topology may still be structurally valid.
    if (solid) {
        EXPECT_EQ(solid->faceCount(), 6u);
    }
}

TEST(PipelineTest, ExtrudeOpenProfileFails) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(1, 0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(1, 0), Vec2(1, 1)));
    SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3::UnitZ, 1.0, "ext_open");
    EXPECT_EQ(solid, nullptr);
}

TEST(PipelineTest, ExtrudeEmptyProfileFails) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3::UnitZ, 1.0, "ext_empty");
    EXPECT_EQ(solid, nullptr);
}

TEST(PipelineTest, ExtrudeHasValidEuler) {
    auto profile = makeRectProfile(7.0, 4.0);
    SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3::UnitZ, 3.0, "ext_euler");
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
}

TEST(PipelineTest, ExtrudeHasNURBSSurfaces) {
    auto profile = makeRectProfile(4.0, 4.0);
    SketchPlane plane;
    auto solid = Extrude::execute(profile, plane, Vec3::UnitZ, 2.0, "ext_nurbs");
    ASSERT_NE(solid, nullptr);
    for (const auto& f : solid->faces()) {
        EXPECT_NE(f.surface, nullptr) << "Face " << f.id << " has no NURBS surface";
    }
}

// ===========================================================================
// Revolve variations (5 tests)
// ===========================================================================

TEST(PipelineTest, RevolveRectangle) {
    auto profile = makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0);
    SketchPlane plane;
    auto solid = Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY,
                                   kTwoPi, "rev_rect");
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

TEST(PipelineTest, RevolveHasValidEuler) {
    auto profile = makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0);
    SketchPlane plane;
    auto solid = Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY,
                                   kTwoPi, "rev_euler");
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
}

TEST(PipelineTest, RevolveInvalidProfileFails) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 0)));
    SketchPlane plane;
    auto solid = Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY,
                                   kTwoPi, "rev_bad");
    EXPECT_EQ(solid, nullptr);
}

TEST(PipelineTest, RevolveHasTopologyIDs) {
    auto profile = makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0);
    SketchPlane plane;
    auto solid = Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY,
                                   kTwoPi, "rev_id");
    ASSERT_NE(solid, nullptr);
    for (const auto& f : solid->faces()) {
        EXPECT_TRUE(f.topoId.isValid());
    }
    for (const auto& e : solid->edges()) {
        EXPECT_TRUE(e.topoId.isValid());
    }
}

TEST(PipelineTest, RevolveHasNURBSSurfaces) {
    auto profile = makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0);
    SketchPlane plane;
    auto solid = Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY,
                                   kTwoPi, "rev_nurbs");
    ASSERT_NE(solid, nullptr);
    for (const auto& f : solid->faces()) {
        EXPECT_NE(f.surface, nullptr);
    }
}

// ===========================================================================
// Boolean operations (10 tests)
// ===========================================================================

TEST(PipelineTest, BoolUnionTwoBoxes) {
    auto a = PrimitiveFactory::makeBox(10, 10, 10);
    auto b = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*b, Vec3(5, 0, 0));
    auto r = BooleanOp::execute(*a, *b, BooleanType::Union);
    ASSERT_NE(r, nullptr);
    EXPECT_GT(r->faceCount(), 0u);
}

TEST(PipelineTest, BoolSubtractBoxFromBox) {
    auto a = PrimitiveFactory::makeBox(10, 10, 10);
    auto b = PrimitiveFactory::makeBox(4, 4, 20);
    offsetSolid(*b, Vec3(3, 3, -5));
    auto r = BooleanOp::execute(*a, *b, BooleanType::Subtract);
    ASSERT_NE(r, nullptr);
    EXPECT_GT(r->faceCount(), 0u);
}

TEST(PipelineTest, BoolIntersectTwoBoxes) {
    auto a = PrimitiveFactory::makeBox(10, 10, 10);
    auto b = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*b, Vec3(5, 5, 5));
    auto r = BooleanOp::execute(*a, *b, BooleanType::Intersect);
    ASSERT_NE(r, nullptr);
    EXPECT_GT(r->faceCount(), 0u);
}

TEST(PipelineTest, BoolNonOverlappingUnion) {
    auto a = PrimitiveFactory::makeBox(10, 10, 10);
    auto b = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*b, Vec3(100, 0, 0));
    auto r = BooleanOp::execute(*a, *b, BooleanType::Union);
    ASSERT_NE(r, nullptr);
    EXPECT_GE(r->faceCount(), 10u);
}

TEST(PipelineTest, BoolNonOverlappingSubtract) {
    auto a = PrimitiveFactory::makeBox(10, 10, 10);
    auto b = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*b, Vec3(100, 0, 0));
    auto r = BooleanOp::execute(*a, *b, BooleanType::Subtract);
    ASSERT_NE(r, nullptr);
    EXPECT_GE(r->faceCount(), 4u);
}

TEST(PipelineTest, BoolResultHasTopologyIDs) {
    auto a = PrimitiveFactory::makeBox(10, 10, 10);
    auto b = PrimitiveFactory::makeBox(4, 4, 20);
    offsetSolid(*b, Vec3(3, 3, -5));
    auto r = BooleanOp::execute(*a, *b, BooleanType::Subtract);
    ASSERT_NE(r, nullptr);
    int validIds = 0;
    for (const auto& f : r->faces()) {
        if (f.topoId.isValid()) ++validIds;
    }
    EXPECT_GT(validIds, 0);
}

TEST(PipelineTest, BoolSubtractPreservesAFaces) {
    auto a = PrimitiveFactory::makeBox(10, 10, 10);
    auto b = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*b, Vec3(100, 0, 0));
    // Non-overlapping subtract should preserve A's faces
    auto r = BooleanOp::execute(*a, *b, BooleanType::Subtract);
    ASSERT_NE(r, nullptr);
    EXPECT_GE(r->faceCount(), 4u);
    EXPECT_LE(r->faceCount(), 8u);
}

TEST(PipelineTest, BoolUnionHasFaces) {
    auto a = PrimitiveFactory::makeBox(10, 10, 10);
    auto b = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*b, Vec3(5, 5, 5));
    auto r = BooleanOp::execute(*a, *b, BooleanType::Union);
    ASSERT_NE(r, nullptr);
    EXPECT_GE(r->faceCount(), 6u);
}

TEST(PipelineTest, BoolIntersectHasFaces) {
    auto a = PrimitiveFactory::makeBox(10, 10, 10);
    auto b = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*b, Vec3(5, 5, 5));
    auto r = BooleanOp::execute(*a, *b, BooleanType::Intersect);
    ASSERT_NE(r, nullptr);
    EXPECT_GT(r->faceCount(), 0u);
}

TEST(PipelineTest, BoolResultTessellates) {
    auto a = PrimitiveFactory::makeBox(10, 10, 10);
    auto b = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*b, Vec3(5, 0, 0));
    auto r = BooleanOp::execute(*a, *b, BooleanType::Union);
    ASSERT_NE(r, nullptr);
    auto mesh = SolidTessellator::tessellate(*r, 0.5);
    // The result should produce at least some triangles
    EXPECT_GT(mesh.positions.size(), 0u);
}

// ===========================================================================
// Fillet/Chamfer (8 tests)
// ===========================================================================

TEST(PipelineTest, FilletOneEdge) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};
    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_pipe");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_GT(result.solid->faceCount(), box->faceCount());
}

TEST(PipelineTest, FilletHasEuler) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    std::vector<TopologyID> edgeIds = {edges.front().topoId};
    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_euler");
    ASSERT_NE(result.solid, nullptr);
    EXPECT_TRUE(result.solid->checkEulerFormula());
}

TEST(PipelineTest, FilletHasTopologyIDs) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    std::vector<TopologyID> edgeIds = {edges.front().topoId};
    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_id");
    ASSERT_NE(result.solid, nullptr);
    for (const auto& f : result.solid->faces()) {
        EXPECT_TRUE(f.topoId.isValid());
    }
}

TEST(PipelineTest, FilletTooLargeFails) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    std::vector<TopologyID> edgeIds = {edges.front().topoId};
    auto result = FilletOp::execute(*box, edgeIds, 100.0, "fillet_big");
    EXPECT_FALSE(result.errorMessage.empty());
}

TEST(PipelineTest, FilletInvalidEdgeFails) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    std::vector<TopologyID> edgeIds = {TopologyID::make("nonexistent", "edge")};
    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_bad");
    EXPECT_FALSE(result.errorMessage.empty());
}

TEST(PipelineTest, ChamferOneEdge) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};
    auto result = ChamferOp::executeEqual(*box, edgeIds, 1.0, "cham_pipe");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_GT(result.solid->faceCount(), box->faceCount());
}

TEST(PipelineTest, ChamferTwoDistance) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};
    auto result = ChamferOp::executeTwoDistance(*box, edgeIds, 1.0, 2.0, "cham_2d");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_TRUE(result.solid->checkEulerFormula());
}

TEST(PipelineTest, ChamferHasEuler) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    std::vector<TopologyID> edgeIds = {edges.front().topoId};
    auto result = ChamferOp::executeEqual(*box, edgeIds, 1.0, "cham_euler");
    ASSERT_NE(result.solid, nullptr);
    EXPECT_TRUE(result.solid->checkEulerFormula());
}

// ===========================================================================
// Feature tree (7 tests)
// ===========================================================================

TEST(PipelineTest, FeatureTreeSingleExtrude) {
    FeatureTree tree;
    auto sketch = makeRectSketch(10.0, 5.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3::UnitZ, 3.0));
    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

TEST(PipelineTest, FeatureTreeReplayConsistent) {
    FeatureTree tree;
    auto sketch = makeRectSketch(4.0, 3.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3::UnitZ, 2.0));
    auto s1 = tree.build();
    auto s2 = tree.build();
    ASSERT_NE(s1, nullptr);
    ASSERT_NE(s2, nullptr);
    EXPECT_EQ(s1->vertexCount(), s2->vertexCount());
    EXPECT_EQ(s1->edgeCount(), s2->edgeCount());
    EXPECT_EQ(s1->faceCount(), s2->faceCount());
}

TEST(PipelineTest, FeatureTreeEditParameter) {
    FeatureTree tree;
    auto sketch = makeRectSketch(5.0, 5.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3::UnitZ, 5.0));

    auto solid1 = tree.build();
    ASSERT_NE(solid1, nullptr);

    // Change the extrusion distance
    Feature* feat = tree.feature(0);
    ASSERT_NE(feat, nullptr);
    EXPECT_TRUE(feat->setParameter("distance", 10.0));

    auto solid2 = tree.build();
    ASSERT_NE(solid2, nullptr);
    // Same topology, just different vertex positions
    EXPECT_EQ(solid1->faceCount(), solid2->faceCount());
}

TEST(PipelineTest, FeatureTreeRollback) {
    FeatureTree tree;
    auto sketch1 = makeRectSketch(10.0, 5.0);
    auto sketch2 = makeRectSketch(3.0, 3.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch1, Vec3::UnitZ, 3.0));
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch2, Vec3::UnitZ, 2.0));

    // Rollback to feature 0 (suppress feature 1)
    tree.setRollbackIndex(0);
    auto result = tree.buildWithDiagnostics();
    ASSERT_NE(result.solid, nullptr);
    EXPECT_EQ(result.lastSuccessfulFeature, 0);
}

TEST(PipelineTest, FeatureTreeRemoveFeature) {
    FeatureTree tree;
    auto sketch = makeRectSketch(5.0, 5.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3::UnitZ, 1.0));
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3::UnitZ, 2.0));
    EXPECT_EQ(tree.featureCount(), 2u);

    tree.removeFeature(0);
    EXPECT_EQ(tree.featureCount(), 1u);

    auto solid = tree.build();
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid());
}

TEST(PipelineTest, FeatureTreeEmpty) {
    FeatureTree tree;
    EXPECT_EQ(tree.build(), nullptr);
    auto result = tree.buildWithDiagnostics();
    EXPECT_EQ(result.solid, nullptr);
}

TEST(PipelineTest, FeatureTreeBuildWithDiagnostics) {
    FeatureTree tree;
    auto sketch = makeRectSketch(5.0, 5.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3::UnitZ, 3.0));

    auto result = tree.buildWithDiagnostics();
    ASSERT_NE(result.solid, nullptr);
    EXPECT_EQ(result.lastSuccessfulFeature, 0);
    EXPECT_EQ(result.failedFeatureIndex, -1);
    EXPECT_TRUE(result.failureMessage.empty());
}

// ===========================================================================
// Primitives (5 tests)
// ===========================================================================

TEST(PipelineTest, PrimitiveBoxValid) {
    auto box = PrimitiveFactory::makeBox(10, 5, 3);
    ASSERT_NE(box, nullptr);
    EXPECT_EQ(box->vertexCount(), 8u);
    EXPECT_EQ(box->edgeCount(), 12u);
    EXPECT_EQ(box->faceCount(), 6u);
    EXPECT_TRUE(box->checkEulerFormula());
    EXPECT_TRUE(box->checkManifold());
    EXPECT_TRUE(box->isValid()) << box->validationReport();
}

TEST(PipelineTest, PrimitiveCylinderValid) {
    auto cyl = PrimitiveFactory::makeCylinder(5.0, 10.0);
    ASSERT_NE(cyl, nullptr);
    EXPECT_TRUE(cyl->checkEulerFormula());
    EXPECT_TRUE(cyl->checkManifold());
    EXPECT_TRUE(cyl->isValid()) << cyl->validationReport();
}

TEST(PipelineTest, PrimitiveSphereValid) {
    auto sph = PrimitiveFactory::makeSphere(5.0);
    ASSERT_NE(sph, nullptr);
    EXPECT_TRUE(sph->checkEulerFormula());
    EXPECT_TRUE(sph->checkManifold());
    EXPECT_TRUE(sph->isValid()) << sph->validationReport();
}

TEST(PipelineTest, PrimitiveConeValid) {
    auto cone = PrimitiveFactory::makeCone(5.0, 2.0, 10.0);
    ASSERT_NE(cone, nullptr);
    EXPECT_TRUE(cone->checkEulerFormula());
    EXPECT_TRUE(cone->checkManifold());
    EXPECT_TRUE(cone->isValid()) << cone->validationReport();
}

TEST(PipelineTest, PrimitiveTorusValid) {
    auto torus = PrimitiveFactory::makeTorus(5.0, 1.5);
    ASSERT_NE(torus, nullptr);
    EXPECT_TRUE(torus->checkEulerFormula());
    EXPECT_TRUE(torus->checkManifold());
    EXPECT_TRUE(torus->isValid()) << torus->validationReport();
}

// ===========================================================================
// Fuzz tests (5 tests)
// ===========================================================================

TEST(PipelineTest, FuzzRandomExtrudes) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> sizeDist(1.0, 20.0);
    for (int i = 0; i < 10; ++i) {
        double w = sizeDist(rng), h = sizeDist(rng), d = sizeDist(rng);
        auto box = PrimitiveFactory::makeBox(w, h, d);
        EXPECT_TRUE(box->checkEulerFormula()) << "Failed at iteration " << i;
        EXPECT_TRUE(box->isValid()) << "Invalid at iteration " << i;
    }
}

TEST(PipelineTest, FuzzRandomBooleans) {
    std::mt19937 rng(123);
    std::uniform_real_distribution<double> offsetDist(-5.0, 15.0);
    int success = 0;
    for (int i = 0; i < 10; ++i) {
        auto a = PrimitiveFactory::makeBox(10, 10, 10);
        auto b = PrimitiveFactory::makeBox(10, 10, 10);
        offsetSolid(*b, Vec3(offsetDist(rng), offsetDist(rng), offsetDist(rng)));
        auto r = BooleanOp::execute(*a, *b, BooleanType::Subtract);
        if (r && r->faceCount() > 0) ++success;
    }
    EXPECT_GE(success, 3) << "Too many Boolean failures";
}

TEST(PipelineTest, FuzzRandomFillets) {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> sizeDist(5.0, 20.0);
    std::uniform_real_distribution<double> radiusDist(0.1, 0.5);
    int success = 0;
    for (int i = 0; i < 5; ++i) {
        double s = sizeDist(rng);
        auto box = PrimitiveFactory::makeBox(s, s, s);
        auto& edges = box->edges();
        if (edges.empty()) continue;
        std::vector<TopologyID> edgeIds = {edges.front().topoId};
        auto result = FilletOp::execute(*box, edgeIds, radiusDist(rng), "fuzz_fillet");
        if (result.solid && result.solid->checkEulerFormula()) ++success;
    }
    EXPECT_GE(success, 3) << "Too many fillet failures";
}

TEST(PipelineTest, FuzzManifoldInvariant) {
    // Every primitive should be manifold
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    EXPECT_TRUE(box->checkManifold());

    auto cyl = PrimitiveFactory::makeCylinder(5, 10);
    EXPECT_TRUE(cyl->checkManifold());

    auto sph = PrimitiveFactory::makeSphere(5);
    EXPECT_TRUE(sph->checkManifold());

    auto cone = PrimitiveFactory::makeCone(5, 2, 10);
    EXPECT_TRUE(cone->checkManifold());

    auto torus = PrimitiveFactory::makeTorus(5, 1.5);
    EXPECT_TRUE(torus->checkManifold());
}

TEST(PipelineTest, FuzzEulerInvariant) {
    // Every primitive and extrude should satisfy Euler
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    EXPECT_TRUE(box->checkEulerFormula());

    auto cyl = PrimitiveFactory::makeCylinder(5, 10);
    EXPECT_TRUE(cyl->checkEulerFormula());

    auto sph = PrimitiveFactory::makeSphere(5);
    EXPECT_TRUE(sph->checkEulerFormula());

    auto profile = makeRectProfile(5.0, 3.0);
    SketchPlane plane;
    auto extruded = Extrude::execute(profile, plane, Vec3::UnitZ, 4.0, "fuzz_ext");
    ASSERT_NE(extruded, nullptr);
    EXPECT_TRUE(extruded->checkEulerFormula());

    auto triProf = makeTriangleProfile();
    auto triSolid = Extrude::execute(triProf, plane, Vec3::UnitZ, 3.0, "fuzz_tri");
    ASSERT_NE(triSolid, nullptr);
    EXPECT_TRUE(triSolid->checkEulerFormula());
}

// ===========================================================================
// TNP Regression (3 tests)
// ===========================================================================

TEST(TNPRegressionTest, ExtrudeTopologyIDsStable) {
    auto profile = makeRectProfile(5.0, 5.0);
    SketchPlane plane;

    // Build once, record face TopologyID tags
    auto solid1 = Extrude::execute(profile, plane, Vec3::UnitZ, 3.0, "tnp_ext");
    ASSERT_NE(solid1, nullptr);
    std::set<std::string> tags1;
    for (const auto& f : solid1->faces()) {
        tags1.insert(f.topoId.tag());
    }

    // Build again with same featureID
    auto solid2 = Extrude::execute(profile, plane, Vec3::UnitZ, 3.0, "tnp_ext");
    ASSERT_NE(solid2, nullptr);
    std::set<std::string> tags2;
    for (const auto& f : solid2->faces()) {
        tags2.insert(f.topoId.tag());
    }

    EXPECT_EQ(tags1, tags2) << "TopologyIDs changed between rebuilds";
}

TEST(TNPRegressionTest, FilletResolvesEdgeAfterRebuild) {
    // Build a box, record an edge TopologyID, rebuild, fillet using the recorded ID
    auto box1 = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_FALSE(box1->edges().empty());
    TopologyID targetEdge = box1->edges().front().topoId;

    // Rebuild box (simulates model rebuild)
    auto box2 = PrimitiveFactory::makeBox(10, 10, 10);

    // Collect all edge IDs from box2
    std::vector<TopologyID> candidates;
    for (const auto& e : box2->edges()) {
        candidates.push_back(e.topoId);
    }

    // Resolve the original edge ID against the rebuilt solid
    auto resolved = TopologyID::resolve(targetEdge, candidates);
    ASSERT_TRUE(resolved.has_value()) << "Failed to resolve edge " << targetEdge.tag();

    // Fillet using the resolved ID
    std::vector<TopologyID> edgeIds = {resolved.value()};
    auto result = FilletOp::execute(*box2, edgeIds, 1.0, "tnp_fillet");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_TRUE(result.solid->checkEulerFormula());
}

TEST(TNPRegressionTest, FeatureTreePreservesIDs) {
    FeatureTree tree;
    auto sketch = makeRectSketch(5.0, 5.0);
    tree.addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3::UnitZ, 3.0));

    auto solid1 = tree.build();
    ASSERT_NE(solid1, nullptr);
    std::set<std::string> faceTags1;
    for (const auto& f : solid1->faces()) {
        faceTags1.insert(f.topoId.tag());
    }

    // Rebuild via feature tree
    auto solid2 = tree.build();
    ASSERT_NE(solid2, nullptr);
    std::set<std::string> faceTags2;
    for (const auto& f : solid2->faces()) {
        faceTags2.insert(f.topoId.tag());
    }

    EXPECT_EQ(faceTags1, faceTags2)
        << "Feature tree rebuild changed TopologyIDs";
}

// ===========================================================================
// Performance benchmarks (2 tests)
// ===========================================================================

TEST(PerfTest, PrimitiveTessellation) {
    using Clock = std::chrono::high_resolution_clock;

    // Box tessellation at various tolerances
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    {
        auto start = Clock::now();
        auto mesh = SolidTessellator::tessellate(*box, 1.0);
        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start);
        EXPECT_LT(elapsed.count(), 1000) << "Box tessellation (tol=1.0) too slow";
        EXPECT_FALSE(mesh.positions.empty());
    }
    {
        auto start = Clock::now();
        auto mesh = SolidTessellator::tessellate(*box, 0.01);
        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start);
        // Generous thresholds for Debug builds (unoptimized code)
        EXPECT_LT(elapsed.count(), 60000) << "Box tessellation (tol=0.01) too slow";
        EXPECT_FALSE(mesh.positions.empty());
    }

    // Cylinder tessellation
    auto cyl = PrimitiveFactory::makeCylinder(5, 10);
    {
        auto start = Clock::now();
        auto mesh = SolidTessellator::tessellate(*cyl, 0.1);
        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start);
        EXPECT_LT(elapsed.count(), 30000) << "Cylinder tessellation too slow";
        EXPECT_FALSE(mesh.positions.empty());
    }

    // Sphere tessellation
    auto sph = PrimitiveFactory::makeSphere(5);
    {
        auto start = Clock::now();
        auto mesh = SolidTessellator::tessellate(*sph, 0.1);
        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start);
        EXPECT_LT(elapsed.count(), 30000) << "Sphere tessellation too slow";
        EXPECT_FALSE(mesh.positions.empty());
    }
}

TEST(PerfTest, FeatureTreeRebuild) {
    using Clock = std::chrono::high_resolution_clock;

    FeatureTree tree;
    // Build a 5-feature tree (each feature is an independent extrude for now)
    for (int i = 0; i < 5; ++i) {
        auto sketch = makeRectSketch(5.0 + i, 3.0 + i);
        tree.addFeature(std::make_unique<ExtrudeFeature>(
            sketch, Vec3::UnitZ, 2.0 + i));
    }

    auto start = Clock::now();
    for (int iter = 0; iter < 10; ++iter) {
        auto solid = tree.build();
        EXPECT_NE(solid, nullptr);
    }
    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start);
    // 10 rebuilds of 5-feature tree should complete in under 10 seconds
    EXPECT_LT(elapsed.count(), 10000)
        << "Feature tree rebuild too slow: " << elapsed.count() << "ms for 10 rebuilds";
}
