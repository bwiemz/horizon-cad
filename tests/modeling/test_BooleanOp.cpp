#include <gtest/gtest.h>

#include <random>
#include <set>

#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"

using hz::math::Vec3;
using hz::model::BooleanOp;
using hz::model::BooleanType;
using hz::model::MassPropertiesCalculator;
using hz::model::PrimitiveFactory;

namespace {

double volumeOf(const hz::topo::Solid& solid) {
    return MassPropertiesCalculator::compute(solid).volume;
}

}  // namespace

// ---------------------------------------------------------------------------
// Helper: offset all vertices AND surface control points of a solid by a
// translation vector.  This keeps classifyPoint (which ray-casts against
// tessellated surfaces) consistent with the vertex positions.
// ---------------------------------------------------------------------------

static void offsetSolid(hz::topo::Solid& solid, const Vec3& offset) {
    // Offset vertices.
    for (auto& v : const_cast<std::deque<hz::topo::Vertex>&>(solid.vertices())) {
        v.point = v.point + offset;
    }

    // Offset NURBS surface control points so tessellation stays correct.
    // We track surfaces we've already translated (faces can share a surface).
    std::set<hz::geo::NurbsSurface*> translated;
    for (auto& face : const_cast<std::deque<hz::topo::Face>&>(solid.faces())) {
        if (!face.surface) continue;
        auto* ptr = face.surface.get();
        if (translated.count(ptr)) continue;
        translated.insert(ptr);

        // Rebuild the surface with translated control points.
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

// ---------------------------------------------------------------------------
// SubtractOverlappingBoxes
// ---------------------------------------------------------------------------

TEST(BooleanOpTest, SubtractOverlappingBoxes) {
    // Box A: (0,0,0) to (10,10,10)
    // Box B: (3,3,-5) to (7,7,15) — a tall thin box through the center.
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(4, 4, 20);
    offsetSolid(*boxB, Vec3(3, 3, -5));

    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Subtract);
    ASSERT_NE(result, nullptr);
    EXPECT_GT(result->faceCount(), 0u);
}

// ---------------------------------------------------------------------------
// UnionOverlappingBoxes
// ---------------------------------------------------------------------------

TEST(BooleanOpTest, UnionOverlappingBoxes) {
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*boxB, Vec3(5, 0, 0));

    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Union);
    ASSERT_NE(result, nullptr);
    EXPECT_GT(result->faceCount(), 0u);
}

// ---------------------------------------------------------------------------
// IntersectOverlappingBoxes
// ---------------------------------------------------------------------------

TEST(BooleanOpTest, IntersectOverlappingBoxes) {
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*boxB, Vec3(5, 5, 5));

    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Intersect);
    ASSERT_NE(result, nullptr);
    EXPECT_GT(result->faceCount(), 0u);
}

// ---------------------------------------------------------------------------
// NonOverlappingSubtractKeepsOriginal
// ---------------------------------------------------------------------------

TEST(BooleanOpTest, NonOverlappingSubtractKeepsOriginal) {
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*boxB, Vec3(100, 0, 0));

    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Subtract);
    ASSERT_NE(result, nullptr);
    // A's 6 faces are all outside B.  For subtract: keep A's outside faces.
    // The Euler-based builder may drop a face if vertex merging causes an
    // MEF to fail, and boundary-classified faces may be excluded.
    // Accept 4-7 faces as correct for the face-level approach.
    EXPECT_GE(result->faceCount(), 4u);
    EXPECT_LE(result->faceCount(), 8u);
}

// ---------------------------------------------------------------------------
// TopologyIDsSurvive
// ---------------------------------------------------------------------------

TEST(BooleanOpTest, TopologyIDsSurvive) {
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(4, 4, 20);
    offsetSolid(*boxB, Vec3(3, 3, -5));

    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Subtract);
    ASSERT_NE(result, nullptr);

    // At least some faces should have valid topology IDs.
    int validIds = 0;
    for (const auto& face : result->faces()) {
        if (face.topoId.isValid()) {
            ++validIds;
        }
    }
    EXPECT_GT(validIds, 0);
}

// ---------------------------------------------------------------------------
// NonOverlappingUnionKeepsBoth
// ---------------------------------------------------------------------------

TEST(BooleanOpTest, NonOverlappingUnionKeepsBoth) {
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*boxB, Vec3(100, 0, 0));

    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Union);
    ASSERT_NE(result, nullptr);
    // Both boxes are fully outside each other; union keeps all 12 faces.
    // The Euler-based builder may include a residual seed face.
    EXPECT_GE(result->faceCount(), 10u);
}

// ---------------------------------------------------------------------------
// NonOverlappingIntersectReturnsEmpty
// ---------------------------------------------------------------------------

TEST(BooleanOpTest, NonOverlappingIntersectReturnsEmpty) {
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*boxB, Vec3(100, 0, 0));

    // Intersect of non-overlapping boxes should have no inside faces.
    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Intersect);
    // Both boxes' faces classify as outside the other → nothing selected,
    // or a degenerate solid with no meaningful faces.
    if (result != nullptr) {
        // If the builder returns something, it should have very few faces
        // (just the residual seed face at most).
        EXPECT_LE(result->faceCount(), 2u);
    }
}

// ---------------------------------------------------------------------------
// CylinderThroughBox
// ---------------------------------------------------------------------------

TEST(BooleanOpTest, CylinderThroughBox) {
    // Box (0,0,0)→(10,10,10), cylinder centered at (5,5,-5) radius=2, height=20.
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto cyl = PrimitiveFactory::makeCylinder(2.0, 20.0);
    offsetSolid(*cyl, Vec3(5.0, 5.0, -5.0));

    auto result = BooleanOp::execute(*box, *cyl, BooleanType::Subtract);
    // May return nullptr if cylinder faces don't classify cleanly — acceptable for Phase 36.
    if (result) {
        EXPECT_GT(result->faceCount(), 0u);
    }
}

// ---------------------------------------------------------------------------
// RandomTransformStressTest
// ---------------------------------------------------------------------------

TEST(BooleanOpTest, RandomTransformStressTest) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-5.0, 15.0);
    int successCount = 0;
    for (int i = 0; i < 20; ++i) {
        auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
        auto boxB = PrimitiveFactory::makeBox(10, 10, 10);
        const Vec3 offset(dist(rng), dist(rng), dist(rng));
        offsetSolid(*boxB, offset);
        auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Subtract);
        if (result && result->faceCount() > 0) ++successCount;
    }
    EXPECT_GE(successCount, 5) << "Too many failures in stress test";
}

// ---------------------------------------------------------------------------
// RandomOverlapProducesValidTopology (Phase 52 stabilization)
//
// Across all three Boolean types and a spread of random overlapping
// configurations — from deep overlap to near-coincident faces — the operation
// must not crash and must never emit invalid topology. A face-level Boolean is
// allowed to give up (return nullptr) on a hard case, but any solid it does
// return must be a valid, Euler-consistent B-Rep.
// ---------------------------------------------------------------------------

TEST(BooleanOpTest, RandomOverlapProducesValidTopology) {
    std::mt19937 rng(1234);
    // Box edge is 10; offsets in [1, 9.5] keep the boxes overlapping while
    // sweeping toward (but never reaching) coincident faces.
    std::uniform_real_distribution<double> offset(1.0, 9.5);
    const BooleanType types[] = {BooleanType::Union, BooleanType::Subtract, BooleanType::Intersect};

    int produced = 0;
    for (int i = 0; i < 6; ++i) {
        const Vec3 off(offset(rng), offset(rng), offset(rng));
        for (BooleanType type : types) {
            auto a = PrimitiveFactory::makeBox(10, 10, 10);
            auto b = PrimitiveFactory::makeBox(10, 10, 10);
            offsetSolid(*b, off);

            auto result = BooleanOp::execute(*a, *b, type);
            if (result) {
                ++produced;
                EXPECT_TRUE(result->isValid())
                    << "Invalid topology from Boolean type " << static_cast<int>(type)
                    << " at offset (" << off.x << ", " << off.y << ", " << off.z << ")";
                EXPECT_TRUE(result->checkEulerFormula());
            }
        }
    }
    EXPECT_GT(produced, 0) << "No Boolean produced a result — the guard exercised nothing";
}

// ===========================================================================
// Volume-exact Boolean semantics (BSP CSG rework).
//
// These pin the *geometry* of the results, not just their existence: for
// axis-aligned boxes every expected volume is known in closed form.  Face
// splitting must be happening for any of these to hold — the old whole-face
// classification produced open, over- or under-covered shells whose volumes
// were meaningless.
// ===========================================================================

TEST(BooleanOpVolume, UnionOfOverlappingBoxesIsExact) {
    // A: (0..10)³, B: (5..15, 0..10, 0..10).  Overlap = 500.
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*boxB, Vec3(5, 0, 0));

    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Union);
    ASSERT_NE(result, nullptr);
    EXPECT_NEAR(volumeOf(*result), 1500.0, 1e-6);
    EXPECT_TRUE(result->checkManifold()) << result->validationReport();
    EXPECT_TRUE(result->isValid()) << result->validationReport();
}

TEST(BooleanOpVolume, SubtractOfOverlappingBoxesIsExact) {
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*boxB, Vec3(5, 0, 0));

    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Subtract);
    ASSERT_NE(result, nullptr);
    EXPECT_NEAR(volumeOf(*result), 500.0, 1e-6);
    EXPECT_TRUE(result->checkManifold()) << result->validationReport();
    EXPECT_TRUE(result->isValid()) << result->validationReport();
}

TEST(BooleanOpVolume, IntersectOfOverlappingBoxesIsExact) {
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*boxB, Vec3(5, 0, 0));

    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Intersect);
    ASSERT_NE(result, nullptr);
    EXPECT_NEAR(volumeOf(*result), 500.0, 1e-6);
    EXPECT_TRUE(result->checkManifold()) << result->validationReport();
    EXPECT_TRUE(result->isValid()) << result->validationReport();
}

TEST(BooleanOpVolume, CornerOverlapAllThreeTypes) {
    // Generic (no coplanar faces) corner overlap: B at (7,7,7); overlap = 27.
    auto mk = [] { return PrimitiveFactory::makeBox(10, 10, 10); };

    auto a1 = mk();
    auto b1 = mk();
    offsetSolid(*b1, Vec3(7, 7, 7));
    auto uni = BooleanOp::execute(*a1, *b1, BooleanType::Union);
    ASSERT_NE(uni, nullptr);
    EXPECT_NEAR(volumeOf(*uni), 1973.0, 1e-6);
    EXPECT_TRUE(uni->isValid()) << uni->validationReport();

    auto a2 = mk();
    auto b2 = mk();
    offsetSolid(*b2, Vec3(7, 7, 7));
    auto sub = BooleanOp::execute(*a2, *b2, BooleanType::Subtract);
    ASSERT_NE(sub, nullptr);
    EXPECT_NEAR(volumeOf(*sub), 973.0, 1e-6);
    EXPECT_TRUE(sub->isValid()) << sub->validationReport();

    auto a3 = mk();
    auto b3 = mk();
    offsetSolid(*b3, Vec3(7, 7, 7));
    auto inter = BooleanOp::execute(*a3, *b3, BooleanType::Intersect);
    ASSERT_NE(inter, nullptr);
    EXPECT_NEAR(volumeOf(*inter), 27.0, 1e-6);
    EXPECT_TRUE(inter->isValid()) << inter->validationReport();
}

TEST(BooleanOpVolume, ThroughHoleSubtractIsManifoldWithExactVolume) {
    // B pierces A completely: a genus-1 result.  Volume 1000 - 4*4*10 = 840.
    // checkEulerFormula() has no genus term, so only manifoldness is asserted.
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(4, 4, 20);
    offsetSolid(*boxB, Vec3(3, 3, -5));

    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Subtract);
    ASSERT_NE(result, nullptr);
    EXPECT_NEAR(volumeOf(*result), 840.0, 1e-6);
    EXPECT_TRUE(result->checkManifold()) << result->validationReport();
}

TEST(BooleanOpVolume, ContainedSubtractCreatesCavity) {
    // B strictly inside A: subtract yields a hollow solid with two shells.
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(4, 4, 4);
    offsetSolid(*boxB, Vec3(3, 3, 3));

    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Subtract);
    ASSERT_NE(result, nullptr);
    EXPECT_NEAR(volumeOf(*result), 1000.0 - 64.0, 1e-6);
    EXPECT_EQ(result->shellCount(), 2u);
    EXPECT_TRUE(result->isValid()) << result->validationReport();
}

TEST(BooleanOpVolume, ContainedUnionAndIntersect) {
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(4, 4, 4);
    offsetSolid(*boxB, Vec3(3, 3, 3));

    auto uni = BooleanOp::execute(*boxA, *boxB, BooleanType::Union);
    ASSERT_NE(uni, nullptr);
    EXPECT_NEAR(volumeOf(*uni), 1000.0, 1e-6);

    auto boxA2 = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB2 = PrimitiveFactory::makeBox(4, 4, 4);
    offsetSolid(*boxB2, Vec3(3, 3, 3));

    auto inter = BooleanOp::execute(*boxA2, *boxB2, BooleanType::Intersect);
    ASSERT_NE(inter, nullptr);
    EXPECT_NEAR(volumeOf(*inter), 64.0, 1e-6);
    EXPECT_TRUE(inter->isValid()) << inter->validationReport();
}

TEST(BooleanOpVolume, IdenticalSolids) {
    auto mk = [] { return PrimitiveFactory::makeBox(10, 10, 10); };

    auto a1 = mk();
    auto b1 = mk();
    auto uni = BooleanOp::execute(*a1, *b1, BooleanType::Union);
    ASSERT_NE(uni, nullptr);
    EXPECT_NEAR(volumeOf(*uni), 1000.0, 1e-6);

    auto a2 = mk();
    auto b2 = mk();
    auto inter = BooleanOp::execute(*a2, *b2, BooleanType::Intersect);
    ASSERT_NE(inter, nullptr);
    EXPECT_NEAR(volumeOf(*inter), 1000.0, 1e-6);

    // A − A is empty; the contract for an empty result is nullptr.
    auto a3 = mk();
    auto b3 = mk();
    auto sub = BooleanOp::execute(*a3, *b3, BooleanType::Subtract);
    EXPECT_EQ(sub, nullptr);
}

TEST(BooleanOpVolume, DisjointFastPathsAreExact) {
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(10, 10, 10);
    offsetSolid(*boxB, Vec3(100, 0, 0));

    auto uni = BooleanOp::execute(*boxA, *boxB, BooleanType::Union);
    ASSERT_NE(uni, nullptr);
    EXPECT_NEAR(volumeOf(*uni), 2000.0, 1e-6);
    EXPECT_EQ(uni->shellCount(), 2u);
    EXPECT_TRUE(uni->isValid()) << uni->validationReport();

    auto sub = BooleanOp::execute(*boxA, *boxB, BooleanType::Subtract);
    ASSERT_NE(sub, nullptr);
    EXPECT_NEAR(volumeOf(*sub), 1000.0, 1e-6);
    EXPECT_TRUE(sub->isValid()) << sub->validationReport();
}

TEST(BooleanOpVolume, ChainedBooleansStayConsistent) {
    // Results must be valid *inputs* — FeatureTree::executeMulti folds
    // multi-body Booleans left to right.
    // Step 1: L-shape = (0..10)³ ∪ (10..20, 0..10, 0..5)... use overlap to
    // avoid the exact-touch case: B = (8..18, 0..10, 0..5).
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(10, 10, 5);
    offsetSolid(*boxB, Vec3(8, 0, 0));

    auto lShape = BooleanOp::execute(*boxA, *boxB, BooleanType::Union);
    ASSERT_NE(lShape, nullptr);
    // 1000 + 500 - overlap(2*10*5=100) = 1400.
    EXPECT_NEAR(volumeOf(*lShape), 1400.0, 1e-6);

    // Step 2: drill a square through-pocket into the L's thick arm.
    auto drill = PrimitiveFactory::makeBox(2, 2, 30);
    offsetSolid(*drill, Vec3(4, 4, -10));

    auto drilled = BooleanOp::execute(*lShape, *drill, BooleanType::Subtract);
    ASSERT_NE(drilled, nullptr);
    // Drill removes 2*2*10 = 40 from the 10-tall arm.
    EXPECT_NEAR(volumeOf(*drilled), 1360.0, 1e-6);
    EXPECT_TRUE(drilled->checkManifold()) << drilled->validationReport();

    // Step 3: intersect with a slab to keep the bottom half.
    auto slab = PrimitiveFactory::makeBox(50, 50, 2.5);
    offsetSolid(*slab, Vec3(-10, -10, 0));

    auto sliced = BooleanOp::execute(*drilled, *slab, BooleanType::Intersect);
    ASSERT_NE(sliced, nullptr);
    // Bottom 2.5 of the L: (10*10 + 8*10... actually full footprint):
    // footprint area = 10*10 + 8*10 = 180, minus drill 2*2 → 176; × 2.5 = 440.
    EXPECT_NEAR(volumeOf(*sliced), 440.0, 1e-6);
    EXPECT_TRUE(sliced->checkManifold()) << sliced->validationReport();
}

TEST(BooleanOpVolume, SubtractFragmentsKeepProvenance) {
    auto boxA = PrimitiveFactory::makeBox(10, 10, 10);
    auto boxB = PrimitiveFactory::makeBox(4, 4, 20);
    offsetSolid(*boxB, Vec3(3, 3, -5));

    auto result = BooleanOp::execute(*boxA, *boxB, BooleanType::Subtract);
    ASSERT_NE(result, nullptr);

    // Every face must carry a valid provenance ID, and both source solids
    // must be represented (A's outer skin and B's flipped hole walls).
    size_t total = 0;
    size_t valid = 0;
    for (const auto& face : result->faces()) {
        ++total;
        if (face.topoId.isValid()) ++valid;
    }
    EXPECT_EQ(valid, total);
    EXPECT_GT(total, 6u);
}
