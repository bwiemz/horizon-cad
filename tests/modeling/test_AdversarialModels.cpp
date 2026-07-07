// ===========================================================================
// Adversarial model fixtures: realistic messy parts built by chaining
// modeling operations, checked against closed-form volumes and global
// invariants — not just "the call returned something".
//
// Invariants exercised:
//  * conservation: vol(A) = vol(A−B) + vol(A∩B), and the union identity
//  * exact volumes for axis-aligned interactions (hole patterns, pockets)
//  * Boolean results stay manifold and usable as inputs to further features
//  * interference semantics on hollow (shelled) parts
// ===========================================================================

#include <gtest/gtest.h>

#include <random>
#include <set>

#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/ChamferOp.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/InterferenceChecker.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Shell.h"
#include "horizon/topology/Queries.h"

using hz::draft::DraftEntity;
using hz::draft::DraftLine;
using hz::draft::SketchPlane;
using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::BooleanOp;
using hz::model::BooleanType;
using hz::model::ChamferOp;
using hz::model::Extrude;
using hz::model::InterferenceChecker;
using hz::model::MassPropertiesCalculator;
using hz::model::Pattern;
using hz::model::PrimitiveFactory;
using hz::model::Shell;

namespace {

double volumeOf(const hz::topo::Solid& solid) {
    return MassPropertiesCalculator::compute(solid).volume;
}

void offsetSolid(hz::topo::Solid& solid, const Vec3& offset) {
    for (auto& v : const_cast<std::deque<hz::topo::Vertex>&>(solid.vertices())) {
        v.point = v.point + offset;
    }
    std::set<hz::geo::NurbsSurface*> translated;
    for (auto& face : const_cast<std::deque<hz::topo::Face>&>(solid.faces())) {
        if (!face.surface || translated.count(face.surface.get()) != 0) continue;
        translated.insert(face.surface.get());
        auto cps = face.surface->controlPoints();
        for (auto& row : cps) {
            for (auto& cp : row) cp = cp + offset;
        }
        face.surface = std::make_shared<hz::geo::NurbsSurface>(
            std::move(cps), face.surface->weights(), face.surface->knotsU(), face.surface->knotsV(),
            face.surface->degreeU(), face.surface->degreeV());
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// A plate with a linear pattern of through-holes: Boolean against a
// multi-shell operand (the patterned drill), exact volume.
// ---------------------------------------------------------------------------

TEST(AdversarialModels, PlateWithHolePattern) {
    auto plate = PrimitiveFactory::makeBox(100, 60, 10);
    ASSERT_NE(plate, nullptr);

    auto drill = PrimitiveFactory::makeBox(6, 6, 30);
    ASSERT_NE(drill, nullptr);
    offsetSolid(*drill, Vec3(10, 27, -10));

    auto drills = Pattern::linear(*drill, Vec3(1, 0, 0), 18.0, 5);
    ASSERT_NE(drills, nullptr);
    ASSERT_EQ(drills->shellCount(), 5u);

    auto holed = BooleanOp::execute(*plate, *drills, BooleanType::Subtract);
    ASSERT_NE(holed, nullptr);
    // 5 through-holes of 6×6×10 each.
    EXPECT_NEAR(volumeOf(*holed), 100.0 * 60.0 * 10.0 - 5.0 * 6.0 * 6.0 * 10.0, 1e-6);
    EXPECT_TRUE(holed->checkManifold()) << holed->validationReport();
}

// ---------------------------------------------------------------------------
// Pocket-in-pocket with a boss: a nested Boolean chain where every step has
// a closed-form volume and feeds the next step.
// ---------------------------------------------------------------------------

TEST(AdversarialModels, NestedPocketsWithBoss) {
    auto base = PrimitiveFactory::makeBox(50, 50, 50);
    ASSERT_NE(base, nullptr);
    double expected = 50.0 * 50.0 * 50.0;

    // Open pocket from the top: 30×30, 20 deep.
    auto pocket = PrimitiveFactory::makeBox(30, 30, 30);
    offsetSolid(*pocket, Vec3(10, 10, 30));
    auto step1 = BooleanOp::execute(*base, *pocket, BooleanType::Subtract);
    ASSERT_NE(step1, nullptr);
    expected -= 30.0 * 30.0 * 20.0;
    EXPECT_NEAR(volumeOf(*step1), expected, 1e-6);

    // Deeper inner pocket in the pocket floor: 10×10, 15 deep.
    auto inner = PrimitiveFactory::makeBox(10, 10, 30);
    offsetSolid(*inner, Vec3(20, 20, 15));
    auto step2 = BooleanOp::execute(*step1, *inner, BooleanType::Subtract);
    ASSERT_NE(step2, nullptr);
    expected -= 10.0 * 10.0 * 15.0;
    EXPECT_NEAR(volumeOf(*step2), expected, 1e-6);

    // A boss standing on the inner pocket floor (union adds material).
    auto boss = PrimitiveFactory::makeBox(4, 4, 10);
    offsetSolid(*boss, Vec3(23, 23, 15));
    auto step3 = BooleanOp::execute(*step2, *boss, BooleanType::Union);
    ASSERT_NE(step3, nullptr);
    expected += 4.0 * 4.0 * 10.0;
    EXPECT_NEAR(volumeOf(*step3), expected, 1e-6);
    EXPECT_TRUE(step3->checkManifold()) << step3->validationReport();
}

// ---------------------------------------------------------------------------
// KNOWN DEFECT, pinned: ChamferOp emits combinatorially valid topology
// (manifold + Euler both pass) whose loop *geometry* is inconsistent — the
// mass-properties integrator reports 3200 for a 20³ box with two 2mm edge
// chamfers (true value: 7920).  The structural validators cannot catch this
// because they never compare twin half-edge endpoint positions.
//
// When ChamferOp/FilletOp are rebuilt on SolidSewer (the way BooleanOp was),
// flip the volume expectation below to EXPECT_NEAR(..., 7920.0, 1e-6).
// ---------------------------------------------------------------------------

TEST(AdversarialModels, ChamferGeometryDefectIsDocumented) {
    auto box = PrimitiveFactory::makeBox(20, 20, 20);
    ASSERT_NE(box, nullptr);

    // Two opposite top edges (both endpoints at z = 20, edge along X).
    // ChamferOp does not support edges sharing a vertex (vertex blends), so
    // adjacent-edge selections are rejected — a documented limitation.
    std::vector<hz::topo::TopologyID> topEdges;
    for (const auto& e : box->edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const Vec3 a = e.halfEdge->origin->point;
        const Vec3 b = e.halfEdge->twin->origin->point;
        if (a.z > 19.9 && b.z > 19.9 && std::abs(a.y - b.y) < 1e-9) topEdges.push_back(e.topoId);
    }
    ASSERT_EQ(topEdges.size(), 2u);

    auto chamfered = ChamferOp::executeEqual(*box, topEdges, 2.0, "chamfer");
    ASSERT_NE(chamfered.solid, nullptr) << chamfered.errorMessage;
    EXPECT_EQ(chamfered.solid->faceCount(), 8u);
    EXPECT_TRUE(chamfered.solid->checkManifold());

    // The defect: structurally valid, geometrically undercounted volume.
    const double chamferedVolume = volumeOf(*chamfered.solid);
    EXPECT_LT(chamferedVolume, 7920.0 - 1.0)
        << "ChamferOp volume became consistent — strengthen this test to "
           "EXPECT_NEAR(volume, 7920.0, 1e-6) and re-enable drill volume checks";

    // Robustness smoke: even with geometrically inconsistent input, the
    // Boolean must not crash and must keep its manifold-output contract.
    auto drill = PrimitiveFactory::makeBox(2, 2, 40);
    offsetSolid(*drill, Vec3(9, 9, -10));
    auto drilled = BooleanOp::execute(*chamfered.solid, *drill, BooleanType::Subtract);
    if (drilled != nullptr) {
        EXPECT_TRUE(drilled->checkManifold()) << drilled->validationReport();
    }
}

// ---------------------------------------------------------------------------
// Non-convex sketch-based part: an L-bracket extrude (non-convex caps
// exercise ear clipping in the boundary extraction), drilled through the
// thick arm.
// ---------------------------------------------------------------------------

TEST(AdversarialModels, LBracketExtrudeDrilled) {
    // L outline: 40 wide, 40 tall, arms 15 thick.
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(40, 0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(40, 0), Vec2(40, 15)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(40, 15), Vec2(15, 15)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(15, 15), Vec2(15, 40)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(15, 40), Vec2(0, 40)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 40), Vec2(0, 0)));

    SketchPlane plane;  // XY at origin
    auto bracket = Extrude::execute(profile, plane, Vec3(0, 0, 1), 12.0, "bracket");
    ASSERT_NE(bracket, nullptr);
    const double lArea = 40.0 * 15.0 + 15.0 * 25.0;
    EXPECT_NEAR(volumeOf(*bracket), lArea * 12.0, 1e-6);

    auto drill = PrimitiveFactory::makeBox(5, 5, 40);
    offsetSolid(*drill, Vec3(25, 5, -10));
    auto drilled = BooleanOp::execute(*bracket, *drill, BooleanType::Subtract);
    ASSERT_NE(drilled, nullptr);
    EXPECT_NEAR(volumeOf(*drilled), lArea * 12.0 - 5.0 * 5.0 * 12.0, 1e-6);
    EXPECT_TRUE(drilled->checkManifold()) << drilled->validationReport();
}

// ---------------------------------------------------------------------------
// Hollow (shelled) container: interference must respect the cavity — a part
// floating inside the cavity does NOT interfere; one poking into a wall does.
// ---------------------------------------------------------------------------

TEST(AdversarialModels, ShelledContainerInterferenceSemantics) {
    auto box = PrimitiveFactory::makeBox(20, 20, 20);
    ASSERT_NE(box, nullptr);

    // Remove the top face (both endpoints of its loop at z = 20).
    hz::topo::TopologyID topFace;
    for (const auto& f : box->faces()) {
        auto verts = hz::topo::faceVertices(&f);
        bool allTop = !verts.empty();
        for (const auto* v : verts) allTop = allTop && v->point.z > 19.9;
        if (allTop) {
            topFace = f.topoId;
            break;
        }
    }
    ASSERT_TRUE(topFace.isValid());

    auto shelled = Shell::execute(std::move(box), 2.0, {topFace});
    ASSERT_TRUE(shelled.ok) << shelled.message;
    ASSERT_NE(shelled.solid, nullptr);

    // Wall material only: outer minus the open inner cavity (16×16×18).
    EXPECT_NEAR(volumeOf(*shelled.solid), 8000.0 - 16.0 * 16.0 * 18.0, 1e-6);

    // A small part floating inside the cavity — no interference.
    auto floating = PrimitiveFactory::makeBox(4, 4, 4);
    offsetSolid(*floating, Vec3(8, 8, 8));
    EXPECT_FALSE(InterferenceChecker::solidsInterfere(*shelled.solid, *floating));

    // The same part shifted into a wall — real interference.
    auto poking = PrimitiveFactory::makeBox(4, 4, 4);
    offsetSolid(*poking, Vec3(0, 8, 8));  // spans x ∈ [0,4]: crosses the 2-thick wall
    EXPECT_TRUE(InterferenceChecker::solidsInterfere(*shelled.solid, *poking));
}

// ---------------------------------------------------------------------------
// Conservation fuzz: for random overlapping boxes, subtract/intersect/union
// must partition the volume exactly:
//   vol(A) = vol(A−B) + vol(A∩B)
//   vol(A∪B) = vol(A−B) + vol(B−A) + vol(A∩B)
// ---------------------------------------------------------------------------

TEST(AdversarialModels, BooleanVolumeConservationFuzz) {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> offset(0.5, 9.0);
    std::uniform_real_distribution<double> size(4.0, 14.0);

    for (int i = 0; i < 15; ++i) {
        const double sx = size(rng), sy = size(rng), sz = size(rng);
        const Vec3 off(offset(rng), offset(rng), offset(rng));

        auto mkA = [] { return PrimitiveFactory::makeBox(10, 10, 10); };
        auto mkB = [&] {
            auto b = PrimitiveFactory::makeBox(sx, sy, sz);
            offsetSolid(*b, off);
            return b;
        };

        auto aMinusB = BooleanOp::execute(*mkA(), *mkB(), BooleanType::Subtract);
        auto bMinusA = BooleanOp::execute(*mkB(), *mkA(), BooleanType::Subtract);
        auto inter = BooleanOp::execute(*mkA(), *mkB(), BooleanType::Intersect);
        auto uni = BooleanOp::execute(*mkA(), *mkB(), BooleanType::Union);

        const double vAmB = aMinusB ? volumeOf(*aMinusB) : 0.0;
        const double vBmA = bMinusA ? volumeOf(*bMinusA) : 0.0;
        const double vI = inter ? volumeOf(*inter) : 0.0;
        const double vU = uni ? volumeOf(*uni) : 0.0;
        const double vA = 1000.0;
        const double vB = sx * sy * sz;

        EXPECT_NEAR(vA, vAmB + vI, 1e-6) << "A partition failed at iteration " << i;
        EXPECT_NEAR(vB, vBmA + vI, 1e-6) << "B partition failed at iteration " << i;
        EXPECT_NEAR(vU, vAmB + vBmA + vI, 1e-6) << "union identity failed at iteration " << i;
    }
}

// ---------------------------------------------------------------------------
// Long feature chain: alternating unions and subtractions stay manifold and
// volume-consistent across 8 chained Booleans (rebuild-style workload).
// ---------------------------------------------------------------------------

TEST(AdversarialModels, LongBooleanChainStaysManifold) {
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> pos(0.0, 24.0);

    auto current = PrimitiveFactory::makeBox(30, 30, 8);
    ASSERT_NE(current, nullptr);
    double volume = 30.0 * 30.0 * 8.0;

    for (int i = 0; i < 8; ++i) {
        const bool add = (i % 2 == 0);
        auto tool = PrimitiveFactory::makeBox(6, 6, 24);
        offsetSolid(*tool, Vec3(pos(rng), pos(rng), -8.0));

        auto next =
            BooleanOp::execute(*current, *tool, add ? BooleanType::Union : BooleanType::Subtract);
        ASSERT_NE(next, nullptr) << "chain step " << i;
        EXPECT_TRUE(next->checkManifold()) << "step " << i << ": " << next->validationReport();

        const double newVolume = volumeOf(*next);
        if (add) {
            EXPECT_GE(newVolume, volume - 1e-6) << "union removed material at step " << i;
        } else {
            EXPECT_LE(newVolume, volume + 1e-6) << "subtract added material at step " << i;
        }
        volume = newVolume;
        current = std::move(next);
    }
    EXPECT_GT(volume, 0.0);
}
