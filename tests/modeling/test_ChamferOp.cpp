#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>
#include <vector>

#include "horizon/modeling/ChamferOp.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Solid.h"

using hz::model::ChamferOp;
using hz::model::ChamferResult;
using hz::model::MassPropertiesCalculator;
using hz::model::PrimitiveFactory;
using hz::topo::TopologyID;

// ---------------------------------------------------------------------------
// Chamfer a single edge with equal distance
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, ChamferOneEdge) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = ChamferOp::executeEqual(*box, edgeIds, 1.0, "chamfer_1");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_GT(result.solid->faceCount(), box->faceCount());
    EXPECT_TRUE(result.solid->checkEulerFormula());
}

// ---------------------------------------------------------------------------
// Two-distance chamfer
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, TwoDistanceChamfer) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = ChamferOp::executeTwoDistance(*box, edgeIds, 1.0, 2.0, "chamfer_1");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_TRUE(result.solid->checkEulerFormula());
}

// ---------------------------------------------------------------------------
// Chamfer faces have valid TopologyIDs
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, ChamferHasTopologyIDs) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = ChamferOp::executeEqual(*box, edgeIds, 1.0, "chamfer_1");
    ASSERT_NE(result.solid, nullptr);
    for (const auto& face : result.solid->faces()) {
        EXPECT_TRUE(face.topoId.isValid());
    }
}

// ---------------------------------------------------------------------------
// Closed-form volumes.
//
// A chamfer of distances (a, b) along an edge of length L removes a
// triangular prism of volume (1/2) a b L.  These are exact, so they are the
// real check on the reconstruction — face counts and Euler consistency were
// already satisfied by the previous implementation while its loop geometry
// was wrong.
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, EqualDistanceRemovesATriangularPrism) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto edgeIds = std::vector<TopologyID>{box->edges().front().topoId};

    auto result = ChamferOp::executeEqual(*box, edgeIds, 1.0, "chamfer_1");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*result.solid).volume,
                1000.0 - 0.5 * 1.0 * 1.0 * 10.0, 1e-9);
}

TEST(ChamferOpTest, TwoDistanceRemovesTheAsymmetricPrism) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto edgeIds = std::vector<TopologyID>{box->edges().front().topoId};

    auto result = ChamferOp::executeTwoDistance(*box, edgeIds, 1.0, 2.0, "chamfer_1");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*result.solid).volume,
                1000.0 - 0.5 * 1.0 * 2.0 * 10.0, 1e-9);
}

TEST(ChamferOpTest, ChamferingTwoParallelEdgesIsAdditive) {
    // Two chamfers that do not share a vertex remove independent prisms.
    auto box = PrimitiveFactory::makeBox(20, 20, 20);
    ASSERT_NE(box, nullptr);

    std::vector<TopologyID> topEdges;
    for (const auto& e : box->edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const auto a = e.halfEdge->origin->point;
        const auto b = e.halfEdge->twin->origin->point;
        if (a.z > 19.9 && b.z > 19.9 && std::abs(a.y - b.y) < 1e-9) topEdges.push_back(e.topoId);
    }
    ASSERT_EQ(topEdges.size(), 2u);

    auto result = ChamferOp::executeEqual(*box, topEdges, 3.0, "chamfer_2");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    EXPECT_NEAR(MassPropertiesCalculator::compute(*result.solid).volume,
                8000.0 - 2.0 * 0.5 * 3.0 * 3.0 * 20.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Geometric validity.
//
// The reconstruction is a SolidSewer sew of rewritten face loops, and the op
// refuses to hand back a result the geometric validator rejects — so these
// assert the contract rather than a happy path.
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, ResultIsGeometricallyValidNotJustManifold) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto edgeIds = std::vector<TopologyID>{box->edges().front().topoId};

    auto result = ChamferOp::executeEqual(*box, edgeIds, 2.0, "chamfer_1");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;
    EXPECT_TRUE(result.solid->isValid()) << result.solid->validationReport();
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*result.solid))
        << hz::topo::GeometryValidator::report(*result.solid);
}

TEST(ChamferOpTest, OriginalFacesKeepTheirIdentityAndCarriers) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto edgeIds = std::vector<TopologyID>{box->edges().front().topoId};

    auto result = ChamferOp::executeEqual(*box, edgeIds, 1.0, "chamfer_1");
    ASSERT_NE(result.solid, nullptr) << result.errorMessage;

    // Six originals plus one chamfer face; the originals keep their tags
    // (offsets slide along the neighbouring faces, so no plane moves).
    EXPECT_EQ(result.solid->faceCount(), 7u);
    std::set<std::string> tags;
    for (const auto& f : result.solid->faces()) {
        EXPECT_NE(f.surface, nullptr);
        tags.insert(f.topoId.tag());
    }
    for (const auto* name :
         {"box/bottom", "box/top", "box/front", "box/back", "box/right", "box/left"}) {
        EXPECT_EQ(tags.count(name), 1u) << "missing original face " << name;
    }
}

// ---------------------------------------------------------------------------
// Chained use: a chamfered solid must be a first-class modeling input.
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, ChamfersChain) {
    auto box = PrimitiveFactory::makeBox(20, 20, 20);
    ASSERT_NE(box, nullptr);

    // Chamfer a bottom edge, then a top edge of the result — the two are
    // disjoint, so the removed prisms simply add up.
    std::vector<TopologyID> first;
    for (const auto& e : box->edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const auto a = e.halfEdge->origin->point;
        const auto b = e.halfEdge->twin->origin->point;
        if (a.z < 0.1 && b.z < 0.1 && std::abs(a.y - b.y) < 1e-9) {
            first.push_back(e.topoId);
            break;
        }
    }
    ASSERT_EQ(first.size(), 1u);

    auto step1 = ChamferOp::executeEqual(*box, first, 2.0, "c1");
    ASSERT_NE(step1.solid, nullptr) << step1.errorMessage;

    std::vector<TopologyID> second;
    for (const auto& e : step1.solid->edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const auto a = e.halfEdge->origin->point;
        const auto b = e.halfEdge->twin->origin->point;
        if (a.z > 19.9 && b.z > 19.9 && std::abs(a.y - b.y) < 1e-9) {
            second.push_back(e.topoId);
            break;
        }
    }
    ASSERT_EQ(second.size(), 1u);

    auto step2 = ChamferOp::executeEqual(*step1.solid, second, 2.0, "c2");
    ASSERT_NE(step2.solid, nullptr) << step2.errorMessage;
    EXPECT_TRUE(hz::topo::GeometryValidator::isGeometricallyValid(*step2.solid))
        << hz::topo::GeometryValidator::report(*step2.solid);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*step2.solid).volume,
                8000.0 - 2.0 * 0.5 * 2.0 * 2.0 * 20.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Refusals.
// ---------------------------------------------------------------------------

TEST(ChamferOpTest, VertexBlendIsRefused) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);

    // Two edges of the top face share a corner.
    std::vector<TopologyID> adjacent;
    const hz::topo::Vertex* shared = nullptr;
    for (const auto& e : box->edges()) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        if (adjacent.empty()) {
            adjacent.push_back(e.topoId);
            shared = e.halfEdge->twin->origin;
            continue;
        }
        if (e.halfEdge->origin == shared || e.halfEdge->twin->origin == shared) {
            adjacent.push_back(e.topoId);
            break;
        }
    }
    ASSERT_EQ(adjacent.size(), 2u);

    auto result = ChamferOp::executeEqual(*box, adjacent, 1.0, "blend");
    EXPECT_EQ(result.solid, nullptr);
    EXPECT_NE(result.errorMessage.find("Vertex blend"), std::string::npos) << result.errorMessage;
}

TEST(ChamferOpTest, InvalidInputsAreRefused) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    ASSERT_NE(box, nullptr);
    const auto edgeIds = std::vector<TopologyID>{box->edges().front().topoId};

    EXPECT_EQ(ChamferOp::executeEqual(*box, edgeIds, 0.0, "c").solid, nullptr);
    EXPECT_EQ(ChamferOp::executeEqual(*box, {}, 1.0, "c").solid, nullptr);
    // Half the shortest adjacent edge is the cap; 10 is far past it.
    EXPECT_EQ(ChamferOp::executeEqual(*box, edgeIds, 10.0, "c").solid, nullptr);
    EXPECT_EQ(ChamferOp::executeEqual(*box, {TopologyID::make("nope", "edge")}, 1.0, "c").solid,
              nullptr);
}
