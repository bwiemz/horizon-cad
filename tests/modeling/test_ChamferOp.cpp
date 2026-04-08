#include <gtest/gtest.h>

#include "horizon/modeling/ChamferOp.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::model::ChamferOp;
using hz::model::ChamferResult;
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
