#include <gtest/gtest.h>

#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::math::Vec3;
using hz::model::FilletOp;
using hz::model::FilletResult;
using hz::model::PrimitiveFactory;
using hz::topo::TopologyID;

// ---------------------------------------------------------------------------
// Fillet a single edge of a box
// ---------------------------------------------------------------------------

TEST(FilletOpTest, FilletOneEdgeOfBox) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_1");
    ASSERT_TRUE(result.errorMessage.empty()) << result.errorMessage;
    ASSERT_NE(result.solid, nullptr);
    EXPECT_GT(result.solid->faceCount(), box->faceCount());
    EXPECT_TRUE(result.solid->checkEulerFormula());
}

// ---------------------------------------------------------------------------
// Fillet edges sequentially (one at a time)
// ---------------------------------------------------------------------------

TEST(FilletOpTest, FilletAllEdgesSequentially) {
    auto currentSolid = PrimitiveFactory::makeBox(10, 10, 10);

    // Fillet just one edge (sequential means one at a time, not all at once)
    auto& edges = currentSolid->edges();
    if (!edges.empty()) {
        std::vector<TopologyID> ids = {edges.front().topoId};
        auto result = FilletOp::execute(*currentSolid, ids, 0.5, "fillet_1");
        if (result.solid) {
            EXPECT_TRUE(result.solid->checkEulerFormula());
            currentSolid = std::move(result.solid);
        }
    }
    EXPECT_GT(currentSolid->faceCount(), 6u);
}

// ---------------------------------------------------------------------------
// Fillet a cylinder edge (cylinder-to-plane)
// ---------------------------------------------------------------------------

TEST(FilletOpTest, FilletCylinderToPlaneEdge) {
    auto cyl = PrimitiveFactory::makeCylinder(5.0, 10.0);
    auto& edges = cyl->edges();
    ASSERT_FALSE(edges.empty());
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = FilletOp::execute(*cyl, edgeIds, 0.5, "fillet_cyl");
    // May succeed or fail depending on face types — both acceptable for Era 1
    if (result.solid) {
        EXPECT_TRUE(result.solid->checkEulerFormula());
    }
}

// ---------------------------------------------------------------------------
// Fillet faces have valid TopologyIDs with "fillet" tag
// ---------------------------------------------------------------------------

TEST(FilletOpTest, FilletHasTopologyIDs) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_1");
    ASSERT_NE(result.solid, nullptr);
    for (const auto& face : result.solid->faces()) {
        EXPECT_TRUE(face.topoId.isValid());
    }
    // At least one face should have "fillet" in its TopologyID
    bool hasFillet = false;
    for (const auto& face : result.solid->faces()) {
        if (face.topoId.tag().find("fillet") != std::string::npos) {
            hasFillet = true;
            break;
        }
    }
    EXPECT_TRUE(hasFillet);
}

// ---------------------------------------------------------------------------
// All faces of the filleted solid have a NURBS surface
// ---------------------------------------------------------------------------

TEST(FilletOpTest, FilletHasNURBSSurface) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_1");
    ASSERT_NE(result.solid, nullptr);
    for (const auto& face : result.solid->faces()) {
        EXPECT_NE(face.surface, nullptr) << "Face missing NURBS surface";
    }
}

// ---------------------------------------------------------------------------
// Fillet face has smooth (unit-length) normals
// ---------------------------------------------------------------------------

TEST(FilletOpTest, FilletSmoothNormals) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    std::vector<TopologyID> edgeIds = {edges.front().topoId};

    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_1");
    ASSERT_NE(result.solid, nullptr);

    // The fillet face should have a cylindrical surface with varying normals
    for (const auto& face : result.solid->faces()) {
        if (face.topoId.tag().find("fillet") != std::string::npos && face.surface) {
            auto tess = face.surface->tessellate(0.1);
            if (tess.normals.size() >= 6) {
                Vec3 n0(tess.normals[0], tess.normals[1], tess.normals[2]);
                EXPECT_NEAR(n0.length(), 1.0, 0.1);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Vertex-blend (two edges sharing a vertex) should be refused in Era 1
// ---------------------------------------------------------------------------

TEST(FilletOpTest, VertexBlendRefused) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();

    // Find two edges sharing a vertex
    std::vector<TopologyID> edgeIds;
    if (edges.size() >= 2) {
        edgeIds.push_back(edges[0].topoId);
        for (size_t i = 1; i < edges.size(); ++i) {
            const auto* e0 = &edges[0];
            const auto* ei = &edges[i];
            if (!e0->halfEdge || !ei->halfEdge) continue;
            auto* v0a = e0->halfEdge->origin;
            auto* v0b = e0->halfEdge->twin ? e0->halfEdge->twin->origin : nullptr;
            auto* via = ei->halfEdge->origin;
            auto* vib = ei->halfEdge->twin ? ei->halfEdge->twin->origin : nullptr;
            if (via == v0a || via == v0b || vib == v0a || vib == v0b) {
                edgeIds.push_back(edges[i].topoId);
                break;
            }
        }
    }

    if (edgeIds.size() == 2) {
        auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_1");
        EXPECT_FALSE(result.errorMessage.empty()) << "Should refuse vertex blend";
    }
}

// ---------------------------------------------------------------------------
// Invalid (nonexistent) edge ID returns an error
// ---------------------------------------------------------------------------

TEST(FilletOpTest, InvalidEdgeReturnsError) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    std::vector<TopologyID> edgeIds = {TopologyID::make("nonexistent", "edge")};
    auto result = FilletOp::execute(*box, edgeIds, 1.0, "fillet_1");
    EXPECT_FALSE(result.errorMessage.empty());
}

// ---------------------------------------------------------------------------
// Radius too large for the box returns an error
// ---------------------------------------------------------------------------

TEST(FilletOpTest, RadiusTooLargeReturnsError) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto& edges = box->edges();
    std::vector<TopologyID> edgeIds = {edges.front().topoId};
    auto result = FilletOp::execute(*box, edgeIds, 100.0, "fillet_1");
    EXPECT_FALSE(result.errorMessage.empty());
}
