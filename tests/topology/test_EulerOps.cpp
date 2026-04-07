#include "horizon/topology/EulerOps.h"
#include "horizon/topology/Solid.h"

#include <gtest/gtest.h>

using namespace hz::topo;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// MVFS
// ---------------------------------------------------------------------------

TEST(EulerOpsTest, MVFSCreatesMinimalTopology) {
    Solid solid;
    auto [v, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
    EXPECT_NE(v, nullptr);
    EXPECT_NE(f, nullptr);
    EXPECT_NE(s, nullptr);
    EXPECT_EQ(solid.vertexCount(), 1u);
    EXPECT_EQ(solid.edgeCount(), 0u);
    EXPECT_EQ(solid.faceCount(), 1u);
    EXPECT_EQ(solid.shellCount(), 1u);
    // Euler: 1 - 0 + 1 = 2, S=1, H=0 → 2*(1-0) = 2.
    EXPECT_TRUE(solid.checkEulerFormula());
}

TEST(EulerOpsTest, MVFSVertexHasCorrectPoint) {
    Solid solid;
    auto [v, f, s] = euler::makeVertexFaceSolid(solid, Vec3(3, 4, 5));
    EXPECT_DOUBLE_EQ(v->point.x, 3.0);
    EXPECT_DOUBLE_EQ(v->point.y, 4.0);
    EXPECT_DOUBLE_EQ(v->point.z, 5.0);
}

TEST(EulerOpsTest, MVFSFaceHasWireButNoEdges) {
    Solid solid;
    auto [v, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
    EXPECT_NE(f->outerLoop, nullptr);
    EXPECT_EQ(f->outerLoop->halfEdge, nullptr);
    EXPECT_TRUE(f->innerLoops.empty());
    EXPECT_EQ(f->shell, s);
}

TEST(EulerOpsTest, MVFSShellReferences) {
    Solid solid;
    auto [v, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
    EXPECT_EQ(s->faces.size(), 1u);
    EXPECT_EQ(s->faces[0], f);
    EXPECT_EQ(s->solid, &solid);
}

// ---------------------------------------------------------------------------
// MEV — first edge on an MVFS face (he == nullptr)
// ---------------------------------------------------------------------------

TEST(EulerOpsTest, MEVFirstEdgeCreatesEdgeAndVertex) {
    Solid solid;
    auto [v0, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
    auto [e1, v1] = euler::makeEdgeVertex(solid, nullptr, f, Vec3(1, 0, 0));

    EXPECT_NE(e1, nullptr);
    EXPECT_NE(v1, nullptr);
    EXPECT_EQ(solid.vertexCount(), 2u);
    EXPECT_EQ(solid.edgeCount(), 1u);
    EXPECT_EQ(solid.faceCount(), 1u);

    // Euler: 2 - 1 + 1 = 2.
    EXPECT_TRUE(solid.checkEulerFormula());
}

TEST(EulerOpsTest, MEVFirstEdgeHalfEdgeLoop) {
    Solid solid;
    auto [v0, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
    auto [e1, v1] = euler::makeEdgeVertex(solid, nullptr, f, Vec3(1, 0, 0));

    // There should be exactly 2 half-edges forming a closed loop.
    HalfEdge* heOut = e1->halfEdge;
    ASSERT_NE(heOut, nullptr);
    HalfEdge* heIn = heOut->twin;
    ASSERT_NE(heIn, nullptr);

    // Loop: heOut → heIn → heOut.
    EXPECT_EQ(heOut->next, heIn);
    EXPECT_EQ(heIn->next, heOut);
    EXPECT_EQ(heOut->prev, heIn);
    EXPECT_EQ(heIn->prev, heOut);

    // Origins.
    EXPECT_EQ(heOut->origin, v0);
    EXPECT_EQ(heIn->origin, v1);

    // Both on the same face.
    EXPECT_EQ(heOut->face, f);
    EXPECT_EQ(heIn->face, f);

    // Wire updated.
    EXPECT_EQ(f->outerLoop->halfEdge, heOut);
}

// ---------------------------------------------------------------------------
// MEV — general case (extending from an existing half-edge)
// ---------------------------------------------------------------------------

TEST(EulerOpsTest, MEVGeneralCaseExtends) {
    Solid solid;
    auto [v0, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
    auto [e01, v1] = euler::makeEdgeVertex(solid, nullptr, f, Vec3(1, 0, 0));

    // After the first MEV we have heOut (v0→v1) and heIn (v1→v0) in a loop.
    // To extend from v1, we need a half-edge with origin = v1.
    // That's heIn (v1→v0).  We'll extend from v1.
    HalfEdge* heFromV1 = e01->halfEdge->twin;  // origin = v1
    ASSERT_EQ(heFromV1->origin, v1);

    auto [e12, v2] = euler::makeEdgeVertex(solid, heFromV1, f, Vec3(1, 1, 0));

    EXPECT_EQ(solid.vertexCount(), 3u);
    EXPECT_EQ(solid.edgeCount(), 2u);
    EXPECT_EQ(solid.faceCount(), 1u);

    // Euler: 3 - 2 + 1 = 2.
    EXPECT_TRUE(solid.checkEulerFormula());
}

TEST(EulerOpsTest, MEVChainFormsCorrectLoop) {
    Solid solid;
    auto [v0, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));

    // MEV: v0 → v1
    auto [e01, v1] = euler::makeEdgeVertex(solid, nullptr, f, Vec3(1, 0, 0));

    // MEV: v1 → v2
    HalfEdge* heFromV1 = e01->halfEdge->twin;
    auto [e12, v2] = euler::makeEdgeVertex(solid, heFromV1, f, Vec3(1, 1, 0));

    // The loop should be:
    // v0→v1 (heOut01), v1→v2 (heOut12), v2→v1 (heIn12), v1→v0 (heIn01)
    // Actually let's just check the loop is closed with the right length.
    HalfEdge* start = f->outerLoop->halfEdge;
    ASSERT_NE(start, nullptr);

    int count = 0;
    HalfEdge* cur = start;
    do {
        cur = cur->next;
        ++count;
        ASSERT_LE(count, 10) << "Loop not closed";
    } while (cur != start);

    // 2 edges = 4 half-edges in the loop.
    EXPECT_EQ(count, 4);

    // Also verify manifold (all next/prev consistent).
    EXPECT_TRUE(solid.checkManifold());
}

// ---------------------------------------------------------------------------
// MEF — build a triangle
// ---------------------------------------------------------------------------

TEST(EulerOpsTest, BuildTriangle) {
    Solid solid;

    // MVFS: v0 at origin.
    auto [v0, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));

    // MEV: v0 → v1.
    auto [e01, v1] = euler::makeEdgeVertex(solid, nullptr, f, Vec3(1, 0, 0));

    // MEV: v1 → v2.
    // heIn01 has origin = v1.
    HalfEdge* heFromV1 = e01->halfEdge->twin;
    ASSERT_EQ(heFromV1->origin, v1);
    auto [e12, v2] = euler::makeEdgeVertex(solid, heFromV1, f, Vec3(0.5, 1, 0));

    // Now we have 3 vertices, 2 edges, 1 face.
    EXPECT_EQ(solid.vertexCount(), 3u);
    EXPECT_EQ(solid.edgeCount(), 2u);
    EXPECT_EQ(solid.faceCount(), 1u);

    // The half-edge loop (starting from wire):
    // We need to find two half-edges with origins v2 and v0 to close the triangle.
    // After the two MEVs, the loop is:
    //   heOut01 (v0→v1) → heOut12 (v1→v2) → heIn12 (v2→v1) → heIn01 (v1→v0) → back to heOut01
    //
    // To close v2→v0, we need:
    //   he1 with origin v2 (that's heIn12)
    //   he2 with origin v0 (that's heOut01)

    // Find the half-edge with origin = v2.
    HalfEdge* heAtV2 = e12->halfEdge->twin;  // heIn12, origin = v2
    ASSERT_EQ(heAtV2->origin, v2);

    // Find the half-edge with origin = v0.
    HalfEdge* heAtV0 = e01->halfEdge;  // heOut01, origin = v0
    ASSERT_EQ(heAtV0->origin, v0);

    // MEF: close the triangle.
    auto [e20, f2] = euler::makeEdgeFace(solid, heAtV2, heAtV0);

    EXPECT_EQ(solid.vertexCount(), 3u);
    EXPECT_EQ(solid.edgeCount(), 3u);
    EXPECT_EQ(solid.faceCount(), 2u);

    // Euler: 3 - 3 + 2 = 2, S=1, H=0. ✓
    EXPECT_TRUE(solid.checkEulerFormula());
    EXPECT_TRUE(solid.checkManifold());
}

TEST(EulerOpsTest, TriangleFacesHaveCorrectLoopSizes) {
    Solid solid;
    auto [v0, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
    auto [e01, v1] = euler::makeEdgeVertex(solid, nullptr, f, Vec3(1, 0, 0));
    HalfEdge* heFromV1 = e01->halfEdge->twin;
    auto [e12, v2] = euler::makeEdgeVertex(solid, heFromV1, f, Vec3(0.5, 1, 0));

    HalfEdge* heAtV2 = e12->halfEdge->twin;
    HalfEdge* heAtV0 = e01->halfEdge;
    auto [e20, f2] = euler::makeEdgeFace(solid, heAtV2, heAtV0);

    // Count half-edges in each face's loop.
    auto countLoop = [](Face* face) -> int {
        if (face->outerLoop == nullptr || face->outerLoop->halfEdge == nullptr) return 0;
        int n = 0;
        HalfEdge* start = face->outerLoop->halfEdge;
        HalfEdge* cur = start;
        do {
            ++n;
            cur = cur->next;
        } while (cur != start && n <= 100);
        return n;
    };

    // A triangle divides the original face into two: both should have 3 half-edges.
    int oldFaceLoop = countLoop(f);
    int newFaceLoop = countLoop(f2);
    EXPECT_EQ(oldFaceLoop, 3);
    EXPECT_EQ(newFaceLoop, 3);
}

// ---------------------------------------------------------------------------
// Build a quadrilateral (v0 → v1 → v2 → v3, close with MEF)
// ---------------------------------------------------------------------------

TEST(EulerOpsTest, BuildQuad) {
    Solid solid;
    auto [v0, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));

    auto [e01, v1] = euler::makeEdgeVertex(solid, nullptr, f, Vec3(1, 0, 0));
    HalfEdge* heFromV1 = e01->halfEdge->twin;
    auto [e12, v2] = euler::makeEdgeVertex(solid, heFromV1, f, Vec3(1, 1, 0));
    HalfEdge* heFromV2 = e12->halfEdge->twin;
    auto [e23, v3] = euler::makeEdgeVertex(solid, heFromV2, f, Vec3(0, 1, 0));

    EXPECT_EQ(solid.vertexCount(), 4u);
    EXPECT_EQ(solid.edgeCount(), 3u);
    EXPECT_EQ(solid.faceCount(), 1u);

    // Close with MEF from v3 back to v0.
    HalfEdge* heAtV3 = e23->halfEdge->twin;
    HalfEdge* heAtV0 = e01->halfEdge;
    ASSERT_EQ(heAtV3->origin, v3);
    ASSERT_EQ(heAtV0->origin, v0);

    auto [e30, f2] = euler::makeEdgeFace(solid, heAtV3, heAtV0);

    EXPECT_EQ(solid.vertexCount(), 4u);
    EXPECT_EQ(solid.edgeCount(), 4u);
    EXPECT_EQ(solid.faceCount(), 2u);

    // Euler: 4 - 4 + 2 = 2. ✓
    EXPECT_TRUE(solid.checkEulerFormula());
    EXPECT_TRUE(solid.checkManifold());
}

// ---------------------------------------------------------------------------
// KEV — undo the last MEV
// ---------------------------------------------------------------------------

TEST(EulerOpsTest, KEVRemovesEdgeAndVertex) {
    Solid solid;
    auto [v0, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
    auto [e01, v1] = euler::makeEdgeVertex(solid, nullptr, f, Vec3(1, 0, 0));

    EXPECT_EQ(solid.vertexCount(), 2u);
    EXPECT_EQ(solid.edgeCount(), 1u);

    euler::killEdgeVertex(solid, e01);

    // The edge and vertex are "dead" (nulled out).
    // The face's wire should have no half-edges (back to MVFS state).
    EXPECT_EQ(f->outerLoop->halfEdge, nullptr);
    EXPECT_EQ(v0->halfEdge, nullptr);  // v0 has no outgoing edges now.
}

TEST(EulerOpsTest, KEVOnChainRemovesLeaf) {
    Solid solid;
    auto [v0, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
    auto [e01, v1] = euler::makeEdgeVertex(solid, nullptr, f, Vec3(1, 0, 0));
    HalfEdge* heFromV1 = e01->halfEdge->twin;
    auto [e12, v2] = euler::makeEdgeVertex(solid, heFromV1, f, Vec3(1, 1, 0));

    // Kill the last edge (e12), removing v2.
    euler::killEdgeVertex(solid, e12);

    // Should be back to just v0-v1 (one edge, closed loop of 2 half-edges).
    EXPECT_NE(f->outerLoop->halfEdge, nullptr);
    HalfEdge* remaining = f->outerLoop->halfEdge;
    EXPECT_EQ(remaining->next->next, remaining);  // 2-element loop.
}

// ---------------------------------------------------------------------------
// KEF — undo an MEF
// ---------------------------------------------------------------------------

TEST(EulerOpsTest, KEFMergesFaces) {
    Solid solid;
    auto [v0, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
    auto [e01, v1] = euler::makeEdgeVertex(solid, nullptr, f, Vec3(1, 0, 0));
    HalfEdge* heFromV1 = e01->halfEdge->twin;
    auto [e12, v2] = euler::makeEdgeVertex(solid, heFromV1, f, Vec3(0.5, 1, 0));

    HalfEdge* heAtV2 = e12->halfEdge->twin;
    HalfEdge* heAtV0 = e01->halfEdge;
    auto [e20, f2] = euler::makeEdgeFace(solid, heAtV2, heAtV0);

    // We have a triangle: 3V, 3E, 2F.
    EXPECT_EQ(solid.shellCount(), 1u);
    size_t facesBefore = s->faces.size();
    EXPECT_EQ(facesBefore, 2u);

    // Kill the diagonal edge that MEF created, merging faces.
    euler::killEdgeFace(solid, e20);

    // The shell should have only 1 face now.
    EXPECT_EQ(s->faces.size(), 1u);

    // The remaining face's loop should have 4 half-edges (two edges, each with 2 HEs).
    HalfEdge* start = f->outerLoop->halfEdge;
    ASSERT_NE(start, nullptr);
    int count = 0;
    HalfEdge* cur = start;
    do {
        cur = cur->next;
        ++count;
        ASSERT_LE(count, 20) << "Loop not closed after KEF";
    } while (cur != start);
    EXPECT_EQ(count, 4);
}

// ---------------------------------------------------------------------------
// Full round-trip: build and tear down
// ---------------------------------------------------------------------------

TEST(EulerOpsTest, BuildAndTearDownTriangle) {
    Solid solid;
    auto [v0, f, s] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
    auto [e01, v1] = euler::makeEdgeVertex(solid, nullptr, f, Vec3(1, 0, 0));
    HalfEdge* heFromV1 = e01->halfEdge->twin;
    auto [e12, v2] = euler::makeEdgeVertex(solid, heFromV1, f, Vec3(0.5, 1, 0));

    HalfEdge* heAtV2 = e12->halfEdge->twin;
    HalfEdge* heAtV0 = e01->halfEdge;
    auto [e20, f2] = euler::makeEdgeFace(solid, heAtV2, heAtV0);

    // Triangle built: 3V, 3E, 2F, 1S.
    EXPECT_TRUE(solid.checkEulerFormula());
    EXPECT_TRUE(solid.checkManifold());

    // Tear down in reverse order.
    euler::killEdgeFace(solid, e20);
    euler::killEdgeVertex(solid, e12);
    euler::killEdgeVertex(solid, e01);

    // Back to MVFS state: wire has no half-edges.
    EXPECT_EQ(f->outerLoop->halfEdge, nullptr);
    EXPECT_EQ(v0->halfEdge, nullptr);
}
