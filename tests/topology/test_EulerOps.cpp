#include "horizon/topology/EulerOps.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

#include <algorithm>
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

// ===========================================================================
// Task 4: Topological Query Function Tests
// ===========================================================================

// Helper: build a triangle and return all the parts we need for query tests.
struct TriangleFixture {
    Solid solid;
    Vertex* v0 = nullptr;
    Vertex* v1 = nullptr;
    Vertex* v2 = nullptr;
    Edge* e01 = nullptr;
    Edge* e12 = nullptr;
    Edge* e20 = nullptr;
    Face* f1 = nullptr;   // original face (one triangle)
    Face* f2 = nullptr;   // new face from MEF (the other triangle)
    Shell* shell = nullptr;

    TriangleFixture() {
        auto [v0_, f_, s_] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
        v0 = v0_;
        f1 = f_;
        shell = s_;

        auto [e01_, v1_] = euler::makeEdgeVertex(solid, nullptr, f1, Vec3(1, 0, 0));
        e01 = e01_;
        v1 = v1_;

        HalfEdge* heFromV1 = e01->halfEdge->twin;
        auto [e12_, v2_] = euler::makeEdgeVertex(solid, heFromV1, f1, Vec3(0.5, 1, 0));
        e12 = e12_;
        v2 = v2_;

        HalfEdge* heAtV2 = e12->halfEdge->twin;
        HalfEdge* heAtV0 = e01->halfEdge;
        auto [e20_, f2_] = euler::makeEdgeFace(solid, heAtV2, heAtV0);
        e20 = e20_;
        f2 = f2_;
    }
};

TEST(TopologyQueryTest, TriangleFaceVertices) {
    TriangleFixture tri;

    auto verts1 = faceVertices(tri.f1);
    auto verts2 = faceVertices(tri.f2);

    // Both faces of a triangle should have exactly 3 vertices.
    EXPECT_EQ(verts1.size(), 3u);
    EXPECT_EQ(verts2.size(), 3u);

    // Each face should reference v0, v1, v2 (in some order).
    auto containsAll = [&](const std::vector<Vertex*>& verts) {
        return std::find(verts.begin(), verts.end(), tri.v0) != verts.end() &&
               std::find(verts.begin(), verts.end(), tri.v1) != verts.end() &&
               std::find(verts.begin(), verts.end(), tri.v2) != verts.end();
    };
    EXPECT_TRUE(containsAll(verts1));
    EXPECT_TRUE(containsAll(verts2));
}

TEST(TopologyQueryTest, TriangleAdjacentFaces) {
    TriangleFixture tri;

    // Triangle has 2 faces. Each should be adjacent to the other.
    auto adj1 = adjacentFaces(tri.f1);
    auto adj2 = adjacentFaces(tri.f2);

    // f1 is adjacent to f2 (and only f2).
    EXPECT_EQ(adj1.size(), 1u);
    EXPECT_EQ(adj1[0], tri.f2);

    // f2 is adjacent to f1 (and only f1).
    EXPECT_EQ(adj2.size(), 1u);
    EXPECT_EQ(adj2[0], tri.f1);
}

TEST(TopologyQueryTest, TriangleIncidentEdges) {
    TriangleFixture tri;

    // Each vertex of a triangle has exactly 2 incident edges... wait.
    // Actually in a triangle with 3 edges, each vertex has degree 2
    // (two edges meet at each vertex). But our "triangle" is a closed
    // surface (3V, 3E, 2F) on a sphere — every vertex has exactly 2 edges.
    // Wait, no. 3 vertices, 3 edges. Each vertex is shared by 2 edges.
    // But actually vertex v0 connects to v1 (e01) and to v2 (e20), so 2 edges.
    // v1 connects to v0 (e01) and v2 (e12), so 2 edges.
    // v2 connects to v1 (e12) and v0 (e20), so 2 edges.

    auto edges0 = incidentEdges(tri.v0);
    auto edges1 = incidentEdges(tri.v1);
    auto edges2 = incidentEdges(tri.v2);

    EXPECT_EQ(edges0.size(), 2u);
    EXPECT_EQ(edges1.size(), 2u);
    EXPECT_EQ(edges2.size(), 2u);
}

TEST(TopologyQueryTest, LeftAndRightFace) {
    TriangleFixture tri;

    // For each edge of the triangle, leftFace and rightFace should be the two faces.
    auto checkEdge = [&](Edge* edge) {
        Face* lf = leftFace(edge);
        Face* rf = rightFace(edge);
        EXPECT_NE(lf, nullptr);
        EXPECT_NE(rf, nullptr);
        EXPECT_NE(lf, rf);
        // One should be f1, the other f2.
        bool valid = (lf == tri.f1 && rf == tri.f2) || (lf == tri.f2 && rf == tri.f1);
        EXPECT_TRUE(valid) << "Edge's left/right faces don't match f1/f2";
    };

    checkEdge(tri.e01);
    checkEdge(tri.e12);
    checkEdge(tri.e20);
}

TEST(TopologyQueryTest, LoopSize) {
    TriangleFixture tri;

    // Both faces of a triangle have loop size 3.
    EXPECT_EQ(loopSize(tri.f1->outerLoop), 3);
    EXPECT_EQ(loopSize(tri.f2->outerLoop), 3);
}

TEST(TopologyQueryTest, EmptyWireLoopSize) {
    // loopSize of nullptr or empty wire should be 0.
    EXPECT_EQ(loopSize(nullptr), 0);

    Wire w;
    w.halfEdge = nullptr;
    EXPECT_EQ(loopSize(&w), 0);
}

TEST(TopologyQueryTest, NullInputs) {
    // Query functions should handle null gracefully.
    EXPECT_TRUE(adjacentFaces(nullptr).empty());
    EXPECT_EQ(leftFace(nullptr), nullptr);
    EXPECT_EQ(rightFace(nullptr), nullptr);
    EXPECT_TRUE(incidentEdges(nullptr).empty());
    EXPECT_TRUE(faceVertices(nullptr).empty());
}

// ===========================================================================
// Task 5: Integration Tests — Tetrahedron
// ===========================================================================

// Helper: build a tetrahedron (4V, 6E, 4F) via Euler operators.
struct TetrahedronFixture {
    Solid solid;
    Vertex* v0 = nullptr;
    Vertex* v1 = nullptr;
    Vertex* v2 = nullptr;
    Vertex* v3 = nullptr;
    Face* f1 = nullptr;  // original triangle face (after step 4)
    Face* f2 = nullptr;  // second face from closing first triangle
    Face* f3 = nullptr;  // third face from MEF connecting v3-v1
    Face* f4 = nullptr;  // fourth face from MEF connecting v3-v2
    Shell* shell = nullptr;

    // Track edges for queries
    Edge* e01 = nullptr;
    Edge* e12 = nullptr;
    Edge* e20 = nullptr;
    Edge* e03 = nullptr;
    Edge* e31 = nullptr;
    Edge* e32 = nullptr;

    TetrahedronFixture() {
        // Step 1: MVFS — v0
        auto [v0_, f_, s_] = euler::makeVertexFaceSolid(solid, Vec3(0, 0, 0));
        v0 = v0_;
        f1 = f_;
        shell = s_;

        // Step 2: MEV — v0 → v1
        auto [e01_, v1_] = euler::makeEdgeVertex(solid, nullptr, f1, Vec3(1, 0, 0));
        e01 = e01_;
        v1 = v1_;

        // Step 3: MEV — v1 → v2
        HalfEdge* heFromV1 = e01->halfEdge->twin;  // origin = v1
        auto [e12_, v2_] = euler::makeEdgeVertex(solid, heFromV1, f1, Vec3(0.5, 1, 0));
        e12 = e12_;
        v2 = v2_;

        // Step 4: MEF — close triangle (v2 → v0), creating f2
        HalfEdge* heAtV2 = e12->halfEdge->twin;  // origin = v2
        HalfEdge* heAtV0 = e01->halfEdge;         // origin = v0
        auto [e20_, f2_] = euler::makeEdgeFace(solid, heAtV2, heAtV0);
        e20 = e20_;
        f2 = f2_;

        // After step 4: 3V, 3E, 2F
        // f1 loop: heNew1(v2→v0) → heOut01(v0→v1) → heOut12(v1→v2) (front triangle)
        // f2 loop: heNew2(v0→v2) → heIn12(v2→v1) → heIn01(v1→v0) (back triangle)
        //
        // We need a half-edge at v0 in face f2.
        // heNew2 from the MEF has origin v0 and face f2.
        // e20->halfEdge is heNew1 (stays in old face f1). Its twin heNew2 is in f2.
        HalfEdge* heV0inF2 = e20->halfEdge->twin;  // origin = v0, face = f2

        // Step 5: MEV — extend v0 → v3 in face f2
        auto [e03_, v3_] = euler::makeEdgeVertex(solid, heV0inF2, f2, Vec3(0.5, 0.5, 1));
        e03 = e03_;
        v3 = v3_;

        // After step 5: 4V, 4E, 2F
        // f2 loop now: heOut03(v0→v3) → heIn03(v3→v0) → heNew2(v0→v2) → heIn12(v2→v1) → heIn01(v1→v0)
        //
        // Step 6: MEF — connect v3 to v1 in face f2
        // Need he1 with origin v3, he2 with origin v1, both in f2.
        HalfEdge* heIn03 = e03->halfEdge->twin;  // origin = v3, face = f2

        // Find the half-edge at v1 in f2. That's heIn01->twin = heOut01 in f1... no.
        // heIn01 has origin = v1 and is in f2 (it was moved there by step 4's MEF).
        // But we need to find it. heIn01 = e01->halfEdge->twin
        HalfEdge* heV1inF2 = e01->halfEdge->twin;  // origin = v1

        auto [e31_, f3_] = euler::makeEdgeFace(solid, heIn03, heV1inF2);
        e31 = e31_;
        f3 = f3_;

        // After step 6: 4V, 5E, 3F
        // f2 keeps: heNew1_s6(v3→v1) → heIn01(v1→v0) → heOut03(v0→v3) (triangle v3-v1-v0)
        // f3 gets: heNew2_s6(v1→v3) → heIn03(v3→v0) → heNew2_s4(v0→v2) → heIn12(v2→v1) (quad v1-v3-v0-v2)
        //
        // Step 7: MEF — connect v3 to v2 in face f3, splitting the quad
        // Need he1 with origin v3, he2 with origin v2, both in f3.
        // heIn03 is now in f3 (moved there by step 6's MEF). origin = v3.
        // heIn12 is in f3 (moved there by step 4's MEF, stayed through step 6). origin = v2.
        HalfEdge* heV3inF3 = e03->halfEdge->twin;   // heIn03, origin = v3, now in f3
        HalfEdge* heV2inF3 = e12->halfEdge->twin;    // heIn12, origin = v2, now in f3

        auto [e32_, f4_] = euler::makeEdgeFace(solid, heV3inF3, heV2inF3);
        e32 = e32_;
        f4 = f4_;

        // After step 7: 4V, 6E, 4F → Euler: 4-6+4 = 2 ✓
    }
};

TEST(TopologyIntegrationTest, TetrahedronEulerFormula) {
    TetrahedronFixture tet;

    EXPECT_EQ(tet.solid.vertexCount(), 4u);
    EXPECT_EQ(tet.solid.edgeCount(), 6u);
    EXPECT_EQ(tet.solid.faceCount(), 4u);
    EXPECT_EQ(tet.solid.shellCount(), 1u);

    // V - E + F = 4 - 6 + 4 = 2
    EXPECT_TRUE(tet.solid.checkEulerFormula());
    EXPECT_TRUE(tet.solid.checkManifold());
    EXPECT_TRUE(tet.solid.isValid());
}

TEST(TopologyIntegrationTest, TetrahedronEachFaceHas3Vertices) {
    TetrahedronFixture tet;

    // Every face of a tetrahedron is a triangle.
    for (const auto& face : tet.solid.faces()) {
        auto verts = faceVertices(&face);
        EXPECT_EQ(verts.size(), 3u) << "Face " << face.id << " has " << verts.size()
                                    << " vertices, expected 3";
    }
}

TEST(TopologyIntegrationTest, TetrahedronEachFaceLoopSize3) {
    TetrahedronFixture tet;

    for (const auto& face : tet.solid.faces()) {
        EXPECT_EQ(loopSize(face.outerLoop), 3) << "Face " << face.id << " loop size != 3";
    }
}

TEST(TopologyIntegrationTest, TetrahedronEachVertexHas3Edges) {
    TetrahedronFixture tet;

    // Every vertex of a tetrahedron has exactly 3 incident edges.
    auto edges0 = incidentEdges(tet.v0);
    auto edges1 = incidentEdges(tet.v1);
    auto edges2 = incidentEdges(tet.v2);
    auto edges3 = incidentEdges(tet.v3);

    EXPECT_EQ(edges0.size(), 3u) << "v0 has " << edges0.size() << " incident edges";
    EXPECT_EQ(edges1.size(), 3u) << "v1 has " << edges1.size() << " incident edges";
    EXPECT_EQ(edges2.size(), 3u) << "v2 has " << edges2.size() << " incident edges";
    EXPECT_EQ(edges3.size(), 3u) << "v3 has " << edges3.size() << " incident edges";
}

TEST(TopologyIntegrationTest, TetrahedronEachFaceHas3Neighbors) {
    TetrahedronFixture tet;

    // In a tetrahedron, every face is adjacent to all 3 other faces.
    for (const auto& face : tet.solid.faces()) {
        auto adj = adjacentFaces(&face);
        EXPECT_EQ(adj.size(), 3u) << "Face " << face.id << " has " << adj.size()
                                  << " adjacent faces, expected 3";
    }
}

TEST(TopologyIntegrationTest, TetrahedronEdgesHaveDistinctFaces) {
    TetrahedronFixture tet;

    // For every edge, leftFace and rightFace should be different.
    for (const auto& edge : tet.solid.edges()) {
        Face* lf = leftFace(&edge);
        Face* rf = rightFace(&edge);
        EXPECT_NE(lf, nullptr) << "Edge " << edge.id << " has null leftFace";
        EXPECT_NE(rf, nullptr) << "Edge " << edge.id << " has null rightFace";
        EXPECT_NE(lf, rf) << "Edge " << edge.id << " has same left and right face";
    }
}

TEST(TopologyIntegrationTest, TetrahedronTopologyIDsAssigned) {
    TetrahedronFixture tet;

    // Assign TopologyIDs to faces and verify they survive queries.
    auto id1 = TopologyID::make("tet", "f1");
    auto id2 = TopologyID::make("tet", "f2");
    auto id3 = TopologyID::make("tet", "f3");
    auto id4 = TopologyID::make("tet", "f4");

    tet.f1->topoId = id1;
    tet.f2->topoId = id2;
    tet.f3->topoId = id3;
    tet.f4->topoId = id4;

    // Verify IDs are present.
    EXPECT_EQ(tet.f1->topoId, id1);
    EXPECT_EQ(tet.f2->topoId, id2);
    EXPECT_EQ(tet.f3->topoId, id3);
    EXPECT_EQ(tet.f4->topoId, id4);

    // Query adjacentFaces and verify the returned faces have their IDs intact.
    auto adj = adjacentFaces(tet.f1);
    EXPECT_EQ(adj.size(), 3u);
    for (Face* neighbor : adj) {
        EXPECT_TRUE(neighbor->topoId.isValid()) << "Adjacent face lost its TopologyID";
    }

    // Resolve a TopologyID from a candidate list.
    std::vector<TopologyID> candidates = {id1, id2, id3, id4};
    auto resolved = TopologyID::resolve(id2, candidates);
    ASSERT_TRUE(resolved.has_value());
    EXPECT_EQ(*resolved, id2);
}

TEST(TopologyIntegrationTest, TetrahedronValidationReport) {
    TetrahedronFixture tet;

    std::string report = tet.solid.validationReport();
    // Should contain "OK" for both Euler and manifold.
    EXPECT_NE(report.find("Euler formula OK"), std::string::npos) << "Report: " << report;
    EXPECT_NE(report.find("Manifold checks OK"), std::string::npos) << "Report: " << report;
}
