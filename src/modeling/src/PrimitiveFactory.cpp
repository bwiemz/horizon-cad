#include "horizon/modeling/PrimitiveFactory.h"

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/topology/EulerOps.h"
#include "horizon/topology/Queries.h"

#include <cassert>
#include <cmath>
#include <memory>

namespace hz::model {

using namespace hz::topo;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// Helper: find the half-edge originating from 'origin' on 'face'.
// When multiple half-edges from the same vertex exist on a face, 'prev_origin'
// disambiguates: we want the HE whose prev->origin == prev_origin (i.e. the HE
// that arrives FROM prev_origin, then the NEXT HE departs from 'origin').
// ---------------------------------------------------------------------------

static HalfEdge* findHE(Face* face, Vertex* origin, Vertex* prevOrigin = nullptr) {
    if (face->outerLoop == nullptr || face->outerLoop->halfEdge == nullptr) {
        return nullptr;
    }
    HalfEdge* start = face->outerLoop->halfEdge;
    HalfEdge* cur = start;
    HalfEdge* fallback = nullptr;
    do {
        if (cur->origin == origin) {
            if (prevOrigin == nullptr) {
                return cur;
            }
            if (cur->prev->origin == prevOrigin) {
                return cur;
            }
            fallback = cur;  // right origin but wrong prev; remember as fallback
        }
        cur = cur->next;
    } while (cur != start);
    return fallback;  // may be nullptr if not found at all
}

// ---------------------------------------------------------------------------
// Helper: make a degree-1 (linear) NURBS curve between two points.
// ---------------------------------------------------------------------------

static std::shared_ptr<geo::NurbsCurve> makeLineCurve(const Vec3& a, const Vec3& b) {
    return std::make_shared<geo::NurbsCurve>(
        std::vector<Vec3>{a, b}, std::vector<double>{1.0, 1.0},
        std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
}

// ---------------------------------------------------------------------------
// Helper: assign a line curve to an edge based on its half-edge endpoints.
// ---------------------------------------------------------------------------

static void assignEdgeCurve(Edge* edge) {
    assert(edge->halfEdge != nullptr);
    HalfEdge* he = edge->halfEdge;
    const Vec3& a = he->origin->point;
    const Vec3& b = he->twin->origin->point;
    edge->curve = makeLineCurve(a, b);
}

// ---------------------------------------------------------------------------
// Build the canonical box topology: 8V, 12E, 6F.
//
// Returns vertex pointers in order:
//   v0=(0,0,0)  v1=(w,0,0)  v2=(w,h,0)  v3=(0,h,0)   -- bottom (z=0)
//   v4=(0,0,d)  v5=(w,0,d)  v6=(w,h,d)  v7=(0,h,d)   -- top    (z=d)
//
// The face assignment (after construction) is:
//   bottom (v0-v1-v2-v3), top (v4-v5-v6-v7),
//   front (v0-v1-v5-v4),  back (v2-v3-v7-v6),
//   right (v1-v2-v6-v5),  left (v3-v0-v4-v7)
// ---------------------------------------------------------------------------

struct BoxBuild {
    Vertex* v[8] = {};
    Face* bottom = nullptr;
    Face* top = nullptr;
    Face* front = nullptr;
    Face* right = nullptr;
    Face* back = nullptr;
    Face* left = nullptr;
    Face* outer = nullptr;  // the original MVFS "outer" face — becomes top at the end
};

static BoxBuild buildBoxTopology(Solid& solid, const Vec3 pts[8]) {
    BoxBuild b;

    // Step 1: MVFS — creates v0, face0 (outer), shell.
    auto [v0, fOuter, shell] = euler::makeVertexFaceSolid(solid, pts[0]);
    b.v[0] = v0;
    b.outer = fOuter;

    // Step 2: MEV v0 → v1 (first edge on the face, he = nullptr).
    auto [e01, v1] = euler::makeEdgeVertex(solid, nullptr, fOuter, pts[1]);
    b.v[1] = v1;

    // Step 3: MEV from v1 → v2.
    // After step 2: the face loop is (heOut01: v0→v1) → (heIn01: v1→v0) → cycle.
    // We want to extend from v1. We need a HE originating at v1 on fOuter.
    HalfEdge* heAtV1 = findHE(fOuter, v1);
    auto [e12, v2] = euler::makeEdgeVertex(solid, heAtV1, fOuter, pts[2]);
    b.v[2] = v2;

    // Step 4: MEV from v2 → v3.
    HalfEdge* heAtV2 = findHE(fOuter, v2);
    auto [e23, v3] = euler::makeEdgeVertex(solid, heAtV2, fOuter, pts[3]);
    b.v[3] = v3;

    // Step 5: MEF(v3 → v0) — closes the bottom quad.
    // We need HE at v3 (in fOuter) and HE at v0 (in fOuter).
    // Current loop on fOuter after steps 2-4:
    //   heOut01(v0→v1) → heOut12(v1→v2) → heOut23(v2→v3) → heIn23(v3→v2) →
    //   heIn12(v2→v1) → heIn01(v1→v0) → back to heOut01
    //
    // For MEF we need:
    //   he1 = HE at v3 (origin=v3), he2 = HE at v0 (origin=v0).
    // heIn23 has origin=v3, and heOut01 has origin=v0.
    // But we must ensure they are on the same face (fOuter), which they are.
    //
    // The MEF splits the face loop:
    //   Old face keeps: heNew1(v3→v0) → heOut01(v0→v1) → heOut12 → heOut23(v2→v3) → ...
    //   Wait, we need to trace more carefully.
    //
    // MEF(he1, he2) where he1->origin = v3, he2->origin = v0.
    // he1 = heIn23 (origin=v3), he2 = heOut01 (origin=v0).
    //
    // From the MEF code:
    //   heNew1: he1->origin(v3) → he2->origin(v0), stays in old face.
    //   heNew2: he2->origin(v0) → he1->origin(v3), goes to new face.
    //
    // Old face: heNew1(v3→v0) → he2=heOut01(v0→v1) → heOut12(v1→v2) → heOut23(v2→v3)
    //     → heIn23->prev which is... hmm.
    //
    // Actually the splice is:
    //   heNew1->next = he2 (heOut01), heNew1->prev = he1->prev
    //   heNew2->next = he1 (heIn23), heNew2->prev = he2->prev
    //
    // So old face loop: heNew1 → heOut01 → heOut12 → heOut23 → heNew1
    //    (since heOut23->next was heIn23, now heNew1->prev = he1->prev = heOut23)
    //    That means: heOut23 → heNew1 → heOut01 → heOut12 → heOut23... 4 HEs = bottom quad!
    //
    // New face loop: heNew2 → heIn23 → heIn12 → heIn01 → heNew2
    //    (since heIn01->next was heOut01, now heNew2->prev = he2->prev = heIn01)
    //    That means: heIn01 → heNew2 → heIn23 → heIn12 → heIn01... 4 HEs = outer face.

    HalfEdge* heV3_fOuter = findHE(fOuter, v3);  // heIn23
    HalfEdge* heV0_fOuter = findHE(fOuter, v0);  // heOut01
    auto [e30, fBottom] = euler::makeEdgeFace(solid, heV3_fOuter, heV0_fOuter);
    // After MEF: fOuter is the bottom (has heNew1=v3→v0, heOut01=v0→v1, heOut12=v1→v2, heOut23=v2→v3).
    // fBottom (newFace) is actually the "outer" face now.
    // WAIT: looking at the MEF code again:
    //   heNew1->face = oldFace (fOuter) → the "bottom quad" loop
    //   heNew2->face = newFace (fBottom) → the "outer" face
    // So fOuter now IS the bottom quad face (4 HEs), and fBottom is the remaining face.
    // Let me name them properly:
    b.bottom = fOuter;      // renamed: was outer, is now the bottom quad
    Face* fRemaining = fBottom;  // the remaining face (outer envelope)

    // Step 6: MEV from v0 → v4 (vertical edge going up).
    // v0 is on fRemaining now. We need HE at v0 on fRemaining.
    // fRemaining loop: heNew2(v0→v3) → heIn23(v3→v2) → heIn12(v2→v1) → heIn01(v1→v0) → heNew2
    HalfEdge* heV0_fRem = findHE(fRemaining, v0);
    auto [e04, v4] = euler::makeEdgeVertex(solid, heV0_fRem, fRemaining, pts[4]);
    b.v[4] = v4;

    // Step 7: MEV from v1 → v5 (vertical edge).
    // v1 is on fRemaining. After step 6, the loop was spliced to include the v0→v4 spur.
    // Loop: ... → heIn01(v1→v0) → heOut04(v0→v4) → heIn04(v4→v0) → heNew2(v0→v3) → ...
    // Actually we need to find HE at v1 on fRemaining.
    HalfEdge* heV1_fRem = findHE(fRemaining, v1);
    auto [e15, v5] = euler::makeEdgeVertex(solid, heV1_fRem, fRemaining, pts[5]);
    b.v[5] = v5;

    // Step 8: MEF(v4 → v5) — creates front face (v0-v1-v5-v4).
    // We need HE at v4 on fRemaining and HE at v5 on fRemaining.
    // After the MEV spurs:
    //   From v4: heIn04 (v4→v0)
    //   From v5: heIn15 (v5→v1)
    // MEF(heAtV4, heAtV5) will create edge v4-v5 and carve off a new face.
    //
    // The loop around fRemaining now includes the spurs at v4 and v5.
    // Tracing from heIn04(v4→v0):
    //   heIn04(v4→v0) → heNew2(v0→v3) → heIn23(v3→v2) → heIn12(v2→v1)
    //   → heOut15(v1→v5) → heIn15(v5→v1) → heIn01(v1→v0) → heOut04(v0→v4) → heIn04
    // Wait, that's not right. Let me trace step by step.
    //
    // After step 5 (MEF v3→v0), fRemaining loop:
    //   heNew2(v0→v3) → heIn23(v3→v2) → heIn12(v2→v1) → heIn01(v1→v0) → cycle
    //
    // After step 6 (MEV from v0 → v4 on fRemaining, using heV0_fRem = heNew2 which has origin v0):
    //   MEV splices heOut04, heIn04 before heNew2.
    //   heIn01(v1→v0) → heOut04(v0→v4) → heIn04(v4→v0) → heNew2(v0→v3) → heIn23(v3→v2) → heIn12(v2→v1) → cycle
    //
    // After step 7 (MEV from v1 → v5 on fRemaining, using heV1_fRem = heIn12 which has origin v1):
    //   Wait, heIn12 has origin v2, not v1. Let me reconsider.
    //   heIn12 was the HE v2→v1. No — actually in the original construction:
    //     e12 creates heOut(v1→v2) and heIn(v2→v1).
    //   So heIn12 has origin v2. The HE at v1 in fRemaining is the one that comes after heIn01.
    //   Actually heIn01 has origin v1 (goes v1→v0). So heV1_fRem should be heIn01.
    //   But wait, the findHE searches for origin=v1 on fRemaining.
    //   heIn01 has origin v1 → YES. And heIn12 has origin v2 → no.
    //   So heV1_fRem = heIn01 (origin v1).
    //
    //   MEV splices from heIn01: before heIn01.
    //   Result: ... heIn12(v2→v1) → heOut15(v1→v5) → heIn15(v5→v1) → heIn01(v1→v0) → heOut04(v0→v4) → heIn04(v4→v0) → heNew2(v0→v3) → heIn23(v3→v2) → heIn12 → cycle
    //
    // Now MEF(v4, v5): he1 = HE at v4 on fRemaining, he2 = HE at v5 on fRemaining.
    //   HE at v4 = heIn04 (origin v4)
    //   HE at v5 = heIn15 (origin v5)
    //
    // MEF(heIn04, heIn15):
    //   heNew1: v4→v5, stays in old face (fRemaining)
    //   heNew2: v5→v4, goes to new face
    //
    //   Old face: heNew1(v4→v5) → heIn15(v5→v1) → heIn01(v1→v0) → heOut04(v0→v4) → cycle (front quad!)
    //     Wait, check: heNew1->next = he2 = heIn15. heNew1->prev = he1->prev = heOut04->...
    //     Let me check: he1 = heIn04, he1->prev = heOut04? No.
    //     In the loop: ...heOut04(v0→v4) → heIn04(v4→v0)...
    //     So heIn04->prev = heOut04.
    //     heNew1->prev = he1->prev = heOut04. But that gives: heOut04 → heNew1(v4→v5).
    //     But heOut04 has origin v0, goes to v4. So heOut04→heNew1: arrive at v4, depart v4→v5. Hmm but
    //     heOut04->origin = v0, heNew1->origin = v4. The prev chain is about the half-edge BEFORE in the loop.
    //
    //     Actually: he1Prev = he1->prev = heIn04->prev.
    //     In the current loop: ... → heOut04(v0→v4) → heIn04(v4→v0) → heNew2_mef5(v0→v3) → ...
    //     So heIn04->prev = heOut04.
    //
    //     Old face loop: heNew1(v4→v5) → he2(heIn15, v5→v1) → heIn01(v1→v0) → heOut04(v0→v4) → heNew1
    //     That's 4 HEs: v4→v5→v1→v0→v4 = front face! (vertices v0,v1,v5,v4)
    //
    //   New face loop: heNew2(v5→v4) → he1(heIn04, v4→v0) → heNew2_old(v0→v3) → heIn23(v3→v2) → heIn12(v2→v1) → heOut15(v1→v5) → heNew2(v5→v4)
    //     That's 6 HEs: remaining face.
    //
    // Wait actually:
    //   Old face gets: heNew1→he2→...→he1Prev→heNew1
    //   i.e. heNew1(v4→v5) → heIn15(v5→v1) → heIn01(v1→v0) → heOut04(v0→v4) → heNew1
    //   That IS 4 HEs = the front face.

    HalfEdge* heV4_fRem = findHE(fRemaining, v4);
    HalfEdge* heV5_fRem = findHE(fRemaining, v5);
    auto [e45_front, fFront] = euler::makeEdgeFace(solid, heV4_fRem, heV5_fRem);
    // fRemaining keeps the 4-HE loop (front face):
    //   But wait, from MEF: oldFace keeps heNew1 side. The old face was fRemaining.
    //   Actually, based on MEF code: heNew1->face = oldFace, so heNew1(v4→v5) is on fRemaining.
    //   The 4-HE loop containing heNew1 is the front loop.
    //   But the MEF also reassigns all HEs in the new face loop.
    //   So fRemaining = front face, fFront = remaining (6 HEs).
    // Actually: the old face keeps the loop with heNew1, and heNew1's loop is the smaller one (4 HEs).
    // The new face gets the loop with heNew2 (6 HEs remaining).
    // So: fRemaining IS now the front face (4 HEs), fFront is the new remaining face (6 HEs).
    b.front = fRemaining;
    fRemaining = fFront;

    // Step 9: MEV from v2 → v6 (vertical edge).
    HalfEdge* heV2_fRem = findHE(fRemaining, v2);
    auto [e26, v6] = euler::makeEdgeVertex(solid, heV2_fRem, fRemaining, pts[6]);
    b.v[6] = v6;

    // Step 10: MEF(v5 → v6) — creates right face (v1-v2-v6-v5).
    // We need HE at v5 and HE at v6 on fRemaining.
    // After step 9: fRemaining's loop includes the v2→v6 spur.
    //
    // Before step 9, fRemaining (6 HEs):
    //   heNew2_front(v5→v4) → heIn04(v4→v0) → heNew2_bottom(v0→v3) → heIn23(v3→v2) → heIn12(v2→v1) → heOut15(v1→v5) → cycle
    //
    // After step 9 (MEV v2→v6 at heV2_fRem):
    //   heV2_fRem = findHE(fRemaining, v2) = heIn12? No, heIn12 has origin v2... wait.
    //   Let me re-check: heIn12 was created in step 3 as the "in" halfedge of edge e12.
    //   e12 was MEV from v1→v2. So heOut goes v1→v2, heIn goes v2→v1.
    //   heIn12 has origin v2. But is heIn12 on fRemaining?
    //   After the MEF in step 5, HEs were split. The "remaining" face got: heNew2(v0→v3) → heIn23 → heIn12 → heIn01 → cycle.
    //   Then after MEF in step 8, the new remaining face (fFront→fRemaining) got the 6-HE loop.
    //   heIn12 should be in that loop (origin v2, goes v2→v1). And heIn23 has origin v3.
    //
    //   So heV2_fRem could be either heIn12 or the twin of heOut23.
    //   Actually only heIn12 has origin v2 in this face. heIn23 has origin v3.
    //
    //   MEV from v2 (using heIn12) creates: heOut26(v2→v6), heIn26(v6→v2) spliced before heIn12.
    //   Loop becomes:
    //   ... → heIn23(v3→v2) → heOut26(v2→v6) → heIn26(v6→v2) → heIn12(v2→v1) → heOut15(v1→v5) → heNew2_front(v5→v4) → heIn04(v4→v0) → heNew2_bottom(v0→v3) → heIn23 → cycle

    HalfEdge* heV5_fRem2 = findHE(fRemaining, v5);
    HalfEdge* heV6_fRem = findHE(fRemaining, v6);
    auto [e56_right, fRight] = euler::makeEdgeFace(solid, heV5_fRem2, heV6_fRem);
    // MEF(v5, v6): heNew1(v5→v6) stays in old face, heNew2(v6→v5) goes to new face.
    // Old face loop: heNew1(v5→v6) → heIn26(v6→v2) → heIn12(v2→v1) → heOut15(v1→v5) → cycle
    //   = 4 HEs = right face (v5-v6-v2-v1... or equivalently v1-v2-v6-v5).
    // New face: heNew2(v6→v5) → heNew2_front(v5→v4) → heIn04(v4→v0) → heNew2_bottom(v0→v3)
    //   → heIn23(v3→v2) → heOut26(v2→v6) → cycle = 6 HEs remaining.
    b.right = fRemaining;
    fRemaining = fRight;

    // Step 11: MEV from v3 → v7 (vertical edge).
    HalfEdge* heV3_fRem = findHE(fRemaining, v3);
    auto [e37, v7] = euler::makeEdgeVertex(solid, heV3_fRem, fRemaining, pts[7]);
    b.v[7] = v7;

    // Step 12: MEF(v6 → v7) — creates back face (v2-v3-v7-v6).
    HalfEdge* heV6_fRem2 = findHE(fRemaining, v6);
    HalfEdge* heV7_fRem = findHE(fRemaining, v7);
    auto [e67_back, fBack] = euler::makeEdgeFace(solid, heV6_fRem2, heV7_fRem);
    // MEF(v6, v7): heNew1(v6→v7) stays in old face, heNew2(v7→v6) goes to new face.
    // Old face: heNew1(v6→v7) → heIn37(v7→v3) → heIn23(v3→v2) → heOut26(v2→v6) → cycle
    //   = 4 HEs = back face (v6-v7-v3-v2... or v2-v3-v7-v6).
    // New face: heNew2(v7→v6) → heNew2_right(v6→v5) → heNew2_front(v5→v4) → heIn04(v4→v0)
    //   → heNew2_bottom(v0→v3) → heOut37(v3→v7) → cycle = 6 HEs remaining.
    b.back = fRemaining;
    fRemaining = fBack;

    // Step 13: MEF(v7 → v4) — closes left face and creates top face.
    HalfEdge* heV7_fRem2 = findHE(fRemaining, v7);
    HalfEdge* heV4_fRem2 = findHE(fRemaining, v4);
    auto [e74_left, fLeft] = euler::makeEdgeFace(solid, heV7_fRem2, heV4_fRem2);
    // MEF(v7, v4): heNew1(v7→v4) stays in old face, heNew2(v4→v7) goes to new face.
    // Old face: heNew1(v7→v4) → heIn04(v4→v0) → heNew2_bottom(v0→v3) → heOut37(v3→v7) → cycle
    //   = 4 HEs = left face (v7-v4-v0-v3... or v3-v0-v4-v7).
    // New face: heNew2(v4→v7) → heNew2_back(v7→v6) → heNew2_right(v6→v5) → heNew2_front(v5→v4) → cycle
    //   = 4 HEs = top face (v4-v7-v6-v5... or v4-v5-v6-v7).
    b.left = fRemaining;
    b.top = fLeft;

    return b;
}

// ---------------------------------------------------------------------------
// makeBox
// ---------------------------------------------------------------------------

std::unique_ptr<topo::Solid> PrimitiveFactory::makeBox(double width, double height, double depth) {
    auto solid = std::make_unique<topo::Solid>();

    const double w = width;
    const double h = height;
    const double d = depth;

    const Vec3 pts[8] = {
        {0, 0, 0}, {w, 0, 0}, {w, h, 0}, {0, h, 0},  // bottom
        {0, 0, d}, {w, 0, d}, {w, h, d}, {0, h, d},  // top
    };

    BoxBuild bb = buildBoxTopology(*solid, pts);

    // -- Assign TopologyIDs to faces ---
    bb.bottom->topoId = TopologyID::make("box", "bottom");
    bb.top->topoId = TopologyID::make("box", "top");
    bb.front->topoId = TopologyID::make("box", "front");
    bb.back->topoId = TopologyID::make("box", "back");
    bb.right->topoId = TopologyID::make("box", "right");
    bb.left->topoId = TopologyID::make("box", "left");

    // -- Assign TopologyIDs to edges (by index) ---
    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make("box", "edge" + std::to_string(idx));
            ++idx;
        }
    }

    // -- Bind NURBS line curves to all edges ---
    for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
        assignEdgeCurve(&e);
    }

    // -- Bind planar NURBS surfaces to all faces ---
    // Bottom face: z=0 plane.  Vertices: v0,v1,v2,v3.
    bb.bottom->surface = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makePlane(pts[0], Vec3(1, 0, 0), Vec3(0, 1, 0), w, h));

    // Top face: z=d plane.  Vertices: v4,v5,v6,v7.
    bb.top->surface = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makePlane(pts[4], Vec3(1, 0, 0), Vec3(0, 1, 0), w, h));

    // Front face: y=0 plane.  Vertices: v0,v1,v5,v4.
    bb.front->surface = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makePlane(pts[0], Vec3(1, 0, 0), Vec3(0, 0, 1), w, d));

    // Back face: y=h plane.  Vertices: v2,v3,v7,v6.
    bb.back->surface = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makePlane(pts[3], Vec3(1, 0, 0), Vec3(0, 0, 1), w, d));

    // Right face: x=w plane.  Vertices: v1,v2,v6,v5.
    bb.right->surface = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makePlane(pts[1], Vec3(0, 1, 0), Vec3(0, 0, 1), h, d));

    // Left face: x=0 plane.  Vertices: v3,v0,v4,v7.
    bb.left->surface = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makePlane(pts[0], Vec3(0, 1, 0), Vec3(0, 0, 1), h, d));

    return solid;
}

// ---------------------------------------------------------------------------
// makeCylinder
// ---------------------------------------------------------------------------

std::unique_ptr<topo::Solid> PrimitiveFactory::makeCylinder(double radius, double height) {
    auto solid = std::make_unique<topo::Solid>();

    const double r = radius;
    const double h = height;

    // Use box topology with cylinder-appropriate vertex positions.
    // Place 4 points around the circle at z=0 and z=h.
    // Angles: 0, 90, 180, 270 degrees.
    const Vec3 pts[8] = {
        {r, 0, 0},   {0, r, 0},   {-r, 0, 0},  {0, -r, 0},   // bottom circle
        {r, 0, h},   {0, r, h},   {-r, 0, h},   {0, -r, h},   // top circle
    };

    BoxBuild bb = buildBoxTopology(*solid, pts);

    // -- TopologyIDs ---
    bb.bottom->topoId = TopologyID::make("cylinder", "bottom");
    bb.top->topoId = TopologyID::make("cylinder", "top");
    bb.front->topoId = TopologyID::make("cylinder", "side0");
    bb.right->topoId = TopologyID::make("cylinder", "side1");
    bb.back->topoId = TopologyID::make("cylinder", "side2");
    bb.left->topoId = TopologyID::make("cylinder", "side3");

    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make("cylinder", "edge" + std::to_string(idx));
            ++idx;
        }
    }

    // -- Edge curves (linear for vertical edges, arc segments for circle edges) ---
    for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
        assignEdgeCurve(&e);
    }

    // -- Surfaces ---
    // Bottom cap: planar circle at z=0.
    bb.bottom->surface = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makePlane(Vec3(-r, -r, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), 2 * r, 2 * r));

    // Top cap: planar circle at z=h.
    bb.top->surface = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makePlane(Vec3(-r, -r, h), Vec3(1, 0, 0), Vec3(0, 1, 0), 2 * r, 2 * r));

    // Lateral faces: cylindrical surface.
    auto cylSurf = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makeCylinder(Vec3(0, 0, 0), Vec3(0, 0, 1), r, h));
    bb.front->surface = cylSurf;
    bb.right->surface = cylSurf;
    bb.back->surface = cylSurf;
    bb.left->surface = cylSurf;

    return solid;
}

// ---------------------------------------------------------------------------
// makeSphere
// ---------------------------------------------------------------------------

std::unique_ptr<topo::Solid> PrimitiveFactory::makeSphere(double radius) {
    auto solid = std::make_unique<topo::Solid>();

    const double r = radius;

    // Use box-sphere approach: box topology with vertices on the sphere.
    // 8 vertices at the corners of a cube inscribed in the sphere.
    const double c = r / std::sqrt(3.0);  // coordinate magnitude for cube corner on sphere
    const Vec3 pts[8] = {
        {-c, -c, -c}, {c, -c, -c}, {c, c, -c}, {-c, c, -c},  // bottom
        {-c, -c, c},  {c, -c, c},  {c, c, c},  {-c, c, c},   // top
    };

    BoxBuild bb = buildBoxTopology(*solid, pts);

    // -- TopologyIDs ---
    bb.bottom->topoId = TopologyID::make("sphere", "bottom");
    bb.top->topoId = TopologyID::make("sphere", "top");
    bb.front->topoId = TopologyID::make("sphere", "front");
    bb.right->topoId = TopologyID::make("sphere", "right");
    bb.back->topoId = TopologyID::make("sphere", "back");
    bb.left->topoId = TopologyID::make("sphere", "left");

    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make("sphere", "edge" + std::to_string(idx));
            ++idx;
        }
    }

    // -- Edge curves ---
    for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
        assignEdgeCurve(&e);
    }

    // -- Surfaces: spherical NURBS ---
    auto sphereSurf = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makeSphere(Vec3(0, 0, 0), r));
    bb.bottom->surface = sphereSurf;
    bb.top->surface = sphereSurf;
    bb.front->surface = sphereSurf;
    bb.right->surface = sphereSurf;
    bb.back->surface = sphereSurf;
    bb.left->surface = sphereSurf;

    return solid;
}

// ---------------------------------------------------------------------------
// makeCone
// ---------------------------------------------------------------------------

std::unique_ptr<topo::Solid> PrimitiveFactory::makeCone(double bottomRadius, double topRadius,
                                                         double height) {
    auto solid = std::make_unique<topo::Solid>();

    const double rb = bottomRadius;
    const double rt = topRadius;
    const double h = height;

    // 4 points on the bottom circle, 4 on the top circle.
    const Vec3 pts[8] = {
        {rb, 0, 0},  {0, rb, 0},  {-rb, 0, 0}, {0, -rb, 0},  // bottom
        {rt, 0, h},  {0, rt, h},  {-rt, 0, h},  {0, -rt, h},  // top
    };

    BoxBuild bb = buildBoxTopology(*solid, pts);

    // -- TopologyIDs ---
    bb.bottom->topoId = TopologyID::make("cone", "bottom");
    bb.top->topoId = TopologyID::make("cone", "top");
    bb.front->topoId = TopologyID::make("cone", "side0");
    bb.right->topoId = TopologyID::make("cone", "side1");
    bb.back->topoId = TopologyID::make("cone", "side2");
    bb.left->topoId = TopologyID::make("cone", "side3");

    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make("cone", "edge" + std::to_string(idx));
            ++idx;
        }
    }

    // -- Edge curves ---
    for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
        assignEdgeCurve(&e);
    }

    // -- Surfaces ---
    // Caps: planar.
    bb.bottom->surface = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makePlane(Vec3(-rb, -rb, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), 2 * rb,
                                     2 * rb));
    bb.top->surface = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makePlane(Vec3(-rt, -rt, h), Vec3(1, 0, 0), Vec3(0, 1, 0), 2 * rt,
                                     2 * rt));

    // Lateral faces: conical surface.
    // Compute half-angle from the geometry: tan(halfAngle) = bottomRadius / height.
    // The cone factory takes apex, axis, halfAngle, height (from apex).
    // For a frustum this is approximate — we use the bottom radius cone.
    if (rb > 1e-12) {
        double halfAngle = std::atan2(rb, h);
        auto coneSurf = std::make_shared<geo::NurbsSurface>(
            geo::NurbsSurface::makeCone(Vec3(0, 0, h), Vec3(0, 0, -1), halfAngle, h));
        bb.front->surface = coneSurf;
        bb.right->surface = coneSurf;
        bb.back->surface = coneSurf;
        bb.left->surface = coneSurf;
    } else {
        // Degenerate: inverted cone with apex at bottom.
        double halfAngle = std::atan2(rt, h);
        auto coneSurf = std::make_shared<geo::NurbsSurface>(
            geo::NurbsSurface::makeCone(Vec3(0, 0, 0), Vec3(0, 0, 1), halfAngle, h));
        bb.front->surface = coneSurf;
        bb.right->surface = coneSurf;
        bb.back->surface = coneSurf;
        bb.left->surface = coneSurf;
    }

    return solid;
}

// ---------------------------------------------------------------------------
// makeTorus
// ---------------------------------------------------------------------------

std::unique_ptr<topo::Solid> PrimitiveFactory::makeTorus(double majorRadius, double minorRadius) {
    auto solid = std::make_unique<topo::Solid>();

    const double R = majorRadius;
    const double r = minorRadius;

    // 4 points on the outer ring and 4 on the inner ring.
    // Outer ring: at distance R+r from center.
    // Inner ring: at distance R-r from center.
    const double outer = R + r;
    const double inner = R - r;

    const Vec3 pts[8] = {
        {outer, 0, 0},  {0, outer, 0},  {-outer, 0, 0}, {0, -outer, 0},  // outer ring (z=0)
        {inner, 0, 0},  {0, inner, 0},  {-inner, 0, 0},  {0, -inner, 0},  // inner ring (z=0)
    };

    BoxBuild bb = buildBoxTopology(*solid, pts);

    // -- TopologyIDs ---
    bb.bottom->topoId = TopologyID::make("torus", "outer");
    bb.top->topoId = TopologyID::make("torus", "inner");
    bb.front->topoId = TopologyID::make("torus", "side0");
    bb.right->topoId = TopologyID::make("torus", "side1");
    bb.back->topoId = TopologyID::make("torus", "side2");
    bb.left->topoId = TopologyID::make("torus", "side3");

    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make("torus", "edge" + std::to_string(idx));
            ++idx;
        }
    }

    // -- Edge curves ---
    for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
        assignEdgeCurve(&e);
    }

    // -- Surfaces: toroidal NURBS ---
    auto torusSurf = std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makeTorus(Vec3(0, 0, 0), Vec3(0, 0, 1), R, r));
    bb.bottom->surface = torusSurf;
    bb.top->surface = torusSurf;
    bb.front->surface = torusSurf;
    bb.right->surface = torusSurf;
    bb.back->surface = torusSurf;
    bb.left->surface = torusSurf;

    return solid;
}

}  // namespace hz::model
