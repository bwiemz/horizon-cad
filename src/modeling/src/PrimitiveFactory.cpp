#include "horizon/modeling/PrimitiveFactory.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/SolidSewer.h"
#include "horizon/topology/EulerOps.h"
#include "horizon/topology/Queries.h"

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
    return std::make_shared<geo::NurbsCurve>(std::vector<Vec3>{a, b}, std::vector<double>{1.0, 1.0},
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
    // After MEF: fOuter is the bottom (has heNew1=v3→v0, heOut01=v0→v1, heOut12=v1→v2,
    // heOut23=v2→v3). fBottom (newFace) is actually the "outer" face now. WAIT: looking at the MEF
    // code again:
    //   heNew1->face = oldFace (fOuter) → the "bottom quad" loop
    //   heNew2->face = newFace (fBottom) → the "outer" face
    // So fOuter now IS the bottom quad face (4 HEs), and fBottom is the remaining face.
    // Let me name them properly:
    b.bottom = fOuter;           // renamed: was outer, is now the bottom quad
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
    //   heIn01(v1→v0) → heOut04(v0→v4) → heIn04(v4→v0) → heNew2(v0→v3) → heIn23(v3→v2) →
    //   heIn12(v2→v1) → cycle
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
    //   Result: ... heIn12(v2→v1) → heOut15(v1→v5) → heIn15(v5→v1) → heIn01(v1→v0) → heOut04(v0→v4)
    //   → heIn04(v4→v0) → heNew2(v0→v3) → heIn23(v3→v2) → heIn12 → cycle
    //
    // Now MEF(v4, v5): he1 = HE at v4 on fRemaining, he2 = HE at v5 on fRemaining.
    //   HE at v4 = heIn04 (origin v4)
    //   HE at v5 = heIn15 (origin v5)
    //
    // MEF(heIn04, heIn15):
    //   heNew1: v4→v5, stays in old face (fRemaining)
    //   heNew2: v5→v4, goes to new face
    //
    //   Old face: heNew1(v4→v5) → heIn15(v5→v1) → heIn01(v1→v0) → heOut04(v0→v4) → cycle (front
    //   quad!)
    //     Wait, check: heNew1->next = he2 = heIn15. heNew1->prev = he1->prev = heOut04->...
    //     Let me check: he1 = heIn04, he1->prev = heOut04? No.
    //     In the loop: ...heOut04(v0→v4) → heIn04(v4→v0)...
    //     So heIn04->prev = heOut04.
    //     heNew1->prev = he1->prev = heOut04. But that gives: heOut04 → heNew1(v4→v5).
    //     But heOut04 has origin v0, goes to v4. So heOut04→heNew1: arrive at v4, depart v4→v5. Hmm
    //     but heOut04->origin = v0, heNew1->origin = v4. The prev chain is about the half-edge
    //     BEFORE in the loop.
    //
    //     Actually: he1Prev = he1->prev = heIn04->prev.
    //     In the current loop: ... → heOut04(v0→v4) → heIn04(v4→v0) → heNew2_mef5(v0→v3) → ...
    //     So heIn04->prev = heOut04.
    //
    //     Old face loop: heNew1(v4→v5) → he2(heIn15, v5→v1) → heIn01(v1→v0) → heOut04(v0→v4) →
    //     heNew1 That's 4 HEs: v4→v5→v1→v0→v4 = front face! (vertices v0,v1,v5,v4)
    //
    //   New face loop: heNew2(v5→v4) → he1(heIn04, v4→v0) → heNew2_old(v0→v3) → heIn23(v3→v2) →
    //   heIn12(v2→v1) → heOut15(v1→v5) → heNew2(v5→v4)
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
    // Actually: the old face keeps the loop with heNew1, and heNew1's loop is the smaller one (4
    // HEs). The new face gets the loop with heNew2 (6 HEs remaining). So: fRemaining IS now the
    // front face (4 HEs), fFront is the new remaining face (6 HEs).
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
    //   heNew2_front(v5→v4) → heIn04(v4→v0) → heNew2_bottom(v0→v3) → heIn23(v3→v2) → heIn12(v2→v1)
    //   → heOut15(v1→v5) → cycle
    //
    // After step 9 (MEV v2→v6 at heV2_fRem):
    //   heV2_fRem = findHE(fRemaining, v2) = heIn12? No, heIn12 has origin v2... wait.
    //   Let me re-check: heIn12 was created in step 3 as the "in" halfedge of edge e12.
    //   e12 was MEV from v1→v2. So heOut goes v1→v2, heIn goes v2→v1.
    //   heIn12 has origin v2. But is heIn12 on fRemaining?
    //   After the MEF in step 5, HEs were split. The "remaining" face got: heNew2(v0→v3) → heIn23 →
    //   heIn12 → heIn01 → cycle. Then after MEF in step 8, the new remaining face
    //   (fFront→fRemaining) got the 6-HE loop. heIn12 should be in that loop (origin v2, goes
    //   v2→v1). And heIn23 has origin v3.
    //
    //   So heV2_fRem could be either heIn12 or the twin of heOut23.
    //   Actually only heIn12 has origin v2 in this face. heIn23 has origin v3.
    //
    //   MEV from v2 (using heIn12) creates: heOut26(v2→v6), heIn26(v6→v2) spliced before heIn12.
    //   Loop becomes:
    //   ... → heIn23(v3→v2) → heOut26(v2→v6) → heIn26(v6→v2) → heIn12(v2→v1) → heOut15(v1→v5) →
    //   heNew2_front(v5→v4) → heIn04(v4→v0) → heNew2_bottom(v0→v3) → heIn23 → cycle

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
    // New face: heNew2(v4→v7) → heNew2_back(v7→v6) → heNew2_right(v6→v5) → heNew2_front(v5→v4) →
    // cycle
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
// Faceted curved primitives
//
// Every downstream path in the kernel — Boolean classification, interference,
// mass properties, drawing projection, export — evaluates a solid from its
// face loops.  A curved NURBS surface bound to a coarse loop is therefore
// decoration: it makes the display disagree with every computation.  The old
// builders leaned on that, wrapping box topology (8V/12E/6F) in cylindrical or
// spherical patches, so a cylinder was a square prism to everything but the
// renderer (volume 500 instead of 785) and a sphere was a cube (192 instead of
// 524).  It also made the renderer emit each shared surface once per face —
// a sphere drew six full spheres, 480k triangles for a shape that needs 2k.
//
// Curved primitives are now faceted at construction and their facets carry
// planar patches that match them.  The B-Rep is exactly what the rest of the
// kernel treats it as, and volumes converge to the analytic value from below.
// ---------------------------------------------------------------------------

namespace {

/// Points of a circle of @p radius at height @p z, counter-clockwise seen
/// from +Z, starting on the +X axis.
std::vector<Vec3> ringPoints(double radius, double z, int segments) {
    std::vector<Vec3> ring;
    ring.reserve(static_cast<size_t>(segments));
    for (int i = 0; i < segments; ++i) {
        const double a = 2.0 * math::kPi * static_cast<double>(i) / static_cast<double>(segments);
        ring.push_back(Vec3(radius * std::cos(a), radius * std::sin(a), z));
    }
    return ring;
}

/// Signed volume of a closed polygon soup (divergence theorem, fan per face),
/// about a point of its own, so its rounding does not grow with the soup's
/// distance from the origin.
double soupSignedVolume(const std::vector<SolidSewer::InputFace>& faces) {
    Vec3 o;
    for (const auto& f : faces) {
        if (!f.points.empty()) {
            o = f.points[0];
            break;
        }
    }
    double vol6 = 0.0;
    for (const auto& f : faces) {
        for (size_t i = 1; i + 1 < f.points.size(); ++i) {
            vol6 += (f.points[0] - o).dot((f.points[i] - o).cross(f.points[i + 1] - o));
        }
    }
    return vol6 / 6.0;
}

/// Normalize a closed soup to outward winding, which is what SolidSewer
/// documents as its input contract.  Building each face's loop by hand and
/// reasoning about its handedness is where winding bugs come from; deriving it
/// once from the enclosed volume cannot get it wrong.
void orientOutward(std::vector<SolidSewer::InputFace>& faces) {
    if (soupSignedVolume(faces) >= 0.0) {
        return;
    }
    for (auto& f : faces) {
        std::reverse(f.points.begin(), f.points.end());
    }
}

/// A quad, dropping to a triangle where one edge is degenerate (the ring
/// beside a cone apex or a sphere pole).
void addQuad(std::vector<SolidSewer::InputFace>& faces, const Vec3& a, const Vec3& b, const Vec3& c,
             const Vec3& d, const TopologyID& id) {
    std::vector<Vec3> loop;
    for (const Vec3& p : {a, b, c, d}) {
        if (loop.empty() || p.distanceTo(loop.back()) > 1e-12) {
            loop.push_back(p);
        }
    }
    while (loop.size() > 1 && loop.front().distanceTo(loop.back()) <= 1e-12) {
        loop.pop_back();
    }
    if (loop.size() < 3) {
        return;
    }
    SolidSewer::InputFace f;
    f.points = std::move(loop);
    f.topoId = id;
    faces.push_back(std::move(f));
}

/// Record on each face the ideal surface its facets approximate, and on each
/// rim edge the arc its chord replaces.  These are not carriers (see
/// topo::Face::analyticSurface) — they are what feature recognition needs to
/// answer "this facet belongs to a cylinder of radius r" from a single pick,
/// which faceting would otherwise throw away.
void tagAnalyticSurface(topo::Solid& solid, const std::string& facePrefix,
                        const std::shared_ptr<geo::NurbsSurface>& surface) {
    if (surface == nullptr) {
        return;
    }
    for (auto& f : const_cast<std::deque<Face>&>(solid.faces())) {
        if (f.topoId.tag().find(facePrefix) != std::string::npos) {
            f.analyticSurface = surface;
        }
    }
}

/// Tag every edge whose two endpoints lie on the circle of @p radius about
/// @p axisPoint in the plane z = @p z with the arc it chords.
void tagRimArcs(topo::Solid& solid, double radius, double z) {
    for (auto& e : const_cast<std::deque<Edge>&>(solid.edges())) {
        if (e.halfEdge == nullptr || e.halfEdge->twin == nullptr) continue;
        const Vec3 a = e.halfEdge->origin->point;
        const Vec3 b = e.halfEdge->twin->origin->point;
        if (std::abs(a.z - z) > 1e-9 || std::abs(b.z - z) > 1e-9) continue;
        if (std::abs(std::hypot(a.x, a.y) - radius) > 1e-9) continue;
        if (std::abs(std::hypot(b.x, b.y) - radius) > 1e-9) continue;

        double a0 = std::atan2(a.y, a.x);
        double a1 = std::atan2(b.y, b.x);
        double sweep = a1 - a0;
        while (sweep <= 0.0) sweep += 2.0 * math::kPi;
        if (sweep > math::kPi) std::swap(a0, a1);
        e.analyticCurve = std::make_shared<geo::NurbsCurve>(
            geo::NurbsCurve::makeArc(Vec3(0, 0, z), radius, a0, a1, Vec3(0, 0, 1)));
    }
}

}  // namespace

int PrimitiveFactory::segmentsForTolerance(double radius, double tolerance) {
    if (!(radius > 0.0) || !(tolerance > 0.0)) {
        return kDefaultSegments;
    }
    const double ratio = 1.0 - tolerance / radius;
    if (ratio <= -1.0) {
        return 3;  // Tolerance exceeds the diameter: any polygon will do.
    }
    const double n = math::kPi / std::acos(std::min(1.0, ratio));
    return std::clamp(static_cast<int>(std::ceil(n)), 3, 4096);
}

// ---------------------------------------------------------------------------
// makeCylinder
// ---------------------------------------------------------------------------

std::unique_ptr<topo::Solid> PrimitiveFactory::makeCylinder(double radius, double height,
                                                            int segments) {
    if (!(radius > 0.0) || !(height > 0.0) || segments < 3) {
        return nullptr;
    }

    const auto bottom = ringPoints(radius, 0.0, segments);
    const auto top = ringPoints(radius, height, segments);

    std::vector<SolidSewer::InputFace> faces;
    faces.reserve(static_cast<size_t>(segments) + 2);

    // The lateral quads traverse the bottom ring counter-clockwise (seen from
    // +Z), so the bottom cap must traverse it the other way for the shared
    // edges to pair into twins.  The top ring is already opposed.
    SolidSewer::InputFace cap;
    cap.points.assign(bottom.rbegin(), bottom.rend());
    cap.topoId = TopologyID::make("cylinder", "bottom");
    faces.push_back(std::move(cap));

    SolidSewer::InputFace lid;
    lid.points = top;
    lid.topoId = TopologyID::make("cylinder", "top");
    faces.push_back(std::move(lid));

    for (int i = 0; i < segments; ++i) {
        const int j = (i + 1) % segments;
        addQuad(faces, bottom[static_cast<size_t>(i)], bottom[static_cast<size_t>(j)],
                top[static_cast<size_t>(j)], top[static_cast<size_t>(i)],
                TopologyID::make("cylinder", "side" + std::to_string(i)));
    }

    orientOutward(faces);
    auto solid = SolidSewer::sew(faces);
    if (solid != nullptr) {
        tagAnalyticSurface(*solid, "cylinder/side",
                           std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makeCylinder(
                               Vec3(0, 0, 0), Vec3(0, 0, 1), radius, height)));
        tagRimArcs(*solid, radius, 0.0);
        tagRimArcs(*solid, radius, height);
    }
    return solid;
}

// ---------------------------------------------------------------------------
// makeSphere
// ---------------------------------------------------------------------------

std::unique_ptr<topo::Solid> PrimitiveFactory::makeSphere(double radius, int segments, int stacks) {
    if (stacks <= 0) {
        stacks = std::max(2, segments / 2);
    }
    if (!(radius > 0.0) || segments < 3 || stacks < 2) {
        return nullptr;
    }

    // Rings of constant polar angle; the poles are single vertices, so the
    // bands beside them collapse to triangle fans (addQuad drops the
    // degenerate edge).
    std::vector<std::vector<Vec3>> rings;
    rings.reserve(static_cast<size_t>(stacks) + 1);
    for (int j = 0; j <= stacks; ++j) {
        const double phi = math::kPi * static_cast<double>(j) / static_cast<double>(stacks);
        const double z = radius * std::cos(phi);
        const double r = radius * std::sin(phi);
        if (j == 0 || j == stacks) {
            rings.push_back(std::vector<Vec3>(static_cast<size_t>(segments), Vec3(0, 0, z)));
        } else {
            rings.push_back(ringPoints(r, z, segments));
        }
    }

    std::vector<SolidSewer::InputFace> faces;
    faces.reserve(static_cast<size_t>(segments) * static_cast<size_t>(stacks));
    for (int j = 0; j < stacks; ++j) {
        const auto& lower = rings[static_cast<size_t>(j) + 1];
        const auto& upper = rings[static_cast<size_t>(j)];
        for (int i = 0; i < segments; ++i) {
            const int k = (i + 1) % segments;
            addQuad(
                faces, lower[static_cast<size_t>(i)], lower[static_cast<size_t>(k)],
                upper[static_cast<size_t>(k)], upper[static_cast<size_t>(i)],
                TopologyID::make("sphere", "band" + std::to_string(j) + "_" + std::to_string(i)));
        }
    }

    orientOutward(faces);
    auto solid = SolidSewer::sew(faces);
    if (solid != nullptr) {
        tagAnalyticSurface(*solid, "sphere/band",
                           std::make_shared<geo::NurbsSurface>(
                               geo::NurbsSurface::makeSphere(Vec3(0, 0, 0), radius)));
    }
    return solid;
}

// ---------------------------------------------------------------------------
// makeCone
// ---------------------------------------------------------------------------

std::unique_ptr<topo::Solid> PrimitiveFactory::makeCone(double bottomRadius, double topRadius,
                                                        double height, int segments) {
    constexpr double kRingEps = 1e-12;
    if (bottomRadius <= kRingEps && topRadius <= kRingEps) {
        return nullptr;  // Both ends degenerate: no solid to build.
    }
    if (height <= kRingEps || segments < 3) {
        return nullptr;
    }

    // A zero radius collapses its ring to a point, so that end is an apex
    // rather than a cap: n triangles instead of a cap plus n quads.
    const bool sharpTop = topRadius <= kRingEps;
    const bool sharpBottom = bottomRadius <= kRingEps;

    const auto bottom = sharpBottom
                            ? std::vector<Vec3>(static_cast<size_t>(segments), Vec3(0, 0, 0))
                            : ringPoints(bottomRadius, 0.0, segments);
    const auto top = sharpTop ? std::vector<Vec3>(static_cast<size_t>(segments), Vec3(0, 0, height))
                              : ringPoints(topRadius, height, segments);

    std::vector<SolidSewer::InputFace> faces;
    faces.reserve(static_cast<size_t>(segments) + 2);

    if (!sharpBottom) {
        // Opposed to the lateral quads' traversal of the same ring; see
        // makeCylinder.
        SolidSewer::InputFace cap;
        cap.points.assign(bottom.rbegin(), bottom.rend());
        cap.topoId = TopologyID::make("cone", "bottom");
        faces.push_back(std::move(cap));
    }
    if (!sharpTop) {
        SolidSewer::InputFace lid;
        lid.points = top;
        lid.topoId = TopologyID::make("cone", "top");
        faces.push_back(std::move(lid));
    }

    for (int i = 0; i < segments; ++i) {
        const int j = (i + 1) % segments;
        addQuad(faces, bottom[static_cast<size_t>(i)], bottom[static_cast<size_t>(j)],
                top[static_cast<size_t>(j)], top[static_cast<size_t>(i)],
                TopologyID::make("cone", "side" + std::to_string(i)));
    }

    orientOutward(faces);
    auto solid = SolidSewer::sew(faces);
    if (solid != nullptr) {
        // The carrier cone runs from its apex: where the side meets the axis,
        // at an end that comes to a point, or beyond the narrower end of a
        // frustum. It used to start at the bottom whatever the radii, so a
        // frustum's ideal was another cone (Phase 160 found it, fitting one).
        // Radii alike: a cylinder.
        if (std::abs(bottomRadius - topRadius) <= kRingEps) {
            tagAnalyticSurface(*solid, "cone/side",
                               std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makeCylinder(
                                   Vec3(0, 0, 0), Vec3(0, 0, 1), bottomRadius, height)));
        } else {
            const double apexZ = height * bottomRadius / (bottomRadius - topRadius);
            const bool narrowing = bottomRadius > topRadius;  // toward the top
            const Vec3 apexDir = narrowing ? Vec3(0, 0, -1) : Vec3(0, 0, 1);
            const double reach = narrowing ? apexZ : height - apexZ;  // apex to the far end
            const double halfAngle = std::atan2(std::abs(bottomRadius - topRadius), height);
            tagAnalyticSurface(*solid, "cone/side",
                               std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makeCone(
                                   Vec3(0, 0, apexZ), apexDir, halfAngle, reach)));
        }
        if (!sharpBottom) tagRimArcs(*solid, bottomRadius, 0.0);
        if (!sharpTop) tagRimArcs(*solid, topRadius, height);
    }
    return solid;
}

// ---------------------------------------------------------------------------
// makeTorus
// ---------------------------------------------------------------------------

std::unique_ptr<topo::Solid> PrimitiveFactory::makeTorus(double majorRadius, double minorRadius,
                                                         int segments, int tubeSegments) {
    if (tubeSegments <= 0) {
        tubeSegments = std::max(3, segments / 2);
    }
    if (!(minorRadius > 0.0) || !(majorRadius > minorRadius) || segments < 3 || tubeSegments < 3) {
        return nullptr;
    }

    // Grid of tube cross-sections around the major circle.  Unlike the other
    // primitives this shell has genus 1, so V - E + F is 0 rather than 2.
    std::vector<std::vector<Vec3>> rings;
    rings.reserve(static_cast<size_t>(segments));
    for (int i = 0; i < segments; ++i) {
        const double theta =
            2.0 * math::kPi * static_cast<double>(i) / static_cast<double>(segments);
        const Vec3 outward(std::cos(theta), std::sin(theta), 0.0);
        std::vector<Vec3> ring;
        ring.reserve(static_cast<size_t>(tubeSegments));
        for (int j = 0; j < tubeSegments; ++j) {
            const double psi =
                2.0 * math::kPi * static_cast<double>(j) / static_cast<double>(tubeSegments);
            ring.push_back(outward * (majorRadius + minorRadius * std::cos(psi)) +
                           Vec3(0, 0, minorRadius * std::sin(psi)));
        }
        rings.push_back(std::move(ring));
    }

    std::vector<SolidSewer::InputFace> faces;
    faces.reserve(static_cast<size_t>(segments) * static_cast<size_t>(tubeSegments));
    for (int i = 0; i < segments; ++i) {
        const auto& a = rings[static_cast<size_t>(i)];
        const auto& b = rings[static_cast<size_t>((i + 1) % segments)];
        for (int j = 0; j < tubeSegments; ++j) {
            const int k = (j + 1) % tubeSegments;
            addQuad(
                faces, a[static_cast<size_t>(j)], b[static_cast<size_t>(j)],
                b[static_cast<size_t>(k)], a[static_cast<size_t>(k)],
                TopologyID::make("torus", "patch" + std::to_string(i) + "_" + std::to_string(j)));
        }
    }

    orientOutward(faces);
    auto solid = SolidSewer::sew(faces);
    if (solid != nullptr) {
        tagAnalyticSurface(*solid, "torus/patch",
                           std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makeTorus(
                               Vec3(0, 0, 0), Vec3(0, 0, 1), majorRadius, minorRadius)));
    }
    return solid;
}

}  // namespace hz::model
