#include "horizon/modeling/Extrude.h"

#include <cassert>
#include <cmath>

#include "RingStack.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/topology/EulerOps.h"

namespace hz::model {

using namespace hz::topo;
using hz::math::Vec2;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// Helpers (same pattern as PrimitiveFactory.cpp)
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
            if (prevOrigin == nullptr) return cur;
            if (cur->prev->origin == prevOrigin) return cur;
            fallback = cur;
        }
        cur = cur->next;
    } while (cur != start);
    return fallback;
}

static std::shared_ptr<geo::NurbsCurve> makeLineCurve(const Vec3& a, const Vec3& b) {
    return std::make_shared<geo::NurbsCurve>(std::vector<Vec3>{a, b}, std::vector<double>{1.0, 1.0},
                                             std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
}

static void assignEdgeCurve(Edge* edge) {
    assert(edge->halfEdge != nullptr);
    HalfEdge* he = edge->halfEdge;
    edge->curve = makeLineCurve(he->origin->point, he->twin->origin->point);
}

// ---------------------------------------------------------------------------
// Box-topology builder (reused for 4-vertex rectangular extrude)
// Identical algorithm to PrimitiveFactory::buildBoxTopology.
// ---------------------------------------------------------------------------

struct BoxBuild {
    Vertex* v[8] = {};
    Face* bottom = nullptr;
    Face* top = nullptr;
    Face* front = nullptr;
    Face* right = nullptr;
    Face* back = nullptr;
    Face* left = nullptr;
};

static BoxBuild buildBoxTopology(Solid& solid, const Vec3 pts[8]) {
    BoxBuild b;

    auto [v0, fOuter, shell] = euler::makeVertexFaceSolid(solid, pts[0]);
    b.v[0] = v0;

    auto [e01, v1] = euler::makeEdgeVertex(solid, nullptr, fOuter, pts[1]);
    b.v[1] = v1;

    HalfEdge* heAtV1 = findHE(fOuter, v1);
    auto [e12, v2] = euler::makeEdgeVertex(solid, heAtV1, fOuter, pts[2]);
    b.v[2] = v2;

    HalfEdge* heAtV2 = findHE(fOuter, v2);
    auto [e23, v3] = euler::makeEdgeVertex(solid, heAtV2, fOuter, pts[3]);
    b.v[3] = v3;

    HalfEdge* heV3_fOuter = findHE(fOuter, v3);
    HalfEdge* heV0_fOuter = findHE(fOuter, v0);
    auto [e30, fBottom] = euler::makeEdgeFace(solid, heV3_fOuter, heV0_fOuter);
    b.bottom = fOuter;
    Face* fRemaining = fBottom;

    HalfEdge* heV0_fRem = findHE(fRemaining, v0);
    auto [e04, v4] = euler::makeEdgeVertex(solid, heV0_fRem, fRemaining, pts[4]);
    b.v[4] = v4;

    HalfEdge* heV1_fRem = findHE(fRemaining, v1);
    auto [e15, v5] = euler::makeEdgeVertex(solid, heV1_fRem, fRemaining, pts[5]);
    b.v[5] = v5;

    HalfEdge* heV4_fRem = findHE(fRemaining, v4);
    HalfEdge* heV5_fRem = findHE(fRemaining, v5);
    auto [e45_front, fFront] = euler::makeEdgeFace(solid, heV4_fRem, heV5_fRem);
    b.front = fRemaining;
    fRemaining = fFront;

    HalfEdge* heV2_fRem = findHE(fRemaining, v2);
    auto [e26, v6] = euler::makeEdgeVertex(solid, heV2_fRem, fRemaining, pts[6]);
    b.v[6] = v6;

    HalfEdge* heV5_fRem2 = findHE(fRemaining, v5);
    HalfEdge* heV6_fRem = findHE(fRemaining, v6);
    auto [e56_right, fRight] = euler::makeEdgeFace(solid, heV5_fRem2, heV6_fRem);
    b.right = fRemaining;
    fRemaining = fRight;

    HalfEdge* heV3_fRem = findHE(fRemaining, v3);
    auto [e37, v7] = euler::makeEdgeVertex(solid, heV3_fRem, fRemaining, pts[7]);
    b.v[7] = v7;

    HalfEdge* heV6_fRem2 = findHE(fRemaining, v6);
    HalfEdge* heV7_fRem = findHE(fRemaining, v7);
    auto [e67_back, fBack] = euler::makeEdgeFace(solid, heV6_fRem2, heV7_fRem);
    b.back = fRemaining;
    fRemaining = fBack;

    HalfEdge* heV7_fRem2 = findHE(fRemaining, v7);
    HalfEdge* heV4_fRem2 = findHE(fRemaining, v4);
    auto [e74_left, fLeft] = euler::makeEdgeFace(solid, heV7_fRem2, heV4_fRem2);
    b.left = fRemaining;
    b.top = fLeft;

    return b;
}

// ---------------------------------------------------------------------------
// General N-gon prism builder using Euler operators.
//
// bottomPts[0..N-1] = bottom polygon vertices
// topPts[0..N-1]    = top polygon vertices (bottomPts + direction*distance)
//
// Result: 2N vertices, 3N edges, N+2 faces.
// Euler: 2N - 3N + (N+2) = 2. ✓
// ---------------------------------------------------------------------------

struct PrismBuild {
    std::vector<Vertex*> bottomVerts;
    std::vector<Vertex*> topVerts;
    Face* bottomFace = nullptr;
    Face* topFace = nullptr;
    std::vector<Face*> lateralFaces;
};

static PrismBuild buildPrismTopology(Solid& solid, const std::vector<Vec3>& bottomPts,
                                     const std::vector<Vec3>& topPts) {
    const size_t N = bottomPts.size();
    assert(N >= 3);
    assert(topPts.size() == N);

    PrismBuild pb;
    pb.bottomVerts.resize(N);
    pb.topVerts.resize(N);
    pb.lateralFaces.resize(N);

    // Step 1: MVFS — first bottom vertex.
    auto [v0, fOuter, shell] = euler::makeVertexFaceSolid(solid, bottomPts[0]);
    pb.bottomVerts[0] = v0;

    // Step 2: MEV to create bottom polygon vertices v1..v(N-1).
    for (size_t i = 1; i < N; ++i) {
        HalfEdge* hePrev = findHE(fOuter, pb.bottomVerts[i - 1]);
        auto [edge, vi] =
            euler::makeEdgeVertex(solid, (i == 1) ? nullptr : hePrev, fOuter, bottomPts[i]);
        pb.bottomVerts[i] = vi;
    }

    // Step 3: MEF to close the bottom polygon (vN-1 → v0).
    HalfEdge* heLast = findHE(fOuter, pb.bottomVerts[N - 1]);
    HalfEdge* heFirst = findHE(fOuter, pb.bottomVerts[0]);
    auto [eClose, fBottom] = euler::makeEdgeFace(solid, heLast, heFirst);
    // fOuter is now the bottom face (N HEs), fBottom is the remaining face.
    pb.bottomFace = fOuter;
    Face* fRemaining = fBottom;

    // Step 4: MEV to create vertical edges from each bottom vertex to its top counterpart.
    // We create them in order 0, 1, 2, ..., N-1.
    for (size_t i = 0; i < N; ++i) {
        HalfEdge* heAtBotI = findHE(fRemaining, pb.bottomVerts[i]);
        auto [eVert, topV] = euler::makeEdgeVertex(solid, heAtBotI, fRemaining, topPts[i]);
        pb.topVerts[i] = topV;
    }

    // Step 5: MEF to create lateral faces and the top face.
    // For each lateral face i, connect topVerts[i] to topVerts[(i+1)%N].
    // The last MEF also closes the top face.
    for (size_t i = 0; i < N; ++i) {
        size_t next = (i + 1) % N;
        HalfEdge* heTopI = findHE(fRemaining, pb.topVerts[i]);
        HalfEdge* heTopNext = findHE(fRemaining, pb.topVerts[next]);
        auto [eLat, fLat] = euler::makeEdgeFace(solid, heTopI, heTopNext);
        // Old face (fRemaining) becomes the lateral face for this quad.
        pb.lateralFaces[i] = fRemaining;
        fRemaining = fLat;
    }
    // After the last MEF, fRemaining is the top face.
    pb.topFace = fRemaining;

    return pb;
}

// ---------------------------------------------------------------------------
// Record on the facets of each profile arc the ideal geometry they
// approximate: the cylinder on the lateral faces (only for an extrusion along
// the sketch normal — an oblique one sweeps an elliptic cylinder), and the
// arc's circle on each chord of both caps.
// ---------------------------------------------------------------------------

static void tagArcIdeals(const ringstack::SampledProfile& sampled, const draft::SketchPlane& plane,
                         const Vec3& offset, const std::vector<Face*>& laterals,
                         const std::vector<Vertex*>& bottomVerts,
                         const std::vector<Vertex*>& topVerts) {
    if (sampled.arcs.empty()) return;
    const double height = offset.length();
    const Vec3 axis = offset.normalized();
    const bool alongNormal = axis.cross(plane.normal()).length() < 1e-9;

    std::vector<std::shared_ptr<geo::NurbsSurface>> cylinders(sampled.arcs.size());
    std::vector<std::shared_ptr<geo::NurbsCurve>> bottomCircles(sampled.arcs.size());
    std::vector<std::shared_ptr<geo::NurbsCurve>> topCircles(sampled.arcs.size());
    for (size_t a = 0; a < sampled.arcs.size(); ++a) {
        const Vec3 center = plane.localToWorld(sampled.arcs[a].center);
        const double r = sampled.arcs[a].radius;
        if (alongNormal) {
            cylinders[a] = std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makeCylinder(center, axis, r, height));
        }
        bottomCircles[a] = std::make_shared<geo::NurbsCurve>(
            geo::NurbsCurve::makeCircle(center, r, plane.normal()));
        topCircles[a] = std::make_shared<geo::NurbsCurve>(
            geo::NurbsCurve::makeCircle(center + offset, r, plane.normal()));
    }

    const size_t N = laterals.size();
    for (size_t i = 0; i < N; ++i) {
        const int a = sampled.edgeArc[i];
        if (a < 0) continue;
        const size_t j = (i + 1) % N;
        Face* face = laterals[i];
        face->analyticSurface = cylinders[static_cast<size_t>(a)];

        HalfEdge* start = face->outerLoop->halfEdge;
        HalfEdge* he = start;
        do {
            const Vertex* p = he->origin;
            const Vertex* q = he->twin->origin;
            const bool bottom = (p == bottomVerts[i] && q == bottomVerts[j]) ||
                                (p == bottomVerts[j] && q == bottomVerts[i]);
            const bool top =
                (p == topVerts[i] && q == topVerts[j]) || (p == topVerts[j] && q == topVerts[i]);
            if (bottom) he->edge->analyticCurve = bottomCircles[static_cast<size_t>(a)];
            if (top) he->edge->analyticCurve = topCircles[static_cast<size_t>(a)];
            he = he->next;
        } while (he != start);
    }
}

// ---------------------------------------------------------------------------
// Extrude::execute
// ---------------------------------------------------------------------------

std::unique_ptr<topo::Solid> Extrude::execute(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
    const draft::SketchPlane& plane, const Vec3& direction, double distance,
    const std::string& featureID, int segments, double chordTolerance) {
    // -----------------------------------------------------------------------
    // 1. Validate profile
    // -----------------------------------------------------------------------
    auto validation = ProfileValidator::validate(profile);
    if (!validation.isClosed) {
        return nullptr;
    }

    const Vec3 offset = direction * distance;

    // -----------------------------------------------------------------------
    // 2. Facet the profile.  Arcs and circles are followed along their curve;
    //    a circle used to become four points and so a square prism.
    // -----------------------------------------------------------------------
    if (segments < 3) {
        return nullptr;
    }
    const ringstack::SampledProfile sampled = ringstack::sampleProfile(
        validation.orderedEdges, 1e-6, ringstack::ProfileResolution{segments, chordTolerance});
    const std::vector<Vec2>& verts2D = sampled.vertices;
    const size_t N = verts2D.size();
    if (N < 3) {
        return nullptr;
    }

    // Transform to 3D and compute top vertices.
    std::vector<Vec3> bottomPts(N);
    std::vector<Vec3> topPts(N);
    for (size_t i = 0; i < N; ++i) {
        bottomPts[i] = plane.localToWorld(verts2D[i]);
        topPts[i] = bottomPts[i] + offset;
    }

    // -----------------------------------------------------------------------
    // 3a. Rectangle (N==4) → box topology (reuse proven buildBoxTopology)
    // -----------------------------------------------------------------------
    if (N == 4 && sampled.arcs.empty()) {
        const Vec3 pts[8] = {
            bottomPts[0], bottomPts[1], bottomPts[2], bottomPts[3],
            topPts[0],    topPts[1],    topPts[2],    topPts[3],
        };

        auto solid = std::make_unique<topo::Solid>();
        BoxBuild bb = buildBoxTopology(*solid, pts);

        bb.bottom->topoId = TopologyID::make(featureID, "cap_bottom");
        bb.top->topoId = TopologyID::make(featureID, "cap_top");
        bb.front->topoId = TopologyID::make(featureID, "lateral_0");
        bb.right->topoId = TopologyID::make(featureID, "lateral_1");
        bb.back->topoId = TopologyID::make(featureID, "lateral_2");
        bb.left->topoId = TopologyID::make(featureID, "lateral_3");

        {
            int idx = 0;
            for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
                e.topoId = TopologyID::make(featureID, "edge" + std::to_string(idx));
                ++idx;
            }
        }

        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            assignEdgeCurve(&e);
        }

        // Surfaces — all planar for a rectangular extrude.
        // Bottom cap
        {
            const Vec3 u = (bottomPts[1] - bottomPts[0]);
            const Vec3 v = (bottomPts[3] - bottomPts[0]);
            bb.bottom->surface = std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makePlane(
                bottomPts[0], u.normalized(), v.normalized(), u.length(), v.length()));
        }
        // Top cap
        {
            const Vec3 u = (topPts[1] - topPts[0]);
            const Vec3 v = (topPts[3] - topPts[0]);
            bb.top->surface = std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makePlane(
                topPts[0], u.normalized(), v.normalized(), u.length(), v.length()));
        }
        // Lateral faces: front(0-1), right(1-2), back(2-3), left(3-0)
        Face* laterals[4] = {bb.front, bb.right, bb.back, bb.left};
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            const Vec3 uDir = (bottomPts[j] - bottomPts[i]);
            const Vec3 vDir = direction.normalized();
            laterals[i]->surface = std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makePlane(
                bottomPts[i], uDir.normalized(), vDir, uDir.length(), distance));
        }

        return solid;
    }

    // -----------------------------------------------------------------------
    // 3b. General N-gon → prism topology via Euler ops
    // -----------------------------------------------------------------------
    auto solid = std::make_unique<topo::Solid>();
    PrismBuild pb = buildPrismTopology(*solid, bottomPts, topPts);

    // TopologyIDs
    pb.bottomFace->topoId = TopologyID::make(featureID, "cap_bottom");
    pb.topFace->topoId = TopologyID::make(featureID, "cap_top");
    for (size_t i = 0; i < N; ++i) {
        pb.lateralFaces[i]->topoId = TopologyID::make(featureID, "lateral_" + std::to_string(i));
    }

    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make(featureID, "edge" + std::to_string(idx));
            ++idx;
        }
    }

    // Edge curves
    for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
        assignEdgeCurve(&e);
    }

    // Surfaces — planar for caps and lateral faces.
    // Bottom cap: use first two edges to define U/V directions.
    {
        const Vec3 u = (bottomPts[1] - bottomPts[0]).normalized();
        const Vec3 n = plane.normal();
        const Vec3 v = n.cross(u).normalized();
        // Compute bounding extent for the cap plane.
        double uMin = 0, uMax = 0, vMin = 0, vMax = 0;
        for (size_t i = 0; i < N; ++i) {
            const Vec3 d = bottomPts[i] - bottomPts[0];
            double uProj = d.x * u.x + d.y * u.y + d.z * u.z;
            double vProj = d.x * v.x + d.y * v.y + d.z * v.z;
            if (uProj < uMin) uMin = uProj;
            if (uProj > uMax) uMax = uProj;
            if (vProj < vMin) vMin = vProj;
            if (vProj > vMax) vMax = vProj;
        }
        Vec3 origin = bottomPts[0] + u * uMin + v * vMin;
        pb.bottomFace->surface = std::make_shared<geo::NurbsSurface>(
            geo::NurbsSurface::makePlane(origin, u, v, uMax - uMin, vMax - vMin));
    }

    // Top cap
    {
        const Vec3 u = (topPts[1] - topPts[0]).normalized();
        const Vec3 n = plane.normal();
        const Vec3 v = n.cross(u).normalized();
        double uMin = 0, uMax = 0, vMin = 0, vMax = 0;
        for (size_t i = 0; i < N; ++i) {
            const Vec3 d = topPts[i] - topPts[0];
            double uProj = d.x * u.x + d.y * u.y + d.z * u.z;
            double vProj = d.x * v.x + d.y * v.y + d.z * v.z;
            if (uProj < uMin) uMin = uProj;
            if (uProj > uMax) uMax = uProj;
            if (vProj < vMin) vMin = vProj;
            if (vProj > vMax) vMax = vProj;
        }
        Vec3 origin = topPts[0] + u * uMin + v * vMin;
        pb.topFace->surface = std::make_shared<geo::NurbsSurface>(
            geo::NurbsSurface::makePlane(origin, u, v, uMax - uMin, vMax - vMin));
    }

    // Lateral faces
    const Vec3 vDir = direction.normalized();
    for (size_t i = 0; i < N; ++i) {
        size_t j = (i + 1) % N;
        const Vec3 uDir = (bottomPts[j] - bottomPts[i]);
        pb.lateralFaces[i]->surface =
            std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makePlane(
                bottomPts[i], uDir.normalized(), vDir, uDir.length(), distance));
    }

    tagArcIdeals(sampled, plane, offset, pb.lateralFaces, pb.bottomVerts, pb.topVerts);
    return solid;
}

}  // namespace hz::model
