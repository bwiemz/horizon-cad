#include "horizon/modeling/Extrude.h"

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/topology/EulerOps.h"

#include <cassert>
#include <cmath>

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
    return std::make_shared<geo::NurbsCurve>(
        std::vector<Vec3>{a, b}, std::vector<double>{1.0, 1.0},
        std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
}

static void assignEdgeCurve(Edge* edge) {
    assert(edge->halfEdge != nullptr);
    HalfEdge* he = edge->halfEdge;
    edge->curve = makeLineCurve(he->origin->point, he->twin->origin->point);
}

// ---------------------------------------------------------------------------
// Extract 2D vertices from a closed profile (lines/arcs chain).
// Returns the unique vertices in chain order (the closing point == first point
// is NOT duplicated).
// ---------------------------------------------------------------------------

static std::vector<Vec2> extractVertices2D(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& orderedEdges, double tolerance) {
    std::vector<Vec2> verts;
    for (const auto& ent : orderedEdges) {
        Vec2 s, e;
        if (auto* line = dynamic_cast<draft::DraftLine*>(ent.get())) {
            s = line->start();
            e = line->end();
        } else if (auto* arc = dynamic_cast<draft::DraftArc*>(ent.get())) {
            s = arc->startPoint();
            e = arc->endPoint();
        } else {
            continue;
        }

        // If chain is empty or the start of this edge doesn't match the last vertex
        // added, add the start point. Otherwise the chain is continuing from the last
        // vertex and we only need to add the end.
        if (verts.empty()) {
            verts.push_back(s);
        } else {
            // Check whether this edge is reversed relative to the chain.
            const double ds = (verts.back() - s).length();
            const double de = (verts.back() - e).length();
            if (de < ds) {
                // Entity is reversed — swap s and e.
                std::swap(s, e);
            }
        }
        verts.push_back(e);
    }

    // Remove the duplicate closing vertex (last == first within tolerance).
    if (verts.size() >= 2) {
        if ((verts.back() - verts.front()).length() <= tolerance) {
            verts.pop_back();
        }
    }

    return verts;
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
        auto [edge, vi] = euler::makeEdgeVertex(solid, (i == 1) ? nullptr : hePrev,
                                                 fOuter, bottomPts[i]);
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
// Extrude::execute
// ---------------------------------------------------------------------------

std::unique_ptr<topo::Solid> Extrude::execute(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
    const draft::SketchPlane& plane,
    const Vec3& direction,
    double distance,
    const std::string& featureID) {
    // -----------------------------------------------------------------------
    // 1. Validate profile
    // -----------------------------------------------------------------------
    auto validation = ProfileValidator::validate(profile);
    if (!validation.isClosed) {
        return nullptr;
    }

    const Vec3 offset = direction * distance;

    // -----------------------------------------------------------------------
    // 2. Circle profile → cylinder
    // -----------------------------------------------------------------------
    if (validation.orderedEdges.size() == 1) {
        auto* circle = dynamic_cast<draft::DraftCircle*>(validation.orderedEdges[0].get());
        if (circle != nullptr) {
            const double r = circle->radius();
            const Vec3 center3D = plane.localToWorld(circle->center());
            const Vec3 topCenter = center3D + offset;

            // Use box topology with 4 points around circle at bottom and top.
            const Vec3 xA = plane.xAxis();
            const Vec3 yA = plane.yAxis();

            const Vec3 pts[8] = {
                center3D + xA * r,
                center3D + yA * r,
                center3D - xA * r,
                center3D - yA * r,
                topCenter + xA * r,
                topCenter + yA * r,
                topCenter - xA * r,
                topCenter - yA * r,
            };

            auto solid = std::make_unique<topo::Solid>();
            BoxBuild bb = buildBoxTopology(*solid, pts);

            // TopologyIDs
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

            // Surfaces — planar caps, cylindrical lateral
            bb.bottom->surface = std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makePlane(center3D - xA * r - yA * r, xA, yA, 2 * r, 2 * r));
            bb.top->surface = std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makePlane(topCenter - xA * r - yA * r, xA, yA, 2 * r, 2 * r));

            auto cylSurf = std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makeCylinder(center3D, direction.normalized(), r, distance));
            bb.front->surface = cylSurf;
            bb.right->surface = cylSurf;
            bb.back->surface = cylSurf;
            bb.left->surface = cylSurf;

            return solid;
        }
    }

    // -----------------------------------------------------------------------
    // 3. Line/arc profile → prism extrude
    // -----------------------------------------------------------------------
    std::vector<Vec2> verts2D = extractVertices2D(validation.orderedEdges, 1e-6);
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
    if (N == 4) {
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
            bb.bottom->surface = std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makePlane(bottomPts[0], u.normalized(), v.normalized(),
                                             u.length(), v.length()));
        }
        // Top cap
        {
            const Vec3 u = (topPts[1] - topPts[0]);
            const Vec3 v = (topPts[3] - topPts[0]);
            bb.top->surface = std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makePlane(topPts[0], u.normalized(), v.normalized(),
                                             u.length(), v.length()));
        }
        // Lateral faces: front(0-1), right(1-2), back(2-3), left(3-0)
        Face* laterals[4] = {bb.front, bb.right, bb.back, bb.left};
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            const Vec3 uDir = (bottomPts[j] - bottomPts[i]);
            const Vec3 vDir = direction.normalized();
            laterals[i]->surface = std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makePlane(bottomPts[i], uDir.normalized(), vDir,
                                             uDir.length(), distance));
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
        pb.lateralFaces[i]->topoId =
            TopologyID::make(featureID, "lateral_" + std::to_string(i));
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
        pb.lateralFaces[i]->surface = std::make_shared<geo::NurbsSurface>(
            geo::NurbsSurface::makePlane(bottomPts[i], uDir.normalized(), vDir,
                                         uDir.length(), distance));
    }

    return solid;
}

}  // namespace hz::model
