#include "RingStack.h"

#include <cassert>
#include <cmath>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/topology/EulerOps.h"

namespace hz::model::ringstack {

using hz::math::Vec2;
using hz::math::Vec3;
using namespace hz::topo;

namespace {

// Find a half-edge on @p face originating at @p origin. When @p prevOrigin is
// given, prefer the half-edge whose predecessor starts at prevOrigin (matches
// the helper in Extrude.cpp / PrimitiveFactory.cpp).
HalfEdge* findHE(Face* face, Vertex* origin, Vertex* prevOrigin = nullptr) {
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

std::shared_ptr<geo::NurbsCurve> makeLineCurve(const Vec3& a, const Vec3& b) {
    return std::make_shared<geo::NurbsCurve>(std::vector<Vec3>{a, b}, std::vector<double>{1.0, 1.0},
                                             std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
}

}  // namespace

RingStackBuild build(Solid& solid, const std::vector<std::vector<Vec3>>& rings) {
    RingStackBuild out;
    if (rings.size() < 2) return out;
    const size_t N = rings[0].size();
    if (N < 3) return out;
    for (const auto& ring : rings) {
        if (ring.size() != N) return out;
    }
    const size_t levels = rings.size() - 1;  // S

    out.rings.assign(rings.size(), std::vector<Vertex*>(N, nullptr));
    out.lateralFaces.assign(levels, std::vector<Face*>(N, nullptr));

    // Step 1: MVFS with the first vertex of ring 0.
    auto [v0, fOuter, shell] = euler::makeVertexFaceSolid(solid, rings[0][0]);
    out.rings[0][0] = v0;

    // Step 2: MEV to create the rest of ring 0.
    for (size_t i = 1; i < N; ++i) {
        HalfEdge* hePrev = findHE(fOuter, out.rings[0][i - 1]);
        auto [edge, vi] =
            euler::makeEdgeVertex(solid, (i == 1) ? nullptr : hePrev, fOuter, rings[0][i]);
        out.rings[0][i] = vi;
    }

    // Step 3: MEF to close ring 0 → the bottom cap.
    HalfEdge* heLast = findHE(fOuter, out.rings[0][N - 1]);
    HalfEdge* heFirst = findHE(fOuter, out.rings[0][0]);
    auto [eClose, fRemaining] = euler::makeEdgeFace(solid, heLast, heFirst);
    out.bottomFace = fOuter;

    // Steps 4-5 per level: spur the next ring's vertices off the working face,
    // then close each lateral quad. The final MEF of the last level yields the
    // top cap.
    for (size_t L = 0; L < levels; ++L) {
        const auto& lower = out.rings[L];
        auto& upper = out.rings[L + 1];

        // MEV: vertical edges lower[i] → upper[i].
        for (size_t i = 0; i < N; ++i) {
            HalfEdge* heAtLower = findHE(fRemaining, lower[i]);
            auto [eVert, topV] =
                euler::makeEdgeVertex(solid, heAtLower, fRemaining, rings[L + 1][i]);
            upper[i] = topV;
        }

        // MEF: close each lateral quad upper[i] → upper[(i+1)%N].
        for (size_t i = 0; i < N; ++i) {
            size_t next = (i + 1) % N;
            HalfEdge* heUpperI = findHE(fRemaining, upper[i]);
            HalfEdge* heUpperNext = findHE(fRemaining, upper[next]);
            auto [eLat, fLat] = euler::makeEdgeFace(solid, heUpperI, heUpperNext);
            out.lateralFaces[L][i] = fRemaining;
            fRemaining = fLat;
        }
    }

    out.topFace = fRemaining;
    return out;
}

void assignEdgeCurves(Solid& solid) {
    for (auto& e : const_cast<std::deque<Edge>&>(solid.edges())) {
        assert(e.halfEdge != nullptr);
        HalfEdge* he = e.halfEdge;
        e.curve = makeLineCurve(he->origin->point, he->twin->origin->point);
    }
}

std::shared_ptr<geo::NurbsSurface> makeBilinearPatch(const Vec3& p00, const Vec3& p10,
                                                     const Vec3& p01, const Vec3& p11) {
    std::vector<std::vector<Vec3>> ctrlPts = {
        {p00, p01},
        {p10, p11},
    };
    std::vector<std::vector<double>> wts = {
        {1.0, 1.0},
        {1.0, 1.0},
    };
    std::vector<double> knots = {0.0, 0.0, 1.0, 1.0};
    return std::make_shared<geo::NurbsSurface>(std::move(ctrlPts), std::move(wts), knots, knots, 1,
                                               1);
}

std::shared_ptr<geo::NurbsSurface> makeCapSurface(const std::vector<Vec3>& ring,
                                                  const Vec3& normal) {
    if (ring.size() < 3) return nullptr;

    // Build an in-plane basis (u, v) from the first edge and the normal, then
    // fit a bounding rectangle covering the ring.
    Vec3 u = (ring[1] - ring[0]).normalized();
    Vec3 n = normal.normalized();
    Vec3 v = n.cross(u).normalized();

    double uMin = 0, uMax = 0, vMin = 0, vMax = 0;
    for (const auto& p : ring) {
        const Vec3 d = p - ring[0];
        double uProj = d.dot(u);
        double vProj = d.dot(v);
        uMin = std::min(uMin, uProj);
        uMax = std::max(uMax, uProj);
        vMin = std::min(vMin, vProj);
        vMax = std::max(vMax, vProj);
    }
    Vec3 origin = ring[0] + u * uMin + v * vMin;
    return std::make_shared<geo::NurbsSurface>(
        geo::NurbsSurface::makePlane(origin, u, v, uMax - uMin, vMax - vMin));
}

std::vector<Vec2> extractProfileVertices(
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
        if (verts.empty()) {
            verts.push_back(s);
        } else {
            // Orient this edge to continue the chain.
            const double ds = (verts.back() - s).length();
            const double de = (verts.back() - e).length();
            if (de < ds) std::swap(s, e);
        }
        verts.push_back(e);
    }

    if (verts.size() >= 2 && (verts.back() - verts.front()).length() <= tolerance) {
        verts.pop_back();
    }
    return verts;
}

}  // namespace hz::model::ringstack
