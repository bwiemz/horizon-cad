#include "horizon/modeling/Revolve.h"

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
// Helpers (same pattern as Extrude.cpp / PrimitiveFactory.cpp)
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
// Rotate a point around an axis by a given angle (Rodrigues' rotation formula).
// ---------------------------------------------------------------------------

static Vec3 rotateAroundAxis(const Vec3& point, const Vec3& axisPoint,
                              const Vec3& axisDir, double angle) {
    const Vec3 p = point - axisPoint;
    const Vec3 k = axisDir;  // must be normalized
    const double cosA = std::cos(angle);
    const double sinA = std::sin(angle);
    // Rodrigues: p*cos(a) + (k x p)*sin(a) + k*(k.p)*(1-cos(a))
    const Vec3 rotated = p * cosA + k.cross(p) * sinA + k * (k.dot(p) * (1.0 - cosA));
    return rotated + axisPoint;
}

// ---------------------------------------------------------------------------
// Extract 2D vertices from a closed profile (same as Extrude.cpp).
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

        if (verts.empty()) {
            verts.push_back(s);
        } else {
            const double ds = (verts.back() - s).length();
            const double de = (verts.back() - e).length();
            if (de < ds) {
                std::swap(s, e);
            }
        }
        verts.push_back(e);
    }

    if (verts.size() >= 2) {
        if ((verts.back() - verts.front()).length() <= tolerance) {
            verts.pop_back();
        }
    }

    return verts;
}

// ---------------------------------------------------------------------------
// Box-topology builder (reused from Extrude.cpp for 8V/12E/6F).
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
// Revolve::execute
// ---------------------------------------------------------------------------

std::unique_ptr<topo::Solid> Revolve::execute(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
    const draft::SketchPlane& plane,
    const Vec3& axisPoint,
    const Vec3& axisDirection,
    double angle,
    const std::string& featureID) {
    // -----------------------------------------------------------------------
    // 1. Validate profile
    // -----------------------------------------------------------------------
    auto validation = ProfileValidator::validate(profile);
    if (!validation.isClosed) {
        return nullptr;
    }

    const Vec3 axisDir = axisDirection.normalized();
    constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
    const bool fullRevolution = (std::abs(angle - kTwoPi) < 1e-6);

    // -----------------------------------------------------------------------
    // 2. Extract 2D profile vertices and transform to 3D
    // -----------------------------------------------------------------------
    std::vector<Vec2> verts2D = extractVertices2D(validation.orderedEdges, 1e-6);
    const size_t N = verts2D.size();
    if (N < 3) {
        return nullptr;
    }

    // Transform vertices to 3D world coordinates.
    std::vector<Vec3> profilePts3D(N);
    for (size_t i = 0; i < N; ++i) {
        profilePts3D[i] = plane.localToWorld(verts2D[i]);
    }

    // -----------------------------------------------------------------------
    // 3. Full 360-degree revolve of a rectangle profile -> torus-like solid
    //    Using box topology (8V, 12E, 6F) — same as PrimitiveFactory::makeTorus.
    //
    //    The 4 profile corners are rotated by 0 and 180 degrees to produce
    //    8 vertices (two quads at opposite sides of the revolution).
    // -----------------------------------------------------------------------
    if (fullRevolution && N == 4) {
        const double halfAngle = kTwoPi / 2.0;  // 180 degrees

        // Bottom quad: profile corners at angle=0 (they already are in world space).
        // Top quad: profile corners rotated 180 degrees around axis.
        Vec3 pts[8];
        for (size_t i = 0; i < 4; ++i) {
            pts[i] = profilePts3D[i];
            pts[i + 4] = rotateAroundAxis(profilePts3D[i], axisPoint, axisDir, halfAngle);
        }

        auto solid = std::make_unique<topo::Solid>();
        BoxBuild bb = buildBoxTopology(*solid, pts);

        // TopologyIDs
        bb.bottom->topoId = TopologyID::make(featureID, "lateral_0");
        bb.top->topoId = TopologyID::make(featureID, "lateral_3");
        bb.front->topoId = TopologyID::make(featureID, "lateral_1");
        bb.right->topoId = TopologyID::make(featureID, "inner");
        bb.back->topoId = TopologyID::make(featureID, "lateral_2");
        bb.left->topoId = TopologyID::make(featureID, "outer");

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

        // Compute the major radius (distance from axis to profile centroid).
        Vec3 centroid(0, 0, 0);
        for (size_t i = 0; i < 4; ++i) {
            centroid = centroid + profilePts3D[i];
        }
        centroid = centroid * 0.25;
        // Project centroid onto axis to find closest point.
        const Vec3 centroidToAxis = centroid - axisPoint;
        const Vec3 axisProjection =
            axisPoint + axisDir * centroidToAxis.dot(axisDir);
        const double majorRadius = (centroid - axisProjection).length();

        // Minor radius: half the profile diagonal (approximation for
        // rectangular cross-section).
        const double minorRadius =
            (profilePts3D[0] - profilePts3D[2]).length() * 0.5;

        // Toroidal surface for all faces.
        auto torusSurf = std::make_shared<geo::NurbsSurface>(
            geo::NurbsSurface::makeTorus(axisProjection, axisDir, majorRadius, minorRadius));

        bb.bottom->surface = torusSurf;
        bb.top->surface = torusSurf;
        bb.front->surface = torusSurf;
        bb.right->surface = torusSurf;
        bb.back->surface = torusSurf;
        bb.left->surface = torusSurf;

        return solid;
    }

    // -----------------------------------------------------------------------
    // 4. Unsupported profile shape or partial revolve — not yet implemented.
    // -----------------------------------------------------------------------
    return nullptr;
}

}  // namespace hz::model
