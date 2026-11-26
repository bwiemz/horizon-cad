#include "horizon/modeling/Extrude.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <exception>
#include <iomanip>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ExactProfile.h"
#include "RingStack.h"
#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/topology/EulerOps.h"
#include "horizon/topology/ExactTolerance.h"

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

    // Each step below takes the half-edge it needs from what the step before
    // it made, where findHE walked the face's whole loop for it: a profile of
    // N points took time in N squared, and one of 20,000 most of a minute.
    // At every step the vertex has one outgoing half-edge on the face, so it
    // is the one findHE would find; a debug build checks that on small
    // prisms, where checking costs little.
    constexpr size_t kCheckUpTo = 64;
    [[maybe_unused]] const auto agrees = [N](HalfEdge* he, Face* face, Vertex* v) {
        return N > kCheckUpTo || he == findHE(face, v);
    };

    // Step 2: MEV to create bottom polygon vertices v1..v(N-1). The tip of
    // the chain so far has one half-edge, back along it.
    for (size_t i = 1; i < N; ++i) {
        HalfEdge* hePrev = i == 1 ? nullptr : pb.bottomVerts[i - 1]->halfEdge;
        assert(i == 1 || agrees(hePrev, fOuter, pb.bottomVerts[i - 1]));
        auto [edge, vi] = euler::makeEdgeVertex(solid, hePrev, fOuter, bottomPts[i]);
        pb.bottomVerts[i] = vi;
    }

    // Step 3: MEF to close the bottom polygon (vN-1 → v0): the chain's two
    // ends, each with its one half-edge.
    HalfEdge* heLast = pb.bottomVerts[N - 1]->halfEdge;
    HalfEdge* heFirst = pb.bottomVerts[0]->halfEdge;
    assert(agrees(heLast, fOuter, pb.bottomVerts[N - 1]));
    assert(agrees(heFirst, fOuter, pb.bottomVerts[0]));
    auto [eClose, fBottom] = euler::makeEdgeFace(solid, heLast, heFirst);
    // fOuter is now the bottom face (N HEs), fBottom is the remaining face.
    pb.bottomFace = fOuter;
    Face* fRemaining = fBottom;

    // Step 4: MEV to create vertical edges from each bottom vertex to its top counterpart.
    // We create them in order 0, 1, 2, ..., N-1. Each bottom vertex has one
    // half-edge on the remaining face, found in one walk round it: an MEV
    // elsewhere adds half-edges but leaves those.
    std::unordered_map<const Vertex*, HalfEdge*> onRemaining;
    onRemaining.reserve(N);
    {
        HalfEdge* start = fRemaining->outerLoop->halfEdge;
        HalfEdge* cur = start;
        do {
            onRemaining.emplace(cur->origin, cur);
            cur = cur->next;
        } while (cur != start);
    }
    for (size_t i = 0; i < N; ++i) {
        HalfEdge* heAtBotI = onRemaining.at(pb.bottomVerts[i]);
        assert(agrees(heAtBotI, fRemaining, pb.bottomVerts[i]));
        auto [eVert, topV] = euler::makeEdgeVertex(solid, heAtBotI, fRemaining, topPts[i]);
        pb.topVerts[i] = topV;
    }

    // Step 5: MEF to create lateral faces and the top face.
    // For each lateral face i, connect topVerts[i] to topVerts[(i+1)%N].
    // The last MEF also closes the top face. A top vertex's one half-edge
    // on the remaining face is the edge down to its bottom vertex until a
    // lateral face is cut off beside it; then it is the new edge's half on
    // the remaining face (the twin of the one that stays on the old face).
    // Each cut leaves the large face as the new one: its half-edges are told
    // their faces once, after the last cut, not at every cut.
    HalfEdge* heTopI = pb.topVerts[0]->halfEdge;
    for (size_t i = 0; i < N; ++i) {
        size_t next = (i + 1) % N;
        HalfEdge* heTopNext = pb.topVerts[next]->halfEdge;
        assert(agrees(heTopI, fRemaining, pb.topVerts[i]));
        assert(agrees(heTopNext, fRemaining, pb.topVerts[next]));
        auto [eLat, fLat] = euler::makeEdgeFaceUnlabelled(solid, fRemaining, heTopI, heTopNext);
        // Old face (fRemaining) becomes the lateral face for this quad.
        pb.lateralFaces[i] = fRemaining;
        fRemaining = fLat;
        heTopI = eLat->halfEdge->twin;
    }
    for (Face* lateral : pb.lateralFaces) euler::labelLoop(lateral);
    euler::labelLoop(fRemaining);
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

namespace {

/// One closed loop, extruded: the solid execute() builds for a profile that
/// is a single loop.
std::unique_ptr<topo::Solid> extrudeLoop(const ProfileValidationResult& validation,
                                         const draft::SketchPlane& plane, const Vec3& direction,
                                         double distance, const std::string& featureID,
                                         int segments, double chordTolerance, std::string* reason,
                                         NamingScheme naming) {
    const auto fail = [reason](std::string why) -> std::unique_ptr<topo::Solid> {
        if (reason) *reason = std::move(why);
        return nullptr;
    };
    const Vec3 offset = direction * distance;

    // -----------------------------------------------------------------------
    // Facet the profile.  Arcs and circles are followed along their curve;
    // a circle used to become four points and so a square prism.
    // -----------------------------------------------------------------------
    const ringstack::SampledProfile sampled = ringstack::sampleProfile(
        validation.orderedEdges, 1e-6, ringstack::ProfileResolution{segments, chordTolerance});
    const std::vector<Vec2>& verts2D = sampled.vertices;
    const size_t N = verts2D.size();
    if (N < 3) {
        return fail("the profile has fewer than three distinct points");
    }

    // The side face over chord i -> i+1: by position, or after the sketch
    // entity the chord was cut from (and which of its chords it is).
    const auto sideRole = [&](size_t i) -> std::string {
        if (naming == NamingScheme::Positional) return "lateral_" + std::to_string(i);
        const int source = i < sampled.edgeSource.size() ? sampled.edgeSource[i] : -1;
        if (source < 0) return "side:closing";
        std::string role = "side:" + validation.edgeSources[static_cast<size_t>(source)];
        if (sampled.sourceFacets[static_cast<size_t>(source)] > 1) {
            // A facet of the curved side the arc sweeps: part of it, under
            // Stable names, so a reference to the side takes them all.
            role += (naming == NamingScheme::Stable ? "/facet:" : ".f") +
                    std::to_string(sampled.edgeFacet[i]);
        }
        return role;
    };
    const auto nameEdges = [&](topo::Solid& solid) {
        if (namesFromGeometry(naming)) {
            hz::model::nameEdges(solid, naming);
            return;
        }
        int idx = 0;
        for (auto& e : solid.edges()) {
            e.topoId = TopologyID::make(featureID, "edge" + std::to_string(idx));
            ++idx;
        }
    };

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
        // front(0-1), right(1-2), back(2-3), left(3-0): the chords in order.
        bb.front->topoId = TopologyID::make(featureID, sideRole(0));
        bb.right->topoId = TopologyID::make(featureID, sideRole(1));
        bb.back->topoId = TopologyID::make(featureID, sideRole(2));
        bb.left->topoId = TopologyID::make(featureID, sideRole(3));
        nameEdges(*solid);

        for (auto& e : solid->edges()) {
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
        pb.lateralFaces[i]->topoId = TopologyID::make(featureID, sideRole(i));
    }
    nameEdges(*solid);

    // Edge curves
    for (auto& e : solid->edges()) {
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

/// Why master refuses to extrude what @p found holds along @p direction by @p distance, in its
/// words; nullopt where it builds it.
std::optional<std::string> refusal(const ProfileRegions& found, const draft::SketchPlane& plane,
                                   const Vec3& direction, double distance) {
    if (!found.ok()) return found.errorMessage;
    if (!(std::abs(distance) > 0.0) || !std::isfinite(distance)) {
        return std::string("the extrusion distance must be a non-zero number");
    }
    if (std::abs(direction.normalized().dot(plane.normal())) < 1e-9) {
        return std::string(
            "the extrusion direction lies in the sketch plane, so it sweeps no volume");
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// The exact extrusion (stage 3): whole faces carrying exact geometry.
// ---------------------------------------------------------------------------

/// Builds the exact extrusion of a profile's regions by an offset, a shell for each region.
///
/// Each loop's material lies on its left (exact::Loop), and the solid lies on the offset's side
/// of the sketch plane ("up" where the offset rises along the plane's normal). Every face's loops
/// run counter-clockwise about its outward normal: in its (u, v), counter-clockwise where the
/// face is not reversed and clockwise where it is.
///
/// - A curve's side face, over its corners i and i + 1 of the loop at the sketch (b0, b1) and at
///   the offset (t0, t1), runs b0, b1, t1, t0 going up and b1, b0, t0, t1 going down. A line's
///   is a plane whose frame's z is the outward normal. An arc's or a circle's is, along the
///   normal, a cylinder about the offset whose u runs the arc's own way (counter-clockwise about
///   the normal) from its own start, its v up the normal from the sketch, reversed where the loop
///   runs the arc backwards (its material outside the circle); along any other direction, the
///   arc swept by the offset (NurbsSurface::makeExtrusion), u its own way, v from 0 at the
///   sketch to 1, reversed where the loop runs it backwards going up or forwards going down.
/// - The caps are planes in the sketch's own frame, so that their (u, v) are the sketch's (x, y):
///   at the sketch (reversed going up) and at the offset (reversed going down). Each runs its
///   loops against the side faces, a hole's as an inner loop.
/// - Each edge's curve is exact: the profile's curve at the sketch or at the offset, and a line
///   along the offset at each corner. A closed curve's corner is the seam of its side.
class ExactExtrusion {
public:
    ExactExtrusion(topo::Solid& solid, const draft::SketchPlane& plane, const Vec3& offset)
        : solid_(solid),
          plane_(plane),
          offset_(offset),
          length_(offset.length()),
          along_(offset * (1.0 / offset.length())),
          up_(offset.dot(plane.normal()) > 0.0) {
        // Within the model's angular resolution of the normal, the offset is along it: an arc
        // sweeps a circular cylinder.
        const math::ModelTolerance resolution;
        alongNormal_ = along_.cross(plane.normal()).length() <= resolution.angular;
    }

    /// Adds @p region as a shell of its own: its caps and its outer loop's sides named for
    /// @p source, each hole's sides for the source @p holeSources gives it.
    void addRegion(const exact::Region& region, const std::string& source,
                   const std::vector<std::string>& holeSources) {
        topo::Shell* shell = solid_.allocShell();
        shell->solid = &solid_;
        geo::Frame atSketch;
        atSketch.origin = plane_.origin();
        atSketch.x = plane_.xAxis();
        atSketch.y = plane_.yAxis();
        atSketch.z = plane_.normal();
        geo::Frame atOffset = atSketch;
        atOffset.origin = plane_.origin() + offset_;
        Face* bottom =
            face(shell, source, "cap_bottom", std::make_shared<geo::Plane>(atSketch), up_);
        Face* top = face(shell, source, "cap_top", std::make_shared<geo::Plane>(atOffset), !up_);
        addLoop(region.outer, source, shell, bottom, top, true);
        for (size_t k = 0; k < region.holes.size(); ++k) {
            addLoop(region.holes[k], holeSources[k], shell, bottom, top, false);
        }
    }

    /// Where the profile's curves miss the corners they meet at by more than the model's
    /// resolution (a sketch's ends join within the profile's tolerance), the corner's vertices
    /// and its line along the offset are given that gap as their tolerance (topo::ToleranceGrowth:
    /// as far as a thousandth of the solid's size). False, with @p why, past that.
    bool coverGaps(std::string* why) {
        if (gaps_.empty()) return true;
        const topo::ToleranceGrowth growth(topo::exactExtent(solid_));
        for (const Gap& g : gaps_) {
            if (!growth.grow(g.bottom->tolerance, g.gap) || !growth.grow(g.top->tolerance, g.gap) ||
                !growth.grow(g.along->tolerance, g.gap)) {
                if (why) {
                    std::ostringstream out;
                    out.imbue(std::locale::classic());
                    out << "the profile's curves miss each other at (" << std::setprecision(6)
                        << g.where.x << ", " << g.where.y << ") by " << g.gap
                        << ", more than its exact form can take at its size";
                    *why = out.str();
                }
                return false;
            }
        }
        return true;
    }

private:
    /// A corner where a curve's own end misses the vertex.
    struct Gap {
        Vertex* bottom = nullptr;
        Vertex* top = nullptr;
        Edge* along = nullptr;
        double gap = 0.0;
        Vec2 where;
    };

    /// A side face and where its corners b0, b1, t1, t0 are in its (u, v).
    struct Side {
        Face* face = nullptr;
        std::array<Vec2, 4> uv;
    };

    Vertex* vertex(const Vec3& p) {
        Vertex* v = solid_.allocVertex();
        v->point = p;
        return v;
    }

    Face* face(topo::Shell* shell, const std::string& source, const std::string& role,
               std::shared_ptr<const geo::Surface> surface, bool reversed) {
        Face* f = solid_.allocFace();
        f->topoId = TopologyID::make(source, role);
        f->exactSurface = std::move(surface);
        f->reversed = reversed;
        f->shell = shell;
        shell->faces.push_back(f);
        return f;
    }

    HalfEdge* halfEdge(Vertex* origin) {
        HalfEdge* he = solid_.allocHalfEdge();
        he->origin = origin;
        if (origin->halfEdge == nullptr) origin->halfEdge = he;
        return he;
    }

    /// Closes @p hes into a loop of @p f: its outer loop, or one more inner loop.
    void close(Face* f, const std::vector<HalfEdge*>& hes, bool outer) {
        topo::Wire* w = solid_.allocWire();
        w->halfEdge = hes.front();
        for (size_t i = 0; i < hes.size(); ++i) {
            HalfEdge* he = hes[i];
            he->face = f;
            he->next = hes[(i + 1) % hes.size()];
            he->next->prev = he;
        }
        if (outer) {
            f->outerLoop = w;
        } else {
            f->innerLoops.push_back(w);
        }
    }

    /// Joins @p a and @p b as the two sides of an edge along @p course, which runs from a's
    /// origin to b's.
    Edge* edge(HalfEdge* a, HalfEdge* b, const exact::Course<geo::Curve3>& course) {
        Edge* e = solid_.allocEdge();
        e->halfEdge = a;
        a->twin = b;
        b->twin = a;
        a->edge = b->edge = e;
        e->exactCurve = course.curve;
        e->curveStart = course.start;
        e->curveEnd = course.end;
        return e;
    }

    static void setPcurve(HalfEdge* he, const exact::Course<geo::Curve2D>& course) {
        he->pcurve = course.curve;
        he->pcurveStart = course.start;
        he->pcurveEnd = course.end;
    }

    static void setLinePcurve(HalfEdge* he, const Vec2& from, const Vec2& to) {
        setPcurve(he, {std::make_shared<geo::Line2D>(from, to), 0.0, 1.0});
    }

    /// The side face @p c sweeps, from corners @p b0 and @p b1 at the sketch to @p t1 and @p t0.
    Side side(const exact::LoopCurve& c, topo::Shell* shell, const std::string& source,
              const Vertex* b0, const Vertex* b1, const Vertex* t1, const Vertex* t0) {
        Side out;
        const std::string role = "side:" + c.source;
        if (!c.curved()) {
            const Vec3 run = (b1->point - b0->point).normalized();
            const Vec3 outward = run.cross(along_) * (up_ ? 1.0 : -1.0);
            const auto plane = std::make_shared<geo::Plane>(
                geo::Frame::fromAxis(b0->point, outward.normalized(), run));
            out.face = face(shell, source, role, plane, false);
            out.uv = {plane->invert(b0->point), plane->invert(b1->point), plane->invert(t1->point),
                      plane->invert(t0->point)};
            return out;
        }
        const double u0 = c.forward ? 0.0 : c.sweep;
        const double u1 = c.forward ? c.sweep : 0.0;
        double vTop = 1.0;
        if (alongNormal_) {
            // About the offset, pointed up the normal: u runs counter-clockwise about it.
            const Vec3 axis = along_ * (up_ ? 1.0 : -1.0);
            const geo::Frame own = c.ownFrame(plane_, Vec3{});
            out.face = face(shell, source, role,
                            std::make_shared<geo::Cylinder>(
                                geo::Frame::fromAxis(own.origin, axis, own.x), c.radius),
                            !c.forward);
            vTop = up_ ? length_ : -length_;
        } else {
            const auto arc = c.curve(plane_, Vec3{}, c.forward);  // its own way, 0 to sweep
            const auto nurbs = arc.curve->toNurbs(0.0, c.sweep);
            out.face = face(shell, source, role,
                            std::make_shared<geo::NurbsSurface>(
                                geo::NurbsSurface::makeExtrusion(*nurbs, offset_)),
                            c.forward != up_);
        }
        out.uv = {Vec2(u0, 0.0), Vec2(u1, 0.0), Vec2(u1, vTop), Vec2(u0, vTop)};
        return out;
    }

    /// How far the curves of @p loop meeting at its corner @p j miss it: an arc's own end may.
    static double gapAt(const exact::Loop& loop, size_t j) {
        const size_t m = loop.curves.size();
        const exact::LoopCurve& leaving = loop.curves[j];
        const exact::LoopCurve& reaching = loop.curves[(j + m - 1) % m];
        const Vec2 corner = leaving.start;
        double gap = 0.0;
        if (leaving.curved()) {
            const Vec2 own = leaving.forward ? leaving.ownStartPoint() : leaving.ownEndPoint();
            gap = std::max(gap, (own - corner).length());
        }
        if (reaching.curved()) {
            const Vec2 own = reaching.forward ? reaching.ownEndPoint() : reaching.ownStartPoint();
            gap = std::max(gap, (own - corner).length());
        }
        return gap;
    }

    void addLoop(const exact::Loop& loop, const std::string& source, topo::Shell* shell,
                 Face* bottomCap, Face* topCap, bool outer) {
        const size_t m = loop.curves.size();
        std::vector<Vertex*> b(m);
        std::vector<Vertex*> t(m);
        for (size_t j = 0; j < m; ++j) {
            b[j] = vertex(plane_.localToWorld(loop.curves[j].start));
            t[j] = vertex(b[j]->point + offset_);
        }
        // Each corner's two half-edges along the offset: up from the sketch and down to it.
        std::vector<HalfEdge*> rising(m, nullptr);
        std::vector<HalfEdge*> falling(m, nullptr);
        std::vector<HalfEdge*> capBottom(m, nullptr);
        std::vector<HalfEdge*> capTop(m, nullptr);
        for (size_t i = 0; i < m; ++i) {
            const exact::LoopCurve& c = loop.curves[i];
            const size_t j0 = i;
            const size_t j1 = (i + 1) % m;
            const Side s = side(c, shell, source, b[j0], b[j1], t[j1], t[j0]);
            const Vec2& b0 = s.uv[0];
            const Vec2& b1 = s.uv[1];
            const Vec2& t1 = s.uv[2];
            const Vec2& t0 = s.uv[3];
            HalfEdge* below = nullptr;  // along the curve at the sketch
            HalfEdge* above = nullptr;  // and at the offset
            if (up_) {
                below = halfEdge(b[j0]);
                HalfEdge* rise = halfEdge(b[j1]);
                above = halfEdge(t[j1]);
                HalfEdge* fall = halfEdge(t[j0]);
                close(s.face, {below, rise, above, fall}, true);
                setLinePcurve(below, b0, b1);
                setLinePcurve(rise, b1, t1);
                setLinePcurve(above, t1, t0);
                setLinePcurve(fall, t0, b0);
                rising[j1] = rise;
                falling[j0] = fall;
            } else {
                below = halfEdge(b[j1]);
                HalfEdge* rise = halfEdge(b[j0]);
                above = halfEdge(t[j0]);
                HalfEdge* fall = halfEdge(t[j1]);
                close(s.face, {below, rise, above, fall}, true);
                setLinePcurve(below, b1, b0);
                setLinePcurve(rise, b0, t0);
                setLinePcurve(above, t0, t1);
                setLinePcurve(fall, t1, b1);
                rising[j0] = rise;
                falling[j1] = fall;
            }
            // The caps run against the side: at the sketch along the loop going down, at the
            // offset going up.
            capBottom[i] = halfEdge(up_ ? b[j1] : b[j0]);
            capTop[i] = halfEdge(up_ ? t[j0] : t[j1]);
            setPcurve(capBottom[i], c.pcurve(!up_));
            setPcurve(capTop[i], c.pcurve(up_));
            edge(below, capBottom[i], c.curve(plane_, Vec3{}, up_));
            edge(above, capTop[i], c.curve(plane_, offset_, !up_));
        }
        // Each cap's loop in the order its half-edges run.
        std::vector<HalfEdge*> alongLoop = up_ ? capTop : capBottom;
        std::vector<HalfEdge*> againstLoop = up_ ? capBottom : capTop;
        std::reverse(againstLoop.begin(), againstLoop.end());
        close(up_ ? topCap : bottomCap, alongLoop, outer);
        close(up_ ? bottomCap : topCap, againstLoop, outer);
        for (size_t j = 0; j < m; ++j) {
            Edge* e = edge(rising[j], falling[j],
                           {std::make_shared<geo::LineCurve>(b[j]->point, along_), 0.0, length_});
            const double gap = gapAt(loop, j);
            if (gap > 0.0) gaps_.push_back({b[j], t[j], e, gap, loop.curves[j].start});
        }
    }

    topo::Solid& solid_;
    const draft::SketchPlane& plane_;
    Vec3 offset_;
    double length_;
    Vec3 along_;  ///< the offset's direction
    bool up_;     ///< the offset rises along the sketch's normal
    bool alongNormal_ = false;
    std::vector<Gap> gaps_;
};

}  // namespace

namespace {

/// The exact extrusion of the regions @p found (which master builds) along @p offset, or null
/// with "could not be built exactly: <why>".
std::unique_ptr<topo::Solid> extrudeExactly(const ProfileRegions& found,
                                            const draft::SketchPlane& plane, const Vec3& offset,
                                            const std::string& featureID, std::string* reason) {
    const auto notExactly = [reason](const std::string& why) -> std::unique_ptr<topo::Solid> {
        if (reason) *reason = "could not be built exactly: " + why;
        return nullptr;
    };
    if (!std::isfinite(offset.length())) return notExactly("its direction is not a finite vector");
    try {
        auto solid = std::make_unique<topo::Solid>();
        ExactExtrusion build(*solid, plane, offset);
        // Named as master named the faceted extrusion's: the first region for the feature, the
        // others `<feature>~region<r>`, and each hole's sides `<feature>~hole<k>`, counted
        // through the regions.
        int holeCount = 0;
        for (size_t r = 0; r < found.regions.size(); ++r) {
            std::string why;
            const auto region = exact::regionOf(found.regions[r], &why);
            if (!region) return notExactly(why);
            std::vector<std::string> holeSources;
            holeSources.reserve(region->holes.size());
            for (size_t k = 0; k < region->holes.size(); ++k) {
                holeSources.push_back(featureID + "~hole" + std::to_string(holeCount++));
            }
            build.addRegion(*region, r == 0 ? featureID : featureID + "~region" + std::to_string(r),
                            holeSources);
        }
        std::string why;
        if (!build.coverGaps(&why)) return notExactly(why);
        nameEdgesLogically(*solid);
        return solid;
    } catch (const std::exception& e) {
        return notExactly(e.what());
    }
}

}  // namespace

std::unique_ptr<topo::Solid> Extrude::exact(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
    const draft::SketchPlane& plane, const Vec3& direction, double distance,
    const std::string& featureID, std::string* reason) {
    const ProfileRegions found = ProfileValidator::regions(profile);
    if (auto refused = refusal(found, plane, direction, distance)) {
        if (reason) *reason = std::move(*refused);
        return nullptr;
    }
    return extrudeExactly(found, plane, direction * distance, featureID, reason);
}

std::unique_ptr<topo::Solid> Extrude::execute(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
    const draft::SketchPlane& plane, const Vec3& direction, double distance,
    const std::string& featureID, int segments, double chordTolerance, std::string* reason,
    NamingScheme naming) {
    const auto fail = [reason](std::string why) -> std::unique_ptr<topo::Solid> {
        if (reason) *reason = std::move(why);
        return nullptr;
    };

    const ProfileRegions found = ProfileValidator::regions(profile);
    if (auto refused = refusal(found, plane, direction, distance)) return fail(*refused);
    if (segments < 3) {
        return fail("arcs need at least 3 segments per turn");
    }
    if (found.isSingleLoop()) {
        return extrudeLoop(found.regions.front().outer, plane, direction, distance, featureID,
                           segments, chordTolerance, reason, naming);
    }

    // Holes and separate regions: each region's outer loop extruded, less
    // each hole's; the regions joined. A hole is cut by its loop extruded a
    // tenth further at both ends, so no face of the cutter lies in a cap.
    // The cutters and the regions after the first are named apart from the
    // first region, whose faces keep the names a single loop's would have.
    const double margin = 0.1 * std::abs(distance);
    const Vec3 grows = (direction * distance).normalized();
    const draft::SketchPlane cutterPlane(plane.origin() - grows * margin, plane.normal(),
                                         plane.xAxis());
    const double cutterDistance = distance + std::copysign(2.0 * margin, distance);
    std::unique_ptr<topo::Solid> result;
    int holeCount = 0;
    for (size_t r = 0; r < found.regions.size(); ++r) {
        const ProfileRegion& region = found.regions[r];
        const std::string regionID = r == 0 ? featureID : featureID + "~region" + std::to_string(r);
        std::string why;
        auto solid = extrudeLoop(region.outer, plane, direction, distance, regionID, segments,
                                 chordTolerance, &why, naming);
        if (!solid) return fail(why);
        for (const auto& hole : region.holes) {
            auto cutter = extrudeLoop(hole, cutterPlane, direction, cutterDistance,
                                      featureID + "~hole" + std::to_string(holeCount++), segments,
                                      chordTolerance, &why, naming);
            if (!cutter) return fail("a hole in the profile: " + why);
            auto cut = BooleanOp::execute(*solid, *cutter, BooleanType::Subtract, &why, naming);
            if (!cut) return fail("a hole in the profile could not be cut: " + why);
            solid = std::move(cut);
        }
        if (!result) {
            result = std::move(solid);
            continue;
        }
        auto joined = BooleanOp::execute(*result, *solid, BooleanType::Union, &why, naming);
        if (!joined) return fail("the profile's regions could not be joined: " + why);
        result = std::move(joined);
    }
    return result;
}

}  // namespace hz::model
