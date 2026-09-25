#include "RingStack.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/PrimitiveFactory.h"
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

namespace {

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

}  // namespace

/// Deriving the handedness from the enclosed volume beats reasoning about
/// each loop by hand, which is where winding bugs come from.
void orientOutward(std::vector<SolidSewer::InputFace>& faces) {
    if (soupSignedVolume(faces) >= 0.0) {
        return;
    }
    for (auto& f : faces) {
        std::reverse(f.points.begin(), f.points.end());
    }
}

int ProfileResolution::stepsFor(double radius, double sweep) const {
    const int perTurn = chordTolerance > 0.0
                            ? PrimitiveFactory::segmentsForTolerance(radius, chordTolerance)
                            : segmentsPerTurn;
    const double steps = sweep / math::kTwoPi * static_cast<double>(std::max(perTurn, 3));
    return std::max(1, static_cast<int>(std::ceil(steps - 1e-9)));
}

SampledProfile sampleProfile(const std::vector<std::shared_ptr<draft::DraftEntity>>& orderedEdges,
                             double tolerance, const ProfileResolution& resolution) {
    SampledProfile out;

    // A lone circle is its own closed loop.
    if (orderedEdges.size() == 1) {
        if (auto* circle = dynamic_cast<draft::DraftCircle*>(orderedEdges[0].get())) {
            if (!(circle->radius() > 0.0)) return out;
            const int n = std::max(3, resolution.stepsFor(circle->radius(), math::kTwoPi));
            out.arcs.push_back({circle->center(), circle->radius()});
            out.sourceFacets.push_back(n);
            for (int k = 0; k < n; ++k) {
                const double a = math::kTwoPi * static_cast<double>(k) / n;
                out.vertices.emplace_back(circle->center().x + circle->radius() * std::cos(a),
                                          circle->center().y + circle->radius() * std::sin(a));
                out.edgeArc.push_back(0);
                out.edgeSource.push_back(0);
                out.edgeFacet.push_back(k);
            }
            return out;
        }
    }

    for (size_t source = 0; source < orderedEdges.size(); ++source) {
        const auto& ent = orderedEdges[source];
        Vec2 s, e;
        std::vector<Vec2> run;  // points after s, ending at e
        int arcIndex = -1;
        if (auto* line = dynamic_cast<draft::DraftLine*>(ent.get())) {
            s = line->start();
            e = line->end();
            run.push_back(e);
        } else if (auto* arc = dynamic_cast<draft::DraftArc*>(ent.get())) {
            s = arc->startPoint();
            e = arc->endPoint();
            const double sweep = arc->sweepAngle();
            const int steps = resolution.stepsFor(arc->radius(), sweep);
            for (int k = 1; k < steps; ++k) {
                const double a = arc->startAngle() + sweep * static_cast<double>(k) / steps;
                run.emplace_back(arc->center().x + arc->radius() * std::cos(a),
                                 arc->center().y + arc->radius() * std::sin(a));
            }
            // End on the stored end point exactly, so it welds with its neighbour.
            run.push_back(e);
            arcIndex = static_cast<int>(out.arcs.size());
            out.arcs.push_back({arc->center(), arc->radius()});
        } else {
            out.sourceFacets.push_back(0);
            continue;
        }

        bool reversed = false;
        if (out.vertices.empty()) {
            out.vertices.push_back(s);
        } else {
            // Orient this edge to continue the chain.
            const double ds = (out.vertices.back() - s).length();
            const double de = (out.vertices.back() - e).length();
            if (de < ds) {
                run.pop_back();
                std::reverse(run.begin(), run.end());
                run.push_back(s);
                reversed = true;
            }
        }
        const int facets = static_cast<int>(run.size());
        out.sourceFacets.push_back(facets);
        for (int m = 0; m < facets; ++m) {
            out.vertices.push_back(run[static_cast<size_t>(m)]);
            out.edgeArc.push_back(arcIndex);
            out.edgeSource.push_back(static_cast<int>(source));
            out.edgeFacet.push_back(reversed ? facets - 1 - m : m);
        }
    }

    // One provenance entry was pushed per point after the first, so
    // edgeArc[i] describes the chord vertices[i] -> vertices[i+1].  Dropping
    // the closing vertex (== first) makes the last chord wrap to vertex 0.
    if (out.vertices.size() >= 2 &&
        (out.vertices.back() - out.vertices.front()).length() <= tolerance) {
        out.vertices.pop_back();
    } else {
        // An open chain: the implicit closing edge is straight.
        out.edgeArc.push_back(-1);
        out.edgeSource.push_back(-1);
        out.edgeFacet.push_back(0);
    }
    return out;
}

std::vector<Vec2> extractProfileVertices(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& orderedEdges, double tolerance) {
    return sampleProfile(orderedEdges, tolerance).vertices;
}

}  // namespace hz::model::ringstack
