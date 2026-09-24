#include "horizon/modeling/ChamferOp.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <set>
#include <vector>

#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/modeling/SolidSewer.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Queries.h"

namespace hz::model {

using namespace hz::topo;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// Helpers (same as FilletOp — duplicated to keep each op self-contained)
// ---------------------------------------------------------------------------

/// The edges a reference names: the edge itself, or — once an operation has
/// split it into pieces — every piece (its descendants).
static std::vector<const Edge*> findEdges(const Solid& solid, const TopologyID& id) {
    std::vector<const Edge*> pieces;
    for (const auto& e : solid.edges()) {
        if (e.topoId == id) return {&e};
        if (e.topoId.isDescendantOf(id)) pieces.push_back(&e);
    }
    return pieces;
}

// ---------------------------------------------------------------------------
// Chamfer geometry for a single edge
// ---------------------------------------------------------------------------

struct ChamferEdgeInfo {
    const Edge* originalEdge = nullptr;
    Vertex* v1 = nullptr;
    Vertex* v2 = nullptr;
    Face* faceA = nullptr;
    Face* faceB = nullptr;
    Vec3 edgeDir;
    Vec3 offsetA;
    Vec3 offsetB;

    // Offset vertex positions (4 per chamfered edge).
    Vec3 v1_offsetA;
    Vec3 v1_offsetB;
    Vec3 v2_offsetA;
    Vec3 v2_offsetB;

    // The chamfer plane as a half-space: material is kept where
    // dot(p - planePoint, planeNormal) >= 0, so the chamfered edge itself is
    // on the discarded side.  Corner geometry falls out of intersecting these.
    Vec3 planePoint;
    Vec3 planeNormal;
};

/// Sutherland–Hodgman clip of a planar polygon against a half-space.
/// Works directly in 3D — the polygon stays in its own plane, so no
/// projection is needed.
///
/// Intersections are **snapped to the segment endpoint they land on**.  A cut
/// passing through an existing vertex recomputes that vertex as
/// `cur + (nxt - cur) * t` with t at one end of its range, where cancellation
/// leaves the result a fraction of a micron away.  That gap is enough to
/// matter: the pair is then too far apart to deduplicate and too close to be a
/// sane polygon edge, so it survives into a loop whose segments read as
/// crossing.  Snapping removes the drift at its source, which keeps
/// deduplication exact rather than tolerance-dependent.
static std::vector<Vec3> clipToHalfSpace(const std::vector<Vec3>& poly, const Vec3& planePoint,
                                         const Vec3& planeNormal, double eps) {
    std::vector<Vec3> out;
    const size_t n = poly.size();
    if (n < 3) {
        return out;
    }
    // Vertex identity is scale-relative, not absolute.  The sewer welds at an
    // absolute tolerance while the geometric validator judges a loop
    // degenerate relative to its own extent, so on a loop several units across
    // a segment can be long enough to survive sewing and short enough to read
    // as degenerate afterwards.  Deduplicating against the loop's extent keeps
    // both consistent, and it is the tolerance the incoming points are
    // actually built to: an offset shared by two edges is only equal to within
    // the rounding of two independent constructions.
    double scale = 0.0;
    for (const auto& p : poly) {
        scale = std::max(scale, std::max(std::abs(p.x), std::max(std::abs(p.y), std::abs(p.z))));
    }
    const double vertexEps = eps * std::max(1.0, scale);

    out.reserve(n + 2);
    for (size_t i = 0; i < n; ++i) {
        const Vec3& cur = poly[i];
        const Vec3& nxt = poly[(i + 1) % n];
        const double dCur = (cur - planePoint).dot(planeNormal);
        const double dNxt = (nxt - planePoint).dot(planeNormal);
        const bool inCur = dCur >= -eps;
        const bool inNxt = dNxt >= -eps;

        if (inCur) {
            out.push_back(cur);
        }
        if (inCur != inNxt) {
            // A segment nearly parallel to the plane makes `t` ill-conditioned:
            // the denominator is the difference of two nearly equal signed
            // distances, so the "intersection" lands microns from an endpoint
            // rather than on it.  When the two endpoints are within tolerance
            // of the same side, the segment lies in the plane and the endpoint
            // already kept represents the crossing; computing a point there
            // manufactures the near-duplicate instead of finding one.
            const double denom = dCur - dNxt;
            if (std::abs(denom) > eps) {
                const double t = dCur / denom;
                Vec3 hit = cur + (nxt - cur) * t;
                if (hit.distanceTo(cur) <= vertexEps) {
                    hit = cur;
                } else if (hit.distanceTo(nxt) <= vertexEps) {
                    hit = nxt;
                }
                out.push_back(hit);
            }
        }
    }

    // Drop points the clip duplicated (a cut through an existing vertex).
    std::vector<Vec3> deduped;
    deduped.reserve(out.size());
    for (const auto& p : out) {
        if (deduped.empty() || p.distanceTo(deduped.back()) > vertexEps) {
            deduped.push_back(p);
        }
    }
    while (deduped.size() > 1 && deduped.front().distanceTo(deduped.back()) <= vertexEps) {
        deduped.pop_back();
    }
    return deduped;
}

static bool computeChamferGeometry(const Edge* edge, double distA, double distB,
                                   ChamferEdgeInfo& info) {
    info.originalEdge = edge;
    info.faceA = leftFace(edge);
    info.faceB = rightFace(edge);
    if (!info.faceA || !info.faceB) {
        return false;
    }

    HalfEdge* he = edge->halfEdge;
    info.v1 = he->origin;
    info.v2 = he->twin->origin;

    Vec3 dir = info.v2->point - info.v1->point;
    double edgeLen = dir.length();
    if (edgeLen < 1e-12) {
        return false;
    }
    info.edgeDir = dir * (1.0 / edgeLen);

    // Inward on each face from the loop's winding: walking a loop with its
    // normal up, the face lies to the left. faceA's loop runs v1->v2 and
    // faceB's v2->v1. Comparing with a face's centroid picked the wrong side
    // on non-convex faces, an L-shaped cap for one.
    const Vec3 nA = loopNormal(info.faceA);
    const Vec3 nB = loopNormal(info.faceB);
    Vec3 candidateA = nA.cross(info.edgeDir);
    Vec3 candidateB = nB.cross(info.edgeDir * (-1.0));

    double lenA = candidateA.length();
    double lenB = candidateB.length();
    if (lenA < 1e-12 || lenB < 1e-12) {
        return false;
    }
    candidateA = candidateA * (1.0 / lenA);
    candidateB = candidateB * (1.0 / lenB);

    info.offsetA = candidateA;
    info.offsetB = candidateB;

    // Offset by possibly different distances on each face.
    info.v1_offsetA = info.v1->point + candidateA * distA;
    info.v1_offsetB = info.v1->point + candidateB * distB;
    info.v2_offsetA = info.v2->point + candidateA * distA;
    info.v2_offsetB = info.v2->point + candidateB * distB;

    // The chamfer plane, oriented so the edge being cut away is outside it.
    Vec3 normal = (info.v1_offsetB - info.v1_offsetA).cross(info.v2_offsetA - info.v1_offsetA);
    const double nLen = normal.length();
    if (nLen < 1e-12) {
        return false;  // Offsets collapse onto the edge line.
    }
    normal = normal * (1.0 / nLen);
    if ((info.v1->point - info.v1_offsetA).dot(normal) > 0.0) {
        normal = normal * (-1.0);
    }
    info.planePoint = info.v1_offsetA;
    info.planeNormal = normal;

    return true;
}

// ---------------------------------------------------------------------------
// Core chamfer implementation shared by both variants
// ---------------------------------------------------------------------------

static ChamferResult chamferImpl(const Solid& inputSolid, const std::vector<TopologyID>& edgeIds,
                                 double distA, double distB, const std::string& featureID) {
    ChamferResult result;

    if (distA <= 0.0 || distB <= 0.0) {
        result.errorMessage = "Chamfer distance must be positive";
        return result;
    }
    if (edgeIds.empty()) {
        result.errorMessage = "No edges specified for chamfer";
        return result;
    }

    // Resolve edges.
    std::vector<ChamferEdgeInfo> chamferEdges;
    chamferEdges.reserve(edgeIds.size());

    std::vector<std::pair<const Edge*, const TopologyID*>> chosen;
    for (const auto& eid : edgeIds) {
        const auto edges = findEdges(inputSolid, eid);
        if (edges.empty()) {
            result.errorMessage = "Edge not found: " + eid.tag();
            return result;
        }
        for (const Edge* edge : edges) {
            // Once each, however many references reach it.
            const bool listed = std::any_of(chosen.begin(), chosen.end(),
                                            [edge](const auto& c) { return c.first == edge; });
            if (!listed) chosen.emplace_back(edge, &eid);
        }
    }
    for (const auto& [edge, idPtr] : chosen) {
        const TopologyID& eid = *idPtr;
        ChamferEdgeInfo info;
        if (!computeChamferGeometry(edge, distA, distB, info)) {
            result.errorMessage = "Cannot compute chamfer geometry for edge: " + eid.tag();
            return result;
        }
        chamferEdges.push_back(info);
    }

    // Capacity: an offset cannot travel further into a face than the face
    // itself extends.
    //
    // The previous check capped the distance at half the shortest edge of the
    // adjacent face.  That is a proxy for "do not overrun the neighbouring
    // geometry" which happens to hold for a box and is meaningless once faces
    // are faceted: on a 32-sided cylinder the shortest edge is the facet
    // chord (0.98 for r = 5), so it refused every chamfer over 0.49 even
    // though the geometry is exact well past 2.  It was also wrong for boxes,
    // rejecting a 6mm chamfer on a 10mm box that builds correctly.
    //
    // What actually bounds the offset is how far the face reaches in the
    // offset direction.  Past that the offset edge has left the face
    // entirely, which no amount of downstream repair can rescue.  This is a
    // necessary condition only — it never rejects a distance that would have
    // worked — and the geometric gate after sewing is what actually
    // guarantees the result.
    for (const auto& ce : chamferEdges) {
        const Vec3 onEdge = ce.v1->point;
        auto reachOf = [&onEdge](const Face* face, const Vec3& dir) {
            double reach = 0.0;
            for (const auto* v : faceVertices(face)) {
                reach = std::max(reach, (v->point - onEdge).dot(dir));
            }
            return reach;
        };
        const double reachA = reachOf(ce.faceA, ce.offsetA);
        const double reachB = reachOf(ce.faceB, ce.offsetB);
        if (distA > reachA + 1e-9 || distB > reachB + 1e-9) {
            result.errorMessage =
                "Chamfer distance exceeds the width of a face adjacent to edge: " +
                ce.originalEdge->topoId.tag();
            return result;
        }
    }

    // Every vertex touched by a chamfer, mapped to the chamfers meeting there.
    // Two selected edges sharing a vertex is a *vertex blend*: the chamfer
    // planes cut each other, and the corner is whatever their intersection
    // leaves.  That is handled uniformly below rather than refused.
    std::map<uint32_t, std::vector<size_t>> vertexChamfers;
    for (size_t i = 0; i < chamferEdges.size(); ++i) {
        vertexChamfers[chamferEdges[i].v1->id].push_back(i);
        vertexChamfers[chamferEdges[i].v2->id].push_back(i);
    }

    double maxOffset = 0.0;
    for (const auto& ce : chamferEdges) {
        maxOffset = std::max({maxOffset, ce.v1->point.distanceTo(ce.v1_offsetA),
                              ce.v1->point.distanceTo(ce.v1_offsetB)});
    }
    // Deduplicate clipped loops at the tolerance the sewer welds at, not a
    // tighter one.  A pair of points closer than the weld tolerance but
    // further apart than the clip's own epsilon survives into the loop and
    // then collapses during sewing, leaving a zero-length segment that reads
    // downstream as a degenerate — and, once two of them meet, a
    // self-intersecting — boundary.
    const double clipEps = SolidSewer::kDefaultWeldTol;

    // -----------------------------------------------------------------------
    // Rewrite every original face loop, then sew.
    //
    // A chamfer is a local edit of the boundary polygons: the material a
    // chamfer removes is the half-space on the far side of its chamfer plane,
    // so what a corner becomes is just that corner clipped by the planes of
    // every chamfer meeting there.  Doing it by clipping — rather than by
    // pasting precomputed offset points into the loop — is what makes vertex
    // blends work: where two or three chamfers meet, their planes cut each
    // other and the corner points fall out of the same code path that
    // produces the single-chamfer offsets.
    //
    // The clip runs on a *wedge*: the corner's two loop directions extended
    // far enough that every cut lands well inside it.  Output points near the
    // apex are the new corner chain, in loop order; points out at the far
    // edge are the wedge's own boundary and are dropped.  Working on a local
    // wedge rather than the whole face keeps a chamfer plane — which is
    // unbounded — from slicing a distant part of a non-convex face.
    //
    // Reconstruction is a SolidSewer sew of the rewritten soup, the same
    // welding / T-junction / twin-pairing pipeline BooleanOp uses.
    // -----------------------------------------------------------------------

    std::vector<SolidSewer::InputFace> sewFaces;
    sewFaces.reserve(inputSolid.faceCount() + chamferEdges.size());

    for (const auto& face : inputSolid.faces()) {
        if (face.outerLoop == nullptr || face.outerLoop->halfEdge == nullptr) {
            continue;
        }
        SolidSewer::InputFace fd;
        fd.topoId = face.topoId;
        // Chamfers cut a face's corners inside its own plane, so every
        // original face keeps its carrier — only its boundary changes.
        fd.surface = face.surface;

        HalfEdge* start = face.outerLoop->halfEdge;
        HalfEdge* cur = start;
        do {
            Vertex* v = cur->origin;
            auto it = vertexChamfers.find(v->id);
            if (it == vertexChamfers.end()) {
                fd.points.push_back(v->point);
                cur = cur->next;
                continue;
            }

            // The corner wedge, listed in loop order: in from the previous
            // vertex, through v, out toward the next.
            const Vec3 prevP = cur->prev->origin->point;
            const Vec3 nextP = cur->next->origin->point;
            Vec3 dirIn = prevP - v->point;
            Vec3 dirOut = nextP - v->point;
            const double lenIn = dirIn.length();
            const double lenOut = dirOut.length();
            if (lenIn < 1e-12 || lenOut < 1e-12) {
                fd.points.push_back(v->point);
                cur = cur->next;
                continue;
            }
            const double reach = 64.0 * maxOffset;
            dirIn = dirIn * (reach / lenIn);
            dirOut = dirOut * (reach / lenOut);

            std::vector<Vec3> wedge = {v->point + dirIn, v->point, v->point + dirOut};
            for (size_t ci : it->second) {
                wedge = clipToHalfSpace(wedge, chamferEdges[ci].planePoint,
                                        chamferEdges[ci].planeNormal, clipEps);
                if (wedge.size() < 3) {
                    break;
                }
            }

            // Keep the points near the apex; the rest are the wedge's own
            // far boundary, which stands in for the untouched face beyond
            // this corner.
            const double nearRadius = reach * 0.5;
            bool emitted = false;
            for (const auto& p : wedge) {
                if (p.distanceTo(v->point) <= nearRadius) {
                    fd.points.push_back(p);
                    emitted = true;
                }
            }
            if (!emitted) {
                // Over-chamfered past this corner: nothing of it survives.
                // Leave the vertex out; the geometric gate below reports the
                // damage rather than this loop guessing at a repair.
            }
            cur = cur->next;
        } while (cur != start);

        if (fd.points.size() >= 3) {
            sewFaces.push_back(std::move(fd));
        }
    }

    // One face per chamfered edge: the quad between the two offset lines,
    // clipped by the chamfers it meets at either end.  With no blend the clip
    // is a no-op and the face stays a quad; with a blend it becomes the
    // pentagon (or triangle) the intersecting planes leave.
    for (size_t i = 0; i < chamferEdges.size(); ++i) {
        const auto& ce = chamferEdges[i];
        SolidSewer::InputFace fd;
        fd.topoId = TopologyID::make(featureID, "chamfer").child(ce.originalEdge->topoId.tag(), 0);
        fd.points = {ce.v1_offsetA, ce.v2_offsetA, ce.v2_offsetB, ce.v1_offsetB};

        for (const uint32_t vid : {ce.v1->id, ce.v2->id}) {
            for (size_t other : vertexChamfers[vid]) {
                if (other == i) {
                    continue;
                }
                fd.points = clipToHalfSpace(fd.points, chamferEdges[other].planePoint,
                                            chamferEdges[other].planeNormal, clipEps);
            }
        }
        if (fd.points.size() < 3) {
            result.errorMessage = "Chamfer face vanished under an adjacent chamfer for edge: " +
                                  ce.originalEdge->topoId.tag();
            return result;
        }

        Vec3 areaVec(0, 0, 0);
        for (size_t k = 0; k < fd.points.size(); ++k) {
            areaVec = areaVec + fd.points[k].cross(fd.points[(k + 1) % fd.points.size()]);
        }
        // Match the input's loop-winding sense rather than assuming one: the
        // chamfer face sits between faceA and faceB, so its loop normal must
        // point the same way theirs do.  Deriving the reference from the loops
        // (not the bound surfaces, whose normals need not agree with the
        // winding) makes this correct for either convention.
        const Vec3 reference = GeometryValidator::loopAreaVector(*ce.faceA) +
                               GeometryValidator::loopAreaVector(*ce.faceB);
        if (areaVec.dot(reference) < 0.0) {
            std::reverse(fd.points.begin(), fd.points.end());
        }
        sewFaces.push_back(std::move(fd));
    }

    auto solid = SolidSewer::sew(sewFaces);
    if (solid == nullptr) {
        result.errorMessage = "Chamfer produced no sewable geometry";
        return result;
    }
    if (!solid->checkManifold()) {
        result.errorMessage = "Chamfer produced non-manifold topology";
        return result;
    }
    // The whole point of the rebuild: refuse to hand back a solid whose loops
    // are geometrically inconsistent, however well-formed its linkage is.
    const auto issues = GeometryValidator::check(*solid);
    if (!issues.ok()) {
        result.errorMessage =
            "Chamfer produced invalid geometry:\n" + GeometryValidator::report(*solid);
        return result;
    }

    result.solid = std::move(solid);
    return result;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

ChamferResult ChamferOp::executeEqual(const Solid& inputSolid,
                                      const std::vector<TopologyID>& edgeIds, double distance,
                                      const std::string& featureID) {
    return chamferImpl(inputSolid, edgeIds, distance, distance, featureID);
}

ChamferResult ChamferOp::executeTwoDistance(const Solid& inputSolid,
                                            const std::vector<TopologyID>& edgeIds,
                                            double distance1, double distance2,
                                            const std::string& featureID) {
    return chamferImpl(inputSolid, edgeIds, distance1, distance2, featureID);
}

}  // namespace hz::model
