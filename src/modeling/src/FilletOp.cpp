#include "horizon/modeling/FilletOp.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <set>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/topology/Queries.h"

namespace hz::model {

using namespace hz::topo;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Find the Edge in the solid whose topoId matches the given id.
static const Edge* findEdge(const Solid& solid, const TopologyID& id) {
    for (const auto& e : solid.edges()) {
        if (e.topoId == id) {
            return &e;
        }
    }
    return nullptr;
}

/// Face normal derived from the loop winding (Newell). Unlike the surface
/// carrier's normal — whose sign is not globally consistent — the Newell
/// normal is tied to the traversal order, so "interior lies to the left when
/// walking the loop with this normal up" holds by construction.
static Vec3 newellNormal(const Face* face) {
    auto verts = faceVertices(face);
    Vec3 n(0, 0, 0);
    for (size_t i = 0; i < verts.size(); ++i) {
        const Vec3& a = verts[i]->point;
        const Vec3& b = verts[(i + 1) % verts.size()]->point;
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    const double len = n.length();
    return len > 1e-12 ? n * (1.0 / len) : Vec3(0, 0, 1);
}

/// Signed volume of the solid computed from its face loops (divergence
/// theorem over loop fans). Positive exactly when the loops' Newell normals
/// point outward — calibrates convexity tests independently of the kernel's
/// winding convention.
static double signedLoopVolume(const Solid& solid) {
    double vol = 0.0;
    for (const auto& f : solid.faces()) {
        auto verts = faceVertices(&f);
        if (verts.size() < 3) continue;
        const Vec3& a = verts[0]->point;
        for (size_t i = 1; i + 1 < verts.size(); ++i) {
            vol += a.dot(verts[i]->point.cross(verts[i + 1]->point)) / 6.0;
        }
    }
    return vol;
}

/// Make a degree-1 linear NURBS curve between two points.
static std::shared_ptr<geo::NurbsCurve> makeLineCurve(const Vec3& a, const Vec3& b) {
    return std::make_shared<geo::NurbsCurve>(std::vector<Vec3>{a, b}, std::vector<double>{1.0, 1.0},
                                             std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
}

/// Assign a line curve to an edge based on half-edge endpoint positions.
static void assignEdgeCurve(Edge* edge) {
    assert(edge->halfEdge != nullptr);
    HalfEdge* he = edge->halfEdge;
    const Vec3& a = he->origin->point;
    const Vec3& b = he->twin->origin->point;
    edge->curve = makeLineCurve(a, b);
}

// ---------------------------------------------------------------------------
// Fillet geometry computation
// ---------------------------------------------------------------------------

/// One cross-section of a fillet: the rolling-ball state at parameter t.
struct FilletStop {
    double t = 0.0;  ///< Normalized position along v1→v2.
    double r = 0.0;  ///< Fillet radius at this stop.
    Vec3 posA;       ///< Tangent point on faceA.
    Vec3 posB;       ///< Tangent point on faceB.
    Vec3 arcCenter;  ///< Rolling-ball center.
};

/// Data describing one edge to be filleted.
struct FilletEdgeInfo {
    const Edge* originalEdge = nullptr;
    Vertex* v1 = nullptr;   ///< First endpoint (t = 0).
    Vertex* v2 = nullptr;   ///< Second endpoint (t = 1).
    Face* faceA = nullptr;  ///< Left face.
    Face* faceB = nullptr;  ///< Right face.
    Vec3 edgeDir;           ///< Unit direction v1→v2.
    double edgeLen = 0.0;

    // Offset directions (inward along each face, perpendicular to edge).
    Vec3 offsetA;  ///< Direction on faceA from the edge inward.
    Vec3 offsetB;  ///< Direction on faceB from the edge inward.

    std::vector<FilletStop> stops;  ///< ≥ 2, increasing t; ends define topology.

    double maxRadius() const {
        double r = 0.0;
        for (const auto& s : stops) r = std::max(r, s.r);
        return r;
    }
    const FilletStop& front() const { return stops.front(); }
    const FilletStop& back() const { return stops.back(); }
};

/// Compute the frame of a fillet edge: adjacent faces, direction, and inward
/// offset directions. Radius-independent. @p outwardSign is +1 when the
/// solid's loop Newell normals point outward, -1 when inward (from
/// signedLoopVolume).
static bool computeFilletFrame(const Edge* edge, double outwardSign, FilletEdgeInfo& info) {
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
    info.edgeLen = dir.length();
    if (info.edgeLen < 1e-12) {
        return false;
    }
    info.edgeDir = dir * (1.0 / info.edgeLen);

    // Loop-winding (Newell) normals — sign-consistent with traversal order.
    Vec3 nA = newellNormal(info.faceA);
    Vec3 nB = newellNormal(info.faceB);

    // Inward offset on each face: walking the loop with the Newell normal up,
    // the interior lies to the LEFT — an identity of the winding, so no
    // centroid heuristic (which mis-picks the side on non-convex faces).
    // faceA owns the half-edge that traverses v1→v2; faceB traverses v2→v1.
    Vec3 candidateA = nA.cross(info.edgeDir);
    Vec3 candidateB = nB.cross(info.edgeDir * (-1.0));

    double lenA = candidateA.length();
    double lenB = candidateB.length();
    if (lenA < 1e-12 || lenB < 1e-12) {
        return false;
    }
    info.offsetA = candidateA * (1.0 / lenA);
    info.offsetB = candidateB * (1.0 / lenB);

    // Only perpendicular, CONVEX dihedral edges are supported: every formula
    // below (tangent offset = r, arc center at offsetA+offsetB, cos 45° arc
    // weight) is exact only there. At a convex right angle each inward offset
    // is exactly anti-parallel to the other face's OUTWARD normal; anything
    // else (oblique dihedrals, reentrant edges, curved faces) must be refused
    // rather than silently emitting wrong geometry.
    const Vec3 outwardB = nB * outwardSign;
    const Vec3 outwardA = nA * outwardSign;
    if (info.offsetA.dot(outwardB) > -1.0 + 1e-6 || info.offsetB.dot(outwardA) > -1.0 + 1e-6) {
        return false;
    }
    return true;
}

/// Materialize a stop: positions of the rolling ball at parameter t.
static FilletStop makeStop(const FilletEdgeInfo& info, double t, double r) {
    FilletStop s;
    s.t = t;
    s.r = r;
    const Vec3 p = info.v1->point + info.edgeDir * (t * info.edgeLen);
    s.posA = p + info.offsetA * r;
    s.posB = p + info.offsetB * r;
    s.arcCenter = p + (info.offsetA + info.offsetB) * r;
    return s;
}

/// Ruled NURBS surface between two quarter arcs (the fillet cross-sections at
/// consecutive stops). Equal radii produce an exact cylindrical patch;
/// different radii an exact conical blend. The patch spans exactly the face
/// extent (unlike a full-cylinder carrier).
static std::shared_ptr<geo::NurbsSurface> makeArcLoft(const FilletEdgeInfo& info,
                                                      const FilletStop& s0, const FilletStop& s1) {
    const double w = std::cos(hz::math::kPi / 4.0);  // 90° arc middle weight

    // Arc control points at a stop: from the faceA tangent to the faceB
    // tangent, bulging toward the edge. Directions from the arc center:
    // toward posA is -offsetB, toward posB is -offsetA (perpendicular pair).
    auto arcRow = [&](const FilletStop& s) {
        const Vec3 dirA = info.offsetB * (-1.0);
        const Vec3 dirB = info.offsetA * (-1.0);
        return std::vector<Vec3>{s.arcCenter + dirA * s.r, s.arcCenter + (dirA + dirB) * s.r,
                                 s.arcCenter + dirB * s.r};
    };

    std::vector<std::vector<Vec3>> cps{arcRow(s0), arcRow(s1)};
    std::vector<std::vector<double>> weights{{1.0, w, 1.0}, {1.0, w, 1.0}};
    std::vector<double> knotsU{0.0, 0.0, 1.0, 1.0};
    std::vector<double> knotsV{0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    return std::make_shared<geo::NurbsSurface>(std::move(cps), std::move(weights),
                                               std::move(knotsU), std::move(knotsV), 1, 2);
}

// ---------------------------------------------------------------------------
// Corner blends (Phase 61): three filleted edges meeting at a vertex
// ---------------------------------------------------------------------------

struct CornerBlend {
    const Vertex* vertex = nullptr;
    std::vector<size_t> edgeIndices;  ///< Indices into filletEdges (size 3).
    double radius = 0.0;
    Vec3 sphereCenter;
    std::vector<Vec3> cornerPoints;  ///< The three blended tangent points.
};

/// Direction of edge fe pointing AWAY from vertex v (into the solid).
static Vec3 dirFromVertex(const FilletEdgeInfo& fe, const Vertex* v) {
    return (fe.v1 == v) ? fe.edgeDir : fe.edgeDir * (-1.0);
}

/// Radius of the fillet at the end that touches vertex v.
static double radiusAtVertex(const FilletEdgeInfo& fe, const Vertex* v) {
    return (fe.v1 == v) ? fe.front().r : fe.back().r;
}

/// Trim the fillet's end at vertex v back by @p trim (edge-length units),
/// keeping the end radius constant over the trimmed span.
static void trimAtVertex(FilletEdgeInfo& fe, const Vertex* v, double trim) {
    const double dt = trim / fe.edgeLen;
    if (fe.v1 == v) {
        const double r = fe.front().r;
        const double t0 = dt;
        auto& stops = fe.stops;
        stops.erase(std::remove_if(stops.begin(), stops.end(),
                                   [t0](const FilletStop& s) { return s.t < t0 + 1e-12; }),
                    stops.end());
        stops.insert(stops.begin(), makeStop(fe, t0, r));
    } else {
        const double r = fe.back().r;
        const double t1 = 1.0 - dt;
        auto& stops = fe.stops;
        stops.erase(std::remove_if(stops.begin(), stops.end(),
                                   [t1](const FilletStop& s) { return s.t > t1 - 1e-12; }),
                    stops.end());
        stops.push_back(makeStop(fe, t1, r));
    }
}

/// Validate and build a corner blend for three fillets sharing vertex v.
/// On success the three fillets are trimmed and @p blend is filled.
static bool buildCornerBlend(std::vector<FilletEdgeInfo>& filletEdges,
                             const std::vector<size_t>& indices, const Vertex* v,
                             CornerBlend& blend, std::string& error) {
    // Equal radii at the shared corner.
    const double r = radiusAtVertex(filletEdges[indices[0]], v);
    for (size_t k : indices) {
        if (std::abs(radiusAtVertex(filletEdges[k], v) - r) > 1e-9) {
            error = "Vertex blend requires equal fillet radii at the shared corner";
            return false;
        }
    }
    // Each pair of edges must share a face (a genuine solid corner).
    for (size_t i = 0; i < indices.size(); ++i) {
        for (size_t j = i + 1; j < indices.size(); ++j) {
            const auto& a = filletEdges[indices[i]];
            const auto& b = filletEdges[indices[j]];
            const bool share = a.faceA == b.faceA || a.faceA == b.faceB || a.faceB == b.faceA ||
                               a.faceB == b.faceB;
            if (!share) {
                error = "Vertex blend edges must pairwise share a face";
                return false;
            }
        }
    }
    // Trim feasibility: the retracted span must leave room for the rest.
    for (size_t k : indices) {
        const auto& fe = filletEdges[k];
        if (r >= fe.edgeLen * 0.5) {
            error = "Vertex blend radius too large for edge: " + fe.originalEdge->topoId.tag();
            return false;
        }
    }

    blend.vertex = v;
    blend.edgeIndices = indices;
    blend.radius = r;

    // Rolling-ball corner center: one radius along each edge into the solid.
    Vec3 center = v->point;
    for (size_t k : indices) {
        center = center + dirFromVertex(filletEdges[k], v) * r;
    }
    blend.sphereCenter = center;

    // Trim the fillets and collect the (pairwise coincident) tangent points.
    std::vector<Vec3> points;
    for (size_t k : indices) {
        auto& fe = filletEdges[k];
        trimAtVertex(fe, v, r);
        const FilletStop& end = (fe.v1 == v) ? fe.front() : fe.back();
        points.push_back(end.posA);
        points.push_back(end.posB);
    }
    for (const Vec3& p : points) {
        bool found = false;
        for (const Vec3& q : blend.cornerPoints) {
            if (p.distanceTo(q) < 1e-9) {
                found = true;
                break;
            }
        }
        if (!found) blend.cornerPoints.push_back(p);
    }
    if (blend.cornerPoints.size() != 3) {
        error = "Vertex blend corner did not resolve to three tangent points";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Core execution over prepared fillet edges
// ---------------------------------------------------------------------------

static FilletResult executeCore(const Solid& inputSolid, std::vector<FilletEdgeInfo>& filletEdges,
                                const std::string& featureID) {
    FilletResult result;

    // -- Shared-vertex analysis: three edges → corner blend, two → refuse --
    std::map<const Vertex*, std::vector<size_t>> vertexEdges;
    for (size_t i = 0; i < filletEdges.size(); ++i) {
        vertexEdges[filletEdges[i].v1].push_back(i);
        vertexEdges[filletEdges[i].v2].push_back(i);
    }
    std::vector<CornerBlend> blends;
    for (const auto& [v, indices] : vertexEdges) {
        if (indices.size() == 1) continue;
        if (indices.size() == 3) {
            CornerBlend blend;
            if (!buildCornerBlend(filletEdges, indices, v, blend, result.errorMessage)) {
                return result;
            }
            blends.push_back(std::move(blend));
            continue;
        }
        result.errorMessage =
            "Vertex blend not supported: " + std::to_string(indices.size()) +
            " selected edges share a vertex (exactly three are required for a corner blend)";
        return result;
    }

    // -- Validate radius against face dimensions --
    for (const auto& fe : filletEdges) {
        auto minEdgeLen = [](const std::vector<Vertex*>& verts) -> double {
            double minLen = 1e30;
            for (size_t i = 0; i < verts.size(); ++i) {
                size_t j = (i + 1) % verts.size();
                double len = verts[i]->point.distanceTo(verts[j]->point);
                if (len < minLen) {
                    minLen = len;
                }
            }
            return minLen;
        };

        double minA = minEdgeLen(faceVertices(fe.faceA));
        double minB = minEdgeLen(faceVertices(fe.faceB));
        double limit = std::min(minA, minB) * 0.5;
        if (fe.maxRadius() > limit + 1e-9) {
            result.errorMessage =
                "Fillet radius too large for edge: " + fe.originalEdge->topoId.tag();
            return result;
        }
    }

    // -- Build the new solid --
    //
    // Face loops are emitted as consistently oriented vertex-position rings
    // (walking the original solid's half-edge loops keeps original faces
    // consistent; fillet/blend faces are oriented to oppose them), then the
    // half-edge structure is assembled directly with strict twin linking —
    // no heuristics, and Solid::isValid() gates the result.

    // Map from original edge id to fillet info.
    std::map<uint32_t, size_t> edgeToFillet;  // edge.id -> index in filletEdges
    for (size_t i = 0; i < filletEdges.size(); ++i) {
        edgeToFillet[filletEdges[i].originalEdge->id] = i;
    }

    // Map from unblended fillet endpoints to their fillet: these vertices get
    // chord-cut on the end face (the face at the vertex that is not adjacent
    // to the filleted edge).
    std::map<const Vertex*, size_t> endVertexToFillet;
    for (size_t i = 0; i < filletEdges.size(); ++i) {
        const auto& fe = filletEdges[i];
        auto isBlendedAt = [&](const Vertex* v) {
            return std::any_of(blends.begin(), blends.end(),
                               [v](const CornerBlend& b) { return b.vertex == v; });
        };
        if (!isBlendedAt(fe.v1)) {
            if (endVertexToFillet.count(fe.v1) != 0) {
                result.errorMessage = "Vertex blend not supported: 2 selected edges share a vertex";
                return result;
            }
            endVertexToFillet[fe.v1] = i;
        }
        if (!isBlendedAt(fe.v2)) {
            if (endVertexToFillet.count(fe.v2) != 0) {
                result.errorMessage = "Vertex blend not supported: 2 selected edges share a vertex";
                return result;
            }
            endVertexToFillet[fe.v2] = i;
        }
    }

    struct NewFaceData {
        std::vector<Vec3> vertices;
        TopologyID topoId;
        bool isOriginal = true;
        std::shared_ptr<geo::NurbsSurface> surface;  ///< Prebuilt (fillet/blend faces).
    };

    std::vector<NewFaceData> newFaces;

    // The tangent point of fillet fe at endpoint v on original face F.
    auto tangentOn = [&](const FilletEdgeInfo& fe, const Vertex* v, const Face* f) -> Vec3 {
        const FilletStop& end = (fe.v1 == v) ? fe.front() : fe.back();
        return (f == fe.faceA) ? end.posA : end.posB;
    };

    // Process each original face. Its surface carrier travels with it —
    // curved faces (including fillet patches from a previous FilletOp call)
    // must not be re-bound as planes.
    for (const auto& face : inputSolid.faces()) {
        NewFaceData fd;
        fd.topoId = face.topoId;
        fd.isOriginal = true;
        fd.surface = face.surface;

        HalfEdge* start = face.outerLoop->halfEdge;
        HalfEdge* cur = start;
        do {
            Edge* edge = cur->edge;
            auto fitIt = edgeToFillet.find(edge->id);

            if (fitIt != edgeToFillet.end()) {
                const auto& fe = filletEdges[fitIt->second];
                const bool forward = (cur->origin == fe.v1);
                const bool isFaceA = (cur->face == fe.faceA);

                // Emit the tangent chain for this side, in traversal order.
                if (forward) {
                    for (const auto& s : fe.stops) {
                        fd.vertices.push_back(isFaceA ? s.posA : s.posB);
                    }
                } else {
                    for (auto it = fe.stops.rbegin(); it != fe.stops.rend(); ++it) {
                        fd.vertices.push_back(isFaceA ? it->posA : it->posB);
                    }
                }
            } else {
                const Vertex* v = cur->origin;

                // Corner-blend vertices vanish — the adjacent chains meet.
                bool blended = false;
                for (const auto& b : blends) {
                    if (v == b.vertex) {
                        blended = true;
                        break;
                    }
                }
                if (blended) {
                    cur = cur->next;
                    continue;
                }

                // Fillet endpoints never survive: on the fillet's own faces
                // the tangent chain already replaced them; on the end face
                // (the third face at the vertex) the corner is chord-cut with
                // the two tangent points, ordered to match the neighbouring
                // faces: arrive on the side shared with the previous face,
                // leave on the side shared with the next.
                auto endIt = endVertexToFillet.find(v);
                if (endIt != endVertexToFillet.end()) {
                    const auto& fe = filletEdges[endIt->second];
                    if (cur->face == fe.faceA || cur->face == fe.faceB) {
                        cur = cur->next;  // chain covers this corner
                        continue;
                    }
                    const Face* arrivingFace =
                        cur->prev->twin != nullptr ? cur->prev->twin->face : nullptr;
                    const Face* leavingFace = cur->twin != nullptr ? cur->twin->face : nullptr;
                    if ((arrivingFace != fe.faceA && arrivingFace != fe.faceB) ||
                        (leavingFace != fe.faceA && leavingFace != fe.faceB)) {
                        result.errorMessage =
                            "Unsupported fillet-end configuration at vertex (non box-like corner)";
                        return result;
                    }
                    fd.vertices.push_back(tangentOn(fe, v, arrivingFace));
                    fd.vertices.push_back(tangentOn(fe, v, leavingFace));
                    cur = cur->next;
                    continue;
                }

                fd.vertices.push_back(v->point);
            }
            cur = cur->next;
        } while (cur != start);

        newFaces.push_back(std::move(fd));
    }

    // Add fillet faces: one ruled patch per stop segment, oriented to oppose
    // the adjacent face chains (faceA traverses stops forward, faceB
    // backward).
    for (size_t i = 0; i < filletEdges.size(); ++i) {
        const auto& fe = filletEdges[i];
        for (size_t s = 0; s + 1 < fe.stops.size(); ++s) {
            NewFaceData fd;
            fd.isOriginal = false;
            fd.topoId = TopologyID::make(featureID, "fillet")
                            .child(fe.originalEdge->topoId.tag(), static_cast<int>(s));
            fd.vertices.push_back(fe.stops[s + 1].posA);
            fd.vertices.push_back(fe.stops[s].posA);
            fd.vertices.push_back(fe.stops[s].posB);
            fd.vertices.push_back(fe.stops[s + 1].posB);
            fd.surface = makeArcLoft(fe, fe.stops[s], fe.stops[s + 1]);
            newFaces.push_back(std::move(fd));
        }
    }

    // Add spherical corner-blend faces. Orientation: the three fillet end
    // arcs, as traversed by their quads, form a directed 3-cycle around the
    // corner; the sphere loop is that cycle reversed.
    for (size_t bi = 0; bi < blends.size(); ++bi) {
        const auto& b = blends[bi];

        auto keyOf = [&](const Vec3& p) -> int {
            for (size_t k = 0; k < b.cornerPoints.size(); ++k) {
                if (p.distanceTo(b.cornerPoints[k]) < 1e-9) return static_cast<int>(k);
            }
            return -1;
        };

        // Directed end arcs at this corner, as the quads traverse them.
        // Quad loop [A_next, A_end, B_end, B_next] contains A_end → B_end at
        // the corner end when the corner is at the front (v1) stop, and
        // B_end → A_end when at the back (v2) stop.
        std::map<int, int> arcNext;
        bool arcsOk = true;
        for (size_t k : b.edgeIndices) {
            const auto& fe = filletEdges[k];
            const bool atFront = (fe.v1 == b.vertex);
            const FilletStop& end = atFront ? fe.front() : fe.back();
            const int a = keyOf(end.posA);
            const int c = keyOf(end.posB);
            if (a < 0 || c < 0) {
                arcsOk = false;
                break;
            }
            if (atFront) {
                arcNext[a] = c;  // quad traverses A_end → B_end
            } else {
                arcNext[c] = a;  // quad traverses B_end → A_end
            }
        }
        if (!arcsOk || arcNext.size() != 3) {
            result.errorMessage = "Vertex blend arcs do not form a corner cycle";
            return result;
        }

        // Walk the cycle and reverse it for the sphere loop.
        std::vector<int> cycle;
        int at = arcNext.begin()->first;
        for (int step = 0; step < 3; ++step) {
            cycle.push_back(at);
            auto it = arcNext.find(at);
            if (it == arcNext.end()) {
                arcsOk = false;
                break;
            }
            at = it->second;
        }
        if (!arcsOk || at != cycle.front()) {
            result.errorMessage = "Vertex blend arcs do not close into a corner cycle";
            return result;
        }

        NewFaceData fd;
        fd.isOriginal = false;
        fd.topoId = TopologyID::make(featureID, "blend").child("corner", static_cast<int>(bi));
        for (auto it = cycle.rbegin(); it != cycle.rend(); ++it) {
            fd.vertices.push_back(b.cornerPoints[static_cast<size_t>(*it)]);
        }
        fd.surface = std::make_shared<geo::NurbsSurface>(
            geo::NurbsSurface::makeSphere(b.sphereCenter, b.radius));
        newFaces.push_back(std::move(fd));
    }

    // -- Deduplicate positions into indexed loops --
    std::vector<Vec3> uniquePositions;
    std::map<size_t, std::vector<int>> faceVertexIndices;

    auto findOrAddVertex = [&](const Vec3& pos) -> int {
        for (size_t i = 0; i < uniquePositions.size(); ++i) {
            if (pos.distanceTo(uniquePositions[i]) < 1e-9) {
                return static_cast<int>(i);
            }
        }
        uniquePositions.push_back(pos);
        return static_cast<int>(uniquePositions.size() - 1);
    };

    for (size_t fi = 0; fi < newFaces.size(); ++fi) {
        std::vector<int> indices;
        for (const auto& pos : newFaces[fi].vertices) {
            indices.push_back(findOrAddVertex(pos));
        }
        // Corner-blend chains meet at shared tangent points: drop consecutive
        // duplicates (and the wraparound duplicate) so no degenerate edges
        // enter the loop.
        std::vector<int> cleaned;
        for (int idx : indices) {
            if (cleaned.empty() || cleaned.back() != idx) {
                cleaned.push_back(idx);
            }
        }
        while (cleaned.size() > 1 && cleaned.front() == cleaned.back()) {
            cleaned.pop_back();
        }
        if (cleaned.size() < 3) {
            result.errorMessage = "Degenerate face loop after fillet";
            return result;
        }
        faceVertexIndices[fi] = std::move(cleaned);
    }

    const int numVerts = static_cast<int>(uniquePositions.size());
    if (numVerts < 4 || newFaces.size() < 4) {
        result.errorMessage = "Degenerate solid after fillet (too few vertices or faces)";
        return result;
    }

    // -- Assemble the half-edge structure directly --
    // Each face loop spawns its half-edges; twins link by strict (a, b) /
    // (b, a) pairing. Any unmatched or doubly-used directed pair means the
    // emitted loops are not a closed 2-manifold — fail cleanly.
    auto solid = std::make_unique<Solid>();
    Shell* shell = solid->allocShell();
    shell->solid = solid.get();

    std::vector<Vertex*> verts(static_cast<size_t>(numVerts), nullptr);
    for (int i = 0; i < numVerts; ++i) {
        Vertex* v = solid->allocVertex();
        v->point = uniquePositions[static_cast<size_t>(i)];
        verts[static_cast<size_t>(i)] = v;
    }

    std::map<std::pair<int, int>, HalfEdge*> directedEdges;
    std::vector<Face*> builtFaces;

    for (size_t fi = 0; fi < newFaces.size(); ++fi) {
        const auto& indices = faceVertexIndices[fi];
        const size_t n = indices.size();

        Face* face = solid->allocFace();
        face->shell = shell;
        face->topoId = newFaces[fi].topoId;
        shell->faces.push_back(face);
        builtFaces.push_back(face);

        Wire* wire = solid->allocWire();
        std::vector<HalfEdge*> hes(n, nullptr);
        for (size_t k = 0; k < n; ++k) {
            HalfEdge* he = solid->allocHalfEdge();
            he->origin = verts[static_cast<size_t>(indices[k])];
            he->face = face;
            hes[k] = he;

            const std::pair<int, int> key{indices[k], indices[(k + 1) % n]};
            if (directedEdges.count(key) != 0) {
                result.errorMessage = "Fillet produced a non-manifold face loop";
                return result;
            }
            directedEdges[key] = he;
        }
        for (size_t k = 0; k < n; ++k) {
            hes[k]->next = hes[(k + 1) % n];
            hes[k]->prev = hes[(k + n - 1) % n];
            if (hes[k]->origin->halfEdge == nullptr) {
                hes[k]->origin->halfEdge = hes[k];
            }
        }
        wire->halfEdge = hes.front();
        face->outerLoop = wire;
    }

    // Twin-link and create one Edge per undirected pair.
    for (auto& [key, he] : directedEdges) {
        if (he->twin != nullptr) continue;
        auto opposite = directedEdges.find({key.second, key.first});
        if (opposite == directedEdges.end()) {
            result.errorMessage = "Fillet left an open boundary (unmatched edge " +
                                  std::to_string(key.first) + "-" + std::to_string(key.second) +
                                  ")";
            return result;
        }
        HalfEdge* twin = opposite->second;
        he->twin = twin;
        twin->twin = he;

        Edge* edge = solid->allocEdge();
        edge->halfEdge = he;
        he->edge = edge;
        twin->edge = edge;
    }

    if (!solid->isValid()) {
        result.errorMessage = "Fillet produced invalid topology:\n" + solid->validationReport();
        return result;
    }

    // -- Assign edge TopologyIDs and curves --
    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make(featureID, "edge" + std::to_string(idx));
            ++idx;
            assignEdgeCurve(&e);
        }
    }

    // -- Bind NURBS surfaces (faces map 1:1 to the emitted loops) --
    for (size_t fi = 0; fi < builtFaces.size(); ++fi) {
        Face* f = builtFaces[fi];
        if (newFaces[fi].surface != nullptr) {
            f->surface = newFaces[fi].surface;
            continue;
        }

        auto fv = faceVertices(f);
        if (fv.size() < 3) {
            continue;
        }
        // Bind planar NURBS surface computed from the vertex loop.
        Vec3 origin = fv[0]->point;
        Vec3 u = fv[1]->point - fv[0]->point;
        Vec3 v_dir(0, 0, 0);
        for (size_t i = 2; i < fv.size(); ++i) {
            v_dir = fv[i]->point - fv[0]->point;
            Vec3 cross = u.cross(v_dir);
            if (cross.length() > 1e-9) {
                break;
            }
        }
        double uSize = u.length();
        double vSize = v_dir.length();
        if (uSize < 1e-12) {
            uSize = 1.0;
        }
        if (vSize < 1e-12) {
            vSize = 1.0;
        }
        Vec3 uDir = u * (1.0 / uSize);
        Vec3 vDir = v_dir * (1.0 / vSize);

        // Align the carrier normal (u × v) with the loop's outward Newell
        // normal so planar faces keep the kernel's surface-normal-points-out
        // convention regardless of where the loop starts.
        Vec3 newell(0, 0, 0);
        for (size_t i = 0; i < fv.size(); ++i) {
            const Vec3& a = fv[i]->point;
            const Vec3& b = fv[(i + 1) % fv.size()]->point;
            newell.x += (a.y - b.y) * (a.z + b.z);
            newell.y += (a.z - b.z) * (a.x + b.x);
            newell.z += (a.x - b.x) * (a.y + b.y);
        }
        if (uDir.cross(vDir).dot(newell) < 0.0) {
            std::swap(uDir, vDir);
            std::swap(uSize, vSize);
        }

        f->surface = std::make_shared<geo::NurbsSurface>(
            geo::NurbsSurface::makePlane(origin, uDir, vDir, uSize, vSize));
    }

    result.solid = std::move(solid);
    return result;
}

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

FilletResult FilletOp::execute(const Solid& inputSolid, const std::vector<TopologyID>& edgeIds,
                               double radius, const std::string& featureID) {
    FilletResult result;

    if (radius <= 0.0) {
        result.errorMessage = "Fillet radius must be positive";
        return result;
    }
    if (edgeIds.empty()) {
        result.errorMessage = "No edges specified for fillet";
        return result;
    }

    const double outwardSign = signedLoopVolume(inputSolid) >= 0.0 ? 1.0 : -1.0;

    std::vector<FilletEdgeInfo> filletEdges;
    filletEdges.reserve(edgeIds.size());
    for (const auto& eid : edgeIds) {
        const Edge* edge = findEdge(inputSolid, eid);
        if (!edge) {
            result.errorMessage = "Edge not found: " + eid.tag();
            return result;
        }
        FilletEdgeInfo info;
        if (!computeFilletFrame(edge, outwardSign, info)) {
            result.errorMessage = "Cannot compute fillet geometry for edge: " + eid.tag();
            return result;
        }
        info.stops.push_back(makeStop(info, 0.0, radius));
        info.stops.push_back(makeStop(info, 1.0, radius));
        filletEdges.push_back(std::move(info));
    }

    return executeCore(inputSolid, filletEdges, featureID);
}

FilletResult FilletOp::executeVariable(const Solid& inputSolid, const TopologyID& edgeId,
                                       const std::vector<RadiusStop>& stops,
                                       const std::string& featureID) {
    FilletResult result;

    if (stops.size() < 2) {
        result.errorMessage = "Variable fillet needs at least two radius stops";
        return result;
    }
    for (size_t i = 0; i < stops.size(); ++i) {
        if (stops[i].radius <= 0.0) {
            result.errorMessage = "Fillet radius must be positive";
            return result;
        }
        if (i > 0 && stops[i].t <= stops[i - 1].t + 1e-12) {
            result.errorMessage = "Radius stops must have strictly increasing parameters";
            return result;
        }
    }
    if (std::abs(stops.front().t) > 1e-9 || std::abs(stops.back().t - 1.0) > 1e-9) {
        result.errorMessage = "Radius stops must cover t = 0 and t = 1";
        return result;
    }

    const Edge* edge = findEdge(inputSolid, edgeId);
    if (!edge) {
        result.errorMessage = "Edge not found: " + edgeId.tag();
        return result;
    }
    FilletEdgeInfo info;
    const double outwardSign = signedLoopVolume(inputSolid) >= 0.0 ? 1.0 : -1.0;
    if (!computeFilletFrame(edge, outwardSign, info)) {
        result.errorMessage = "Cannot compute fillet geometry for edge: " + edgeId.tag();
        return result;
    }
    for (const auto& s : stops) {
        info.stops.push_back(makeStop(info, s.t, s.radius));
    }

    std::vector<FilletEdgeInfo> filletEdges;
    filletEdges.push_back(std::move(info));
    return executeCore(inputSolid, filletEdges, featureID);
}

}  // namespace hz::model
