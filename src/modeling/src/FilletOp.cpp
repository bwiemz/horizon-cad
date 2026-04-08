#include "horizon/modeling/FilletOp.h"

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/topology/EulerOps.h"
#include "horizon/topology/Queries.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <set>

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

/// Compute face normal from the NURBS surface at its center parameter.
static Vec3 faceNormal(const Face* face) {
    if (face->surface) {
        double uMid = (face->surface->uMin() + face->surface->uMax()) * 0.5;
        double vMid = (face->surface->vMin() + face->surface->vMax()) * 0.5;
        return face->surface->normal(uMid, vMid);
    }
    // Fallback: compute from vertex positions.
    auto verts = faceVertices(face);
    if (verts.size() >= 3) {
        Vec3 a = verts[1]->point - verts[0]->point;
        Vec3 b = verts[2]->point - verts[0]->point;
        Vec3 n = a.cross(b);
        double len = n.length();
        if (len > 1e-12) {
            return n * (1.0 / len);
        }
    }
    return Vec3(0, 0, 1);
}

/// Make a degree-1 linear NURBS curve between two points.
static std::shared_ptr<geo::NurbsCurve> makeLineCurve(const Vec3& a, const Vec3& b) {
    return std::make_shared<geo::NurbsCurve>(
        std::vector<Vec3>{a, b}, std::vector<double>{1.0, 1.0},
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

/// Find the half-edge on a face originating from a given vertex.
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
            fallback = cur;
        }
        cur = cur->next;
    } while (cur != start);
    return fallback;
}

// ---------------------------------------------------------------------------
// Fillet geometry computation for a single edge
// ---------------------------------------------------------------------------

/// Data describing one edge to be filleted.
struct FilletEdgeInfo {
    const Edge* originalEdge = nullptr;
    Vertex* v1 = nullptr;  ///< First endpoint.
    Vertex* v2 = nullptr;  ///< Second endpoint.
    Face* faceA = nullptr;  ///< Left face.
    Face* faceB = nullptr;  ///< Right face.
    Vec3 edgeDir;           ///< Unit direction v1→v2.

    // Offset directions (inward along each face, perpendicular to edge).
    Vec3 offsetA;  ///< Direction on faceA from the edge inward.
    Vec3 offsetB;  ///< Direction on faceB from the edge inward.

    // Computed offset vertex positions (4 per filleted edge).
    Vec3 v1_offsetA;  ///< v1 offset along faceA.
    Vec3 v1_offsetB;  ///< v1 offset along faceB.
    Vec3 v2_offsetA;  ///< v2 offset along faceA.
    Vec3 v2_offsetB;  ///< v2 offset along faceB.

    // Cylinder surface parameters for the fillet.
    Vec3 cylCenter1;  ///< Center of fillet arc at v1.
    Vec3 cylCenter2;  ///< Center of fillet arc at v2.
    Vec3 cylAxis;     ///< Cylinder axis direction (along the edge).
};

/// Compute fillet geometry for a single edge.
static bool computeFilletGeometry(const Edge* edge, double radius, FilletEdgeInfo& info) {
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

    // Get face normals.
    Vec3 nA = faceNormal(info.faceA);
    Vec3 nB = faceNormal(info.faceB);

    // Compute the inward offset direction on each face: perpendicular to the edge,
    // in the face plane, pointing away from the edge (into the face interior).
    // offsetA = edgeDir x nA  (or -edgeDir x nA, whichever points inward).
    Vec3 candidateA = info.edgeDir.cross(nA);
    Vec3 candidateB = info.edgeDir.cross(nB);

    // Normalize offset directions.
    double lenA = candidateA.length();
    double lenB = candidateB.length();
    if (lenA < 1e-12 || lenB < 1e-12) {
        return false;
    }
    candidateA = candidateA * (1.0 / lenA);
    candidateB = candidateB * (1.0 / lenB);

    // Ensure offset directions point inward by checking against face centroid.
    auto vertsA = faceVertices(info.faceA);
    Vec3 centroidA(0, 0, 0);
    for (const auto* v : vertsA) {
        centroidA = centroidA + v->point;
    }
    if (!vertsA.empty()) {
        centroidA = centroidA * (1.0 / static_cast<double>(vertsA.size()));
    }
    Vec3 edgeMid = (info.v1->point + info.v2->point) * 0.5;
    if (candidateA.dot(centroidA - edgeMid) < 0) {
        candidateA = candidateA * (-1.0);
    }

    auto vertsB = faceVertices(info.faceB);
    Vec3 centroidB(0, 0, 0);
    for (const auto* v : vertsB) {
        centroidB = centroidB + v->point;
    }
    if (!vertsB.empty()) {
        centroidB = centroidB * (1.0 / static_cast<double>(vertsB.size()));
    }
    if (candidateB.dot(centroidB - edgeMid) < 0) {
        candidateB = candidateB * (-1.0);
    }

    info.offsetA = candidateA;
    info.offsetB = candidateB;

    // Compute the 4 offset vertex positions.
    info.v1_offsetA = info.v1->point + candidateA * radius;
    info.v1_offsetB = info.v1->point + candidateB * radius;
    info.v2_offsetA = info.v2->point + candidateA * radius;
    info.v2_offsetB = info.v2->point + candidateB * radius;

    // Cylinder arc centers: the point equidistant from both offset vertices at each endpoint.
    // For a 90-degree fillet at a box edge, the center is at v + offsetA*radius + offsetB*radius.
    info.cylCenter1 = info.v1->point + candidateA * radius + candidateB * radius;
    info.cylCenter2 = info.v2->point + candidateA * radius + candidateB * radius;
    info.cylAxis = info.edgeDir;

    return true;
}

// ---------------------------------------------------------------------------
// Build the filleted solid topology
// ---------------------------------------------------------------------------

/// Information for rebuilding the solid with fillet faces.
/// Maps original vertex positions to their (potentially split) new positions.
struct VertexMapping {
    /// For each original vertex index: list of new vertex positions.
    /// Usually 1 (unchanged), but 2 if the vertex is an endpoint of a filleted edge.
    std::map<uint32_t, std::vector<Vec3>> splits;
};

// ---------------------------------------------------------------------------
// FilletOp::execute
// ---------------------------------------------------------------------------

FilletResult FilletOp::execute(const Solid& inputSolid,
                               const std::vector<TopologyID>& edgeIds, double radius,
                               const std::string& featureID) {
    FilletResult result;

    // -- Validate inputs --
    if (radius <= 0.0) {
        result.errorMessage = "Fillet radius must be positive";
        return result;
    }
    if (edgeIds.empty()) {
        result.errorMessage = "No edges specified for fillet";
        return result;
    }

    // -- Resolve edges --
    std::vector<FilletEdgeInfo> filletEdges;
    filletEdges.reserve(edgeIds.size());

    for (const auto& eid : edgeIds) {
        const Edge* edge = findEdge(inputSolid, eid);
        if (!edge) {
            result.errorMessage = "Edge not found: " + eid.tag();
            return result;
        }
        FilletEdgeInfo info;
        if (!computeFilletGeometry(edge, radius, info)) {
            result.errorMessage = "Cannot compute fillet geometry for edge: " + eid.tag();
            return result;
        }
        filletEdges.push_back(info);
    }

    // -- Check for vertex blends (two selected edges sharing a vertex) --
    std::set<uint32_t> usedVertices;
    for (const auto& fe : filletEdges) {
        if (usedVertices.count(fe.v1->id) || usedVertices.count(fe.v2->id)) {
            result.errorMessage = "Vertex blend not supported: two selected edges share a vertex";
            return result;
        }
        usedVertices.insert(fe.v1->id);
        usedVertices.insert(fe.v2->id);
    }

    // -- Validate radius against face dimensions --
    for (const auto& fe : filletEdges) {
        // Check that radius is not too large for adjacent faces.
        auto vertsA = faceVertices(fe.faceA);
        auto vertsB = faceVertices(fe.faceB);

        // Find the shortest edge of each adjacent face as a conservative bound.
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

        double minA = minEdgeLen(vertsA);
        double minB = minEdgeLen(vertsB);
        double limit = std::min(minA, minB) * 0.5;
        if (radius > limit + 1e-9) {
            result.errorMessage = "Fillet radius too large for edge: " + fe.originalEdge->topoId.tag();
            return result;
        }
    }

    // -- Build the new solid --
    //
    // Strategy: collect all vertex positions for the new solid (original vertices
    // with filleted ones replaced by offset pairs), then rebuild topology using
    // the same Euler operator sequence as PrimitiveFactory, extended to handle
    // the extra vertices and faces from fillets.
    //
    // For simplicity in Phase 37, we support filleting edges of a box (8V, 12E, 6F).
    // Each filleted edge adds 2 new vertices, 3 new edges, and 1 new face.

    // Collect all original vertex positions, indexed by vertex id.
    std::map<uint32_t, Vec3> origPositions;
    for (const auto& v : inputSolid.vertices()) {
        origPositions[v.id] = v.point;
    }

    // Map from original edge id to fillet info.
    std::map<uint32_t, size_t> edgeToFillet;  // edge.id -> index in filletEdges
    for (size_t i = 0; i < filletEdges.size(); ++i) {
        edgeToFillet[filletEdges[i].originalEdge->id] = i;
    }

    // For each original face, collect its vertex loop (as positions).
    // For filleted edges, replace the two original vertices with offset pairs.
    struct NewFaceData {
        std::vector<Vec3> vertices;
        TopologyID topoId;
        bool isOriginal = true;  // true = original face, false = fillet face
    };

    std::vector<NewFaceData> newFaces;

    // Process each original face: for each edge of the face, check if it's filleted.
    // If so, replace the two vertices with offset positions.
    for (const auto& face : inputSolid.faces()) {
        auto verts = faceVertices(&face);
        NewFaceData fd;
        fd.topoId = face.topoId;
        fd.isOriginal = true;

        // Walk the face loop, checking each edge.
        HalfEdge* start = face.outerLoop->halfEdge;
        HalfEdge* cur = start;
        do {
            Edge* edge = cur->edge;
            auto fitIt = edgeToFillet.find(edge->id);

            if (fitIt != edgeToFillet.end()) {
                // This edge is being filleted.
                const auto& fe = filletEdges[fitIt->second];

                // Determine which direction we're traversing (v1→v2 or v2→v1).
                bool forward = (cur->origin == fe.v1);

                // On the current (original) face, we need to figure out which
                // face we are (faceA or faceB) to pick the correct offset vertices.
                bool isFaceA = (cur->face == fe.faceA);

                if (forward) {
                    // Traversing v1→v2.  Replace v1 with the offset on this face,
                    // and the next vertex (v2) will be replaced at the next edge or here.
                    if (isFaceA) {
                        fd.vertices.push_back(fe.v1_offsetA);
                        fd.vertices.push_back(fe.v2_offsetA);
                    } else {
                        fd.vertices.push_back(fe.v1_offsetB);
                        fd.vertices.push_back(fe.v2_offsetB);
                    }
                } else {
                    // Traversing v2→v1.
                    if (isFaceA) {
                        fd.vertices.push_back(fe.v2_offsetA);
                        fd.vertices.push_back(fe.v1_offsetA);
                    } else {
                        fd.vertices.push_back(fe.v2_offsetB);
                        fd.vertices.push_back(fe.v1_offsetB);
                    }
                }
            } else {
                // Not filleted — keep original vertex.
                fd.vertices.push_back(cur->origin->point);
            }
            cur = cur->next;
        } while (cur != start);

        newFaces.push_back(std::move(fd));
    }

    // Add fillet faces (one per filleted edge).
    for (size_t i = 0; i < filletEdges.size(); ++i) {
        const auto& fe = filletEdges[i];
        NewFaceData fd;
        fd.isOriginal = false;
        fd.topoId =
            TopologyID::make(featureID, "fillet").child(fe.originalEdge->topoId.tag(), 0);

        // The fillet face is a quad: v1_offsetA → v2_offsetA → v2_offsetB → v1_offsetB.
        fd.vertices.push_back(fe.v1_offsetA);
        fd.vertices.push_back(fe.v2_offsetA);
        fd.vertices.push_back(fe.v2_offsetB);
        fd.vertices.push_back(fe.v1_offsetB);

        newFaces.push_back(std::move(fd));
    }

    // -- Build the new solid using Euler operators --
    // Collect unique vertex positions.
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
        faceVertexIndices[fi] = std::move(indices);
    }

    int numVerts = static_cast<int>(uniquePositions.size());
    int numFaces = static_cast<int>(newFaces.size());
    if (numVerts < 4 || numFaces < 4) {
        result.errorMessage = "Degenerate solid after fillet (too few vertices or faces)";
        return result;
    }

    // Build an adjacency structure: for each vertex, which faces use it.
    std::map<int, std::vector<size_t>> vertToFaces;
    for (size_t fi = 0; fi < newFaces.size(); ++fi) {
        for (int vi : faceVertexIndices[fi]) {
            vertToFaces[vi].push_back(fi);
        }
    }

    // Build the solid topology using Euler operators.
    auto solid = std::make_unique<Solid>();
    std::vector<Vertex*> verts(numVerts, nullptr);

    // MVFS: first vertex + outer face.
    auto [v0, fOuter, shell] = euler::makeVertexFaceSolid(*solid, uniquePositions[0]);
    verts[0] = v0;

    // MEV: create all vertices as a chain from v0 on fOuter.
    for (int i = 1; i < numVerts; ++i) {
        HalfEdge* heAt = findHE(fOuter, verts[i - 1]);
        auto [edge, newV] = euler::makeEdgeVertex(*solid, heAt, fOuter, uniquePositions[i]);
        verts[i] = newV;
    }

    // Track which vertex pairs have edges.
    std::set<std::pair<int, int>> existingEdges;
    for (int i = 0; i + 1 < numVerts; ++i) {
        existingEdges.insert({i, i + 1});
        existingEdges.insert({i + 1, i});
    }

    // Close chain: connect last vertex to first.
    {
        int last = numVerts - 1;
        HalfEdge* heA = findHE(fOuter, verts[last]);
        HalfEdge* heB = findHE(fOuter, verts[0]);
        if (heA && heB) {
            euler::makeEdgeFace(*solid, heA, heB);
            existingEdges.insert({last, 0});
            existingEdges.insert({0, last});
        }
    }

    // Now systematically close faces using MEF.
    // Strategy: process all faces except the last (which closes automatically).
    // For each face, find a diagonal (non-adjacent pair of vertices) that doesn't
    // have an edge yet and where both vertices are on the same topological face.
    // Repeat until all faces are closed.
    bool progress = true;
    int maxIter = numFaces * numVerts;  // Safety limit.
    while (progress && maxIter-- > 0) {
        progress = false;
        for (size_t fi = 0; fi + 1 < newFaces.size(); ++fi) {
            const auto& indices = faceVertexIndices[fi];
            int n = static_cast<int>(indices.size());

            for (int ei = 0; ei < n; ++ei) {
                int a = indices[ei];
                int b = indices[(ei + 1) % n];
                if (existingEdges.count({a, b})) continue;

                // Find a topological face containing both vertices.
                Face* targetFace = nullptr;
                for (auto& f : const_cast<std::deque<Face>&>(solid->faces())) {
                    auto fv = faceVertices(&f);
                    bool hasA = false, hasB = false;
                    for (auto* fvv : fv) {
                        if (fvv == verts[a]) hasA = true;
                        if (fvv == verts[b]) hasB = true;
                    }
                    if (hasA && hasB) {
                        targetFace = &f;
                        break;
                    }
                }
                if (!targetFace) continue;

                HalfEdge* heA = findHE(targetFace, verts[a]);
                HalfEdge* heB = findHE(targetFace, verts[b]);
                if (heA && heB) {
                    euler::makeEdgeFace(*solid, heA, heB);
                    existingEdges.insert({a, b});
                    existingEdges.insert({b, a});
                    progress = true;
                }
            }
        }
    }

    // -- Assign TopologyIDs --
    // First pass: match rebuilt faces to newFaces by checking if every vertex of
    // the newFace data set is present in the rebuilt face's vertex set.
    std::set<size_t> assignedNewFaces;
    for (auto& f : const_cast<std::deque<Face>&>(solid->faces())) {
        auto fv = faceVertices(&f);
        // Build a set of vertex positions for fast lookup.
        std::vector<Vec3> fvPositions;
        fvPositions.reserve(fv.size());
        for (auto* v : fv) {
            fvPositions.push_back(v->point);
        }

        auto containsPoint = [&](const Vec3& p) -> bool {
            for (const auto& q : fvPositions) {
                if (p.distanceTo(q) < 1e-6) return true;
            }
            return false;
        };

        // Find a newFace whose vertices are all present in this rebuilt face.
        double bestDist = 1e30;
        size_t bestFi = newFaces.size();
        for (size_t fi = 0; fi < newFaces.size(); ++fi) {
            if (assignedNewFaces.count(fi)) continue;
            const auto& nfVerts = newFaces[fi].vertices;
            bool allFound = true;
            for (const auto& p : nfVerts) {
                if (!containsPoint(p)) {
                    allFound = false;
                    break;
                }
            }
            if (allFound) {
                // Compute centroid distance for tie-breaking.
                Vec3 centroid(0, 0, 0);
                for (auto* v : fv) centroid = centroid + v->point;
                if (!fv.empty()) centroid = centroid * (1.0 / static_cast<double>(fv.size()));
                Vec3 nfCentroid(0, 0, 0);
                for (const auto& p : nfVerts) nfCentroid = nfCentroid + p;
                if (!nfVerts.empty())
                    nfCentroid = nfCentroid * (1.0 / static_cast<double>(nfVerts.size()));
                double d = centroid.distanceTo(nfCentroid);
                if (d < bestDist) {
                    bestDist = d;
                    bestFi = fi;
                }
            }
        }
        if (bestFi < newFaces.size()) {
            f.topoId = newFaces[bestFi].topoId;
            assignedNewFaces.insert(bestFi);
        }
    }

    // Second pass: any face still without a topoId gets a generic derived ID.
    {
        int unassigned = 0;
        for (auto& f : const_cast<std::deque<Face>&>(solid->faces())) {
            if (!f.topoId.isValid()) {
                f.topoId = TopologyID::make(featureID, "face" + std::to_string(unassigned));
                ++unassigned;
            }
        }
    }

    // Edges: assign derived TopologyIDs.
    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make(featureID, "edge" + std::to_string(idx));
            ++idx;
        }
    }

    // -- Assign edge curves --
    for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
        assignEdgeCurve(&e);
    }

    // -- Bind NURBS surfaces --
    for (auto& f : const_cast<std::deque<Face>&>(solid->faces())) {
        auto fv = faceVertices(&f);
        if (fv.size() < 3) {
            continue;
        }

        // Check if this face is a fillet face.
        bool isFillet = false;
        for (size_t fi = 0; fi < newFaces.size(); ++fi) {
            if (!newFaces[fi].isOriginal && f.topoId == newFaces[fi].topoId) {
                isFillet = true;
                break;
            }
        }

        if (isFillet) {
            // Bind cylindrical NURBS surface for the fillet face.
            // Find the corresponding fillet edge info.
            for (const auto& fe : filletEdges) {
                // Match by checking if face vertices are near the fillet offset positions.
                bool match = false;
                for (auto* v : fv) {
                    if (v->point.distanceTo(fe.v1_offsetA) < 1e-6 ||
                        v->point.distanceTo(fe.v1_offsetB) < 1e-6) {
                        match = true;
                        break;
                    }
                }
                if (match) {
                    // Compute cylinder parameters.
                    Vec3 center = (fe.cylCenter1 + fe.cylCenter2) * 0.5;
                    double height = fe.cylCenter1.distanceTo(fe.cylCenter2);
                    if (height < 1e-12) {
                        height = 1e-6;
                    }
                    f.surface = std::make_shared<geo::NurbsSurface>(
                        geo::NurbsSurface::makeCylinder(fe.cylCenter1, fe.cylAxis, radius,
                                                        height));
                    break;
                }
            }
        } else {
            // Bind planar NURBS surface.
            // Compute face plane from vertices.
            Vec3 origin = fv[0]->point;
            Vec3 u = fv[1]->point - fv[0]->point;
            Vec3 v_dir(0, 0, 0);
            // Find a non-collinear vertex for the v direction.
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

            f.surface = std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makePlane(origin, uDir, vDir, uSize, vSize));
        }
    }

    result.solid = std::move(solid);
    return result;
}

}  // namespace hz::model
