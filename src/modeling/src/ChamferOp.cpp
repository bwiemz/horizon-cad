#include "horizon/modeling/ChamferOp.h"

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
// Helpers (same as FilletOp — duplicated to keep each op self-contained)
// ---------------------------------------------------------------------------

static const Edge* findEdge(const Solid& solid, const TopologyID& id) {
    for (const auto& e : solid.edges()) {
        if (e.topoId == id) {
            return &e;
        }
    }
    return nullptr;
}

static Vec3 faceNormal(const Face* face) {
    if (face->surface) {
        double uMid = (face->surface->uMin() + face->surface->uMax()) * 0.5;
        double vMid = (face->surface->vMin() + face->surface->vMax()) * 0.5;
        return face->surface->normal(uMid, vMid);
    }
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

static std::shared_ptr<geo::NurbsCurve> makeLineCurve(const Vec3& a, const Vec3& b) {
    return std::make_shared<geo::NurbsCurve>(
        std::vector<Vec3>{a, b}, std::vector<double>{1.0, 1.0},
        std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
}

static void assignEdgeCurve(Edge* edge) {
    assert(edge->halfEdge != nullptr);
    HalfEdge* he = edge->halfEdge;
    const Vec3& a = he->origin->point;
    const Vec3& b = he->twin->origin->point;
    edge->curve = makeLineCurve(a, b);
}

static HalfEdge* findHE(Face* face, Vertex* origin, Vertex* /*prevOrigin*/ = nullptr) {
    if (face->outerLoop == nullptr || face->outerLoop->halfEdge == nullptr) {
        return nullptr;
    }
    HalfEdge* start = face->outerLoop->halfEdge;
    HalfEdge* cur = start;
    do {
        if (cur->origin == origin) {
            return cur;
        }
        cur = cur->next;
    } while (cur != start);
    return nullptr;
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
};

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

    Vec3 nA = faceNormal(info.faceA);
    Vec3 nB = faceNormal(info.faceB);

    Vec3 candidateA = info.edgeDir.cross(nA);
    Vec3 candidateB = info.edgeDir.cross(nB);

    double lenA = candidateA.length();
    double lenB = candidateB.length();
    if (lenA < 1e-12 || lenB < 1e-12) {
        return false;
    }
    candidateA = candidateA * (1.0 / lenA);
    candidateB = candidateB * (1.0 / lenB);

    // Ensure offset directions point inward.
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

    // Offset by possibly different distances on each face.
    info.v1_offsetA = info.v1->point + candidateA * distA;
    info.v1_offsetB = info.v1->point + candidateB * distB;
    info.v2_offsetA = info.v2->point + candidateA * distA;
    info.v2_offsetB = info.v2->point + candidateB * distB;

    return true;
}

// ---------------------------------------------------------------------------
// Core chamfer implementation shared by both variants
// ---------------------------------------------------------------------------

static ChamferResult chamferImpl(const Solid& inputSolid,
                                 const std::vector<TopologyID>& edgeIds, double distA,
                                 double distB, const std::string& featureID) {
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

    for (const auto& eid : edgeIds) {
        const Edge* edge = findEdge(inputSolid, eid);
        if (!edge) {
            result.errorMessage = "Edge not found: " + eid.tag();
            return result;
        }
        ChamferEdgeInfo info;
        if (!computeChamferGeometry(edge, distA, distB, info)) {
            result.errorMessage = "Cannot compute chamfer geometry for edge: " + eid.tag();
            return result;
        }
        chamferEdges.push_back(info);
    }

    // Check for vertex blends.
    std::set<uint32_t> usedVertices;
    for (const auto& ce : chamferEdges) {
        if (usedVertices.count(ce.v1->id) || usedVertices.count(ce.v2->id)) {
            result.errorMessage = "Vertex blend not supported: two selected edges share a vertex";
            return result;
        }
        usedVertices.insert(ce.v1->id);
        usedVertices.insert(ce.v2->id);
    }

    // Validate distances against face dimensions.
    for (const auto& ce : chamferEdges) {
        auto vertsA = faceVertices(ce.faceA);
        auto vertsB = faceVertices(ce.faceB);

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
        if (distA > minA * 0.5 + 1e-9 || distB > minB * 0.5 + 1e-9) {
            result.errorMessage =
                "Chamfer distance too large for edge: " + ce.originalEdge->topoId.tag();
            return result;
        }
    }

    // Map from original edge id to chamfer info index.
    std::map<uint32_t, size_t> edgeToChamfer;
    for (size_t i = 0; i < chamferEdges.size(); ++i) {
        edgeToChamfer[chamferEdges[i].originalEdge->id] = i;
    }

    // Build face data.
    struct NewFaceData {
        std::vector<Vec3> vertices;
        TopologyID topoId;
        bool isOriginal = true;
    };

    std::vector<NewFaceData> newFaces;

    for (const auto& face : inputSolid.faces()) {
        NewFaceData fd;
        fd.topoId = face.topoId;
        fd.isOriginal = true;

        HalfEdge* start = face.outerLoop->halfEdge;
        HalfEdge* cur = start;
        do {
            Edge* edge = cur->edge;
            auto fitIt = edgeToChamfer.find(edge->id);

            if (fitIt != edgeToChamfer.end()) {
                const auto& ce = chamferEdges[fitIt->second];
                bool forward = (cur->origin == ce.v1);
                bool isFaceA = (cur->face == ce.faceA);

                if (forward) {
                    if (isFaceA) {
                        fd.vertices.push_back(ce.v1_offsetA);
                        fd.vertices.push_back(ce.v2_offsetA);
                    } else {
                        fd.vertices.push_back(ce.v1_offsetB);
                        fd.vertices.push_back(ce.v2_offsetB);
                    }
                } else {
                    if (isFaceA) {
                        fd.vertices.push_back(ce.v2_offsetA);
                        fd.vertices.push_back(ce.v1_offsetA);
                    } else {
                        fd.vertices.push_back(ce.v2_offsetB);
                        fd.vertices.push_back(ce.v1_offsetB);
                    }
                }
            } else {
                fd.vertices.push_back(cur->origin->point);
            }
            cur = cur->next;
        } while (cur != start);

        newFaces.push_back(std::move(fd));
    }

    // Add chamfer faces.
    for (size_t i = 0; i < chamferEdges.size(); ++i) {
        const auto& ce = chamferEdges[i];
        NewFaceData fd;
        fd.isOriginal = false;
        fd.topoId =
            TopologyID::make(featureID, "chamfer").child(ce.originalEdge->topoId.tag(), 0);

        // The chamfer face is a quad: v1_offsetA → v2_offsetA → v2_offsetB → v1_offsetB.
        fd.vertices.push_back(ce.v1_offsetA);
        fd.vertices.push_back(ce.v2_offsetA);
        fd.vertices.push_back(ce.v2_offsetB);
        fd.vertices.push_back(ce.v1_offsetB);

        newFaces.push_back(std::move(fd));
    }

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

    // Build topology with Euler operators.
    auto solid = std::make_unique<Solid>();

    int numVerts = static_cast<int>(uniquePositions.size());
    if (numVerts < 4 || static_cast<int>(newFaces.size()) < 4) {
        result.errorMessage = "Degenerate solid after chamfer";
        return result;
    }

    std::vector<Vertex*> verts(numVerts, nullptr);

    auto [v0, fOuter, shell] = euler::makeVertexFaceSolid(*solid, uniquePositions[0]);
    verts[0] = v0;

    for (int i = 1; i < numVerts; ++i) {
        HalfEdge* heAt = findHE(fOuter, verts[i - 1]);
        auto [edge, newV] = euler::makeEdgeVertex(*solid, heAt, fOuter, uniquePositions[i]);
        verts[i] = newV;
    }

    std::set<std::pair<int, int>> existingEdges;
    for (int i = 0; i + 1 < numVerts; ++i) {
        existingEdges.insert({i, i + 1});
        existingEdges.insert({i + 1, i});
    }

    auto hasEdge = [&](int a, int b) -> bool {
        return existingEdges.count({a, b}) > 0;
    };

    for (size_t fi = 0; fi + 1 < newFaces.size(); ++fi) {
        const auto& indices = faceVertexIndices[fi];
        int n = static_cast<int>(indices.size());

        for (int ei = 0; ei < n; ++ei) {
            int a = indices[ei];
            int b = indices[(ei + 1) % n];
            if (!hasEdge(a, b)) {
                Face* targetFace = nullptr;
                for (auto& f : const_cast<std::deque<Face>&>(solid->faces())) {
                    auto fv = faceVertices(&f);
                    bool hasA = false;
                    bool hasB = false;
                    for (auto* fvv : fv) {
                        if (fvv == verts[a]) {
                            hasA = true;
                        }
                        if (fvv == verts[b]) {
                            hasB = true;
                        }
                    }
                    if (hasA && hasB) {
                        targetFace = &f;
                        break;
                    }
                }
                if (!targetFace) {
                    continue;
                }

                HalfEdge* heA = findHE(targetFace, verts[a]);
                HalfEdge* heB = findHE(targetFace, verts[b]);
                if (heA && heB) {
                    euler::makeEdgeFace(*solid, heA, heB);
                    existingEdges.insert({a, b});
                    existingEdges.insert({b, a});
                }
            }
        }
    }

    // Assign TopologyIDs to faces (match by vertex positions).
    for (auto& f : const_cast<std::deque<Face>&>(solid->faces())) {
        auto fv = faceVertices(&f);
        for (size_t fi = 0; fi < newFaces.size(); ++fi) {
            const auto& nfVerts = newFaces[fi].vertices;
            if (fv.size() != nfVerts.size()) {
                continue;
            }
            bool matched = false;
            for (size_t start = 0; start < nfVerts.size(); ++start) {
                bool ok = true;
                for (size_t j = 0; j < nfVerts.size(); ++j) {
                    size_t idx = (start + j) % nfVerts.size();
                    if (fv[j]->point.distanceTo(nfVerts[idx]) > 1e-6) {
                        ok = false;
                        break;
                    }
                }
                if (ok) {
                    matched = true;
                    break;
                }
                ok = true;
                for (size_t j = 0; j < nfVerts.size(); ++j) {
                    size_t idx = (start + nfVerts.size() - j) % nfVerts.size();
                    if (fv[j]->point.distanceTo(nfVerts[idx]) > 1e-6) {
                        ok = false;
                        break;
                    }
                }
                if (ok) {
                    matched = true;
                    break;
                }
            }
            if (matched) {
                f.topoId = newFaces[fi].topoId;
                break;
            }
        }
    }

    // Assign edge TopologyIDs.
    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make(featureID, "edge" + std::to_string(idx));
            ++idx;
        }
    }

    // Assign edge curves.
    for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
        assignEdgeCurve(&e);
    }

    // Bind NURBS surfaces — ALL planar for chamfer.
    for (auto& f : const_cast<std::deque<Face>&>(solid->faces())) {
        auto fv = faceVertices(&f);
        if (fv.size() < 3) {
            continue;
        }

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

        f.surface = std::make_shared<geo::NurbsSurface>(
            geo::NurbsSurface::makePlane(origin, uDir, vDir, uSize, vSize));
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
