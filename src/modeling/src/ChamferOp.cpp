#include "horizon/modeling/ChamferOp.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <set>

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

    // Every vertex touched by a chamfer, mapped to its chamfer.  The
    // vertex-blend rejection above guarantees at most one chamfer per vertex.
    std::map<uint32_t, size_t> vertexToChamfer;
    for (size_t i = 0; i < chamferEdges.size(); ++i) {
        vertexToChamfer[chamferEdges[i].v1->id] = i;
        vertexToChamfer[chamferEdges[i].v2->id] = i;
    }

    // -----------------------------------------------------------------------
    // Rewrite every original face loop, then sew.
    //
    // A chamfer is a purely local edit of the boundary polygons: each vertex
    // an edge chamfer touches is replaced by its offset point(s), and one new
    // quad is added per chamfered edge.  Handing that polygon soup to
    // SolidSewer — the same reconstruction BooleanOp uses — is what keeps the
    // result's geometry consistent.  The previous implementation replayed the
    // soup through Euler operators with a convergence loop that picked target
    // faces by vertex-set containment; it produced combinatorially valid
    // topology whose loops were self-intersecting and non-planar, so the
    // volume integrator disagreed with the model.
    //
    // Three cases per loop vertex v:
    //   * v is untouched                → emit v.
    //   * this face carries the chamfered edge (it is faceA or faceB) → emit
    //     the single offset on this face's side.
    //   * this face only meets v        → the corner opens up, so emit both
    //     offsets, the one on the side we arrived from first.
    // -----------------------------------------------------------------------

    std::vector<SolidSewer::InputFace> sewFaces;
    sewFaces.reserve(inputSolid.faceCount() + chamferEdges.size());

    for (const auto& face : inputSolid.faces()) {
        if (face.outerLoop == nullptr || face.outerLoop->halfEdge == nullptr) {
            continue;
        }
        SolidSewer::InputFace fd;
        fd.topoId = face.topoId;
        // Offsets slide along the neighbouring faces, so every original face
        // keeps its own carrier — only its boundary changes.
        fd.surface = face.surface;

        HalfEdge* start = face.outerLoop->halfEdge;
        HalfEdge* cur = start;
        do {
            Vertex* v = cur->origin;
            auto it = vertexToChamfer.find(v->id);
            if (it == vertexToChamfer.end()) {
                fd.points.push_back(v->point);
                cur = cur->next;
                continue;
            }

            const auto& ce = chamferEdges[it->second];
            const bool isV1 = (v == ce.v1);
            const Vec3 offA = isV1 ? ce.v1_offsetA : ce.v2_offsetA;
            const Vec3 offB = isV1 ? ce.v1_offsetB : ce.v2_offsetB;

            if (&face == ce.faceA) {
                fd.points.push_back(offA);
            } else if (&face == ce.faceB) {
                fd.points.push_back(offB);
            } else {
                // Which side did the loop arrive from?  The incoming edge is
                // shared with faceA or faceB at a 3-valent vertex; where it is
                // not, fall back to whichever offset is nearer the vertex we
                // came from, which gives the same answer for the 3-valent case.
                const Face* across = (cur->prev != nullptr && cur->prev->twin != nullptr)
                                         ? cur->prev->twin->face
                                         : nullptr;
                bool aFirst;
                if (across == ce.faceA) {
                    aFirst = true;
                } else if (across == ce.faceB) {
                    aFirst = false;
                } else {
                    const Vec3 from = cur->prev != nullptr ? cur->prev->origin->point : v->point;
                    aFirst = from.distanceTo(offA) <= from.distanceTo(offB);
                }
                if (aFirst) {
                    fd.points.push_back(offA);
                    fd.points.push_back(offB);
                } else {
                    fd.points.push_back(offB);
                    fd.points.push_back(offA);
                }
            }
            cur = cur->next;
        } while (cur != start);

        if (fd.points.size() >= 3) {
            sewFaces.push_back(std::move(fd));
        }
    }

    // One quad per chamfered edge, wound to agree with the two faces it
    // bridges.
    for (const auto& ce : chamferEdges) {
        SolidSewer::InputFace fd;
        fd.topoId = TopologyID::make(featureID, "chamfer").child(ce.originalEdge->topoId.tag(), 0);
        fd.points = {ce.v1_offsetA, ce.v2_offsetA, ce.v2_offsetB, ce.v1_offsetB};

        Vec3 areaVec(0, 0, 0);
        for (size_t i = 0; i < fd.points.size(); ++i) {
            areaVec = areaVec + fd.points[i].cross(fd.points[(i + 1) % fd.points.size()]);
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
