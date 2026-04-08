#include "horizon/modeling/BooleanOp.h"

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/modeling/ExactPredicates.h"
#include "horizon/topology/EulerOps.h"
#include "horizon/topology/Queries.h"

#include <cassert>
#include <unordered_map>
#include <vector>

namespace hz::model {

using namespace hz::topo;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// Compute the centroid of a face from its vertex loop.
// ---------------------------------------------------------------------------

static Vec3 computeFaceCentroid(const Face& face) {
    auto verts = faceVertices(&face);
    if (verts.empty()) return Vec3::Zero;
    Vec3 sum = Vec3::Zero;
    for (const auto* v : verts) {
        sum = sum + v->point;
    }
    return sum * (1.0 / static_cast<double>(verts.size()));
}

// ---------------------------------------------------------------------------
// Per-face classification record.
// ---------------------------------------------------------------------------

struct FaceRecord {
    const Face* face = nullptr;
    const Solid* fromSolid = nullptr;
    int classification = 0;   // +1 outside, -1 inside, 0 boundary
    bool flipNormal = false;  // true when B's inside faces are used for Subtract
};

// ---------------------------------------------------------------------------
// Build a result Solid from the selected faces.
//
// Strategy: for each selected face, we recreate its vertex loop as a face
// in the new solid.  Vertices are shared where they coincide (within tol).
// The result is topologically a set of independent faces connected at shared
// edges where vertex positions match.
// ---------------------------------------------------------------------------

/// Helper: make a degree-1 (linear) NURBS curve between two points.
static std::shared_ptr<geo::NurbsCurve> makeLineCurve(const Vec3& a, const Vec3& b) {
    return std::make_shared<geo::NurbsCurve>(std::vector<Vec3>{a, b},
                                              std::vector<double>{1.0, 1.0},
                                              std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
}

/// Helper: find a HE originating at 'origin' on 'face'.
static HalfEdge* findHE(Face* face, Vertex* origin) {
    if (face->outerLoop == nullptr || face->outerLoop->halfEdge == nullptr) return nullptr;
    HalfEdge* start = face->outerLoop->halfEdge;
    HalfEdge* cur = start;
    do {
        if (cur->origin == origin) return cur;
        cur = cur->next;
    } while (cur != start);
    return nullptr;
}

static std::unique_ptr<Solid> buildResultSolid(const std::vector<FaceRecord>& selected) {
    if (selected.empty()) return nullptr;

    auto result = std::make_unique<Solid>();

    // Map from original vertex pointer to result vertex pointer.
    // We merge vertices that are at the same position (within tolerance).
    constexpr double kMergeTol = 1e-6;

    // We'll build each face independently, then try to merge shared edges.
    // Step 1: Create the initial topology with MVFS.
    // Step 2: For each face, add its vertices and close the face with MEF.

    // Collect all unique vertex positions first.
    struct VertexKey {
        Vec3 pos;
        Vertex* resultVert = nullptr;
    };
    std::vector<VertexKey> vertexPool;

    auto findOrCreateVertex = [&](const Vec3& pos) -> Vertex* {
        for (auto& vk : vertexPool) {
            if (vk.pos.distanceTo(pos) < kMergeTol) {
                return vk.resultVert;
            }
        }
        // Will be allocated later.
        return nullptr;
    };

    // Pre-collect all vertex positions from selected faces.
    std::vector<std::vector<Vec3>> faceVertexPositions;
    std::vector<TopologyID> faceTopoIds;
    std::vector<std::shared_ptr<geo::NurbsSurface>> faceSurfaces;

    for (const auto& fc : selected) {
        auto verts = faceVertices(fc.face);
        std::vector<Vec3> positions;
        positions.reserve(verts.size());
        for (const auto* v : verts) {
            positions.push_back(v->point);
        }
        if (fc.flipNormal) {
            // Reverse the vertex order to flip the face normal.
            std::reverse(positions.begin(), positions.end());
        }
        faceVertexPositions.push_back(std::move(positions));
        faceTopoIds.push_back(fc.face->topoId);
        faceSurfaces.push_back(fc.face->surface);
    }

    if (faceVertexPositions.empty()) return nullptr;

    // Build the solid using Euler operators.
    // First face seeds the solid via MVFS, then we extend with MEV/MEF.

    // MVFS with the first vertex of the first face.
    const auto& firstFaceVerts = faceVertexPositions[0];
    if (firstFaceVerts.empty()) return nullptr;

    auto [seedVertex, seedFace, seedShell] =
        euler::makeVertexFaceSolid(*result, firstFaceVerts[0]);
    vertexPool.push_back({firstFaceVerts[0], seedVertex});

    // For each face, we need to create a closed polygon in the solid.
    // The approach: use a "spur and close" method.
    //   1. Find or create the first vertex (may already exist from a previous face).
    //   2. MEV to extend to each subsequent vertex.
    //   3. MEF to close the polygon back to the first vertex.
    //
    // The tricky part: all new geometry initially lives on seedFace (the single
    // face from MVFS). Each MEF carves off a new face from that seed face.

    // Track which face in the result corresponds to each input face.
    std::vector<Face*> resultFaces;

    for (size_t fi = 0; fi < faceVertexPositions.size(); ++fi) {
        const auto& verts = faceVertexPositions[fi];
        if (verts.size() < 3) continue;

        // For each face we need to:
        // 1. Ensure all vertices exist in the solid (via MEV from seedFace).
        // 2. Connect them with MEF to form the face.

        // Find or create vertices for this face.
        std::vector<Vertex*> faceVerts;
        faceVerts.reserve(verts.size());

        for (size_t vi = 0; vi < verts.size(); ++vi) {
            Vertex* existing = findOrCreateVertex(verts[vi]);
            if (existing != nullptr) {
                faceVerts.push_back(existing);
            } else {
                // Create new vertex via MEV from some existing vertex on seedFace.
                // Pick any vertex that's on seedFace.
                Vertex* attachVert = nullptr;
                HalfEdge* attachHE = nullptr;

                // Try to attach from the previous vertex in this face if it's on seedFace.
                if (!faceVerts.empty()) {
                    attachVert = faceVerts.back();
                    attachHE = findHE(seedFace, attachVert);
                }

                // If that didn't work, find any vertex on seedFace.
                if (attachHE == nullptr) {
                    if (seedFace->outerLoop && seedFace->outerLoop->halfEdge) {
                        attachHE = seedFace->outerLoop->halfEdge;
                        attachVert = attachHE->origin;
                    }
                }

                if (attachHE == nullptr) {
                    // Shouldn't happen — seedFace always has at least one HE after MVFS+MEV.
                    // For MVFS with 0 edges, pass nullptr.
                    auto [edge, newVert] =
                        euler::makeEdgeVertex(*result, nullptr, seedFace, verts[vi]);
                    vertexPool.push_back({verts[vi], newVert});
                    faceVerts.push_back(newVert);
                } else {
                    auto [edge, newVert] =
                        euler::makeEdgeVertex(*result, attachHE, seedFace, verts[vi]);
                    vertexPool.push_back({verts[vi], newVert});
                    faceVerts.push_back(newVert);
                }
            }
        }

        // Now close the face: MEF from last vertex to first vertex.
        if (faceVerts.size() >= 3) {
            Vertex* vFirst = faceVerts[0];
            Vertex* vLast = faceVerts.back();

            // We need half-edges on seedFace at vLast and vFirst.
            HalfEdge* heLast = findHE(seedFace, vLast);
            HalfEdge* heFirst = findHE(seedFace, vFirst);

            if (heLast != nullptr && heFirst != nullptr && heLast != heFirst) {
                auto [closingEdge, newFace] = euler::makeEdgeFace(*result, heLast, heFirst);

                // Determine which of the two faces (seedFace or newFace) is the
                // face we just created.  The new face has the loop containing
                // our vertices.  Check which one has our vertex count.
                auto seedVerts = faceVertices(seedFace);
                auto newVerts = faceVertices(newFace);

                Face* ourFace = nullptr;
                if (static_cast<int>(newVerts.size()) == static_cast<int>(verts.size())) {
                    ourFace = newFace;
                } else if (static_cast<int>(seedVerts.size()) ==
                           static_cast<int>(verts.size())) {
                    ourFace = seedFace;
                    // seedFace was carved, the "remaining" becomes the new seedFace.
                    seedFace = newFace;
                } else {
                    // Neither matches exactly — just pick the new face.
                    ourFace = newFace;
                }

                // Assign topology ID and surface from the original face.
                ourFace->topoId = faceTopoIds[fi];
                ourFace->surface = faceSurfaces[fi];
                resultFaces.push_back(ourFace);
            } else {
                // Cannot close the face — skip it.
                resultFaces.push_back(nullptr);
            }
        } else {
            resultFaces.push_back(nullptr);
        }
    }

    // Assign line curves to all edges.
    for (auto& e : const_cast<std::deque<Edge>&>(result->edges())) {
        if (e.halfEdge != nullptr && e.halfEdge->twin != nullptr) {
            const Vec3& a = e.halfEdge->origin->point;
            const Vec3& b = e.halfEdge->twin->origin->point;
            e.curve = makeLineCurve(a, b);
        }
    }

    // If we created at least one face, return the result.
    if (result->faceCount() > 1) {  // >1 because seedFace is always there
        return result;
    }

    return nullptr;
}

// ---------------------------------------------------------------------------
// BooleanOp::execute
// ---------------------------------------------------------------------------

std::unique_ptr<Solid> BooleanOp::execute(const Solid& solidA, const Solid& solidB,
                                           BooleanType type) {
    // Step 1: Classify each face of A against B, and each face of B against A.
    std::vector<FaceRecord> allFaces;

    for (const auto& face : solidA.faces()) {
        Vec3 centroid = computeFaceCentroid(face);
        int cls = ExactPredicates::classifyPoint(centroid, solidB);
        allFaces.push_back({&face, &solidA, cls, false});
    }

    for (const auto& face : solidB.faces()) {
        Vec3 centroid = computeFaceCentroid(face);
        int cls = ExactPredicates::classifyPoint(centroid, solidA);
        allFaces.push_back({&face, &solidB, cls, false});
    }

    // Step 2: Select faces based on Boolean type.
    std::vector<FaceRecord> selected;

    for (auto& fc : allFaces) {
        bool keep = false;
        bool isFromA = (fc.fromSolid == &solidA);

        if (isFromA) {
            switch (type) {
                case BooleanType::Union:
                    // Keep A's faces that are outside B (or on boundary).
                    keep = (fc.classification >= 0);
                    break;
                case BooleanType::Subtract:
                    // Keep A's faces that are outside B (or on boundary).
                    keep = (fc.classification >= 0);
                    break;
                case BooleanType::Intersect:
                    // Keep A's faces that are inside B (or on boundary).
                    keep = (fc.classification <= 0);
                    break;
            }
        } else {
            // From solid B.
            switch (type) {
                case BooleanType::Union:
                    // Keep B's faces that are outside A (or on boundary).
                    keep = (fc.classification >= 0);
                    break;
                case BooleanType::Subtract:
                    // Keep B's faces that are inside A, but flip normals.
                    keep = (fc.classification <= 0);
                    fc.flipNormal = keep;
                    break;
                case BooleanType::Intersect:
                    // Keep B's faces that are inside A (or on boundary).
                    keep = (fc.classification <= 0);
                    break;
            }
        }

        if (keep) {
            selected.push_back(fc);
        }
    }

    if (selected.empty()) return nullptr;

    // Step 3: Build result solid from selected faces.
    return buildResultSolid(selected);
}

}  // namespace hz::model
