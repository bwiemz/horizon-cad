#include "horizon/modeling/BooleanOp.h"

#include <utility>
#include <vector>

#include "MeshCsg.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/SolidSewer.h"

namespace hz::model {

using hz::math::BoundingBox;
using hz::math::Vec3;

namespace {

/// The sewer's tight default weld tolerance (matches SolidSewer::sew's
/// default), used on paths where no CSG-eps seams exist to reconcile.
constexpr double kSewerDefaultWeldTol = 1e-7;

BoundingBox boundsOf(const std::vector<BoundaryPolygon>& polygons) {
    BoundingBox box;
    for (const auto& poly : polygons) {
        for (const auto& p : poly.points) box.expand(p);
    }
    return box;
}

/// Fast paths keep the original loops (and surfaces) instead of fragments.
void appendAsInputFaces(const std::vector<BoundaryPolygon>& polygons,
                        std::vector<SolidSewer::InputFace>& out) {
    for (const auto& poly : polygons) {
        SolidSewer::InputFace face;
        face.points = poly.points;
        face.topoId = poly.topoId;
        face.surface = poly.surface;
        out.push_back(std::move(face));
    }
}

/// Sew and enforce the public contract: any solid BooleanOp returns passes
/// Solid::checkManifold().  checkManifold() (not checkEulerFormula()) is the
/// right gate — the Euler check has no genus term, so a legitimate manifold
/// through-hole result would fail it.  Applies to the CSG and disjoint fast
/// paths alike, so a near-touching "disjoint" pair the weld could fuse into
/// broken topology is rejected rather than returned.
std::unique_ptr<topo::Solid> sewChecked(const std::vector<SolidSewer::InputFace>& faces,
                                        double weldTol) {
    auto solid = SolidSewer::sew(faces, weldTol);
    if (!solid || solid->faceCount() == 0) return nullptr;
    if (!solid->checkManifold()) return nullptr;
    return solid;
}

}  // namespace

std::unique_ptr<topo::Solid> BooleanOp::execute(const topo::Solid& solidA,
                                                const topo::Solid& solidB, BooleanType type) {
    auto polysA = BoundaryMesh::extractFacePolygons(solidA);
    auto polysB = BoundaryMesh::extractFacePolygons(solidB);
    if (polysA.empty() || polysB.empty()) return nullptr;

    // Disjoint solids never interact — resolve without splitting so the
    // original face loops (and their surfaces) survive verbatim.  Weld at the
    // sewer's tight default here: these loops were never split, so there are
    // no CSG-eps seams to reconcile, and a tight weld avoids fusing genuinely
    // separate near-touching bodies.
    if (!boundsOf(polysA).intersects(boundsOf(polysB))) {
        std::vector<SolidSewer::InputFace> faces;
        switch (type) {
            case BooleanType::Union:
                appendAsInputFaces(polysA, faces);
                appendAsInputFaces(polysB, faces);
                return sewChecked(faces, kSewerDefaultWeldTol);
            case BooleanType::Subtract:
                appendAsInputFaces(polysA, faces);
                return sewChecked(faces, kSewerDefaultWeldTol);
            case BooleanType::Intersect:
                return nullptr;
        }
    }

    auto fragments = csgExecute(csgTriangles(polysA, true), csgTriangles(polysB, false), type);
    if (fragments.empty()) return nullptr;

    std::vector<SolidSewer::InputFace> faces;
    faces.reserve(fragments.size());
    for (auto& fragment : fragments) {
        SolidSewer::InputFace face;
        face.points = std::move(fragment.points);
        face.topoId = fragment.topoId;
        face.surface = nullptr;  // fragments get honest planar patches from the sewer
        faces.push_back(std::move(face));
    }

    // Weld at the CSG plane epsilon: fragments carry split points the BSP
    // treated as on-plane coincident, so healing must reach that far.
    // sewChecked enforces the checkManifold() contract for this path too.
    return sewChecked(faces, kCsgPlaneEps);
}

}  // namespace hz::model
