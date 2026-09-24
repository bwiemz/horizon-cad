#include "horizon/modeling/BooleanOp.h"

#include <algorithm>
#include <deque>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "MeshCsg.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
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
        face.analyticSurface = poly.analyticSurface;
        out.push_back(std::move(face));
    }
}

/// The ideal surface of each of one operand's faces, by TopologyID.  CSG
/// fragments carry their source face's ID and operand, so this is how a
/// fragment of a faceted cylinder keeps the cylinder it approximates: a bored
/// part must still offer its bore to a concentric mate.  The operands are
/// kept apart because two primitives of one kind share face IDs.
std::unordered_map<std::string, std::shared_ptr<geo::NurbsSurface>> idealsById(
    const std::vector<BoundaryPolygon>& polys) {
    std::unordered_map<std::string, std::shared_ptr<geo::NurbsSurface>> out;
    for (const auto& poly : polys) {
        if (poly.analyticSurface && poly.topoId.isValid()) {
            out.emplace(poly.topoId.tag(), poly.analyticSurface);
        }
    }
    return out;
}

/// Carry each source edge's ideal curve onto the result edges that lie along
/// it.  The sewer rebuilds every edge, and a CSG split can cut a source edge
/// into pieces, so an edge inherits the curve when both its ends lie on a
/// source edge segment that carries one.
void inheritEdgeIdeals(topo::Solid& result, const topo::Solid& a, const topo::Solid& b) {
    struct Source {
        Vec3 p, q;
        std::shared_ptr<geo::NurbsCurve> curve;
    };
    std::vector<Source> sources;
    for (const auto* solid : {&a, &b}) {
        for (const auto& e : solid->edges()) {
            if (!e.analyticCurve || !e.halfEdge || !e.halfEdge->twin) continue;
            sources.push_back(
                {e.halfEdge->origin->point, e.halfEdge->twin->origin->point, e.analyticCurve});
        }
    }
    if (sources.empty()) return;

    auto onSegment = [](const Vec3& x, const Source& s, double tol) {
        const Vec3 d = s.q - s.p;
        const double len2 = d.dot(d);
        if (len2 <= 0.0) return false;
        const double t = (x - s.p).dot(d) / len2;
        if (t < -1e-9 || t > 1.0 + 1e-9) return false;
        return (s.p + d * t - x).length() <= tol;
    };
    for (auto& e : const_cast<std::deque<topo::Edge>&>(result.edges())) {
        if (!e.halfEdge || !e.halfEdge->twin) continue;
        const Vec3 x = e.halfEdge->origin->point;
        const Vec3 y = e.halfEdge->twin->origin->point;
        const double tol = 1e-7 * std::max(1.0, (y - x).length());
        for (const auto& s : sources) {
            if (onSegment(x, s, tol) && onSegment(y, s, tol)) {
                e.analyticCurve = s.curve;
                break;
            }
        }
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
                                                const topo::Solid& solidB, BooleanType type,
                                                std::string* reason) {
    const auto fail = [reason](const char* why) -> std::unique_ptr<topo::Solid> {
        if (reason) *reason = why;
        return nullptr;
    };
    // The one failure that is not about the operands' shape: the result would
    // not sew into a closed, manifold solid.
    const auto sewn = [&](std::unique_ptr<topo::Solid> result) {
        if (!result) return fail("the result could not be joined into a valid solid");
        inheritEdgeIdeals(*result, solidA, solidB);
        return result;
    };

    auto polysA = BoundaryMesh::extractFacePolygons(solidA);
    auto polysB = BoundaryMesh::extractFacePolygons(solidB);
    if (polysA.empty() || polysB.empty()) return fail("one of the bodies has no faces");

    // Disjoint solids never interact — resolve without splitting so the
    // original face loops (and their surfaces) survive verbatim.  Weld at the
    // sewer's tight default here: these loops were never split, so there are
    // no CSG-eps seams to reconcile, and a tight weld avoids fusing genuinely
    // separate near-touching bodies.
    if (!boundsOf(polysA).intersects(boundsOf(polysB))) {
        std::vector<SolidSewer::InputFace> faces;
        switch (type) {
            case BooleanType::Union: {
                appendAsInputFaces(polysA, faces);
                appendAsInputFaces(polysB, faces);
                return sewn(sewChecked(faces, kSewerDefaultWeldTol));
            }
            case BooleanType::Subtract: {
                appendAsInputFaces(polysA, faces);
                return sewn(sewChecked(faces, kSewerDefaultWeldTol));
            }
            case BooleanType::Intersect:
                return fail("the bodies do not overlap");
        }
    }

    auto fragments = csgExecute(csgTriangles(polysA, true), csgTriangles(polysB, false), type);
    if (fragments.empty()) {
        switch (type) {
            case BooleanType::Subtract:
                return fail("the cut removes the whole body");
            case BooleanType::Intersect:
                return fail("the bodies do not overlap");
            default:
                return fail("the result is empty");
        }
    }

    const auto idealsA = idealsById(polysA);
    const auto idealsB = idealsById(polysB);
    std::vector<SolidSewer::InputFace> faces;
    faces.reserve(fragments.size());
    for (auto& fragment : fragments) {
        SolidSewer::InputFace face;
        face.points = std::move(fragment.points);
        face.topoId = fragment.topoId;
        face.surface = nullptr;  // fragments get honest planar patches from the sewer
        if (fragment.topoId.isValid()) {
            const auto& ideals = fragment.fromA ? idealsA : idealsB;
            auto it = ideals.find(fragment.topoId.tag());
            if (it != ideals.end()) face.analyticSurface = it->second;
        }
        faces.push_back(std::move(face));
    }

    // Weld at the CSG plane epsilon: fragments carry split points the BSP
    // treated as on-plane coincident, so healing must reach that far.
    // sewChecked enforces the checkManifold() contract for this path too.
    return sewn(sewChecked(faces, kCsgPlaneEps));
}

}  // namespace hz::model
