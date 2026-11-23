#include "horizon/modeling/BooleanOp.h"

#include <algorithm>
#include <deque>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "FragmentMerge.h"
#include "MeshCsg.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/RTree.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/Naming.h"
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

/// Fast paths keep the original loops (and surfaces) instead of fragments,
/// a face's holes among them: its keyhole outline, which the CSG reads as the
/// face less its holes, sewed into a face with a bridge to each hole, an edge
/// with the face on both sides, and the result was refused for a boundary
/// that crosses itself.
void appendAsInputFaces(const std::vector<BoundaryPolygon>& polygons,
                        std::vector<SolidSewer::InputFace>& out) {
    for (const auto& poly : polygons) {
        SolidSewer::InputFace face;
        const bool holed = !poly.holes.empty() && std::all_of(poly.holes.begin(), poly.holes.end(),
                                                              [](const std::vector<Vec3>& hole) {
                                                                  return hole.size() >= 3;
                                                              });
        face.points = holed ? poly.outer : poly.points;
        if (holed) face.holes = poly.holes;
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
    // The sources by where they are: a result edge is tried only against
    // those it could lie along. Every pair was tried, and a part of a few
    // thousand facets has tens of thousands of each.
    math::RTree<size_t> index;
    for (size_t i = 0; i < sources.size(); ++i) {
        BoundingBox box;
        box.expand(sources[i].p);
        box.expand(sources[i].q);
        index.insert(i, box);
    }
    for (auto& e : const_cast<std::deque<topo::Edge>&>(result.edges())) {
        if (!e.halfEdge || !e.halfEdge->twin) continue;
        const Vec3 x = e.halfEdge->origin->point;
        const Vec3 y = e.halfEdge->twin->origin->point;
        const double tol = 1e-7 * std::max(1.0, (y - x).length());
        BoundingBox reach;
        reach.expand(x);
        reach.expand(y);
        reach.expand(reach.min() - Vec3(tol, tol, tol));
        reach.expand(reach.max() + Vec3(tol, tol, tol));
        std::vector<size_t> near = index.query(reach);
        // The first source it lies along, in their order, as before.
        std::sort(near.begin(), near.end());
        for (const size_t i : near) {
            if (onSegment(x, sources[i], tol) && onSegment(y, sources[i], tol)) {
                e.analyticCurve = sources[i].curve;
                break;
            }
        }
    }
}

/// Sew and enforce the public contract: any solid BooleanOp returns passes
/// Solid::checkManifold().  Applies to the CSG and disjoint fast paths alike,
/// so a near-touching "disjoint" pair the weld could fuse into broken
/// topology is rejected rather than returned.
std::unique_ptr<topo::Solid> sewChecked(const std::vector<SolidSewer::InputFace>& faces,
                                        double weldTol) {
    auto solid = SolidSewer::sew(faces, weldTol);
    if (!solid || solid->faceCount() == 0) return nullptr;
    if (!solid->checkManifold()) return nullptr;
    // As the header promises: counts a solid can have.
    if (!solid->checkEulerFormula()) return nullptr;
    return solid;
}

}  // namespace

std::unique_ptr<topo::Solid> BooleanOp::execute(const topo::Solid& solidA,
                                                const topo::Solid& solidB, BooleanType type,
                                                std::string* reason, NamingScheme naming) {
    const auto fail = [reason](const char* why) -> std::unique_ptr<topo::Solid> {
        if (reason) *reason = why;
        return nullptr;
    };
    // The one failure that is not about the operands' shape: the result would
    // not sew into a closed, manifold solid.
    const auto sewn = [&](std::unique_ptr<topo::Solid> result) {
        if (!result) return fail("the result could not be joined into a valid solid");
        inheritEdgeIdeals(*result, solidA, solidB);
        // Names that last: a face the operation split gets a name per piece,
        // and an edge is named after the faces it separates — not numbered in
        // sewing order, which any change to either operand reshuffles.
        if (namesFromGeometry(naming)) {
            nameFacePieces(*result);
            nameEdges(*result, naming);
        }
        return result;
    };

    auto polysA = BoundaryMesh::extractFacePolygons(solidA);
    auto polysB = BoundaryMesh::extractFacePolygons(solidB);
    if (polysA.empty() || polysB.empty()) return fail("one of the bodies has no faces");
    // The tolerances from the operands' size, not absolute (Phase 142).
    BoundingBox both = boundsOf(polysA);
    both.expand(boundsOf(polysB));
    const CsgTolerance tol = CsgTolerance::of(both.min(), both.max());

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
                return sewn(sewChecked(faces, std::min(kSewerDefaultWeldTol, tol.plane)));
            }
            case BooleanType::Subtract: {
                appendAsInputFaces(polysA, faces);
                return sewn(sewChecked(faces, std::min(kSewerDefaultWeldTol, tol.plane)));
            }
            case BooleanType::Intersect:
                return fail("the bodies do not overlap");
        }
    }

    // Faces whose fragments are merged back are taken whole where they are
    // convex; references by position were made against the triangulated
    // fragments, and keep them.
    const bool merged = namesFromGeometry(naming);
    std::vector<CsgPolygon> fragments;
    try {
        fragments = merged ? csgExecute(csgPieces(polysA, true, tol.plane),
                                        csgPieces(polysB, false, tol.plane), type, tol.plane,
                                        CsgFragments::Fewest)
                           : csgExecute(csgTriangles(polysA, true), csgTriangles(polysB, false),
                                        type, tol.plane);
    } catch (const CsgCancelled&) {
        if (reason) *reason = "cancelled";
        return nullptr;
    } catch (const CsgSeriesTooLong& e) {
        if (reason) {
            *reason = std::string("the copies are too many to combine where they overlap: ") +
                      e.what() + ". Fewer copies, or copies clear of one another, take far less.";
        }
        return nullptr;
    } catch (const CsgTooComplex& e) {
        // Said, and nothing else changed, where it ran for an hour or until
        // memory ran out.
        if (reason) {
            *reason = std::string(
                          "the bodies have too many facets to combine where they "
                          "overlap: ") +
                      e.what() + ". A coarser chord tolerance makes fewer.";
        }
        return nullptr;
    }
    // Merging, sewing and naming a result of thousands of faces each take
    // seconds in a debug build, and ask nothing: a cancel is heard between
    // them.
    if (BooleanCancellation::requested()) return fail("cancelled");
    if (merged && !fragments.empty()) {
        // The CSG works on triangles and leaves every face in pieces, even a
        // face the cut never reached; put each face back together.
        fragments = mergeFragments(std::move(fragments), tol.plane);
        if (BooleanCancellation::requested()) return fail("cancelled");
    }
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
    auto sewnSolid = sewChecked(faces, tol.plane);
    if (BooleanCancellation::requested()) return fail("cancelled");
    return sewn(std::move(sewnSolid));
}

BooleanWorkLimits::BooleanWorkLimits(std::size_t work, std::size_t splits) {
    CsgLimits& limits = csgLimitsOnThisThread();
    m_savedWork = limits.work;
    m_savedSplits = limits.splits;
    if (work != 0) limits.work = work;
    if (splits != 0) limits.splits = splits;
}

BooleanCancellation::BooleanCancellation(const std::atomic<bool>* cancelled)
    : m_saved(csgLimitsOnThisThread().cancelled) {
    csgLimitsOnThisThread().cancelled = cancelled;
}

BooleanCancellation::~BooleanCancellation() {
    csgLimitsOnThisThread().cancelled = m_saved;
}

bool BooleanCancellation::requested() {
    const std::atomic<bool>* cancelled = csgLimitsOnThisThread().cancelled;
    return cancelled != nullptr && cancelled->load(std::memory_order_relaxed);
}

BooleanSeries::BooleanSeries(std::size_t work) {
    CsgLimits& limits = csgLimitsOnThisThread();
    if (limits.seriesWork != nullptr) return;  // the outer series counts
    m_outermost = true;
    limits.seriesWork = &m_work;
    limits.seriesLimit = work;
}

BooleanSeries::~BooleanSeries() {
    if (!m_outermost) return;
    CsgLimits& limits = csgLimitsOnThisThread();
    limits.seriesWork = nullptr;
    limits.seriesLimit = 0;
}

BooleanWorkLimits::~BooleanWorkLimits() {
    CsgLimits& limits = csgLimitsOnThisThread();
    limits.work = m_savedWork;
    limits.splits = m_savedSplits;
}

}  // namespace hz::model
