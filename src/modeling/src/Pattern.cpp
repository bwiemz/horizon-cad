#include "horizon/modeling/Pattern.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Quaternion.h"

namespace hz::model {

using hz::math::Mat4;
using hz::math::Quaternion;
using hz::math::Vec3;
using namespace hz::topo;

namespace {

std::shared_ptr<geo::NurbsCurve> transformCurve(const geo::NurbsCurve& c, const Mat4& xform) {
    std::vector<Vec3> cp;
    cp.reserve(c.controlPoints().size());
    for (const auto& p : c.controlPoints()) cp.push_back(xform.transformPoint(p));
    return std::make_shared<geo::NurbsCurve>(std::move(cp), c.weights(), c.knots(), c.degree());
}

std::shared_ptr<geo::NurbsSurface> transformSurface(const geo::NurbsSurface& s, const Mat4& xform) {
    std::vector<std::vector<Vec3>> cp;
    cp.reserve(s.controlPoints().size());
    for (const auto& row : s.controlPoints()) {
        std::vector<Vec3> nrow;
        nrow.reserve(row.size());
        for (const auto& p : row) nrow.push_back(xform.transformPoint(p));
        cp.push_back(std::move(nrow));
    }
    return std::make_shared<geo::NurbsSurface>(std::move(cp), s.weights(), s.knotsU(), s.knotsV(),
                                               s.degreeU(), s.degreeV());
}

TopologyID instanceId(const TopologyID& original, int instanceIndex) {
    if (instanceIndex == 0) return original;  // seed keeps identity
    if (!original.isValid()) return original;
    return original.child("pattern", instanceIndex);
}

// Deep-clone src into dst with a rigid transform, as a new set of shells.
void cloneInto(Solid& dst, const Solid& src, const Mat4& xform, int instanceIndex) {
    std::unordered_map<const void*, Vertex*> vmap;
    std::unordered_map<const void*, HalfEdge*> hmap;
    std::unordered_map<const void*, Edge*> emap;
    std::unordered_map<const void*, Wire*> wmap;
    std::unordered_map<const void*, Face*> fmap;
    std::unordered_map<const void*, Shell*> smap;

    // Vertices.
    for (const auto& v : src.vertices()) {
        Vertex* nv = dst.allocVertex();
        nv->point = xform.transformPoint(v.point);
        nv->topoId = instanceId(v.topoId, instanceIndex);
        vmap[&v] = nv;
    }

    // Edges (with transformed curves).
    for (const auto& e : src.edges()) {
        Edge* ne = dst.allocEdge();
        ne->topoId = instanceId(e.topoId, instanceIndex);
        if (e.curve) ne->curve = transformCurve(*e.curve, xform);
        emap[&e] = ne;
    }

    // Shells.
    for (const auto& s : src.shells()) {
        Shell* ns = dst.allocShell();
        ns->solid = &dst;
        smap[&s] = ns;
    }

    // Faces (with transformed surfaces); collect their wires and half-edges.
    std::vector<const Wire*> wires;
    std::vector<const HalfEdge*> halfEdges;
    std::unordered_set<const void*> seenWire, seenHE;
    auto visitWire = [&](const Wire* w) {
        if (!w || !seenWire.insert(w).second) return;
        wires.push_back(w);
        const HalfEdge* start = w->halfEdge;
        const HalfEdge* cur = start;
        do {
            if (cur && seenHE.insert(cur).second) halfEdges.push_back(cur);
            cur = cur ? cur->next : nullptr;
        } while (cur && cur != start);
    };

    for (const auto& f : src.faces()) {
        Face* nf = dst.allocFace();
        nf->topoId = instanceId(f.topoId, instanceIndex);
        if (f.surface) nf->surface = transformSurface(*f.surface, xform);
        fmap[&f] = nf;
        visitWire(f.outerLoop);
        for (const Wire* inner : f.innerLoops) visitWire(inner);
    }

    for (const Wire* w : wires) wmap[w] = dst.allocWire();
    for (const HalfEdge* h : halfEdges) hmap[h] = dst.allocHalfEdge();

    // Remap all cross-references.
    for (const auto& v : src.vertices()) {
        vmap[&v]->halfEdge = v.halfEdge ? hmap[v.halfEdge] : nullptr;
    }
    for (const HalfEdge* h : halfEdges) {
        HalfEdge* nh = hmap[h];
        nh->origin = h->origin ? vmap[h->origin] : nullptr;
        nh->twin = h->twin ? hmap[h->twin] : nullptr;
        nh->next = h->next ? hmap[h->next] : nullptr;
        nh->prev = h->prev ? hmap[h->prev] : nullptr;
        nh->edge = h->edge ? emap[h->edge] : nullptr;
        nh->face = h->face ? fmap[h->face] : nullptr;
    }
    for (const auto& e : src.edges()) {
        emap[&e]->halfEdge = e.halfEdge ? hmap[e.halfEdge] : nullptr;
    }
    for (const Wire* w : wires) {
        wmap[w]->halfEdge = w->halfEdge ? hmap[w->halfEdge] : nullptr;
    }
    for (const auto& f : src.faces()) {
        Face* nf = fmap[&f];
        nf->outerLoop = f.outerLoop ? wmap[f.outerLoop] : nullptr;
        nf->innerLoops.clear();
        for (const Wire* inner : f.innerLoops) nf->innerLoops.push_back(wmap[inner]);
        nf->shell = f.shell ? smap[f.shell] : nullptr;
    }
    for (const auto& s : src.shells()) {
        Shell* ns = smap[&s];
        ns->faces.clear();
        for (const Face* fp : s.faces) ns->faces.push_back(fmap[fp]);
    }
}

std::unique_ptr<topo::Solid> buildPattern(const Solid& source, const std::vector<Mat4>& transforms,
                                          const std::vector<int>& suppressed) {
    auto result = std::make_unique<Solid>();
    std::unordered_set<int> skip(suppressed.begin(), suppressed.end());
    for (size_t k = 0; k < transforms.size(); ++k) {
        if (skip.count(static_cast<int>(k))) continue;
        cloneInto(*result, source, transforms[k], static_cast<int>(k));
    }
    return result;
}

}  // namespace

std::unique_ptr<topo::Solid> Pattern::linear(const topo::Solid& source, const Vec3& direction,
                                             double spacing, int count,
                                             const std::vector<int>& suppressed) {
    if (count < 1) return nullptr;
    Vec3 dir = direction.normalized();
    std::vector<Mat4> transforms;
    transforms.reserve(static_cast<size_t>(count));
    for (int k = 0; k < count; ++k) {
        transforms.push_back(Mat4::translation(dir * (spacing * k)));
    }
    return buildPattern(source, transforms, suppressed);
}

std::unique_ptr<topo::Solid> Pattern::circular(const topo::Solid& source, const Vec3& axisPoint,
                                               const Vec3& axisDir, double angleStepRad, int count,
                                               const std::vector<int>& suppressed) {
    if (count < 1) return nullptr;
    Vec3 axis = axisDir.normalized();
    std::vector<Mat4> transforms;
    transforms.reserve(static_cast<size_t>(count));
    const Mat4 toOrigin = Mat4::translation(-axisPoint);
    const Mat4 fromOrigin = Mat4::translation(axisPoint);
    for (int k = 0; k < count; ++k) {
        Mat4 rot = Mat4::rotation(Quaternion::fromAxisAngle(axis, angleStepRad * k));
        transforms.push_back(fromOrigin * rot * toOrigin);
    }
    return buildPattern(source, transforms, suppressed);
}

}  // namespace hz::model
