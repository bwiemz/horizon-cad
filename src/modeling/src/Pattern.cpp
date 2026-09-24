#include "horizon/modeling/Pattern.h"

#include <algorithm>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Quaternion.h"
#include "horizon/modeling/BooleanOp.h"

namespace hz::model {

using hz::math::BoundingBox;
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

// Deep-clone src — or, given `only`, just those of its shells — into dst with
// a rigid transform, as a new set of shells.
void cloneInto(Solid& dst, const Solid& src, const Mat4& xform, int instanceIndex,
               const std::vector<const Shell*>* only = nullptr) {
    std::unordered_map<const void*, Vertex*> vmap;
    std::unordered_map<const void*, HalfEdge*> hmap;
    std::unordered_map<const void*, Edge*> emap;
    std::unordered_map<const void*, Wire*> wmap;
    std::unordered_map<const void*, Face*> fmap;
    std::unordered_map<const void*, Shell*> smap;
    const auto mapped = [](const auto& map, const void* key) {
        using Target = typename std::decay_t<decltype(map)>::mapped_type;
        const auto it = map.find(key);
        return it == map.end() ? Target{nullptr} : it->second;
    };

    // What to copy: the faces, then the loops, half-edges, vertices and edges
    // they use.
    std::vector<const Face*> faces;
    if (only) {
        for (const Shell* shell : *only)
            faces.insert(faces.end(), shell->faces.begin(), shell->faces.end());
    } else {
        for (const auto& f : src.faces()) faces.push_back(&f);
    }
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
    for (const Face* f : faces) {
        visitWire(f->outerLoop);
        for (const Wire* inner : f->innerLoops) visitWire(inner);
    }
    std::unordered_set<const void*> usedVertices, usedEdges;
    for (const HalfEdge* h : halfEdges) {
        usedVertices.insert(h->origin);
        usedEdges.insert(h->edge);
    }
    const auto copies = [only](const std::unordered_set<const void*>& used, const void* entity) {
        return only == nullptr || used.count(entity) > 0;
    };

    // Vertices.
    for (const auto& v : src.vertices()) {
        if (!copies(usedVertices, &v)) continue;
        Vertex* nv = dst.allocVertex();
        nv->point = xform.transformPoint(v.point);
        nv->topoId = instanceId(v.topoId, instanceIndex);
        vmap[&v] = nv;
    }

    // Edges (with transformed curves).
    for (const auto& e : src.edges()) {
        if (!copies(usedEdges, &e)) continue;
        Edge* ne = dst.allocEdge();
        ne->topoId = instanceId(e.topoId, instanceIndex);
        if (e.curve) ne->curve = transformCurve(*e.curve, xform);
        if (e.analyticCurve) ne->analyticCurve = transformCurve(*e.analyticCurve, xform);
        emap[&e] = ne;
    }

    // Shells.
    for (const auto& s : src.shells()) {
        if (only && std::find(only->begin(), only->end(), &s) == only->end()) continue;
        Shell* ns = dst.allocShell();
        ns->solid = &dst;
        smap[&s] = ns;
    }

    // Faces (with transformed surfaces).
    for (const Face* f : faces) {
        Face* nf = dst.allocFace();
        nf->topoId = instanceId(f->topoId, instanceIndex);
        if (f->surface) nf->surface = transformSurface(*f->surface, xform);
        // Each instance of a faceted boss must still resolve to its own
        // cylinder, so the ideal is carried and moved with the facets.
        if (f->analyticSurface) nf->analyticSurface = transformSurface(*f->analyticSurface, xform);
        fmap[f] = nf;
    }

    for (const Wire* w : wires) wmap[w] = dst.allocWire();
    for (const HalfEdge* h : halfEdges) hmap[h] = dst.allocHalfEdge();

    // Remap all cross-references. (Within one shell of a closed solid every
    // reference stays inside the shell; one that does not is left null.)
    for (const auto& v : src.vertices()) {
        if (Vertex* nv = mapped(vmap, &v)) nv->halfEdge = mapped(hmap, v.halfEdge);
    }
    for (const HalfEdge* h : halfEdges) {
        HalfEdge* nh = hmap[h];
        nh->origin = mapped(vmap, h->origin);
        nh->twin = mapped(hmap, h->twin);
        nh->next = mapped(hmap, h->next);
        nh->prev = mapped(hmap, h->prev);
        nh->edge = mapped(emap, h->edge);
        nh->face = mapped(fmap, h->face);
    }
    for (const auto& e : src.edges()) {
        if (Edge* ne = mapped(emap, &e)) ne->halfEdge = mapped(hmap, e.halfEdge);
    }
    for (const Wire* w : wires) {
        wmap[w]->halfEdge = mapped(hmap, w->halfEdge);
    }
    for (const Face* f : faces) {
        Face* nf = fmap[f];
        nf->outerLoop = mapped(wmap, f->outerLoop);
        nf->innerLoops.clear();
        for (const Wire* inner : f->innerLoops) nf->innerLoops.push_back(mapped(wmap, inner));
        nf->shell = mapped(smap, f->shell);
    }
    for (const auto& shell : src.shells()) {
        Shell* ns = mapped(smap, &shell);
        if (!ns) continue;
        ns->faces.clear();
        for (const Face* fp : shell.faces) ns->faces.push_back(mapped(fmap, fp));
    }
}

// Visit the vertices of every loop of a face.
template <typename Visit>
void forEachLoopVertex(const Face& face, Visit&& visit) {
    const auto walk = [&visit](const Wire* wire) {
        if (!wire || !wire->halfEdge) return;
        const HalfEdge* start = wire->halfEdge;
        const HalfEdge* he = start;
        do {
            if (!he->origin) return;
            visit(he->origin->point);
            he = he->next;
        } while (he && he != start);
    };
    walk(face.outerLoop);
    for (const Wire* inner : face.innerLoops) walk(inner);
}

// The volume a shell encloses, signed by its orientation: positive when its
// faces point away from what it encloses. Each loop is fanned from its first
// vertex (the divergence theorem over the loop's triangles).
double signedVolume(const Shell& shell) {
    double volume = 0.0;
    for (const Face* face : shell.faces) {
        const auto loop = [&volume](const Wire* wire) {
            if (!wire || !wire->halfEdge || !wire->halfEdge->origin) return;
            const HalfEdge* start = wire->halfEdge;
            const Vec3& a = start->origin->point;
            for (const HalfEdge* he = start->next; he && he->next && he->next != start;
                 he = he->next) {
                if (!he->origin || !he->next->origin) return;
                const Vec3& b = he->origin->point;
                const Vec3& c = he->next->origin->point;
                volume += a.dot(b.cross(c)) / 6.0;
            }
        };
        loop(face->outerLoop);
        for (const Wire* inner : face->innerLoops) loop(inner);
    }
    return volume;
}

BoundingBox boundsOf(const Solid& solid) {
    BoundingBox box;
    for (const auto& v : solid.vertices()) box.expand(v.point);
    return box;
}

int findRoot(std::vector<int>& parent, int i) {
    while (parent[static_cast<size_t>(i)] != i) {
        parent[static_cast<size_t>(i)] =
            parent[static_cast<size_t>(parent[static_cast<size_t>(i)])];
        i = parent[static_cast<size_t>(i)];
    }
    return i;
}

std::unique_ptr<topo::Solid> buildPattern(const Solid& source, const std::vector<Mat4>& transforms,
                                          const std::vector<int>& suppressed) {
    std::unordered_set<int> skip(suppressed.begin(), suppressed.end());

    // Each instance as its own solid, so overlapping ones can be merged.
    std::vector<std::unique_ptr<Solid>> instances;
    std::vector<BoundingBox> bounds;
    for (size_t k = 0; k < transforms.size(); ++k) {
        if (skip.count(static_cast<int>(k))) continue;
        auto inst = std::make_unique<Solid>();
        cloneInto(*inst, source, transforms[k], static_cast<int>(k));
        bounds.push_back(boundsOf(*inst));
        instances.push_back(std::move(inst));
    }

    // Group instances whose bounds touch or overlap.  Instances that meet
    // must become one body: left as separate shells they would count shared
    // material once per instance and present faces inside the part.
    const size_t n = instances.size();
    std::vector<int> parent(n);
    for (size_t i = 0; i < n; ++i) parent[i] = static_cast<int>(i);
    for (size_t i = 0; i < n; ++i) {
        const Vec3 span = bounds[i].max() - bounds[i].min();
        const double pad = 1e-9 * std::max(1.0, span.length());
        BoundingBox grown(bounds[i].min() - Vec3(pad, pad, pad),
                          bounds[i].max() + Vec3(pad, pad, pad));
        for (size_t j = i + 1; j < n; ++j) {
            if (grown.intersects(bounds[j])) {
                parent[static_cast<size_t>(findRoot(parent, static_cast<int>(i)))] =
                    findRoot(parent, static_cast<int>(j));
            }
        }
    }

    // Union each group in instance order.  A group the Boolean cannot merge
    // is refused rather than returned as overlapping shells.
    std::vector<std::unique_ptr<Solid>> bodies(n);
    for (size_t i = 0; i < n; ++i) {
        const auto root = static_cast<size_t>(findRoot(parent, static_cast<int>(i)));
        if (!bodies[root]) {
            bodies[root] = std::move(instances[i]);
            continue;
        }
        auto merged = BooleanOp::execute(*bodies[root], *instances[i], BooleanType::Union);
        if (!merged) return nullptr;
        bodies[root] = std::move(merged);
    }

    auto result = std::make_unique<Solid>();
    for (const auto& body : bodies) {
        if (body) cloneInto(*result, *body, Mat4::identity(), 0);
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

std::unique_ptr<topo::Solid> Pattern::collect(const topo::Solid& a, const topo::Solid& b) {
    auto out = std::make_unique<Solid>();
    cloneInto(*out, a, Mat4::identity(), 0);
    cloneInto(*out, b, Mat4::identity(), 0);
    return out;
}

std::vector<std::unique_ptr<topo::Solid>> Pattern::separate(const topo::Solid& solid) {
    // A shell is either the outside of a body or a cavity inside one: an
    // enclosed void comes out of a Boolean as a second shell of the same
    // body, facing into the void. Its signed volume tells them apart —
    // negative for a cavity — once the solid's overall orientation is taken
    // out (some construction paths build a whole solid facing inward).
    struct Part {
        const Shell* shell;
        double volume;
        BoundingBox box;
    };
    std::vector<Part> parts;
    double total = 0.0;
    for (const auto& shell : solid.shells()) {
        if (shell.faces.empty()) continue;
        Part part{&shell, signedVolume(shell), BoundingBox()};
        for (const Face* face : shell.faces) {
            forEachLoopVertex(*face, [&part](const Vec3& p) { part.box.expand(p); });
        }
        total += part.volume;
        parts.push_back(part);
    }
    const double orientation = total < 0.0 ? -1.0 : 1.0;

    // Each body is an outer shell and the cavities it encloses: a cavity
    // belongs to the smallest outer shell whose bounds contain it.
    std::vector<std::vector<const Shell*>> groups;
    std::vector<const Part*> outers;
    for (const Part& part : parts) {
        if (part.volume * orientation >= 0.0) {
            groups.push_back({part.shell});
            outers.push_back(&part);
        }
    }
    const auto boxVolume = [](const BoundingBox& b) {
        const Vec3 d = b.max() - b.min();
        return d.x * d.y * d.z;
    };
    for (const Part& part : parts) {
        if (part.volume * orientation >= 0.0) continue;
        size_t best = groups.size();
        for (size_t i = 0; i < outers.size(); ++i) {
            const BoundingBox& outer = outers[i]->box;
            if (outer.contains(part.box) &&
                (best == groups.size() || boxVolume(outer) < boxVolume(outers[best]->box))) {
                best = i;
            }
        }
        if (best < groups.size()) {
            groups[best].push_back(part.shell);
        } else {
            groups.push_back({part.shell});  // enclosed by nothing: keep it as it is
        }
    }

    std::vector<std::unique_ptr<topo::Solid>> bodies;
    for (const auto& group : groups) {
        auto body = std::make_unique<Solid>();
        cloneInto(*body, solid, Mat4::identity(), 0, &group);
        bodies.push_back(std::move(body));
    }
    return bodies;
}

std::unique_ptr<topo::Solid> Pattern::transformed(const topo::Solid& source, const Mat4& xform) {
    auto out = std::make_unique<Solid>();
    cloneInto(*out, source, xform, 0);
    return out;
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
