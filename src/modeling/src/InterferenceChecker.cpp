#include "horizon/modeling/InterferenceChecker.h"

#include <array>
#include <cmath>
#include <set>

#include "horizon/math/RTree.h"
#include "horizon/modeling/SolidTessellator.h"

namespace hz::model {

using hz::math::BoundingBox;
using hz::math::Vec3;

namespace {

using Triangle = std::array<Vec3, 3>;

// A tessellated body plus an R*-tree over its triangles, so the pair test is
// O((edges) · log(tris)) instead of O(tris²).
struct Mesh {
    std::vector<Triangle> tris;
    math::RTree<size_t> tree;
};

// Möller–Trumbore ray/triangle intersection (no back-face culling). Reports the
// ray parameter @p t and barycentric coords so callers can bound a segment or
// count parity for containment.
bool rayTriangle(const Vec3& orig, const Vec3& dir, const Vec3& v0, const Vec3& v1, const Vec3& v2,
                 double& t, double& u, double& v) {
    constexpr double kEps = 1e-12;
    const Vec3 e1 = v1 - v0;
    const Vec3 e2 = v2 - v0;
    const Vec3 p = dir.cross(e2);
    const double det = e1.dot(p);
    if (std::abs(det) < kEps) return false;  // ray parallel to triangle
    const double inv = 1.0 / det;
    const Vec3 tv = orig - v0;
    u = tv.dot(p) * inv;
    if (u < -kEps || u > 1.0 + kEps) return false;
    const Vec3 q = tv.cross(e1);
    v = dir.dot(q) * inv;
    if (v < -kEps || u + v > 1.0 + kEps) return false;
    t = e2.dot(q) * inv;
    return true;
}

// Does segment [a,b] pass through the interior of triangle (v0,v1,v2)?
bool segmentCrossesTriangle(const Vec3& a, const Vec3& b, const Vec3& v0, const Vec3& v1,
                            const Vec3& v2) {
    constexpr double kEdge = 1e-6;  // require an interior crossing, not a graze
    double t, u, v;
    if (!rayTriangle(a, b - a, v0, v1, v2, t, u, v)) return false;
    if (t <= kEdge || t >= 1.0 - kEdge) return false;
    return u > kEdge && v > kEdge && (u + v) < 1.0 - kEdge;
}

BoundingBox triangleBounds(const Triangle& tri) {
    BoundingBox box;
    box.expand(tri[0]);
    box.expand(tri[1]);
    box.expand(tri[2]);
    return box;
}

BoundingBox segmentBounds(const Vec3& a, const Vec3& b) {
    BoundingBox box;
    box.expand(a);
    box.expand(b);
    return box;
}

Mesh buildMesh(const topo::Solid& solid, const BoundingBox& bounds) {
    // A size-relative tolerance keeps the triangle budget bounded regardless of
    // absolute solid size (the tessellator otherwise emits a dense uniform grid
    // even for flat faces).
    const double diag = bounds.isValid() ? bounds.diagonal() : 1.0;
    const double tol = std::max(diag * 0.2, 1e-6);
    render::MeshData mesh = SolidTessellator::tessellate(solid, tol);

    Mesh out;
    out.tris.reserve(mesh.indices.size() / 3);
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        Triangle tri;
        bool ok = true;
        for (int k = 0; k < 3; ++k) {
            const uint32_t idx = mesh.indices[i + static_cast<size_t>(k)];
            const size_t base = static_cast<size_t>(idx) * 3;
            if (base + 2 >= mesh.positions.size()) {
                ok = false;
                break;
            }
            tri[static_cast<size_t>(k)] =
                Vec3(mesh.positions[base], mesh.positions[base + 1], mesh.positions[base + 2]);
        }
        if (!ok) break;  // malformed mesh
        out.tree.insert(out.tris.size(), triangleBounds(tri));
        out.tris.push_back(tri);
    }
    return out;
}

// True if any edge of a triangle in @p src crosses a face in @p dst.
bool anyEdgeCrossesFace(const std::vector<Triangle>& src, const Mesh& dst) {
    for (const Triangle& s : src) {
        for (int e = 0; e < 3; ++e) {
            const Vec3& a = s[static_cast<size_t>(e)];
            const Vec3& b = s[static_cast<size_t>((e + 1) % 3)];
            for (size_t j : dst.tree.query(segmentBounds(a, b))) {
                const Triangle& d = dst.tris[j];
                if (segmentCrossesTriangle(a, b, d[0], d[1], d[2])) return true;
            }
        }
    }
    return false;
}

// Ray-parity point-in-mesh test using a generic (non-axis-aligned) direction to
// avoid degenerate edge/vertex hits.
bool pointInsideMesh(const Vec3& point, const std::vector<Triangle>& tris) {
    const Vec3 dir = Vec3(0.5773, 0.5774, 0.5775).normalized();
    int crossings = 0;
    for (const Triangle& tri : tris) {
        double t, u, v;
        if (rayTriangle(point, dir, tri[0], tri[1], tri[2], t, u, v) && t > 1e-9) ++crossings;
    }
    return (crossings % 2) == 1;
}

BoundingBox overlapBox(const BoundingBox& a, const BoundingBox& b) {
    const Vec3 mn(std::max(a.min().x, b.min().x), std::max(a.min().y, b.min().y),
                  std::max(a.min().z, b.min().z));
    const Vec3 mx(std::min(a.max().x, b.max().x), std::min(a.max().y, b.max().y),
                  std::min(a.max().z, b.max().z));
    return BoundingBox(mn, mx);
}

}  // namespace

BoundingBox InterferenceChecker::solidBounds(const topo::Solid& solid) {
    BoundingBox box;
    for (const auto& vtx : solid.vertices()) box.expand(vtx.point);
    return box;
}

bool InterferenceChecker::solidsInterfere(const topo::Solid& a, const topo::Solid& b) {
    const BoundingBox ba = solidBounds(a);
    const BoundingBox bb = solidBounds(b);
    if (!ba.intersects(bb)) return false;

    const Mesh ma = buildMesh(a, ba);
    const Mesh mb = buildMesh(b, bb);
    if (ma.tris.empty() || mb.tris.empty()) return false;

    // Surfaces cross?
    if (anyEdgeCrossesFace(ma.tris, mb)) return true;
    if (anyEdgeCrossesFace(mb.tris, ma)) return true;

    // One solid entirely inside the other? (No surface crossing means every
    // vertex of the inner body is interior, so a single sample suffices.)
    if (pointInsideMesh(ma.tris[0][0], mb.tris)) return true;
    if (pointInsideMesh(mb.tris[0][0], ma.tris)) return true;

    return false;
}

std::vector<InterferencePair> InterferenceChecker::check(
    const std::vector<const topo::Solid*>& solids) {
    std::vector<InterferencePair> pairs;

    // Broad phase: index every valid solid's AABB in an R*-tree.
    std::vector<BoundingBox> bounds(solids.size());
    math::RTree<size_t> tree;
    for (size_t i = 0; i < solids.size(); ++i) {
        if (!solids[i]) continue;
        bounds[i] = solidBounds(*solids[i]);
        if (bounds[i].isValid()) tree.insert(i, bounds[i]);
    }

    std::set<std::pair<size_t, size_t>> tested;
    for (size_t i = 0; i < solids.size(); ++i) {
        if (!solids[i] || !bounds[i].isValid()) continue;
        for (size_t j : tree.query(bounds[i])) {
            if (j == i || !solids[j]) continue;
            const auto key = std::minmax(i, j);
            if (!tested.insert({key.first, key.second}).second) continue;  // dedup
            // Narrow phase.
            if (solidsInterfere(*solids[key.first], *solids[key.second])) {
                pairs.push_back(
                    {key.first, key.second, overlapBox(bounds[key.first], bounds[key.second])});
            }
        }
    }
    return pairs;
}

}  // namespace hz::model
