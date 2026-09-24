#include "horizon/modeling/SolidTessellator.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/modeling/BoundaryMesh.h"

namespace hz::model {

using hz::math::Vec3;

namespace {

/// True for the 2×2 degree-1 bounding-rectangle patches produced by
/// NurbsSurface::makePlane (with coplanar corners) — these over-cover the
/// trimmed face, so the loop polygon is the faithful boundary.
bool isPlanarPatch(const geo::NurbsSurface& surface) {
    if (surface.degreeU() != 1 || surface.degreeV() != 1) return false;
    if (surface.controlPointCountU() != 2 || surface.controlPointCountV() != 2) return false;
    const auto& cp = surface.controlPoints();
    const Vec3& p00 = cp[0][0];
    const Vec3 u = cp[1][0] - p00;
    const Vec3 v = cp[0][1] - p00;
    const Vec3 d = cp[1][1] - p00;
    const Vec3 n = u.cross(v);
    const double nLen = n.length();
    if (nLen < 1e-30) return true;  // degenerate patch — treat as planar
    const double scale = std::max({u.length(), v.length(), d.length(), 1e-12});
    return std::abs(d.dot(n / nLen)) < 1e-9 * scale;
}

void appendLoopTriangles(const BoundaryPolygon& poly, geo::MeshData& out) {
    const auto tris = BoundaryMesh::triangulatePolygon(poly.points);
    for (const auto& tri : tris) {
        Vec3 n = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
        const double len = n.length();
        n = (len > 1e-30) ? n / len : Vec3(0, 0, 1);

        const auto base = static_cast<uint32_t>(out.positions.size() / 3);
        for (const Vec3& p : tri) {
            out.positions.push_back(static_cast<float>(p.x));
            out.positions.push_back(static_cast<float>(p.y));
            out.positions.push_back(static_cast<float>(p.z));
            out.normals.push_back(static_cast<float>(n.x));
            out.normals.push_back(static_cast<float>(n.y));
            out.normals.push_back(static_cast<float>(n.z));
        }
        out.indices.push_back(base);
        out.indices.push_back(base + 1);
        out.indices.push_back(base + 2);
    }
}

void appendSurfaceTriangles(const geo::NurbsSurface& surface, double tolerance,
                            geo::MeshData& out) {
    const auto faceMesh = surface.tessellate(tolerance);
    const auto offset = static_cast<uint32_t>(out.positions.size() / 3);
    out.positions.insert(out.positions.end(), faceMesh.positions.begin(), faceMesh.positions.end());
    out.normals.insert(out.normals.end(), faceMesh.normals.begin(), faceMesh.normals.end());
    for (uint32_t idx : faceMesh.indices) {
        out.indices.push_back(idx + offset);
    }
}

/// A face's unit normal from its outer loop (Newell's method), or zero.
Vec3 loopNormal(const topo::Face& face) {
    Vec3 normal;
    if (!face.outerLoop || !face.outerLoop->halfEdge) return normal;
    const topo::HalfEdge* start = face.outerLoop->halfEdge;
    const topo::HalfEdge* he = start;
    size_t guard = 0;
    do {
        if (!he->origin || !he->next || !he->next->origin) return Vec3();
        const Vec3& a = he->origin->point;
        const Vec3& b = he->next->origin->point;
        normal.x += (a.y - b.y) * (a.z + b.z);
        normal.y += (a.z - b.z) * (a.x + b.x);
        normal.z += (a.x - b.x) * (a.y + b.y);
        he = he->next;
    } while (he && he != start && ++guard < 1000000);
    const double len = normal.length();
    return len > 1e-30 ? normal / len : Vec3();
}

/// Whether the edge between faces @p a and @p b is one the part shows: not
/// a seam between two facets of one curved surface (the same ideal surface,
/// or two ideal surfaces meeting at a gentle bend, as a patterned cylinder's
/// facets do), nor a line across one flat face split in two.
bool showsAsEdge(const topo::Face& a, const topo::Face& b,
                 std::unordered_map<const topo::Face*, Vec3>& normals) {
    if (a.analyticSurface && a.analyticSurface == b.analyticSurface) return false;
    const auto normalOf = [&normals](const topo::Face& f) {
        const auto it = normals.find(&f);
        if (it != normals.end()) return it->second;
        return normals.emplace(&f, loopNormal(f)).first->second;
    };
    const double cosine = normalOf(a).dot(normalOf(b));
    if (cosine > 1.0 - 1e-9) return false;  // coplanar pieces
    // Two facets of curved surfaces bending by under 30 degrees: one smooth
    // surface, faceted.
    constexpr double kSmooth = 0.8660254037844387;  // cos 30 degrees
    return !(a.analyticSurface && b.analyticSurface && cosine > kSmooth);
}

/// The edges the part shows, as polylines: a line for a straight edge,
/// sampled along its curve for a curved one.
void appendEdges(const topo::Solid& solid, geo::MeshData& out) {
    std::unordered_map<const topo::Face*, Vec3> normals;
    for (const auto& edge : solid.edges()) {
        const topo::HalfEdge* he = edge.halfEdge;
        if (!he || !he->origin || !he->next || !he->next->origin || !he->face) continue;
        if (he->twin && he->twin->face && !showsAsEdge(*he->face, *he->twin->face, normals)) {
            continue;
        }
        geo::MeshData::Edge shown;
        shown.tag = edge.topoId.tag();
        const auto add = [&shown](const Vec3& p) {
            shown.points.push_back(static_cast<float>(p.x));
            shown.points.push_back(static_cast<float>(p.y));
            shown.points.push_back(static_cast<float>(p.z));
        };
        if (edge.curve && edge.curve->degree() > 1) {
            constexpr int kSteps = 16;
            const double t0 = edge.curve->tMin();
            const double t1 = edge.curve->tMax();
            for (int k = 0; k <= kSteps; ++k) {
                add(edge.curve->evaluate(t0 + (t1 - t0) * static_cast<double>(k) / kSteps));
            }
        } else {
            add(he->origin->point);
            add(he->next->origin->point);
        }
        out.edges.push_back(std::move(shown));
    }
}

}  // namespace

geo::MeshData SolidTessellator::tessellate(const topo::Solid& solid, double tolerance) {
    geo::MeshData result;

    // extractFacePolygons keeps face order and normalizes loop orientation
    // globally, so loop-triangle normals face outward. Each triangle records
    // the face it came from, by name, so a click on it names the face.
    const auto polygons = BoundaryMesh::extractFacePolygons(solid);
    for (const auto& poly : polygons) {
        if (poly.surface && !isPlanarPatch(*poly.surface)) {
            appendSurfaceTriangles(*poly.surface, tolerance, result);
        } else {
            appendLoopTriangles(poly, result);
        }
        const auto face = static_cast<uint32_t>(result.faceTags.size());
        result.faceTags.push_back(poly.topoId.tag());
        result.triangleFaces.resize(result.indices.size() / 3, face);
    }
    appendEdges(solid, result);

    return result;
}

}  // namespace hz::model
