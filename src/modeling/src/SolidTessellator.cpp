#include "horizon/modeling/SolidTessellator.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

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

}  // namespace

geo::MeshData SolidTessellator::tessellate(const topo::Solid& solid, double tolerance) {
    geo::MeshData result;

    // extractFacePolygons keeps face order and normalizes loop orientation
    // globally, so loop-triangle normals face outward.
    const auto polygons = BoundaryMesh::extractFacePolygons(solid);
    for (const auto& poly : polygons) {
        if (poly.surface && !isPlanarPatch(*poly.surface)) {
            appendSurfaceTriangles(*poly.surface, tolerance, result);
        } else {
            appendLoopTriangles(poly, result);
        }
    }

    return result;
}

}  // namespace hz::model
