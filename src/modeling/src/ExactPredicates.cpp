#include "horizon/modeling/ExactPredicates.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "horizon/geometry/MeshData.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

int ExactPredicates::orient3D(const math::Vec3& planePoint, const math::Vec3& planeNormal,
                              const math::Vec3& testPoint, double tolerance) {
    double dot = (testPoint - planePoint).dot(planeNormal);
    if (std::abs(dot) < tolerance) return 0;
    return dot > 0.0 ? 1 : -1;
}

namespace {

/// Moller-Trumbore ray-triangle intersection test.
/// Returns true if the ray hits the triangle at parameter t > epsilon (forward only).
bool rayTriangleIntersect(const math::Vec3& rayOrigin, const math::Vec3& rayDir,
                          const math::Vec3& v0, const math::Vec3& v1, const math::Vec3& v2,
                          double& t) {
    math::Vec3 e1 = v1 - v0;
    math::Vec3 e2 = v2 - v0;
    math::Vec3 h = rayDir.cross(e2);
    double a = e1.dot(h);
    if (std::abs(a) < 1e-15) return false;
    double f = 1.0 / a;
    math::Vec3 s = rayOrigin - v0;
    double u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return false;
    math::Vec3 q = s.cross(e1);
    double v = f * rayDir.dot(q);
    if (v < 0.0 || u + v > 1.0) return false;
    t = f * e2.dot(q);
    return t > 1e-10;  // Only forward intersections.
}

}  // namespace

std::vector<math::Vec3> ExactPredicates::tessellateSolid(const topo::Solid& solid, double tessTol) {
    // Reuse the display tessellator so classification and rendering agree on
    // the solid's boundary: planar bounding-rectangle patches are triangulated
    // from their trimmed loops (they otherwise over-cover, see Extrude), while
    // curved faces keep smooth surface tessellation — the latter matters for
    // ray-parity on cylinders/spheres/tori, whose coarse box-topology loops
    // enclose little-to-no volume (a torus's are coplanar). Expand the indexed
    // mesh into a flat triangle-corner list.
    const geo::MeshData mesh = SolidTessellator::tessellate(solid, tessTol);
    std::vector<math::Vec3> triangles;
    triangles.reserve(mesh.indices.size());
    for (uint32_t idx : mesh.indices) {
        const size_t base = static_cast<size_t>(idx) * 3;
        if (base + 2 >= mesh.positions.size()) break;
        triangles.emplace_back(mesh.positions[base], mesh.positions[base + 1],
                               mesh.positions[base + 2]);
    }
    return triangles;
}

int ExactPredicates::classifyPointAgainstMesh(const math::Vec3& point,
                                              const std::vector<math::Vec3>& triangles,
                                              double tolerance) {
    // Use a slightly off-axis ray direction to reduce the chance of
    // hitting edges or vertices (which would produce degenerate intersections).
    const math::Vec3 rayDir = math::Vec3(1.0, 0.1, 0.01).normalized();

    int hitCount = 0;
    for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
        const math::Vec3& v0 = triangles[i];
        const math::Vec3& v1 = triangles[i + 1];
        const math::Vec3& v2 = triangles[i + 2];

        // Check if the query point is very close to this triangle (boundary).
        if (point.distanceTo(v0) < tolerance || point.distanceTo(v1) < tolerance ||
            point.distanceTo(v2) < tolerance) {
            return 0;  // On boundary.
        }

        double t = 0.0;
        if (rayTriangleIntersect(point, rayDir, v0, v1, v2, t)) {
            ++hitCount;
        }
    }

    return (hitCount % 2 == 1) ? -1 : 1;
}

int ExactPredicates::classifyPoint(const math::Vec3& point, const topo::Solid& solid,
                                   double tolerance) {
    // The `tolerance` parameter controls boundary detection, not mesh density;
    // a coarse tessellation is sufficient for ray-casting classification.
    return classifyPointAgainstMesh(point, tessellateSolid(solid), tolerance);
}

}  // namespace hz::model
