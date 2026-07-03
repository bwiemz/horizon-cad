#include "horizon/modeling/ExactPredicates.h"

#include <cmath>

#include "horizon/geometry/surfaces/NurbsSurface.h"
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
    std::vector<math::Vec3> triangles;
    for (const auto& face : solid.faces()) {
        if (!face.surface) continue;

        auto tess = face.surface->tessellate(tessTol);
        const auto& pos = tess.positions;
        const auto& idx = tess.indices;

        for (size_t i = 0; i + 2 < idx.size(); i += 3) {
            uint32_t i0 = idx[i];
            uint32_t i1 = idx[i + 1];
            uint32_t i2 = idx[i + 2];
            triangles.emplace_back(pos[i0 * 3], pos[i0 * 3 + 1], pos[i0 * 3 + 2]);
            triangles.emplace_back(pos[i1 * 3], pos[i1 * 3 + 1], pos[i1 * 3 + 2]);
            triangles.emplace_back(pos[i2 * 3], pos[i2 * 3 + 1], pos[i2 * 3 + 2]);
        }
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
