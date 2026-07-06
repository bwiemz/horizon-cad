#include "horizon/render/FrustumCuller.h"

#include <cmath>

namespace hz::render {

using hz::math::BoundingBox;
using hz::math::Mat4;
using hz::math::Vec3;

namespace {

FrustumPlane normalized(double a, double b, double c, double d) {
    const double len = std::sqrt(a * a + b * b + c * c);
    if (len < 1e-12) return {0.0, 0.0, 0.0, 0.0};
    return {a / len, b / len, c / len, d / len};
}

/// Positive-vertex test: the box corner farthest along the plane normal.
/// If even that corner is behind the plane, the whole box is outside.
bool outsidePlane(const BoundingBox& box, const FrustumPlane& p) {
    const Vec3 v(p.a >= 0.0 ? box.max().x : box.min().x, p.b >= 0.0 ? box.max().y : box.min().y,
                 p.c >= 0.0 ? box.max().z : box.min().z);
    return p.a * v.x + p.b * v.y + p.c * v.z + p.d < 0.0;
}

}  // namespace

std::array<FrustumPlane, 6> FrustumCuller::extractPlanes(const Mat4& viewProj) {
    const auto row = [&viewProj](int i, int j) { return viewProj.at(i, j); };
    std::array<FrustumPlane, 6> planes;
    for (int i = 0; i < 3; ++i) {
        // row3 + rowI (left/bottom/near), row3 - rowI (right/top/far).
        planes[2 * i] = normalized(row(3, 0) + row(i, 0), row(3, 1) + row(i, 1),
                                   row(3, 2) + row(i, 2), row(3, 3) + row(i, 3));
        planes[2 * i + 1] = normalized(row(3, 0) - row(i, 0), row(3, 1) - row(i, 1),
                                       row(3, 2) - row(i, 2), row(3, 3) - row(i, 3));
    }
    return planes;
}

BoundingBox FrustumCuller::meshBounds(const MeshData& mesh) {
    BoundingBox box;
    for (size_t i = 0; i + 2 < mesh.positions.size(); i += 3) {
        box.expand(Vec3(mesh.positions[i], mesh.positions[i + 1], mesh.positions[i + 2]));
    }
    return box;
}

bool FrustumCuller::intersects(const BoundingBox& worldBox,
                               const std::array<FrustumPlane, 6>& planes) {
    if (!worldBox.isValid()) return false;
    for (const FrustumPlane& p : planes) {
        // Skip degenerate planes (e.g. from a singular matrix): keep, stay conservative.
        if (p.a == 0.0 && p.b == 0.0 && p.c == 0.0) continue;
        if (outsidePlane(worldBox, p)) return false;
    }
    return true;
}

size_t FrustumCuller::cullBatch(InstanceBatch& batch, const BoundingBox& localBounds,
                                const std::array<FrustumPlane, 6>& planes) {
    if (!localBounds.isValid()) return 0;

    size_t kept = 0;
    const size_t count = batch.transforms.size();
    for (size_t i = 0; i < count; ++i) {
        // World AABB: transform the 8 local corners, re-wrap axis-aligned.
        BoundingBox world;
        for (int c = 0; c < 8; ++c) {
            const Vec3 corner((c & 1) ? localBounds.max().x : localBounds.min().x,
                              (c & 2) ? localBounds.max().y : localBounds.min().y,
                              (c & 4) ? localBounds.max().z : localBounds.min().z);
            world.expand(batch.transforms[i].transformPoint(corner));
        }
        if (!intersects(world, planes)) continue;

        if (kept != i) {
            batch.transforms[kept] = batch.transforms[i];
            batch.materials[kept] = batch.materials[i];
            batch.nodes[kept] = batch.nodes[i];
        }
        ++kept;
    }
    const size_t culled = count - kept;
    batch.transforms.resize(kept);
    batch.materials.resize(kept);
    batch.nodes.resize(kept);
    return culled;
}

}  // namespace hz::render
