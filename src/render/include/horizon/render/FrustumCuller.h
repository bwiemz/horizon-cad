#pragma once

#include <array>

#include "horizon/math/BoundingBox.h"
#include "horizon/math/Mat4.h"
#include "horizon/render/InstanceBatcher.h"

namespace hz::render {

/// One clip plane in world space: ax + by + cz + d >= 0 is the inside.
struct FrustumPlane {
    double a = 0.0, b = 0.0, c = 0.0, d = 0.0;
};

/// View-frustum culling for instanced batches (Phase 78, roadmap §7.14).
///
/// Planes come from the combined view-projection matrix via the
/// Gribb–Hartmann method (rows of M, column-vector convention). Culling is
/// conservative: a box is dropped only when it is fully outside a plane, so
/// nothing visible is ever culled.
class FrustumCuller {
public:
    /// Extract the six world-space clip planes from @p viewProj.
    static std::array<FrustumPlane, 6> extractPlanes(const math::Mat4& viewProj);

    /// Axis-aligned bounds of a mesh in its local frame.
    static math::BoundingBox meshBounds(const MeshData& mesh);

    /// True when @p worldBox is at least partially inside the frustum.
    static bool intersects(const math::BoundingBox& worldBox,
                           const std::array<FrustumPlane, 6>& planes);

    /// Remove instances of @p batch whose transformed @p localBounds lie
    /// fully outside the frustum. transforms/materials/nodes stay aligned.
    /// Returns the number of instances culled.
    static size_t cullBatch(InstanceBatch& batch, const math::BoundingBox& localBounds,
                            const std::array<FrustumPlane, 6>& planes);
};

}  // namespace hz::render
