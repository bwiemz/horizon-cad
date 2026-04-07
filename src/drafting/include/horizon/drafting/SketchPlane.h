#pragma once

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"

namespace hz::draft {

/// Defines a local 2D coordinate frame embedded in 3D space.
/// Origin + orthonormal basis (xAxis, yAxis, normal).
class SketchPlane {
public:
    /// Default: XY plane at origin (normal=Z, xAxis=X, yAxis=Y).
    SketchPlane();

    /// Custom plane. yAxis is derived as normal × xAxis.
    /// xAxis must be perpendicular to normal (it will be orthogonalized if not exactly).
    SketchPlane(const math::Vec3& origin, const math::Vec3& normal,
                const math::Vec3& xAxis);

    // Accessors
    const math::Vec3& origin() const { return m_origin; }
    const math::Vec3& normal() const { return m_normal; }
    const math::Vec3& xAxis() const { return m_xAxis; }
    const math::Vec3& yAxis() const { return m_yAxis; }

    // Coordinate transforms
    [[nodiscard]] math::Vec3 localToWorld(const math::Vec2& local) const;
    [[nodiscard]] math::Vec2 worldToLocal(const math::Vec3& world) const;

    // Transform matrices
    [[nodiscard]] math::Mat4 localToWorldMatrix() const;
    [[nodiscard]] math::Mat4 worldToLocalMatrix() const;

    /// Project a 3D ray onto this plane. Returns the 2D local coordinates
    /// of the intersection point (for mouse picking).
    /// Returns false if the ray is parallel to the plane.
    [[nodiscard]] bool rayIntersect(const math::Vec3& rayOrigin,
                                     const math::Vec3& rayDir,
                                     math::Vec2& localResult) const;

private:
    math::Vec3 m_origin;
    math::Vec3 m_normal;
    math::Vec3 m_xAxis;
    math::Vec3 m_yAxis;
};

}  // namespace hz::draft
