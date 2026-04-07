#include "horizon/drafting/SketchPlane.h"

#include <cmath>

namespace hz::draft {

using math::Mat4;
using math::Vec2;
using math::Vec3;

SketchPlane::SketchPlane()
    : m_origin(Vec3::Zero), m_normal(Vec3::UnitZ), m_xAxis(Vec3::UnitX), m_yAxis(Vec3::UnitY) {}

SketchPlane::SketchPlane(const Vec3& origin, const Vec3& normal, const Vec3& xAxis)
    : m_origin(origin) {
    m_normal = normal.normalized();
    // Orthogonalize xAxis against normal (Gram-Schmidt)
    Vec3 rawX = xAxis - m_normal * xAxis.dot(m_normal);
    m_xAxis = rawX.normalized();
    // Derive yAxis for right-handed system: Y = Z × X
    m_yAxis = m_normal.cross(m_xAxis);
}

Vec3 SketchPlane::localToWorld(const Vec2& local) const {
    return m_origin + m_xAxis * local.x + m_yAxis * local.y;
}

Vec2 SketchPlane::worldToLocal(const Vec3& world) const {
    Vec3 delta = world - m_origin;
    return Vec2(delta.dot(m_xAxis), delta.dot(m_yAxis));
}

Mat4 SketchPlane::localToWorldMatrix() const {
    Mat4 m = Mat4::identity();
    // Column 0: xAxis
    m.at(0, 0) = m_xAxis.x;
    m.at(1, 0) = m_xAxis.y;
    m.at(2, 0) = m_xAxis.z;
    // Column 1: yAxis
    m.at(0, 1) = m_yAxis.x;
    m.at(1, 1) = m_yAxis.y;
    m.at(2, 1) = m_yAxis.z;
    // Column 2: normal
    m.at(0, 2) = m_normal.x;
    m.at(1, 2) = m_normal.y;
    m.at(2, 2) = m_normal.z;
    // Column 3: origin (translation)
    m.at(0, 3) = m_origin.x;
    m.at(1, 3) = m_origin.y;
    m.at(2, 3) = m_origin.z;
    return m;
}

Mat4 SketchPlane::worldToLocalMatrix() const {
    return localToWorldMatrix().inverse();
}

bool SketchPlane::rayIntersect(const Vec3& rayOrigin, const Vec3& rayDir,
                                Vec2& localResult) const {
    double denom = rayDir.dot(m_normal);
    if (std::abs(denom) < 1e-12) return false;  // Ray parallel to plane
    double t = (m_origin - rayOrigin).dot(m_normal) / denom;
    Vec3 hit = rayOrigin + rayDir * t;
    localResult = worldToLocal(hit);
    return true;
}

}  // namespace hz::draft
