#include "horizon/drafting/DraftCircle.h"
#include <cmath>

namespace hz::draft {

DraftCircle::DraftCircle(const math::Vec2& center, double radius)
    : m_center(center), m_radius(radius) {}

math::BoundingBox DraftCircle::boundingBox() const {
    math::Vec3 lo(m_center.x - m_radius, m_center.y - m_radius, 0.0);
    math::Vec3 hi(m_center.x + m_radius, m_center.y + m_radius, 0.0);
    return math::BoundingBox(lo, hi);
}

bool DraftCircle::hitTest(const math::Vec2& point, double tolerance) const {
    // Distance from point to circle edge
    double distToCenter = point.distanceTo(m_center);
    double distToEdge = std::abs(distToCenter - m_radius);
    return distToEdge <= tolerance;
}

std::vector<math::Vec2> DraftCircle::snapPoints() const {
    // Center + 4 quadrant points (right, top, left, bottom)
    return {
        m_center,
        math::Vec2(m_center.x + m_radius, m_center.y),           // right (0 deg)
        math::Vec2(m_center.x,             m_center.y + m_radius), // top (90 deg)
        math::Vec2(m_center.x - m_radius, m_center.y),           // left (180 deg)
        math::Vec2(m_center.x,             m_center.y - m_radius)  // bottom (270 deg)
    };
}

void DraftCircle::translate(const math::Vec2& delta) {
    m_center += delta;
}

}  // namespace hz::draft
