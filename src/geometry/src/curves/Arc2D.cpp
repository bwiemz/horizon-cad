#include "horizon/geometry/curves/Arc2D.h"
#include "horizon/math/Constants.h"
#include <cmath>

namespace hz::geo {

Arc2D::Arc2D(const math::Vec2& center, double radius,
             double startAngle, double endAngle)
    : m_center(center)
    , m_radius(radius)
    , m_startAngle(startAngle)
    , m_endAngle(endAngle) {}

math::Vec2 Arc2D::evaluate(double t) const {
    // Map t in [0, 1] to angle in [startAngle, endAngle]
    double angle = m_startAngle + t * (m_endAngle - m_startAngle);
    return math::Vec2(
        m_center.x + m_radius * std::cos(angle),
        m_center.y + m_radius * std::sin(angle)
    );
}

math::Vec2 Arc2D::derivative(double t, int order) const {
    double angle = m_startAngle + t * (m_endAngle - m_startAngle);
    double dAngle = m_endAngle - m_startAngle;  // dt -> dangle conversion factor

    if (order == 1) {
        // d/dt = d/dangle * dangle/dt
        // d/dangle of (center + r*cos(a), center + r*sin(a)) = (-r*sin(a), r*cos(a))
        // multiply by dangle/dt = (endAngle - startAngle)
        return math::Vec2(
            -m_radius * std::sin(angle) * dAngle,
             m_radius * std::cos(angle) * dAngle
        );
    }
    if (order == 2) {
        double dAngle2 = dAngle * dAngle;
        return math::Vec2(
            -m_radius * std::cos(angle) * dAngle2,
            -m_radius * std::sin(angle) * dAngle2
        );
    }
    // For higher orders, zero is a reasonable approximation
    return math::Vec2(0.0, 0.0);
}

}  // namespace hz::geo
