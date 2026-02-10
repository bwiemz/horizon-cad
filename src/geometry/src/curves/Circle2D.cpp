#include "horizon/geometry/curves/Circle2D.h"
#include "horizon/math/Constants.h"
#include <cmath>

namespace hz::geo {

Circle2D::Circle2D(const math::Vec2& center, double radius)
    : m_center(center), m_radius(radius) {}

math::Vec2 Circle2D::evaluate(double t) const {
    // t is the angle in radians, range [0, 2*pi)
    return math::Vec2(
        m_center.x + m_radius * std::cos(t),
        m_center.y + m_radius * std::sin(t)
    );
}

math::Vec2 Circle2D::derivative(double t, int order) const {
    // First derivative: tangent direction scaled by radius
    // d/dt (center + r*cos(t), center + r*sin(t)) = (-r*sin(t), r*cos(t))
    switch (order) {
        case 1:
            return math::Vec2(
                -m_radius * std::sin(t),
                 m_radius * std::cos(t)
            );
        case 2:
            return math::Vec2(
                -m_radius * std::cos(t),
                -m_radius * std::sin(t)
            );
        case 3:
            return math::Vec2(
                 m_radius * std::sin(t),
                -m_radius * std::cos(t)
            );
        case 4:
            return evaluate(t) - m_center;  // Same as first derivative pattern repeats
        default:
            // For higher orders, cycle with period 4
            return derivative(t, ((order - 1) % 4) + 1);
    }
}

double Circle2D::tMax() const {
    return math::kTwoPi;
}

double Circle2D::length() const {
    return math::kTwoPi * m_radius;
}

}  // namespace hz::geo
