#include "horizon/geometry/curves/Line2D.h"

namespace hz::geo {

Line2D::Line2D(const math::Vec2& start, const math::Vec2& end)
    : m_start(start), m_end(end) {}

math::Vec2 Line2D::evaluate(double t) const {
    return m_start + (m_end - m_start) * t;
}

math::Vec2 Line2D::derivative(double t, int order) const {
    (void)t;
    if (order == 1) {
        return m_end - m_start;
    }
    // Higher-order derivatives of a line are zero
    return math::Vec2(0.0, 0.0);
}

double Line2D::length() const {
    return m_start.distanceTo(m_end);
}

}  // namespace hz::geo
