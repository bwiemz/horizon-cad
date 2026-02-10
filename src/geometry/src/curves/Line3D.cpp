#include "horizon/geometry/curves/Line3D.h"

namespace hz::geo {

Line3D::Line3D(const math::Vec3& origin, const math::Vec3& direction)
    : m_origin(origin), m_direction(direction) {}

math::Vec3 Line3D::evaluate(double t) const {
    return m_origin + m_direction * t;
}

}  // namespace hz::geo
