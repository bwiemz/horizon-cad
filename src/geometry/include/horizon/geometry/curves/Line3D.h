#pragma once

#include "horizon/math/Vec3.h"

namespace hz::geo {

class Line3D {
public:
    Line3D(const math::Vec3& origin, const math::Vec3& direction);

    math::Vec3 evaluate(double t) const;
    math::Vec3 pointAt(double t) const { return evaluate(t); }

    const math::Vec3& origin() const { return m_origin; }
    const math::Vec3& direction() const { return m_direction; }

private:
    math::Vec3 m_origin;
    math::Vec3 m_direction;
};

}  // namespace hz::geo
