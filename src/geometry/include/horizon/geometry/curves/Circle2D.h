#pragma once

#include "Curve2D.h"

namespace hz::geo {

class Circle2D : public Curve2D {
public:
    Circle2D(const math::Vec2& center, double radius);

    math::Vec2 evaluate(double t) const override;
    math::Vec2 derivative(double t, int order) const override;
    double tMin() const override { return 0.0; }
    double tMax() const override;  // 2*pi
    bool isClosed() const override { return true; }
    double length() const override;

    const math::Vec2& center() const { return m_center; }
    double radius() const { return m_radius; }

private:
    math::Vec2 m_center;
    double m_radius;
};

}  // namespace hz::geo
