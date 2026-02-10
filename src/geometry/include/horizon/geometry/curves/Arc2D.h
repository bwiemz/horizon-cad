#pragma once

#include "Curve2D.h"

namespace hz::geo {

class Arc2D : public Curve2D {
public:
    Arc2D(const math::Vec2& center, double radius, double startAngle, double endAngle);

    math::Vec2 evaluate(double t) const override;
    math::Vec2 derivative(double t, int order) const override;
    double tMin() const override { return 0.0; }
    double tMax() const override { return 1.0; }
    bool isClosed() const override { return false; }

    const math::Vec2& center() const { return m_center; }
    double radius() const { return m_radius; }
    double startAngle() const { return m_startAngle; }
    double endAngle() const { return m_endAngle; }

private:
    math::Vec2 m_center;
    double m_radius, m_startAngle, m_endAngle;
};

}  // namespace hz::geo
