#pragma once

#include "Curve2D.h"

namespace hz::geo {

class Line2D : public Curve2D {
public:
    Line2D(const math::Vec2& start, const math::Vec2& end);

    math::Vec2 evaluate(double t) const override;
    math::Vec2 derivative(double t, int order) const override;
    double tMin() const override { return 0.0; }
    double tMax() const override { return 1.0; }
    bool isClosed() const override { return false; }
    double length() const override;

    const math::Vec2& start() const { return m_start; }
    const math::Vec2& end() const { return m_end; }

private:
    math::Vec2 m_start, m_end;
};

}  // namespace hz::geo
