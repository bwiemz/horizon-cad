#pragma once

#include "DraftEntity.h"

namespace hz::draft {

class DraftArc : public DraftEntity {
public:
    DraftArc(const math::Vec2& center, double radius,
             double startAngle, double endAngle);

    math::BoundingBox boundingBox() const override;
    bool hitTest(const math::Vec2& point, double tolerance) const override;
    std::vector<math::Vec2> snapPoints() const override;
    void translate(const math::Vec2& delta) override;
    std::shared_ptr<DraftEntity> clone() const override;
    void mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) override;

    const math::Vec2& center() const { return m_center; }
    double radius() const { return m_radius; }
    double startAngle() const { return m_startAngle; }
    double endAngle() const { return m_endAngle; }

    void setCenter(const math::Vec2& center) { m_center = center; }
    void setRadius(double radius) { m_radius = radius; }
    void setStartAngle(double angle) { m_startAngle = angle; }
    void setEndAngle(double angle) { m_endAngle = angle; }

    math::Vec2 startPoint() const;
    math::Vec2 endPoint() const;
    math::Vec2 midPoint() const;
    double sweepAngle() const;

private:
    bool containsAngle(double angle) const;

    math::Vec2 m_center;
    double m_radius;
    double m_startAngle;  // radians, normalized [0, 2pi)
    double m_endAngle;    // radians, normalized [0, 2pi)
};

}  // namespace hz::draft
