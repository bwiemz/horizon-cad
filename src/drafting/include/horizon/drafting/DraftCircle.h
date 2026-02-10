#pragma once

#include "DraftEntity.h"

namespace hz::draft {

class DraftCircle : public DraftEntity {
public:
    DraftCircle(const math::Vec2& center, double radius);

    math::BoundingBox boundingBox() const override;
    bool hitTest(const math::Vec2& point, double tolerance) const override;
    std::vector<math::Vec2> snapPoints() const override;
    void translate(const math::Vec2& delta) override;
    std::shared_ptr<DraftEntity> clone() const override;
    void mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) override;

    const math::Vec2& center() const { return m_center; }
    double radius() const { return m_radius; }

    void setCenter(const math::Vec2& center) { m_center = center; }
    void setRadius(double radius) { m_radius = radius; }

private:
    math::Vec2 m_center;
    double m_radius;
};

}  // namespace hz::draft
