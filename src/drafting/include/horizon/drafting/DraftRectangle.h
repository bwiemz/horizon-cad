#pragma once

#include "DraftEntity.h"
#include <array>

namespace hz::draft {

class DraftRectangle : public DraftEntity {
public:
    DraftRectangle(const math::Vec2& corner1, const math::Vec2& corner2);

    math::BoundingBox boundingBox() const override;
    bool hitTest(const math::Vec2& point, double tolerance) const override;
    std::vector<math::Vec2> snapPoints() const override;
    void translate(const math::Vec2& delta) override;

    const math::Vec2& corner1() const { return m_corner1; }
    const math::Vec2& corner2() const { return m_corner2; }

    void setCorner1(const math::Vec2& c) { m_corner1 = c; }
    void setCorner2(const math::Vec2& c) { m_corner2 = c; }

    /// Returns the 4 corners: bottom-left, bottom-right, top-right, top-left.
    std::array<math::Vec2, 4> corners() const;

private:
    math::Vec2 m_corner1;
    math::Vec2 m_corner2;
};

}  // namespace hz::draft
