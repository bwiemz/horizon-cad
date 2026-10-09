#pragma once

#include <array>

#include "DraftEntity.h"

namespace hz::draft {

class DraftRectangle : public DraftEntity {
public:
    DraftRectangle(const math::Vec2& corner1, const math::Vec2& corner2);

    math::BoundingBox boundingBox() const override;
    bool hitTest(const math::Vec2& point, double tolerance) const override;
    std::vector<math::Vec2> snapPoints() const override;
    std::vector<SnapPoint> typedSnapPoints() const override;
    void translate(const math::Vec2& delta) override;
    std::shared_ptr<DraftEntity> clone() const override;
    /// Mirror and rotate keep the rectangle's sides along the axes, so they
    /// are exact only when they turn the axes onto the axes: a mirror in a
    /// line along an axis or a diagonal, a quarter turn. mirroredCopy() and
    /// rotatedCopy() give a polyline for any other.
    void mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) override;
    void rotate(const math::Vec2& center, double angle) override;
    void scale(const math::Vec2& center, double factor) override;
    std::shared_ptr<DraftEntity> rotatedCopy(const math::Vec2& center, double angle) const override;
    std::shared_ptr<DraftEntity> mirroredCopy(const math::Vec2& axisP1,
                                              const math::Vec2& axisP2) const override;

    const math::Vec2& corner1() const { return m_corner1; }
    const math::Vec2& corner2() const { return m_corner2; }

    void setCorner1(const math::Vec2& c) { m_corner1 = c; }
    void setCorner2(const math::Vec2& c) { m_corner2 = c; }

    /// Returns the 4 corners: bottom-left, bottom-right, top-right, top-left.
    std::array<math::Vec2, 4> corners() const;

private:
    /// A closed polyline through @p points, in this rectangle's style.
    std::shared_ptr<DraftEntity> outline(const std::array<math::Vec2, 4>& points) const;

    math::Vec2 m_corner1;
    math::Vec2 m_corner2;
};

}  // namespace hz::draft
