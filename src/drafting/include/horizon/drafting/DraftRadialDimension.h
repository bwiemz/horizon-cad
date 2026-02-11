#pragma once

#include "DraftDimension.h"

namespace hz::draft {

/// A dimension measuring radius or diameter of a circle/arc.
class DraftRadialDimension : public DraftDimension {
public:
    DraftRadialDimension(const math::Vec2& center, double radius,
                         const math::Vec2& textPoint, bool isDiameter);

    const math::Vec2& center() const { return m_center; }
    double radius() const { return m_radius; }
    const math::Vec2& textPoint() const { return m_textPoint; }
    bool isDiameter() const { return m_isDiameter; }

    // DraftDimension overrides
    double computedValue() const override;
    std::string displayText(const DimensionStyle& style) const override;
    math::Vec2 textPosition() const override;

    std::vector<std::pair<math::Vec2, math::Vec2>>
        extensionLines(const DimensionStyle& style) const override;
    std::vector<std::pair<math::Vec2, math::Vec2>>
        dimensionLines(const DimensionStyle& style) const override;
    std::vector<std::pair<math::Vec2, math::Vec2>>
        arrowheadLines(const DimensionStyle& style) const override;

    // DraftEntity overrides
    math::BoundingBox boundingBox() const override;
    bool hitTest(const math::Vec2& point, double tolerance) const override;
    std::vector<math::Vec2> snapPoints() const override;
    void translate(const math::Vec2& delta) override;
    std::shared_ptr<DraftEntity> clone() const override;
    void mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) override;
    void rotate(const math::Vec2& center, double angle) override;
    void scale(const math::Vec2& center, double factor) override;

private:
    /// Point on the circle boundary in the direction of textPoint.
    math::Vec2 boundaryPoint() const;

    math::Vec2 m_center;
    double m_radius;
    math::Vec2 m_textPoint;
    bool m_isDiameter;
};

}  // namespace hz::draft
