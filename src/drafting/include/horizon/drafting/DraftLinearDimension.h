#pragma once

#include "DraftDimension.h"

namespace hz::draft {

/// A linear dimension measuring horizontal, vertical, or aligned distance.
class DraftLinearDimension : public DraftDimension {
public:
    enum class Orientation { Horizontal, Vertical, Aligned };

    DraftLinearDimension(const math::Vec2& defPoint1, const math::Vec2& defPoint2,
                         const math::Vec2& dimLinePoint, Orientation orientation);

    const math::Vec2& defPoint1() const { return m_defPoint1; }
    const math::Vec2& defPoint2() const { return m_defPoint2; }
    const math::Vec2& dimLinePoint() const { return m_dimLinePoint; }
    Orientation orientation() const { return m_orientation; }

    // DraftDimension overrides
    double computedValue() const override;
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
    /// Compute the two endpoints of the dimension line.
    std::pair<math::Vec2, math::Vec2> dimLineEndpoints() const;

    math::Vec2 m_defPoint1;
    math::Vec2 m_defPoint2;
    math::Vec2 m_dimLinePoint;  // user's third click — determines offset
    Orientation m_orientation;
};

}  // namespace hz::draft
