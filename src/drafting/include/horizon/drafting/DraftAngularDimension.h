#pragma once

#include "DraftDimension.h"

namespace hz::draft {

/// A dimension measuring the angle between two lines.
class DraftAngularDimension : public DraftDimension {
public:
    /// \param vertex     intersection point of the two lines
    /// \param line1Point point on first line (away from vertex, defines direction)
    /// \param line2Point point on second line (away from vertex, defines direction)
    /// \param arcRadius  distance of the dimension arc from vertex
    DraftAngularDimension(const math::Vec2& vertex,
                          const math::Vec2& line1Point,
                          const math::Vec2& line2Point,
                          double arcRadius);

    const math::Vec2& vertex() const { return m_vertex; }
    const math::Vec2& line1Point() const { return m_line1Point; }
    const math::Vec2& line2Point() const { return m_line2Point; }
    double arcRadius() const { return m_arcRadius; }

    // DraftDimension overrides
    double computedValue() const override;  // angle in degrees
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
    /// Angles of the two directions (radians).
    double startAngle() const;
    double endAngle() const;

    math::Vec2 m_vertex;
    math::Vec2 m_line1Point;
    math::Vec2 m_line2Point;
    double m_arcRadius;
};

}  // namespace hz::draft
