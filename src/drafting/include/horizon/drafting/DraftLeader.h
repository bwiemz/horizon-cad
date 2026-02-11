#pragma once

#include "DraftDimension.h"

namespace hz::draft {

/// A leader annotation: polyline with arrowhead at the start and text at the end.
class DraftLeader : public DraftDimension {
public:
    DraftLeader(const std::vector<math::Vec2>& points, const std::string& text);

    const std::vector<math::Vec2>& points() const { return m_points; }
    const std::string& text() const { return m_text; }
    void setText(const std::string& text) { m_text = text; }

    // DraftDimension overrides
    double computedValue() const override;  // always 0 (not a measurement)
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
    std::vector<math::Vec2> m_points;  // polyline; first = arrow tip, last = text anchor
    std::string m_text;
};

}  // namespace hz::draft
