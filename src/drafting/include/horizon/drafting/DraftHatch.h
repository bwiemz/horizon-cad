#pragma once

#include "DraftEntity.h"
#include <utility>
#include <vector>

namespace hz::draft {

enum class HatchPattern {
    Solid,       // 0 - dense fill lines
    Lines,       // 1 - parallel lines
    CrossHatch   // 2 - two perpendicular sets of lines
};

/// A hatched region defined by a closed boundary polygon.
class DraftHatch : public DraftEntity {
public:
    explicit DraftHatch(const std::vector<math::Vec2>& boundary,
                        HatchPattern pattern = HatchPattern::Lines,
                        double angle = 0.0,
                        double spacing = 1.0);

    math::BoundingBox boundingBox() const override;
    bool hitTest(const math::Vec2& point, double tolerance) const override;
    std::vector<math::Vec2> snapPoints() const override;
    void translate(const math::Vec2& delta) override;
    std::shared_ptr<DraftEntity> clone() const override;
    void mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) override;
    void rotate(const math::Vec2& center, double angle) override;
    void scale(const math::Vec2& center, double factor) override;

    const std::vector<math::Vec2>& boundary() const { return m_boundary; }
    void setBoundary(const std::vector<math::Vec2>& boundary) { m_boundary = boundary; }

    HatchPattern pattern() const { return m_pattern; }
    void setPattern(HatchPattern pattern) { m_pattern = pattern; }

    double angle() const { return m_angle; }
    void setAngle(double angle) { m_angle = angle; }

    double spacing() const { return m_spacing; }
    void setSpacing(double spacing) { m_spacing = spacing; }

    /// Generate hatch fill lines clipped to the boundary polygon.
    std::vector<std::pair<math::Vec2, math::Vec2>> generateHatchLines() const;

private:
    /// Test if a point is inside the boundary polygon (ray casting).
    bool pointInPolygon(const math::Vec2& point) const;

    /// Generate parallel scan lines at a given angle, clipped to the boundary.
    std::vector<std::pair<math::Vec2, math::Vec2>> scanLines(double scanAngle,
                                                              double scanSpacing) const;

    std::vector<math::Vec2> m_boundary;
    HatchPattern m_pattern;
    double m_angle;    // radians
    double m_spacing;  // world units
};

}  // namespace hz::draft
