#pragma once

#include "DraftEntity.h"
#include "DimensionStyle.h"
#include <string>
#include <utility>
#include <vector>

namespace hz::draft {

/// Abstract base class for all dimension/annotation entities.
class DraftDimension : public DraftEntity {
public:
    DraftDimension();
    ~DraftDimension() override = default;

    // ---- Text override ----

    const std::string& textOverride() const { return m_textOverride; }
    void setTextOverride(const std::string& text) { m_textOverride = text; }
    bool hasTextOverride() const { return !m_textOverride.empty(); }

    /// Returns the formatted display text (override or computed value).
    virtual std::string displayText(const DimensionStyle& style) const;

    // ---- Measurement ----

    /// Returns the computed measurement value (distance, angle, etc.).
    virtual double computedValue() const = 0;

    // ---- Geometry for rendering ----

    /// World-space position where the text should be drawn.
    virtual math::Vec2 textPosition() const = 0;

    /// Extension lines (from definition points toward the dimension line).
    virtual std::vector<std::pair<math::Vec2, math::Vec2>>
        extensionLines(const DimensionStyle& style) const = 0;

    /// Dimension line(s) — the main measured line/arc.
    virtual std::vector<std::pair<math::Vec2, math::Vec2>>
        dimensionLines(const DimensionStyle& style) const = 0;

    /// Arrowhead line segments (two lines forming a "V" at each arrowhead).
    virtual std::vector<std::pair<math::Vec2, math::Vec2>>
        arrowheadLines(const DimensionStyle& style) const = 0;

protected:
    /// Helper: generate arrowhead "V" lines at a point, pointing along a direction.
    static std::vector<std::pair<math::Vec2, math::Vec2>>
        makeArrowhead(const math::Vec2& tip, const math::Vec2& direction,
                      double size, double halfAngle);

    std::string m_textOverride;
};

}  // namespace hz::draft
