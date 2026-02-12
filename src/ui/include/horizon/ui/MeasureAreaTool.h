#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"
#include <vector>

namespace hz::ui {

/// Measure the area of a polygon defined by clicking vertices.
/// Click to add vertices, Enter or double-click to finish (min 3 points).
/// Escape cancels.  Result shown in status bar.
class MeasureAreaTool : public Tool {
public:
    std::string name() const override { return "MeasureArea"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

private:
    void finishMeasure();

    /// Compute area of a polygon using the shoelace formula.
    static double shoelaceArea(const std::vector<math::Vec2>& pts);

    std::vector<math::Vec2> m_points;
    math::Vec2 m_currentPos;
    bool m_active = false;
};

}  // namespace hz::ui
