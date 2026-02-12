#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"
#include <vector>

namespace hz::ui {

/// Multi-click spline drawing tool.
///
/// - Each left click adds a control point
/// - Enter or double-click finishes the spline (minimum 4 control points)
/// - Escape cancels
class SplineTool : public Tool {
public:
    std::string name() const override { return "Spline"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;

private:
    void finishSpline();

    /// Evaluate a temporary B-spline from the given control points for preview.
    static std::vector<math::Vec2> evaluatePreview(const std::vector<math::Vec2>& cps);

    std::vector<math::Vec2> m_controlPoints;
    math::Vec2 m_currentPos;
    bool m_active = false;
};

}  // namespace hz::ui
