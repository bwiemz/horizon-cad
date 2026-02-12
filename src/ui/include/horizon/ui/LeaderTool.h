#pragma once

#include "Tool.h"
#include <vector>

namespace hz::ui {

/// Tool for creating leader annotations (polyline with arrowhead + text).
/// Multi-click to add points; Enter or double-click to finish and enter text.
class LeaderTool : public Tool {
public:
    std::string name() const override { return "Leader"; }

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
    void finishLeader();

    std::vector<math::Vec2> m_points;
    math::Vec2 m_currentPos;
    bool m_active = false;
};

}  // namespace hz::ui
