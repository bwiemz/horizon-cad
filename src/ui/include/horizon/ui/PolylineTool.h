#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"
#include <vector>

namespace hz::ui {

/// Multi-click polyline drawing tool.
///
/// - Each left click adds a vertex
/// - Enter or double-click finishes the polyline
/// - Escape cancels
class PolylineTool : public Tool {
public:
    std::string name() const override { return "Polyline"; }

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
    void finishPolyline();

    std::vector<math::Vec2> m_points;
    math::Vec2 m_currentPos;
    bool m_active = false;
};

}  // namespace hz::ui
