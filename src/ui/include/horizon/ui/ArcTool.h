#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Three-click arc drawing tool.
///
/// - First click: set center
/// - Second click: set radius + start angle
/// - Third click: set end angle, create arc
/// - Escape: cancel
class ArcTool : public Tool {
public:
    std::string name() const override { return "Arc"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;
    std::vector<ArcPreview> getPreviewArcs() const override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;

private:
    enum class State { WaitingForCenter, WaitingForStart, WaitingForEnd };
    State m_state = State::WaitingForCenter;
    math::Vec2 m_center;
    double m_radius = 0.0;
    double m_startAngle = 0.0;
    math::Vec2 m_currentPos;
};

}  // namespace hz::ui
