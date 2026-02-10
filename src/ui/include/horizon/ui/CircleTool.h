#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Two-click circle drawing tool.
///
/// - First click: set center
/// - Mouse move: preview circle with radius equal to distance from center to cursor
/// - Second click: finalize circle and add it to the viewport
/// - Escape: cancel current circle
class CircleTool : public Tool {
public:
    std::string name() const override { return "Circle"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, double>> getPreviewCircles() const override;

private:
    enum class State { WaitingForCenter, WaitingForRadius };
    State m_state = State::WaitingForCenter;
    math::Vec2 m_center;
    math::Vec2 m_currentPos;
};

}  // namespace hz::ui
