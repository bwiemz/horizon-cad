#pragma once

#include "Tool.h"

namespace hz::ui {

/// Tool for creating radial or diameter dimensions on circles/arcs.
/// Click a circle/arc, then click to position the text.
class RadialDimensionTool : public Tool {
public:
    std::string name() const override { return "Radial Dimension"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

private:
    enum class State { WaitingForCircle, WaitingForTextPos };

    State m_state = State::WaitingForCircle;
    math::Vec2 m_center;
    double m_radius = 0.0;
    math::Vec2 m_currentPos;
    bool m_isDiameter = false;
};

}  // namespace hz::ui
