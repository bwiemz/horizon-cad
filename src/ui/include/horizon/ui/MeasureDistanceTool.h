#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Measure the distance between two points.
/// Click first point, then second point.  Result shown in status bar.
class MeasureDistanceTool : public Tool {
public:
    std::string name() const override { return "MeasureDistance"; }

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
    enum class State { First, Second };
    State m_state = State::First;
    math::Vec2 m_firstPoint;
    math::Vec2 m_currentPos;
};

}  // namespace hz::ui
