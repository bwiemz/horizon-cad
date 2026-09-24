#pragma once

#include "horizon/math/Vec2.h"
#include "horizon/ui/Tool.h"

namespace hz::ui {

/// Line drawing tool, chaining: each click ends a line and starts the next
/// one there.
///
/// - First click: set start point
/// - Mouse move: preview rubber-band line from start to cursor
/// - Each next click: add a line, and continue from its end
/// - Enter or Escape: finish (the lines drawn stay)
///
/// Points can also be typed (TypedPoint): x,y; @dx,dy or @length<angle from
/// the last point; or a length toward the cursor.
class LineTool : public Tool {
public:
    std::string name() const override { return "Line"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

    bool acceptsTypedPoints() const override { return true; }
    std::optional<math::Vec2> basePoint() const override {
        if (m_state != State::WaitingForEnd) return std::nullopt;
        return m_startPoint;
    }

    std::string promptText() const override;
    bool wantsCrosshair() const override;

private:
    enum class State { WaitingForStart, WaitingForEnd };
    State m_state = State::WaitingForStart;
    math::Vec2 m_startPoint;
    math::Vec2 m_currentPos;
};

}  // namespace hz::ui
