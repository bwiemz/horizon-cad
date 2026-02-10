#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Two-click line drawing tool.
///
/// - First click: set start point
/// - Mouse move: preview rubber-band line from start to cursor
/// - Second click: finalize line and add it to the viewport
/// - Escape: cancel current line
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

private:
    enum class State { WaitingForStart, WaitingForEnd };
    State m_state = State::WaitingForStart;
    math::Vec2 m_startPoint;
    math::Vec2 m_currentPos;
};

}  // namespace hz::ui
