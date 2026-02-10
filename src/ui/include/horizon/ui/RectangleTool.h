#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Two-click rectangle drawing tool.
///
/// - First click: set first corner
/// - Mouse move: preview rectangle outline
/// - Second click: finalize opposite corner
/// - Escape: cancel
class RectangleTool : public Tool {
public:
    std::string name() const override { return "Rectangle"; }

    void activate(ViewportWidget* viewport) override;
    void deactivate() override;

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

private:
    enum class State { WaitingForFirstCorner, WaitingForSecondCorner };
    State m_state = State::WaitingForFirstCorner;
    math::Vec2 m_firstCorner;
    math::Vec2 m_currentPos;
};

}  // namespace hz::ui
