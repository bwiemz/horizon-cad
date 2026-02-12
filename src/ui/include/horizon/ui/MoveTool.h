#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Move tool: click and drag to move selected entities.
///
/// - Click on a selected entity to begin dragging
/// - Drag to new position, release to commit
/// - Escape cancels the drag
class MoveTool : public Tool {
public:
    std::string name() const override { return "Move"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void deactivate() override;
    void cancel() override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;

private:
    bool m_dragging = false;
    math::Vec2 m_dragStart;
    math::Vec2 m_dragCurrent;
    math::Vec2 m_totalDelta;
};

}  // namespace hz::ui
