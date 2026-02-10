#pragma once

#include "horizon/ui/Tool.h"

namespace hz::ui {

/// Selection tool: click to select entities, Shift+click for multi-select,
/// Delete/Backspace to remove selected entities.
class SelectTool : public Tool {
public:
    std::string name() const override { return "Select"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
};

}  // namespace hz::ui
