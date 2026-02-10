#pragma once

#include "horizon/ui/Tool.h"

namespace hz::ui {

/// Basic selection tool (stub for Phase 1).
class SelectTool : public Tool {
public:
    std::string name() const override { return "Select"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
};

}  // namespace hz::ui
