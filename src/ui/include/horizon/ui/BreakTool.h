#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Break tool: click on an entity to split it at the nearest intersection point.
///
/// The entity is split into two pieces at the break point. If there are
/// intersection points with other entities, the nearest one is used.
/// Otherwise, the closest point on the entity to the cursor is used.
class BreakTool : public Tool {
public:
    std::string name() const override { return "Break"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;
};

}  // namespace hz::ui
