#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Trim tool: click on a segment of an entity to remove it at intersections.
///
/// The entity is split at intersection points with other entities,
/// and the segment under the cursor is removed.
class TrimTool : public Tool {
public:
    std::string name() const override { return "Trim"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;
};

}  // namespace hz::ui
