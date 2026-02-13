#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

/// Extend tool: click near an endpoint of an entity to extend it to the
/// nearest boundary entity.
///
/// The entity's closest endpoint to the cursor is extended along its natural
/// direction (line direction or arc curve) until it meets another entity.
class ExtendTool : public Tool {
public:
    std::string name() const override { return "Extend"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;
};

}  // namespace hz::ui
