#pragma once

#include "Tool.h"

namespace hz::ui {

class TextTool : public Tool {
public:
    std::string name() const override { return "Text"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::string promptText() const override;
    bool wantsCrosshair() const override;
};

}  // namespace hz::ui
