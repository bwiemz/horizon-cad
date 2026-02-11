#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"

namespace hz::ui {

class Clipboard;

/// Paste tool: activated after Ctrl+V.  Click to place pasted entities.
class PasteTool : public Tool {
public:
    explicit PasteTool(Clipboard* clipboard);

    std::string name() const override { return "Paste"; }

    void deactivate() override;
    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;
    std::vector<std::pair<math::Vec2, double>> getPreviewCircles() const override;

private:
    Clipboard* m_clipboard;
    math::Vec2 m_currentPos;
};

}  // namespace hz::ui
