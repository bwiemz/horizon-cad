#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"
#include <cstdint>
#include <memory>

namespace hz::draft {
class DraftEntity;
}

namespace hz::ui {

/// Selection tool with grip editing: click to select entities, Shift+click for
/// multi-select, drag grips to reshape, Delete/Backspace to remove.
class SelectTool : public Tool {
public:
    std::string name() const override { return "Select"; }

    bool mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) override;
    bool keyPressEvent(QKeyEvent* event) override;
    void cancel() override;

    std::vector<std::pair<math::Vec2, math::Vec2>> getPreviewLines() const override;

private:
    // Grip dragging state.
    bool m_draggingGrip = false;
    uint64_t m_gripEntityId = 0;
    int m_gripIndex = -1;
    math::Vec2 m_gripOrigPos;
    math::Vec2 m_gripCurrentPos;
    std::shared_ptr<draft::DraftEntity> m_gripBeforeClone;
};

}  // namespace hz::ui
