#pragma once

#include "horizon/ui/Tool.h"
#include "horizon/math/Vec2.h"
#include <QPoint>
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

    std::string promptText() const override;
    bool wantsCrosshair() const override;
    math::Vec3 previewColor() const override;

    /// Whether the tool is currently performing a box-selection drag.
    bool isDraggingBox() const { return m_draggingBox; }

    /// Returns true if box-drag direction is left-to-right (window mode).
    bool isWindowSelection() const;

    /// Selection rectangle corners in world space (valid only when isDraggingBox()).
    math::Vec2 boxCorner1() const { return m_dragStart; }
    math::Vec2 boxCorner2() const { return m_dragCurrent; }

private:
    // Grip dragging state.
    bool m_draggingGrip = false;
    uint64_t m_gripEntityId = 0;
    int m_gripIndex = -1;
    math::Vec2 m_gripOrigPos;
    math::Vec2 m_gripCurrentPos;
    std::shared_ptr<draft::DraftEntity> m_gripBeforeClone;

    // Box selection state.
    bool m_leftButtonDown = false;
    bool m_draggingBox = false;
    math::Vec2 m_dragStart;
    math::Vec2 m_dragCurrent;
    QPoint m_dragStartScreen;
    static constexpr int kDragThreshold = 5;
};

}  // namespace hz::ui
