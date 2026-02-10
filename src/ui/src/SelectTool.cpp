#include "horizon/ui/SelectTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <algorithm>

namespace hz::ui {

bool SelectTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->draftDocument();
    auto& sel = m_viewport->selectionManager();

    // Compute pick tolerance: 10 screen-pixels converted to world space.
    double pixelScale = m_viewport->pixelToWorldScale();
    const double tolerance = std::max(10.0 * pixelScale, 0.15);

    // Find the first entity under the cursor.
    uint64_t hitId = 0;
    for (const auto& entity : doc.entities()) {
        if (entity->hitTest(worldPos, tolerance)) {
            hitId = entity->id();
            break;
        }
    }

    bool shiftHeld = (event->modifiers() & Qt::ShiftModifier);

    if (hitId != 0) {
        if (shiftHeld) {
            sel.toggle(hitId);
        } else {
            sel.clearSelection();
            sel.select(hitId);
        }
    } else {
        if (!shiftHeld) {
            sel.clearSelection();
        }
    }

    return true;
}

bool SelectTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool SelectTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool SelectTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace) {
        if (!m_viewport || !m_viewport->document()) return false;

        auto& sel = m_viewport->selectionManager();
        auto ids = sel.selectedIds();
        if (ids.empty()) return false;

        auto& undoStack = m_viewport->document()->undoStack();
        for (uint64_t id : ids) {
            auto cmd = std::make_unique<doc::RemoveEntityCommand>(
                m_viewport->document()->draftDocument(), id);
            undoStack.push(std::move(cmd));
        }
        sel.clearSelection();
        return true;
    }
    return false;
}

}  // namespace hz::ui
