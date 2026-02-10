#include "horizon/ui/MoveTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <algorithm>

namespace hz::ui {

bool MoveTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& sel = m_viewport->selectionManager();
    auto ids = sel.selectedIds();
    if (ids.empty()) return false;

    // Check if click is on a selected entity.
    auto& doc = m_viewport->document()->draftDocument();
    double pixelScale = m_viewport->pixelToWorldScale();
    double tolerance = std::max(10.0 * pixelScale, 0.15);

    bool hitSelected = false;
    for (const auto& entity : doc.entities()) {
        if (sel.isSelected(entity->id()) && entity->hitTest(worldPos, tolerance)) {
            hitSelected = true;
            break;
        }
    }
    if (!hitSelected) return false;

    // Apply snapping.
    math::Vec2 snappedPos = worldPos;
    auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
    snappedPos = result.point;
    m_viewport->setLastSnapResult(result);

    m_dragging = true;
    m_dragStart = snappedPos;
    m_dragCurrent = snappedPos;
    m_totalDelta = {0.0, 0.0};
    return true;
}

bool MoveTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (!m_dragging) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    math::Vec2 snappedPos = worldPos;
    auto result = m_viewport->snapEngine().snap(
        worldPos, m_viewport->document()->draftDocument().entities());
    snappedPos = result.point;
    m_viewport->setLastSnapResult(result);

    math::Vec2 delta{snappedPos.x - m_dragCurrent.x, snappedPos.y - m_dragCurrent.y};

    // Translate selected entities in real-time.
    auto& doc = m_viewport->document()->draftDocument();
    auto& sel = m_viewport->selectionManager();
    for (const auto& entity : doc.entities()) {
        if (sel.isSelected(entity->id())) {
            entity->translate(delta);
        }
    }

    m_dragCurrent = snappedPos;
    m_totalDelta.x += delta.x;
    m_totalDelta.y += delta.y;
    return true;
}

bool MoveTool::mouseReleaseEvent(QMouseEvent* event, const math::Vec2& /*worldPos*/) {
    if (event->button() != Qt::LeftButton || !m_dragging) return false;
    if (!m_viewport || !m_viewport->document()) {
        m_dragging = false;
        return false;
    }

    // Undo the real-time translation, then push a command for the total delta.
    math::Vec2 neg{-m_totalDelta.x, -m_totalDelta.y};
    auto& doc = m_viewport->document()->draftDocument();
    auto& sel = m_viewport->selectionManager();

    for (const auto& entity : doc.entities()) {
        if (sel.isSelected(entity->id())) {
            entity->translate(neg);
        }
    }

    if (std::abs(m_totalDelta.x) > 1e-10 || std::abs(m_totalDelta.y) > 1e-10) {
        auto ids = sel.selectedIds();
        std::vector<uint64_t> idVec(ids.begin(), ids.end());
        auto cmd = std::make_unique<doc::MoveEntityCommand>(doc, idVec, m_totalDelta);
        m_viewport->document()->undoStack().push(std::move(cmd));
    }

    m_dragging = false;
    m_viewport->setLastSnapResult({});
    return true;
}

bool MoveTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape && m_dragging) {
        cancel();
        return true;
    }
    return false;
}

void MoveTool::cancel() {
    if (m_dragging && m_viewport && m_viewport->document()) {
        // Undo the real-time translation.
        math::Vec2 neg{-m_totalDelta.x, -m_totalDelta.y};
        auto& doc = m_viewport->document()->draftDocument();
        auto& sel = m_viewport->selectionManager();
        for (const auto& entity : doc.entities()) {
            if (sel.isSelected(entity->id())) {
                entity->translate(neg);
            }
        }
    }
    m_dragging = false;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

}  // namespace hz::ui
