#include "horizon/ui/PolylineTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftPolyline.h"

#include <QMouseEvent>
#include <QKeyEvent>

namespace hz::ui {

void PolylineTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_points.clear();
    m_active = false;
}

void PolylineTool::deactivate() {
    m_points.clear();
    m_active = false;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
    Tool::deactivate();
}

bool PolylineTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    // Double-click finishes the polyline.
    if (event->type() == QEvent::MouseButtonDblClick) {
        // The first click of the double-click already added a point; remove it.
        if (m_points.size() > 1) {
            m_points.pop_back();
        }
        finishPolyline();
        return true;
    }

    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }

    m_points.push_back(snappedPos);
    m_currentPos = snappedPos;
    m_active = true;
    return true;
}

bool PolylineTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (!m_active) return false;

    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }
    m_currentPos = snappedPos;
    return true;
}

bool PolylineTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool PolylineTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
        finishPolyline();
        return true;
    }
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void PolylineTool::cancel() {
    m_points.clear();
    m_active = false;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

void PolylineTool::finishPolyline() {
    if (m_points.size() >= 2 && m_viewport && m_viewport->document()) {
        auto polyline = std::make_shared<draft::DraftPolyline>(m_points);
        polyline->setLayer(m_viewport->document()->layerManager().currentLayer());
        auto cmd = std::make_unique<doc::AddEntityCommand>(
            m_viewport->document()->draftDocument(), polyline);
        m_viewport->document()->undoStack().push(std::move(cmd));
    }
    m_points.clear();
    m_active = false;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

std::vector<std::pair<math::Vec2, math::Vec2>> PolylineTool::getPreviewLines() const {
    if (!m_active || m_points.empty()) return {};

    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    for (size_t i = 0; i + 1 < m_points.size(); ++i) {
        lines.push_back({m_points[i], m_points[i + 1]});
    }
    // Rubber-band from last point to cursor.
    lines.push_back({m_points.back(), m_currentPos});
    return lines;
}

std::string PolylineTool::promptText() const {
    if (!m_active || m_points.empty()) return "Specify first point";
    return "Specify next point, Enter to finish";
}

bool PolylineTool::wantsCrosshair() const { return true; }

}  // namespace hz::ui
