#include "horizon/ui/LeaderTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftLeader.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <QInputDialog>

namespace hz::ui {

void LeaderTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_points.clear();
    m_active = false;
}

void LeaderTool::deactivate() {
    m_points.clear();
    m_active = false;
    if (m_viewport) m_viewport->setLastSnapResult({});
    Tool::deactivate();
}

bool LeaderTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    // Snap.
    math::Vec2 snapped = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snapped = result.point;
        m_viewport->setLastSnapResult(result);
    }

    m_points.push_back(snapped);
    m_currentPos = snapped;
    m_active = true;
    return true;
}

bool LeaderTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (!m_active) return false;

    math::Vec2 snapped = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snapped = result.point;
        m_viewport->setLastSnapResult(result);
    }
    m_currentPos = snapped;
    return true;
}

bool LeaderTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool LeaderTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }

    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
        if (m_points.size() >= 2) {
            finishLeader();
            return true;
        }
    }

    return false;
}

void LeaderTool::cancel() {
    m_points.clear();
    m_active = false;
    if (m_viewport) m_viewport->setLastSnapResult({});
}

void LeaderTool::finishLeader() {
    if (!m_viewport || !m_viewport->document()) return;
    if (m_points.size() < 2) return;

    // Prompt for text via dialog.
    bool ok = false;
    QString text = QInputDialog::getText(
        m_viewport, QObject::tr("Leader Text"),
        QObject::tr("Enter annotation text:"),
        QLineEdit::Normal, QString(), &ok);

    if (!ok || text.isEmpty()) {
        cancel();
        return;
    }

    auto leader = std::make_shared<draft::DraftLeader>(
        m_points, text.toStdString());
    leader->setLayer(m_viewport->document()->layerManager().currentLayer());

    auto cmd = std::make_unique<doc::AddEntityCommand>(
        m_viewport->document()->draftDocument(), leader);
    m_viewport->document()->undoStack().push(std::move(cmd));

    m_points.clear();
    m_active = false;
}

std::vector<std::pair<math::Vec2, math::Vec2>> LeaderTool::getPreviewLines() const {
    if (!m_active || m_points.empty()) return {};

    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    // Draw existing segments.
    for (size_t i = 0; i + 1 < m_points.size(); ++i) {
        lines.push_back({m_points[i], m_points[i + 1]});
    }
    // Rubber-band to current position.
    lines.push_back({m_points.back(), m_currentPos});
    return lines;
}

}  // namespace hz::ui
