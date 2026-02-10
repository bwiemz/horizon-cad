#include "horizon/ui/RectangleTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftRectangle.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <algorithm>

namespace hz::ui {

void RectangleTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::WaitingForFirstCorner;
}

void RectangleTool::deactivate() {
    m_state = State::WaitingForFirstCorner;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
    Tool::deactivate();
}

bool RectangleTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }

    switch (m_state) {
        case State::WaitingForFirstCorner:
            m_firstCorner = snappedPos;
            m_currentPos = snappedPos;
            m_state = State::WaitingForSecondCorner;
            return true;

        case State::WaitingForSecondCorner:
            if (m_viewport && m_viewport->document()) {
                auto rect = std::make_shared<draft::DraftRectangle>(m_firstCorner, snappedPos);
                rect->setLayer(m_viewport->document()->layerManager().currentLayer());
                auto cmd = std::make_unique<doc::AddEntityCommand>(
                    m_viewport->document()->draftDocument(), rect);
                m_viewport->document()->undoStack().push(std::move(cmd));
            }
            m_state = State::WaitingForFirstCorner;
            return true;
    }
    return false;
}

bool RectangleTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_state != State::WaitingForSecondCorner) return false;

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

bool RectangleTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool RectangleTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void RectangleTool::cancel() {
    m_state = State::WaitingForFirstCorner;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

std::vector<std::pair<math::Vec2, math::Vec2>> RectangleTool::getPreviewLines() const {
    if (m_state != State::WaitingForSecondCorner) return {};

    double minX = std::min(m_firstCorner.x, m_currentPos.x);
    double minY = std::min(m_firstCorner.y, m_currentPos.y);
    double maxX = std::max(m_firstCorner.x, m_currentPos.x);
    double maxY = std::max(m_firstCorner.y, m_currentPos.y);

    math::Vec2 bl(minX, minY), br(maxX, minY), tr(maxX, maxY), tl(minX, maxY);
    return {{bl, br}, {br, tr}, {tr, tl}, {tl, bl}};
}

}  // namespace hz::ui
