#include "horizon/ui/LineTool.h"

#include <QKeyEvent>
#include <QMouseEvent>

#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/ui/ViewportWidget.h"

namespace hz::ui {

void LineTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::WaitingForStart;
}

void LineTool::deactivate() {
    m_state = State::WaitingForStart;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
    Tool::deactivate();
}

bool LineTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    // Apply snapping.
    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snap(worldPos);
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }

    switch (m_state) {
        case State::WaitingForStart:
            m_startPoint = snappedPos;
            m_currentPos = snappedPos;
            m_state = State::WaitingForEnd;
            return true;

        case State::WaitingForEnd:
            if (snappedPos.distanceTo(m_startPoint) < 1e-12) return true;  // no length
            // Commit the line via undo command.
            if (m_viewport && m_viewport->document()) {
                auto line = std::make_shared<draft::DraftLine>(m_startPoint, snappedPos);
                line->setLayer(m_viewport->document()->layerManager().currentLayer());
                auto cmd = std::make_unique<doc::AddEntityCommand>(
                    m_viewport->document()->activeDrawing(), line);
                m_viewport->document()->undoStack().push(std::move(cmd));
            }
            // The next segment starts where this one ended, until Enter or
            // Escape ends the chain.
            m_startPoint = snappedPos;
            m_currentPos = snappedPos;
            return true;
    }
    return false;
}

bool LineTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_state == State::WaitingForEnd) {
        math::Vec2 snappedPos = worldPos;
        if (m_viewport && m_viewport->document()) {
            auto result = m_viewport->snap(worldPos);
            snappedPos = result.point;
            m_viewport->setLastSnapResult(result);
        }
        m_currentPos = snappedPos;
        return true;
    }
    return false;
}

bool LineTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool LineTool::keyPressEvent(QKeyEvent* event) {
    const bool enter = event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter;
    if (event->key() == Qt::Key_Escape || (enter && m_state == State::WaitingForEnd)) {
        cancel();  // the segments drawn stay
        return true;
    }
    return false;
}

void LineTool::cancel() {
    m_state = State::WaitingForStart;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

std::vector<std::pair<math::Vec2, math::Vec2>> LineTool::getPreviewLines() const {
    if (m_state == State::WaitingForEnd) {
        return {{m_startPoint, m_currentPos}};
    }
    return {};
}

std::string LineTool::promptText() const {
    switch (m_state) {
        case State::WaitingForStart:
            return "Specify first point";
        case State::WaitingForEnd:
            return "Specify next point (Enter or Escape to finish)";
    }
    return "";
}

bool LineTool::wantsCrosshair() const {
    return true;
}

}  // namespace hz::ui
