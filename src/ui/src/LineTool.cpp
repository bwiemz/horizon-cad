#include "horizon/ui/LineTool.h"
#include "horizon/ui/ViewportWidget.h"

#include <QMouseEvent>
#include <QKeyEvent>

namespace hz::ui {

void LineTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::WaitingForStart;
}

void LineTool::deactivate() {
    m_state = State::WaitingForStart;
    Tool::deactivate();
}

bool LineTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    switch (m_state) {
        case State::WaitingForStart:
            m_startPoint = worldPos;
            m_currentPos = worldPos;
            m_state = State::WaitingForEnd;
            return true;

        case State::WaitingForEnd:
            // Commit the line to the viewport.
            if (m_viewport) {
                m_viewport->addLine(m_startPoint, worldPos);
            }
            // Reset for the next line.
            m_state = State::WaitingForStart;
            return true;
    }
    return false;
}

bool LineTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_state == State::WaitingForEnd) {
        m_currentPos = worldPos;
        return true;  // request repaint for preview update
    }
    return false;
}

bool LineTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool LineTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void LineTool::cancel() {
    m_state = State::WaitingForStart;
}

std::vector<std::pair<math::Vec2, math::Vec2>> LineTool::getPreviewLines() const {
    if (m_state == State::WaitingForEnd) {
        return {{m_startPoint, m_currentPos}};
    }
    return {};
}

}  // namespace hz::ui
