#include "horizon/ui/CircleTool.h"
#include "horizon/ui/ViewportWidget.h"

#include <QMouseEvent>
#include <QKeyEvent>

namespace hz::ui {

void CircleTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::WaitingForCenter;
}

void CircleTool::deactivate() {
    m_state = State::WaitingForCenter;
    Tool::deactivate();
}

bool CircleTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    switch (m_state) {
        case State::WaitingForCenter:
            m_center = worldPos;
            m_currentPos = worldPos;
            m_state = State::WaitingForRadius;
            return true;

        case State::WaitingForRadius: {
            double radius = m_center.distanceTo(worldPos);
            if (radius > 1e-6 && m_viewport) {
                m_viewport->addCircle(m_center, radius);
            }
            m_state = State::WaitingForCenter;
            return true;
        }
    }
    return false;
}

bool CircleTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_state == State::WaitingForRadius) {
        m_currentPos = worldPos;
        return true;  // request repaint for preview update
    }
    return false;
}

bool CircleTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool CircleTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void CircleTool::cancel() {
    m_state = State::WaitingForCenter;
}

std::vector<std::pair<math::Vec2, double>> CircleTool::getPreviewCircles() const {
    if (m_state == State::WaitingForRadius) {
        double radius = m_center.distanceTo(m_currentPos);
        if (radius > 1e-6) {
            return {{m_center, radius}};
        }
    }
    return {};
}

}  // namespace hz::ui
