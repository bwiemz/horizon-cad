#include "horizon/ui/CircleTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftCircle.h"

#include <QMouseEvent>
#include <QKeyEvent>

namespace hz::ui {

void CircleTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::WaitingForCenter;
}

void CircleTool::deactivate() {
    m_state = State::WaitingForCenter;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
    Tool::deactivate();
}

bool CircleTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    // Apply snapping.
    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }

    switch (m_state) {
        case State::WaitingForCenter:
            m_center = snappedPos;
            m_currentPos = snappedPos;
            m_state = State::WaitingForRadius;
            return true;

        case State::WaitingForRadius: {
            double radius = m_center.distanceTo(snappedPos);
            if (radius > 1e-6 && m_viewport && m_viewport->document()) {
                auto circle = std::make_shared<draft::DraftCircle>(m_center, radius);
                circle->setLayer(m_viewport->document()->layerManager().currentLayer());
                auto cmd = std::make_unique<doc::AddEntityCommand>(
                    m_viewport->document()->draftDocument(), circle);
                m_viewport->document()->undoStack().push(std::move(cmd));
            }
            m_state = State::WaitingForCenter;
            return true;
        }
    }
    return false;
}

bool CircleTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_state == State::WaitingForRadius) {
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
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
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
