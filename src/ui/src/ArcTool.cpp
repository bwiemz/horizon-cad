#include "horizon/ui/ArcTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/math/Constants.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <cmath>

namespace hz::ui {

void ArcTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::WaitingForCenter;
}

void ArcTool::deactivate() {
    m_state = State::WaitingForCenter;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
    Tool::deactivate();
}

bool ArcTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

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
            m_state = State::WaitingForStart;
            return true;

        case State::WaitingForStart:
            m_radius = m_center.distanceTo(snappedPos);
            if (m_radius < 1e-10) return true;  // ignore degenerate
            m_startAngle = std::atan2(snappedPos.y - m_center.y,
                                      snappedPos.x - m_center.x);
            m_currentPos = snappedPos;
            m_state = State::WaitingForEnd;
            return true;

        case State::WaitingForEnd: {
            double endAngle = std::atan2(snappedPos.y - m_center.y,
                                         snappedPos.x - m_center.x);
            if (m_viewport && m_viewport->document()) {
                auto arc = std::make_shared<draft::DraftArc>(
                    m_center, m_radius, m_startAngle, endAngle);
                arc->setLayer(m_viewport->document()->layerManager().currentLayer());
                auto cmd = std::make_unique<doc::AddEntityCommand>(
                    m_viewport->document()->draftDocument(), arc);
                m_viewport->document()->undoStack().push(std::move(cmd));
            }
            m_state = State::WaitingForCenter;
            return true;
        }
    }
    return false;
}

bool ArcTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_state == State::WaitingForCenter) return false;

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

bool ArcTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool ArcTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void ArcTool::cancel() {
    m_state = State::WaitingForCenter;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

std::vector<std::pair<math::Vec2, math::Vec2>> ArcTool::getPreviewLines() const {
    if (m_state == State::WaitingForStart) {
        // Show radius line from center to cursor.
        return {{m_center, m_currentPos}};
    }
    if (m_state == State::WaitingForEnd) {
        // Show line from center to start point.
        math::Vec2 startPt{m_center.x + m_radius * std::cos(m_startAngle),
                           m_center.y + m_radius * std::sin(m_startAngle)};
        return {{m_center, startPt}};
    }
    return {};
}

std::vector<Tool::ArcPreview> ArcTool::getPreviewArcs() const {
    if (m_state == State::WaitingForEnd) {
        double endAngle = std::atan2(m_currentPos.y - m_center.y,
                                      m_currentPos.x - m_center.x);
        return {{m_center, m_radius, m_startAngle, endAngle}};
    }
    return {};
}

}  // namespace hz::ui
