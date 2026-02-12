#include "horizon/ui/MeasureAngleTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/math/Constants.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <QMainWindow>
#include <QStatusBar>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace hz::ui {

void MeasureAngleTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::Vertex;
}

void MeasureAngleTool::deactivate() {
    m_state = State::Vertex;
    if (m_viewport) m_viewport->setLastSnapResult({});
    Tool::deactivate();
}

bool MeasureAngleTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }

    switch (m_state) {
    case State::Vertex:
        m_vertex = snappedPos;
        m_currentPos = snappedPos;
        m_state = State::Ray1;
        break;

    case State::Ray1:
        m_ray1Point = snappedPos;
        m_currentPos = snappedPos;
        m_state = State::Ray2;
        break;

    case State::Ray2: {
        // Compute angle.
        double a1 = std::atan2(m_ray1Point.y - m_vertex.y,
                                m_ray1Point.x - m_vertex.x);
        double a2 = std::atan2(snappedPos.y - m_vertex.y,
                                snappedPos.x - m_vertex.x);
        double angle = a2 - a1;
        // Normalize to [0, 2*PI).
        while (angle < 0) angle += math::kTwoPi;
        while (angle >= math::kTwoPi) angle -= math::kTwoPi;
        // Use the smaller angle.
        if (angle > math::kPi) angle = math::kTwoPi - angle;

        double degrees = angle * math::kRadToDeg;

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "Angle: " << degrees << "\xC2\xB0";

        if (m_viewport) {
            auto* mainWin = qobject_cast<QMainWindow*>(m_viewport->window());
            if (mainWin && mainWin->statusBar()) {
                mainWin->statusBar()->showMessage(
                    QString::fromStdString(oss.str()), 10000);
            }
        }

        m_state = State::Vertex;
        if (m_viewport) m_viewport->setLastSnapResult({});
        break;
    }
    }

    return true;
}

bool MeasureAngleTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_state == State::Vertex) return false;

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

bool MeasureAngleTool::mouseReleaseEvent(QMouseEvent*, const math::Vec2&) {
    return false;
}

bool MeasureAngleTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void MeasureAngleTool::cancel() {
    m_state = State::Vertex;
    if (m_viewport) m_viewport->setLastSnapResult({});
}

std::vector<std::pair<math::Vec2, math::Vec2>> MeasureAngleTool::getPreviewLines() const {
    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    if (m_state == State::Ray1) {
        lines.push_back({m_vertex, m_currentPos});
    } else if (m_state == State::Ray2) {
        lines.push_back({m_vertex, m_ray1Point});
        lines.push_back({m_vertex, m_currentPos});
    }
    return lines;
}

}  // namespace hz::ui
