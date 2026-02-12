#include "horizon/ui/MeasureDistanceTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <QMainWindow>
#include <QStatusBar>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace hz::ui {

void MeasureDistanceTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::First;
}

void MeasureDistanceTool::deactivate() {
    m_state = State::First;
    if (m_viewport) m_viewport->setLastSnapResult({});
    Tool::deactivate();
}

bool MeasureDistanceTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }

    if (m_state == State::First) {
        m_firstPoint = snappedPos;
        m_currentPos = snappedPos;
        m_state = State::Second;
    } else {
        // Compute and display result.
        double dx = snappedPos.x - m_firstPoint.x;
        double dy = snappedPos.y - m_firstPoint.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << "Distance: " << dist
            << "  (dx=" << dx << ", dy=" << dy << ")";

        // Display in status bar.
        if (m_viewport) {
            auto* mainWin = qobject_cast<QMainWindow*>(m_viewport->window());
            if (mainWin && mainWin->statusBar()) {
                mainWin->statusBar()->showMessage(
                    QString::fromStdString(oss.str()), 10000);
            }
        }

        // Reset for next measurement.
        m_state = State::First;
        if (m_viewport) m_viewport->setLastSnapResult({});
    }

    return true;
}

bool MeasureDistanceTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_state != State::Second) return false;

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

bool MeasureDistanceTool::mouseReleaseEvent(QMouseEvent*, const math::Vec2&) {
    return false;
}

bool MeasureDistanceTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void MeasureDistanceTool::cancel() {
    m_state = State::First;
    if (m_viewport) m_viewport->setLastSnapResult({});
}

std::vector<std::pair<math::Vec2, math::Vec2>> MeasureDistanceTool::getPreviewLines() const {
    if (m_state == State::Second) {
        return {{m_firstPoint, m_currentPos}};
    }
    return {};
}

std::string MeasureDistanceTool::promptText() const {
    switch (m_state) {
        case State::First: return "Specify first point";
        case State::Second: return "Specify second point";
    }
    return "";
}

bool MeasureDistanceTool::wantsCrosshair() const { return true; }

}  // namespace hz::ui
