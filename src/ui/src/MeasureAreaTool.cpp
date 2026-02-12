#include "horizon/ui/MeasureAreaTool.h"
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

void MeasureAreaTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_points.clear();
    m_active = false;
}

void MeasureAreaTool::deactivate() {
    m_points.clear();
    m_active = false;
    if (m_viewport) m_viewport->setLastSnapResult({});
    Tool::deactivate();
}

bool MeasureAreaTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

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

    // Double-click detection: if the new point is very close to the first point
    // and we have at least 3 points, finish.
    if (m_points.size() >= 4) {
        double dist = m_points.front().distanceTo(snappedPos);
        double pixelScale = m_viewport ? m_viewport->pixelToWorldScale() : 0.01;
        if (dist < 10.0 * pixelScale) {
            m_points.pop_back();  // Remove the duplicate closing point.
            finishMeasure();
            return true;
        }
    }

    return true;
}

bool MeasureAreaTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (!m_active && m_points.empty()) return false;

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

bool MeasureAreaTool::mouseReleaseEvent(QMouseEvent*, const math::Vec2&) {
    return false;
}

bool MeasureAreaTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
        if (m_points.size() >= 3) {
            finishMeasure();
        }
        return true;
    }
    return false;
}

void MeasureAreaTool::cancel() {
    m_points.clear();
    m_active = false;
    if (m_viewport) m_viewport->setLastSnapResult({});
}

void MeasureAreaTool::finishMeasure() {
    if (m_points.size() < 3) return;

    double area = shoelaceArea(m_points);

    // Compute perimeter.
    double perimeter = 0.0;
    for (size_t i = 0; i < m_points.size(); ++i) {
        size_t j = (i + 1) % m_points.size();
        perimeter += m_points[i].distanceTo(m_points[j]);
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "Area: " << area << "  Perimeter: " << perimeter;

    if (m_viewport) {
        auto* mainWin = qobject_cast<QMainWindow*>(m_viewport->window());
        if (mainWin && mainWin->statusBar()) {
            mainWin->statusBar()->showMessage(
                QString::fromStdString(oss.str()), 10000);
        }
    }

    m_points.clear();
    m_active = false;
    if (m_viewport) m_viewport->setLastSnapResult({});
}

double MeasureAreaTool::shoelaceArea(const std::vector<math::Vec2>& pts) {
    double sum = 0.0;
    size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        sum += pts[i].x * pts[j].y;
        sum -= pts[j].x * pts[i].y;
    }
    return std::abs(sum) * 0.5;
}

std::vector<std::pair<math::Vec2, math::Vec2>> MeasureAreaTool::getPreviewLines() const {
    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    if (m_points.empty()) return lines;

    // Lines between consecutive points.
    for (size_t i = 0; i + 1 < m_points.size(); ++i) {
        lines.push_back({m_points[i], m_points[i + 1]});
    }
    // Line from last point to cursor.
    lines.push_back({m_points.back(), m_currentPos});
    // Closing line from cursor back to first point (to show polygon shape).
    if (m_points.size() >= 2) {
        lines.push_back({m_currentPos, m_points.front()});
    }
    return lines;
}

std::string MeasureAreaTool::promptText() const {
    if (!m_active || m_points.empty()) return "Specify first vertex";
    return "Specify next vertex, Enter to finish";
}

bool MeasureAreaTool::wantsCrosshair() const { return true; }

}  // namespace hz::ui
