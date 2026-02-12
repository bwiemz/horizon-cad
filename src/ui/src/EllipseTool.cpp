#include "horizon/ui/EllipseTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftEllipse.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <cmath>

namespace hz::ui {

void EllipseTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::Center;
}

void EllipseTool::deactivate() {
    m_state = State::Center;
    if (m_viewport) m_viewport->setLastSnapResult({});
    Tool::deactivate();
}

bool EllipseTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }

    switch (m_state) {
    case State::Center:
        m_center = snappedPos;
        m_currentPos = snappedPos;
        m_state = State::MajorAxis;
        break;

    case State::MajorAxis: {
        m_majorAxisPt = snappedPos;
        double dx = snappedPos.x - m_center.x;
        double dy = snappedPos.y - m_center.y;
        m_semiMajor = std::sqrt(dx * dx + dy * dy);
        m_rotation = std::atan2(dy, dx);
        if (m_semiMajor < 1e-6) {
            // Degenerate — stay in this state.
            break;
        }
        m_currentPos = snappedPos;
        m_state = State::MinorAxis;
        break;
    }

    case State::MinorAxis:
        finishEllipse();
        break;
    }

    return true;
}

bool EllipseTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }
    m_currentPos = snappedPos;
    return m_state != State::Center;
}

bool EllipseTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool EllipseTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void EllipseTool::cancel() {
    m_state = State::Center;
    if (m_viewport) m_viewport->setLastSnapResult({});
}

void EllipseTool::finishEllipse() {
    if (m_semiMajor < 1e-6 || !m_viewport || !m_viewport->document()) {
        m_state = State::Center;
        return;
    }

    // Compute semi-minor: distance from cursor to the major axis line,
    // projected perpendicular.
    double dx = m_currentPos.x - m_center.x;
    double dy = m_currentPos.y - m_center.y;
    // Perpendicular direction to major axis.
    double perpX = -std::sin(m_rotation);
    double perpY =  std::cos(m_rotation);
    double semiMinor = std::abs(dx * perpX + dy * perpY);
    if (semiMinor < 1e-6) semiMinor = m_semiMajor * 0.01;  // Prevent degenerate.

    auto ellipse = std::make_shared<draft::DraftEllipse>(
        m_center, m_semiMajor, semiMinor, m_rotation);
    ellipse->setLayer(m_viewport->document()->layerManager().currentLayer());

    auto cmd = std::make_unique<doc::AddEntityCommand>(
        m_viewport->document()->draftDocument(), ellipse);
    m_viewport->document()->undoStack().push(std::move(cmd));

    m_state = State::Center;
    if (m_viewport) m_viewport->setLastSnapResult({});
}

// ---------------------------------------------------------------------------
// Preview
// ---------------------------------------------------------------------------

std::vector<math::Vec2> EllipseTool::evaluateEllipse(
    const math::Vec2& center, double semiMajor, double semiMinor,
    double rotation, int segments) {
    std::vector<math::Vec2> pts;
    pts.reserve(static_cast<size_t>(segments + 1));
    double cosR = std::cos(rotation);
    double sinR = std::sin(rotation);
    double step = 2.0 * 3.14159265358979323846 / segments;
    for (int i = 0; i <= segments; ++i) {
        double t = i * step;
        double lx = semiMajor * std::cos(t);
        double ly = semiMinor * std::sin(t);
        pts.push_back({center.x + lx * cosR - ly * sinR,
                        center.y + lx * sinR + ly * cosR});
    }
    return pts;
}

std::vector<std::pair<math::Vec2, math::Vec2>> EllipseTool::getPreviewLines() const {
    std::vector<std::pair<math::Vec2, math::Vec2>> lines;

    if (m_state == State::MajorAxis) {
        // Show a line from center to cursor (major axis preview).
        lines.push_back({m_center, m_currentPos});
    } else if (m_state == State::MinorAxis) {
        // Compute the semi-minor from cursor projection.
        double dx = m_currentPos.x - m_center.x;
        double dy = m_currentPos.y - m_center.y;
        double perpX = -std::sin(m_rotation);
        double perpY =  std::cos(m_rotation);
        double semiMinor = std::abs(dx * perpX + dy * perpY);
        if (semiMinor < 1e-6) semiMinor = m_semiMajor * 0.01;

        auto pts = evaluateEllipse(m_center, m_semiMajor, semiMinor, m_rotation);
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            lines.push_back({pts[i], pts[i + 1]});
        }
    }

    return lines;
}

}  // namespace hz::ui
