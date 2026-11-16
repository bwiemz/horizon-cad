#include "horizon/ui/PolygonTool.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <algorithm>
#include <cmath>

#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/math/Constants.h"
#include "horizon/ui/ViewportWidget.h"

namespace hz::ui {

void PolygonTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::WaitingForCenter;
}

void PolygonTool::deactivate() {
    m_state = State::WaitingForCenter;
    if (m_viewport) m_viewport->setLastSnapResult({});
    Tool::deactivate();
}

void PolygonTool::setSides(int sides) {
    m_sides = std::clamp(sides, kFewestSides, kMostSides);
}

std::vector<math::Vec2> PolygonTool::corners(const math::Vec2& center, const math::Vec2& corner,
                                             int sides) {
    const math::Vec2 arm = corner - center;
    const double radius = arm.length();
    if (!std::isfinite(radius) || !std::isfinite(center.x) || !std::isfinite(center.y) ||
        !(radius > 0.0)) {
        return {};
    }
    const int n = std::clamp(sides, kFewestSides, kMostSides);
    const double start = std::atan2(arm.y, arm.x);
    std::vector<math::Vec2> out;
    out.reserve(static_cast<size_t>(n));
    out.push_back(corner);  // exactly where it was put
    for (int k = 1; k < n; ++k) {
        const double angle = start + math::kTwoPi * static_cast<double>(k) / n;
        out.emplace_back(center.x + radius * std::cos(angle), center.y + radius * std::sin(angle));
    }
    return out;
}

bool PolygonTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snap(worldPos);
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }

    switch (m_state) {
        case State::WaitingForCenter:
            m_center = snappedPos;
            m_currentPos = snappedPos;
            m_state = State::WaitingForCorner;
            return true;

        case State::WaitingForCorner: {
            const auto points = corners(m_center, snappedPos, m_sides);
            // A corner on the centre makes no polygon: the tool waits for
            // another, as a circle of no radius is not drawn.
            if (points.empty()) return true;
            if (m_viewport && m_viewport->document()) {
                auto polygon = std::make_shared<draft::DraftPolyline>(points, true);
                polygon->setLayer(m_viewport->document()->layerManager().currentLayer());
                m_viewport->document()->undoStack().push(std::make_unique<doc::AddEntityCommand>(
                    m_viewport->document()->activeDrawing(), polygon));
            }
            m_state = State::WaitingForCenter;
            return true;
        }
    }
    return false;
}

bool PolygonTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_state != State::WaitingForCorner) return false;
    math::Vec2 snappedPos = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snap(worldPos);
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);
    }
    m_currentPos = snappedPos;
    return true;
}

bool PolygonTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool PolygonTool::keyPressEvent(QKeyEvent* event) {
    const int step = (event->modifiers() & Qt::ShiftModifier) ? 10 : 1;
    switch (event->key()) {
        case Qt::Key_Escape:
            cancel();
            return true;
        case Qt::Key_Up:
        case Qt::Key_Plus:
            setSides(m_sides + step);
            return true;
        case Qt::Key_Down:
            setSides(m_sides - step);
            return true;
        default:
            return false;
    }
}

void PolygonTool::cancel() {
    m_state = State::WaitingForCenter;
    if (m_viewport) m_viewport->setLastSnapResult({});
}

std::vector<std::pair<math::Vec2, math::Vec2>> PolygonTool::getPreviewLines() const {
    if (m_state != State::WaitingForCorner) return {};
    const auto points = corners(m_center, m_currentPos, m_sides);
    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    lines.reserve(points.size() + 1);
    for (size_t i = 0; i < points.size(); ++i) {
        lines.emplace_back(points[i], points[(i + 1) % points.size()]);
    }
    if (!points.empty()) lines.emplace_back(m_center, points.front());  // its size and turn
    return lines;
}

std::string PolygonTool::promptText() const {
    const std::string sides = std::to_string(m_sides) + " sides (Up/Down to change)";
    switch (m_state) {
        case State::WaitingForCenter:
            return "Polygon, " + sides + ": specify center point";
        case State::WaitingForCorner:
            return "Polygon, " + sides + ": specify a corner";
    }
    return "";
}

}  // namespace hz::ui
