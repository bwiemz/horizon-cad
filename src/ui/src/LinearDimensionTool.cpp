#include "horizon/ui/LinearDimensionTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"

#include <QMouseEvent>
#include <QKeyEvent>

#include <cmath>

namespace hz::ui {

void LinearDimensionTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::WaitingForPoint1;
    m_orientationOverride = -1;
}

void LinearDimensionTool::deactivate() {
    m_state = State::WaitingForPoint1;
    m_orientationOverride = -1;
    if (m_viewport) m_viewport->setLastSnapResult({});
    Tool::deactivate();
}

bool LinearDimensionTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    // Snap.
    math::Vec2 snapped = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snapped = result.point;
        m_viewport->setLastSnapResult(result);
    }

    switch (m_state) {
        case State::WaitingForPoint1:
            m_point1 = snapped;
            m_currentPos = snapped;
            m_state = State::WaitingForPoint2;
            return true;

        case State::WaitingForPoint2:
            m_point2 = snapped;
            m_currentPos = snapped;
            m_state = State::WaitingForDimLine;
            return true;

        case State::WaitingForDimLine: {
            if (!m_viewport || !m_viewport->document()) return false;

            auto orientation = detectOrientation();
            auto dim = std::make_shared<draft::DraftLinearDimension>(
                m_point1, m_point2, snapped, orientation);
            dim->setLayer(m_viewport->document()->layerManager().currentLayer());

            auto cmd = std::make_unique<doc::AddEntityCommand>(
                m_viewport->document()->draftDocument(), dim);
            m_viewport->document()->undoStack().push(std::move(cmd));

            // Reset for next dimension.
            m_state = State::WaitingForPoint1;
            m_orientationOverride = -1;
            return true;
        }
    }
    return false;
}

bool LinearDimensionTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_state == State::WaitingForPoint1) return false;

    math::Vec2 snapped = worldPos;
    if (m_viewport && m_viewport->document()) {
        auto result = m_viewport->snapEngine().snap(
            worldPos, m_viewport->document()->draftDocument().entities());
        snapped = result.point;
        m_viewport->setLastSnapResult(result);
    }
    m_currentPos = snapped;
    return true;
}

bool LinearDimensionTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool LinearDimensionTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }

    // Tab cycles orientation: Auto → H → V → Aligned → Auto
    if (event->key() == Qt::Key_Tab && m_state == State::WaitingForDimLine) {
        m_orientationOverride = (m_orientationOverride + 2) % 4 - 1;  // -1,0,1,2 cycle
        return true;
    }

    return false;
}

void LinearDimensionTool::cancel() {
    m_state = State::WaitingForPoint1;
    m_orientationOverride = -1;
    if (m_viewport) m_viewport->setLastSnapResult({});
}

draft::DraftLinearDimension::Orientation LinearDimensionTool::detectOrientation() const {
    if (m_orientationOverride >= 0) {
        return static_cast<draft::DraftLinearDimension::Orientation>(m_orientationOverride);
    }

    // Auto-detect: compare displacement of dimLinePoint from the measurement midpoint.
    math::Vec2 mid = (m_point1 + m_point2) * 0.5;
    double dx = std::abs(m_currentPos.x - mid.x);
    double dy = std::abs(m_currentPos.y - mid.y);

    // If the user moved more vertically from the midpoint → horizontal dimension
    // (the offset is perpendicular to the dimension line).
    if (dy > dx) {
        return draft::DraftLinearDimension::Orientation::Horizontal;
    }
    return draft::DraftLinearDimension::Orientation::Vertical;
}

std::vector<std::pair<math::Vec2, math::Vec2>> LinearDimensionTool::getPreviewLines() const {
    if (m_state == State::WaitingForPoint2) {
        // Simple rubber-band line.
        return {{m_point1, m_currentPos}};
    }

    if (m_state == State::WaitingForDimLine) {
        // Show preview of the full dimension geometry.
        auto orientation = detectOrientation();
        draft::DraftLinearDimension previewDim(m_point1, m_point2, m_currentPos, orientation);
        draft::DimensionStyle style;
        if (m_viewport && m_viewport->document()) {
            style = m_viewport->document()->draftDocument().dimensionStyle();
        }

        std::vector<std::pair<math::Vec2, math::Vec2>> lines;
        auto ext = previewDim.extensionLines(style);
        auto dim = previewDim.dimensionLines(style);
        auto arr = previewDim.arrowheadLines(style);
        lines.insert(lines.end(), ext.begin(), ext.end());
        lines.insert(lines.end(), dim.begin(), dim.end());
        lines.insert(lines.end(), arr.begin(), arr.end());
        return lines;
    }

    return {};
}

std::string LinearDimensionTool::promptText() const {
    switch (m_state) {
        case State::WaitingForPoint1: return "Specify first dimension point";
        case State::WaitingForPoint2: return "Specify second dimension point";
        case State::WaitingForDimLine: return "Specify dimension line position";
    }
    return "";
}

bool LinearDimensionTool::wantsCrosshair() const { return true; }

}  // namespace hz::ui
