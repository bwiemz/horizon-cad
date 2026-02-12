#include "horizon/ui/AngularDimensionTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftAngularDimension.h"
#include "horizon/drafting/Layer.h"

#include <QMouseEvent>
#include <QKeyEvent>

#include <cmath>

namespace hz::ui {

void AngularDimensionTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::WaitingForLine1;
}

void AngularDimensionTool::deactivate() {
    m_state = State::WaitingForLine1;
    if (m_viewport) m_viewport->setLastSnapResult({});
    Tool::deactivate();
}

bool AngularDimensionTool::lineIntersection(
    const math::Vec2& a1, const math::Vec2& a2,
    const math::Vec2& b1, const math::Vec2& b2,
    math::Vec2& result) {
    math::Vec2 d1 = a2 - a1;
    math::Vec2 d2 = b2 - b1;
    double denom = d1.cross(d2);
    if (std::abs(denom) < 1e-12) return false;  // parallel

    math::Vec2 diff = b1 - a1;
    double t = diff.cross(d2) / denom;
    result = a1 + d1 * t;
    return true;
}

bool AngularDimensionTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    double tolerance = std::max(10.0 * m_viewport->pixelToWorldScale(), 0.15);
    const auto& entities = m_viewport->document()->draftDocument().entities();
    const auto& layerMgr = m_viewport->document()->layerManager();

    if (m_state == State::WaitingForLine1 || m_state == State::WaitingForLine2) {
        // Find a line entity under the click.
        for (const auto& entity : entities) {
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;

            auto* line = dynamic_cast<const draft::DraftLine*>(entity.get());
            if (!line) continue;
            if (!line->hitTest(worldPos, tolerance)) continue;

            if (m_state == State::WaitingForLine1) {
                m_line1Start = line->start();
                m_line1End = line->end();
                m_state = State::WaitingForLine2;
                return true;
            } else {
                m_line2Start = line->start();
                m_line2End = line->end();

                // Compute intersection.
                if (!lineIntersection(m_line1Start, m_line1End,
                                       m_line2Start, m_line2End, m_vertex)) {
                    // Parallel lines — can't measure angle. Reset.
                    m_state = State::WaitingForLine1;
                    return true;
                }

                m_currentPos = worldPos;
                m_state = State::WaitingForArcPos;
                return true;
            }
        }
        return false;
    }

    if (m_state == State::WaitingForArcPos) {
        double arcRadius = m_vertex.distanceTo(m_currentPos);
        if (arcRadius < 1e-6) arcRadius = 1.0;

        // Determine line1Point and line2Point: points on each line in the
        // direction away from vertex that is closest to the click position.
        auto pickDirection = [&](const math::Vec2& lineStart, const math::Vec2& lineEnd) {
            math::Vec2 d1 = (lineEnd - lineStart).normalized();
            math::Vec2 d2 = -d1;
            math::Vec2 p1 = m_vertex + d1;
            math::Vec2 p2 = m_vertex + d2;
            // Use the direction that is closer to the arc position.
            if (m_currentPos.distanceTo(p1) < m_currentPos.distanceTo(p2))
                return m_vertex + d1 * arcRadius;
            return m_vertex + d2 * arcRadius;
        };

        math::Vec2 line1Pt = pickDirection(m_line1Start, m_line1End);
        math::Vec2 line2Pt = pickDirection(m_line2Start, m_line2End);

        auto dim = std::make_shared<draft::DraftAngularDimension>(
            m_vertex, line1Pt, line2Pt, arcRadius);
        dim->setLayer(m_viewport->document()->layerManager().currentLayer());

        auto cmd = std::make_unique<doc::AddEntityCommand>(
            m_viewport->document()->draftDocument(), dim);
        m_viewport->document()->undoStack().push(std::move(cmd));

        m_state = State::WaitingForLine1;
        return true;
    }

    return false;
}

bool AngularDimensionTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_state == State::WaitingForArcPos) {
        m_currentPos = worldPos;
        return true;
    }
    return false;
}

bool AngularDimensionTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool AngularDimensionTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void AngularDimensionTool::cancel() {
    m_state = State::WaitingForLine1;
    if (m_viewport) m_viewport->setLastSnapResult({});
}

std::vector<std::pair<math::Vec2, math::Vec2>> AngularDimensionTool::getPreviewLines() const {
    if (m_state != State::WaitingForArcPos) return {};

    double arcRadius = m_vertex.distanceTo(m_currentPos);
    if (arcRadius < 1e-6) return {};

    auto pickDirection = [&](const math::Vec2& lineStart, const math::Vec2& lineEnd) {
        math::Vec2 d1 = (lineEnd - lineStart).normalized();
        math::Vec2 d2 = -d1;
        math::Vec2 p1 = m_vertex + d1;
        math::Vec2 p2 = m_vertex + d2;
        if (m_currentPos.distanceTo(p1) < m_currentPos.distanceTo(p2))
            return m_vertex + d1 * arcRadius;
        return m_vertex + d2 * arcRadius;
    };

    math::Vec2 line1Pt = pickDirection(m_line1Start, m_line1End);
    math::Vec2 line2Pt = pickDirection(m_line2Start, m_line2End);

    draft::DraftAngularDimension previewDim(m_vertex, line1Pt, line2Pt, arcRadius);
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

std::string AngularDimensionTool::promptText() const {
    switch (m_state) {
        case State::WaitingForLine1: return "Select first line";
        case State::WaitingForLine2: return "Select second line";
        case State::WaitingForArcPos: return "Specify dimension arc position";
    }
    return "";
}

bool AngularDimensionTool::wantsCrosshair() const { return true; }

}  // namespace hz::ui
