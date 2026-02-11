#include "horizon/ui/RadialDimensionTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRadialDimension.h"
#include "horizon/drafting/Layer.h"

#include <QMouseEvent>
#include <QKeyEvent>

namespace hz::ui {

void RadialDimensionTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::WaitingForCircle;
    m_isDiameter = false;
}

void RadialDimensionTool::deactivate() {
    m_state = State::WaitingForCircle;
    m_isDiameter = false;
    if (m_viewport) m_viewport->setLastSnapResult({});
    Tool::deactivate();
}

bool RadialDimensionTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;

    if (m_state == State::WaitingForCircle) {
        if (!m_viewport || !m_viewport->document()) return false;

        double tolerance = std::max(10.0 * m_viewport->pixelToWorldScale(), 0.15);
        const auto& entities = m_viewport->document()->draftDocument().entities();
        const auto& layerMgr = m_viewport->document()->layerManager();

        for (const auto& entity : entities) {
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;

            if (auto* circle = dynamic_cast<const draft::DraftCircle*>(entity.get())) {
                if (circle->hitTest(worldPos, tolerance)) {
                    m_center = circle->center();
                    m_radius = circle->radius();
                    m_currentPos = worldPos;
                    m_state = State::WaitingForTextPos;
                    return true;
                }
            } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(entity.get())) {
                if (arc->hitTest(worldPos, tolerance)) {
                    m_center = arc->center();
                    m_radius = arc->radius();
                    m_currentPos = worldPos;
                    m_state = State::WaitingForTextPos;
                    return true;
                }
            }
        }
        return false;
    }

    if (m_state == State::WaitingForTextPos) {
        if (!m_viewport || !m_viewport->document()) return false;

        auto dim = std::make_shared<draft::DraftRadialDimension>(
            m_center, m_radius, m_currentPos, m_isDiameter);
        dim->setLayer(m_viewport->document()->layerManager().currentLayer());

        auto cmd = std::make_unique<doc::AddEntityCommand>(
            m_viewport->document()->draftDocument(), dim);
        m_viewport->document()->undoStack().push(std::move(cmd));

        m_state = State::WaitingForCircle;
        return true;
    }

    return false;
}

bool RadialDimensionTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (m_state == State::WaitingForTextPos) {
        m_currentPos = worldPos;
        return true;
    }
    return false;
}

bool RadialDimensionTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool RadialDimensionTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }

    // Tab toggles between radius and diameter.
    if (event->key() == Qt::Key_Tab && m_state == State::WaitingForTextPos) {
        m_isDiameter = !m_isDiameter;
        return true;
    }

    return false;
}

void RadialDimensionTool::cancel() {
    m_state = State::WaitingForCircle;
    m_isDiameter = false;
    if (m_viewport) m_viewport->setLastSnapResult({});
}

std::vector<std::pair<math::Vec2, math::Vec2>> RadialDimensionTool::getPreviewLines() const {
    if (m_state != State::WaitingForTextPos) return {};

    draft::DraftRadialDimension previewDim(m_center, m_radius, m_currentPos, m_isDiameter);
    draft::DimensionStyle style;
    if (m_viewport && m_viewport->document()) {
        style = m_viewport->document()->draftDocument().dimensionStyle();
    }

    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    auto dim = previewDim.dimensionLines(style);
    auto arr = previewDim.arrowheadLines(style);
    lines.insert(lines.end(), dim.begin(), dim.end());
    lines.insert(lines.end(), arr.begin(), arr.end());
    return lines;
}

}  // namespace hz::ui
