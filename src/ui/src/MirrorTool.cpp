#include "horizon/ui/MirrorTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/Intersection.h"
#include "horizon/math/Constants.h"
#include "horizon/math/MathUtils.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <algorithm>
#include <cmath>

namespace hz::ui {

void MirrorTool::deactivate() {
    cancel();
    Tool::deactivate();
}


static math::Vec2 mirrorPoint(const math::Vec2& p,
                               const math::Vec2& axisP1,
                               const math::Vec2& axisP2) {
    math::Vec2 d = (axisP2 - axisP1).normalized();
    math::Vec2 v = p - axisP1;
    return axisP1 + d * (2.0 * v.dot(d)) - v;
}

bool MirrorTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->draftDocument();

    if (m_state == State::SelectFirstPoint) {
        auto& sel = m_viewport->selectionManager();
        if (sel.empty()) return false;  // Nothing selected to mirror.

        // Apply snapping.
        auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
        m_axisP1 = result.point;
        m_viewport->setLastSnapResult(result);

        m_state = State::SelectSecondPoint;
        return true;
    }

    if (m_state == State::SelectSecondPoint) {
        auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
        math::Vec2 axisP2 = result.point;
        m_viewport->setLastSnapResult(result);

        if (m_axisP1.distanceTo(axisP2) < 1e-6) return false;  // Degenerate axis.

        auto& sel = m_viewport->selectionManager();
        const auto& layerMgr = m_viewport->document()->layerManager();
        std::vector<uint64_t> idVec;
        for (const auto& entity : doc.entities()) {
            if (!sel.isSelected(entity->id())) continue;
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;
            idVec.push_back(entity->id());
        }
        if (idVec.empty()) return false;

        auto cmd = std::make_unique<doc::MirrorEntityCommand>(doc, idVec, m_axisP1, axisP2);
        auto* rawCmd = cmd.get();
        m_viewport->document()->undoStack().push(std::move(cmd));

        // Select the mirrored entities.
        sel.clearSelection();
        for (uint64_t id : rawCmd->mirroredIds()) {
            sel.select(id);
        }

        m_state = State::SelectFirstPoint;
        m_viewport->setLastSnapResult({});
        return true;
    }

    return false;
}

bool MirrorTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    m_currentPos = worldPos;
    if (m_state == State::SelectSecondPoint && m_viewport && m_viewport->document()) {
        auto& doc = m_viewport->document()->draftDocument();
        auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
        m_currentPos = result.point;
        m_viewport->setLastSnapResult(result);
    }
    return (m_state == State::SelectSecondPoint);
}

bool MirrorTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool MirrorTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void MirrorTool::cancel() {
    m_state = State::SelectFirstPoint;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

std::vector<std::pair<math::Vec2, math::Vec2>> MirrorTool::getPreviewLines() const {
    std::vector<std::pair<math::Vec2, math::Vec2>> result;
    if (m_state != State::SelectSecondPoint) return result;
    if (!m_viewport || !m_viewport->document()) return result;

    // Draw the mirror axis line.
    result.emplace_back(m_axisP1, m_currentPos);

    if (m_axisP1.distanceTo(m_currentPos) < 1e-6) return result;

    // Preview mirrored entities as lines.
    auto& doc = m_viewport->document()->draftDocument();
    auto& sel = m_viewport->selectionManager();
    for (const auto& entity : doc.entities()) {
        if (!sel.isSelected(entity->id())) continue;

        // Extract segments and mirror them.
        auto segs = draft::extractSegments(*entity);
        for (const auto& [s, e] : segs) {
            result.emplace_back(mirrorPoint(s, m_axisP1, m_currentPos),
                                mirrorPoint(e, m_axisP1, m_currentPos));
        }

        // For circles, approximate as segments for preview.
        if (auto* c = dynamic_cast<const draft::DraftCircle*>(entity.get())) {
            math::Vec2 mc = mirrorPoint(c->center(), m_axisP1, m_currentPos);
            // Show as 32-segment polygon.
            for (int i = 0; i < 32; ++i) {
                double a1 = math::kTwoPi * i / 32.0;
                double a2 = math::kTwoPi * (i + 1) / 32.0;
                math::Vec2 p1(mc.x + c->radius() * std::cos(a1), mc.y + c->radius() * std::sin(a1));
                math::Vec2 p2(mc.x + c->radius() * std::cos(a2), mc.y + c->radius() * std::sin(a2));
                result.emplace_back(p1, p2);
            }
        }

        // For arcs, approximate as segments.
        if (auto* a = dynamic_cast<const draft::DraftArc*>(entity.get())) {
            auto clone = a->clone();
            clone->mirror(m_axisP1, m_currentPos);
            auto* ma = dynamic_cast<const draft::DraftArc*>(clone.get());
            if (ma) {
                double sweep = ma->sweepAngle();
                int segsN = std::max(8, static_cast<int>(sweep * 16.0 / math::kTwoPi));
                for (int i = 0; i < segsN; ++i) {
                    double a1 = ma->startAngle() + sweep * i / segsN;
                    double a2 = ma->startAngle() + sweep * (i + 1) / segsN;
                    math::Vec2 p1(ma->center().x + ma->radius() * std::cos(a1),
                                  ma->center().y + ma->radius() * std::sin(a1));
                    math::Vec2 p2(ma->center().x + ma->radius() * std::cos(a2),
                                  ma->center().y + ma->radius() * std::sin(a2));
                    result.emplace_back(p1, p2);
                }
            }
        }
    }

    return result;
}

}  // namespace hz::ui
