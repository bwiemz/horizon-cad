#include "horizon/ui/ChamferTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <cmath>

namespace hz::ui {

void ChamferTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::SelectFirstLine;
    m_firstEntityId = 0;
    m_distInput.clear();
}

void ChamferTool::deactivate() {
    cancel();
    Tool::deactivate();
}

// ---------------------------------------------------------------------------
// Chamfer computation (line-line only)
// ---------------------------------------------------------------------------

bool ChamferTool::computeChamfer(
    uint64_t lineAId, const math::Vec2& /*clickA*/,
    uint64_t lineBId, const math::Vec2& /*clickB*/,
    math::Vec2& chamferPtA, math::Vec2& chamferPtB,
    math::Vec2& trimA_start, math::Vec2& trimA_end,
    math::Vec2& trimB_start, math::Vec2& trimB_end) const {

    if (!m_viewport || !m_viewport->document()) return false;
    auto& doc = m_viewport->document()->draftDocument();

    // Find the two lines.
    const draft::DraftLine* lineA = nullptr;
    const draft::DraftLine* lineB = nullptr;
    for (const auto& e : doc.entities()) {
        if (e->id() == lineAId) lineA = dynamic_cast<const draft::DraftLine*>(e.get());
        if (e->id() == lineBId) lineB = dynamic_cast<const draft::DraftLine*>(e.get());
    }
    if (!lineA || !lineB) return false;

    // Find infinite-line intersection.
    math::Vec2 d1 = lineA->end() - lineA->start();
    math::Vec2 d2 = lineB->end() - lineB->start();
    double denom = d1.cross(d2);
    if (std::abs(denom) < 1e-10) return false;  // Parallel lines.

    math::Vec2 d3 = lineB->start() - lineA->start();
    double tA = d3.cross(d2) / denom;
    math::Vec2 corner = lineA->start() + d1 * tA;

    // Compute chamfer points at m_chamferDist from the corner along each line.
    // Direction away from corner on line A.
    double lenA = d1.length();
    if (lenA < 1e-10) return false;
    math::Vec2 dirA = d1 * (1.0 / lenA);

    double lenB = d2.length();
    if (lenB < 1e-10) return false;
    math::Vec2 dirB = d2 * (1.0 / lenB);

    // Determine which direction on each line goes away from the corner.
    // The line endpoint closer to the corner determines the "toward corner" direction.
    double distStartA = lineA->start().distanceTo(corner);
    double distEndA = lineA->end().distanceTo(corner);
    math::Vec2 awayDirA = (distStartA < distEndA) ? dirA : -dirA;

    double distStartB = lineB->start().distanceTo(corner);
    double distEndB = lineB->end().distanceTo(corner);
    math::Vec2 awayDirB = (distStartB < distEndB) ? dirB : -dirB;

    // Chamfer points.
    chamferPtA = corner + awayDirA * m_chamferDist;
    chamferPtB = corner + awayDirB * m_chamferDist;

    // Trim: the endpoint closer to the corner gets moved to the chamfer point.
    if (distStartA < distEndA) {
        trimA_start = chamferPtA;
        trimA_end = lineA->end();
    } else {
        trimA_start = lineA->start();
        trimA_end = chamferPtA;
    }

    if (distStartB < distEndB) {
        trimB_start = chamferPtB;
        trimB_end = lineB->end();
    } else {
        trimB_start = lineB->start();
        trimB_end = chamferPtB;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Event handlers
// ---------------------------------------------------------------------------

bool ChamferTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->draftDocument();
    double pixelScale = m_viewport->pixelToWorldScale();
    double tolerance = std::max(10.0 * pixelScale, 0.15);

    if (m_state == State::SelectFirstLine) {
        const auto& layerMgr = m_viewport->document()->layerManager();
        for (const auto& entity : doc.entities()) {
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;
            if (dynamic_cast<const draft::DraftLine*>(entity.get()) &&
                entity->hitTest(worldPos, tolerance)) {
                m_firstEntityId = entity->id();
                m_firstClickPos = worldPos;
                m_state = State::SelectSecondLine;
                return true;
            }
        }
        return false;
    }

    if (m_state == State::SelectSecondLine) {
        const auto& layerMgr = m_viewport->document()->layerManager();
        for (const auto& entity : doc.entities()) {
            if (entity->id() == m_firstEntityId) continue;
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;
            if (!dynamic_cast<const draft::DraftLine*>(entity.get())) continue;
            if (!entity->hitTest(worldPos, tolerance)) continue;

            math::Vec2 chamferPtA, chamferPtB;
            math::Vec2 trimA_start, trimA_end, trimB_start, trimB_end;

            if (computeChamfer(m_firstEntityId, m_firstClickPos,
                               entity->id(), worldPos,
                               chamferPtA, chamferPtB,
                               trimA_start, trimA_end, trimB_start, trimB_end)) {

                auto composite = std::make_unique<doc::CompositeCommand>("Chamfer");

                // Remove original lines.
                composite->addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, m_firstEntityId));
                composite->addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, entity->id()));

                // Find originals for property copying.
                const draft::DraftLine* origA = nullptr;
                const draft::DraftLine* origB = nullptr;
                for (const auto& e : doc.entities()) {
                    if (e->id() == m_firstEntityId) origA = dynamic_cast<const draft::DraftLine*>(e.get());
                    if (e->id() == entity->id()) origB = dynamic_cast<const draft::DraftLine*>(e.get());
                }

                // Add trimmed lines.
                auto newLineA = std::make_shared<draft::DraftLine>(trimA_start, trimA_end);
                if (origA) {
                    newLineA->setLayer(origA->layer());
                    newLineA->setColor(origA->color());
                    newLineA->setLineWidth(origA->lineWidth());
                    newLineA->setLineType(origA->lineType());
                }
                composite->addCommand(std::make_unique<doc::AddEntityCommand>(doc, newLineA));

                auto newLineB = std::make_shared<draft::DraftLine>(trimB_start, trimB_end);
                if (origB) {
                    newLineB->setLayer(origB->layer());
                    newLineB->setColor(origB->color());
                    newLineB->setLineWidth(origB->lineWidth());
                    newLineB->setLineType(origB->lineType());
                }
                composite->addCommand(std::make_unique<doc::AddEntityCommand>(doc, newLineB));

                // Add chamfer line (inherits properties from first line).
                auto chamferLine = std::make_shared<draft::DraftLine>(chamferPtA, chamferPtB);
                if (origA) {
                    chamferLine->setLayer(origA->layer());
                    chamferLine->setColor(origA->color());
                    chamferLine->setLineWidth(origA->lineWidth());
                    chamferLine->setLineType(origA->lineType());
                }
                composite->addCommand(std::make_unique<doc::AddEntityCommand>(doc, chamferLine));

                m_viewport->document()->undoStack().push(std::move(composite));
            }

            m_state = State::SelectFirstLine;
            m_firstEntityId = 0;
            return true;
        }
        return false;
    }

    return false;
}

bool ChamferTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    m_currentPos = worldPos;
    return false;
}

bool ChamferTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool ChamferTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }

    // Capture numeric input for chamfer distance.
    if (event->key() >= Qt::Key_0 && event->key() <= Qt::Key_9) {
        m_distInput += static_cast<char>('0' + (event->key() - Qt::Key_0));
        return true;
    }
    if (event->key() == Qt::Key_Period) {
        m_distInput += '.';
        return true;
    }
    if (event->key() == Qt::Key_Backspace && !m_distInput.empty()) {
        m_distInput.pop_back();
        return true;
    }
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
        if (!m_distInput.empty()) {
            try {
                double d = std::stod(m_distInput);
                if (d > 0.0) m_chamferDist = d;
            } catch (...) {}
            m_distInput.clear();
        }
        return true;
    }

    return false;
}

void ChamferTool::cancel() {
    m_state = State::SelectFirstLine;
    m_firstEntityId = 0;
    m_distInput.clear();
}

std::vector<std::pair<math::Vec2, math::Vec2>> ChamferTool::getPreviewLines() const {
    if (m_state != State::SelectSecondLine) return {};
    if (!m_viewport || !m_viewport->document()) return {};

    auto& doc = m_viewport->document()->draftDocument();
    double pixelScale = m_viewport->pixelToWorldScale();
    double tolerance = std::max(10.0 * pixelScale, 0.15);

    // Find line under cursor for preview.
    for (const auto& entity : doc.entities()) {
        if (entity->id() == m_firstEntityId) continue;
        if (!dynamic_cast<const draft::DraftLine*>(entity.get())) continue;
        if (!entity->hitTest(m_currentPos, tolerance)) continue;

        math::Vec2 chamferPtA, chamferPtB;
        math::Vec2 trimA_start, trimA_end, trimB_start, trimB_end;

        if (computeChamfer(m_firstEntityId, m_firstClickPos,
                           entity->id(), m_currentPos,
                           chamferPtA, chamferPtB,
                           trimA_start, trimA_end, trimB_start, trimB_end)) {
            return {{chamferPtA, chamferPtB}};
        }
    }
    return {};
}

std::string ChamferTool::promptText() const {
    std::string base;
    switch (m_state) {
        case State::SelectFirstLine: base = "Select first line for chamfer"; break;
        case State::SelectSecondLine: base = "Select second line for chamfer"; break;
    }
    if (!m_distInput.empty()) {
        base += "  Distance: " + m_distInput;
    } else {
        base += "  [dist=" + std::to_string(m_chamferDist).substr(0, 5) + "]";
    }
    return base;
}

bool ChamferTool::wantsCrosshair() const { return false; }

}  // namespace hz::ui
