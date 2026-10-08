#include "horizon/ui/ChamferTool.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <cmath>

#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/ui/ViewportWidget.h"

namespace hz::ui {

namespace {

/// The unit a typed length is in: the document's (Phase 154).
math::LengthUnit lengthUnit(const ViewportWidget* viewport) {
    return viewport != nullptr && viewport->document() != nullptr
               ? viewport->document()->lengthUnit()
               : math::LengthUnit::Millimetre;
}

/// The way along @p line from @p corner to the part of it clicked at @p click:
/// the part kept. Clicked at the corner itself, the longer part.
math::Vec2 keptSide(const draft::DraftLine& line, const math::Vec2& corner,
                    const math::Vec2& click) {
    const math::Vec2 along = (line.end() - line.start()).normalized();
    const double t = (click - corner).dot(along);
    if (std::abs(t) > 1e-9) return t > 0.0 ? along : -along;
    return line.end().distanceTo(corner) >= line.start().distanceTo(corner) ? along : -along;
}

/// @p line cut back to @p cut, keeping its end on the @p kept side: the other
/// end moves to @p cut, and the line runs the way it did.
void cutBack(const draft::DraftLine& line, const math::Vec2& kept, const math::Vec2& cut,
             math::Vec2& start, math::Vec2& end) {
    if ((line.end() - line.start()).dot(kept) > 0.0) {
        start = cut;
        end = line.end();
    } else {
        start = line.start();
        end = cut;
    }
}

}  // namespace

void ChamferTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_state = State::SelectFirstLine;
    m_firstEntityId = 0;
    m_distance.clear();
}

void ChamferTool::deactivate() {
    cancel();
    Tool::deactivate();
}

// ---------------------------------------------------------------------------
// Chamfer computation (line-line only)
// ---------------------------------------------------------------------------

bool ChamferTool::computeChamfer(uint64_t lineAId, const math::Vec2& clickA, uint64_t lineBId,
                                 const math::Vec2& clickB, math::Vec2& chamferPtA,
                                 math::Vec2& chamferPtB, math::Vec2& trimA_start,
                                 math::Vec2& trimA_end, math::Vec2& trimB_start,
                                 math::Vec2& trimB_end) const {
    if (!m_viewport || !m_viewport->document()) return false;
    auto& doc = m_viewport->document()->activeDrawing();

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

    // Each line keeps the part on the side of the corner it was clicked on,
    // cut back to the chamfer distance from the corner; the chamfer joins
    // the two cuts.
    const math::Vec2 keptA = keptSide(*lineA, corner, clickA);
    const math::Vec2 keptB = keptSide(*lineB, corner, clickB);
    chamferPtA = corner + keptA * m_distance.value();
    chamferPtB = corner + keptB * m_distance.value();
    cutBack(*lineA, keptA, chamferPtA, trimA_start, trimA_end);
    cutBack(*lineB, keptB, chamferPtB, trimB_start, trimB_end);

    return true;
}

// ---------------------------------------------------------------------------
// Event handlers
// ---------------------------------------------------------------------------

bool ChamferTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->activeDrawing();
    double tolerance = m_viewport->pickTolerance(10.0);

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

            if (computeChamfer(m_firstEntityId, m_firstClickPos, entity->id(), worldPos, chamferPtA,
                               chamferPtB, trimA_start, trimA_end, trimB_start, trimB_end)) {
                auto composite = std::make_unique<doc::CompositeCommand>("Chamfer");

                // Remove original lines.
                composite->addCommand(
                    std::make_unique<doc::RemoveEntityCommand>(doc, m_firstEntityId));
                composite->addCommand(
                    std::make_unique<doc::RemoveEntityCommand>(doc, entity->id()));

                // Find originals for property copying.
                const draft::DraftLine* origA = nullptr;
                const draft::DraftLine* origB = nullptr;
                for (const auto& e : doc.entities()) {
                    if (e->id() == m_firstEntityId)
                        origA = dynamic_cast<const draft::DraftLine*>(e.get());
                    if (e->id() == entity->id())
                        origB = dynamic_cast<const draft::DraftLine*>(e.get());
                }

                // Add trimmed lines.
                auto newLineA = std::make_shared<draft::DraftLine>(trimA_start, trimA_end);
                if (origA) {
                    newLineA->copyStyleFrom(*origA);
                }
                composite->addCommand(std::make_unique<doc::AddEntityCommand>(doc, newLineA));

                auto newLineB = std::make_shared<draft::DraftLine>(trimB_start, trimB_end);
                if (origB) {
                    newLineB->copyStyleFrom(*origB);
                }
                composite->addCommand(std::make_unique<doc::AddEntityCommand>(doc, newLineB));

                // Add chamfer line (inherits properties from first line).
                auto chamferLine = std::make_shared<draft::DraftLine>(chamferPtA, chamferPtB);
                if (origA) {
                    chamferLine->copyStyleFrom(*origA);
                    // It joins a group only if both lines are in it.
                    if (!origB || origB->groupId() != origA->groupId()) chamferLine->setGroupId(0);
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

    // A distance typed while the tool runs.
    return m_distance.key(event->key(), lengthUnit(m_viewport));
}

void ChamferTool::cancel() {
    m_state = State::SelectFirstLine;
    m_firstEntityId = 0;
    m_distance.clear();
}

std::vector<std::pair<math::Vec2, math::Vec2>> ChamferTool::getPreviewLines() const {
    if (m_state != State::SelectSecondLine) return {};
    if (!m_viewport || !m_viewport->document()) return {};

    auto& doc = m_viewport->document()->activeDrawing();
    double tolerance = m_viewport->pickTolerance(10.0);

    // Find line under cursor for preview.
    for (const auto& entity : doc.entities()) {
        if (entity->id() == m_firstEntityId) continue;
        if (!dynamic_cast<const draft::DraftLine*>(entity.get())) continue;
        if (!entity->hitTest(m_currentPos, tolerance)) continue;

        math::Vec2 chamferPtA, chamferPtB;
        math::Vec2 trimA_start, trimA_end, trimB_start, trimB_end;

        if (computeChamfer(m_firstEntityId, m_firstClickPos, entity->id(), m_currentPos, chamferPtA,
                           chamferPtB, trimA_start, trimA_end, trimB_start, trimB_end)) {
            return {{chamferPtA, chamferPtB}};
        }
    }
    return {};
}

std::string ChamferTool::promptText() const {
    std::string base;
    switch (m_state) {
        case State::SelectFirstLine:
            base = "Select first line for chamfer";
            break;
        case State::SelectSecondLine:
            base = "Select second line for chamfer";
            break;
    }
    return base + m_distance.prompt("distance", lengthUnit(m_viewport));
}

bool ChamferTool::wantsCrosshair() const {
    return false;
}

}  // namespace hz::ui
