#include "horizon/ui/MoveTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/ConstraintCommands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/constraint/ParameterTable.h"
#include "horizon/constraint/SketchSolver.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <algorithm>
#include <set>

namespace hz::ui {

void MoveTool::deactivate() {
    cancel();
    Tool::deactivate();
}

bool MoveTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& sel = m_viewport->selectionManager();
    auto ids = sel.selectedIds();
    if (ids.empty()) return false;

    // Check if click is on a selected entity.
    auto& doc = m_viewport->document()->draftDocument();
    double pixelScale = m_viewport->pixelToWorldScale();
    double tolerance = std::max(10.0 * pixelScale, 0.15);

    bool hitSelected = false;
    const auto& layerMgr = m_viewport->document()->layerManager();
    for (const auto& entity : doc.entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        if (entity->hitTest(worldPos, tolerance)) {
            hitSelected = true;
            break;
        }
    }
    if (!hitSelected) return false;

    // Apply snapping.
    math::Vec2 snappedPos = worldPos;
    auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
    snappedPos = result.point;
    m_viewport->setLastSnapResult(result);

    m_dragging = true;
    m_dragStart = snappedPos;
    m_dragCurrent = snappedPos;
    m_totalDelta = {0.0, 0.0};
    return true;
}

bool MoveTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (!m_dragging) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    math::Vec2 snappedPos = worldPos;
    auto result = m_viewport->snapEngine().snap(
        worldPos, m_viewport->document()->draftDocument().entities());
    snappedPos = result.point;
    m_viewport->setLastSnapResult(result);

    math::Vec2 delta{snappedPos.x - m_dragCurrent.x, snappedPos.y - m_dragCurrent.y};

    // Translate selected entities in real-time (skip locked/hidden layers).
    auto& doc = m_viewport->document()->draftDocument();
    auto& sel = m_viewport->selectionManager();
    const auto& layerMgr = m_viewport->document()->layerManager();
    for (const auto& entity : doc.entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        entity->translate(delta);
    }

    m_dragCurrent = snappedPos;
    m_totalDelta.x += delta.x;
    m_totalDelta.y += delta.y;
    return true;
}

bool MoveTool::mouseReleaseEvent(QMouseEvent* event, const math::Vec2& /*worldPos*/) {
    if (event->button() != Qt::LeftButton || !m_dragging) return false;
    if (!m_viewport || !m_viewport->document()) {
        m_dragging = false;
        return false;
    }

    // Undo the real-time translation, then push a command for the total delta.
    math::Vec2 neg{-m_totalDelta.x, -m_totalDelta.y};
    auto& doc = m_viewport->document()->draftDocument();
    auto& sel = m_viewport->selectionManager();
    const auto& layerMgr = m_viewport->document()->layerManager();

    for (const auto& entity : doc.entities()) {
        if (!sel.isSelected(entity->id())) continue;
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        entity->translate(neg);
    }

    if (std::abs(m_totalDelta.x) > 1e-10 || std::abs(m_totalDelta.y) > 1e-10) {
        // Build ID list excluding locked/hidden entities.
        std::vector<uint64_t> idVec;
        for (const auto& entity : doc.entities()) {
            if (!sel.isSelected(entity->id())) continue;
            const auto* lp2 = layerMgr.getLayer(entity->layer());
            if (!lp2 || !lp2->visible || lp2->locked) continue;
            idVec.push_back(entity->id());
        }
        auto cmd = std::make_unique<doc::MoveEntityCommand>(doc, idVec, m_totalDelta);
        m_viewport->document()->undoStack().push(std::move(cmd));

        // Post-move constraint solve: if any moved entity is constrained, re-solve.
        auto& cstrSys = m_viewport->document()->constraintSystem();
        if (!cstrSys.empty()) {
            // Collect all constrained entity IDs.
            std::set<uint64_t> constrainedIds;
            for (const auto& c : cstrSys.constraints()) {
                for (uint64_t eid : c->referencedEntityIds()) {
                    constrainedIds.insert(eid);
                }
            }

            // Check if any moved entity is constrained.
            bool needsSolve = false;
            for (uint64_t id : idVec) {
                if (constrainedIds.count(id)) {
                    needsSolve = true;
                    break;
                }
            }

            if (needsSolve) {
                // Snapshot before-solve state (entities are already moved).
                std::vector<doc::ApplyConstraintSolveCommand::EntitySnapshot> snapshots;
                for (uint64_t eid : constrainedIds) {
                    for (const auto& entity : doc.entities()) {
                        if (entity->id() == eid) {
                            doc::ApplyConstraintSolveCommand::EntitySnapshot snap;
                            snap.entityId = eid;
                            snap.beforeState = entity->clone();
                            snapshots.push_back(std::move(snap));
                            break;
                        }
                    }
                }

                // Build parameter table and solve.
                auto params = cstr::ParameterTable::buildFromEntities(
                    doc.entities(), cstrSys);
                cstr::SketchSolver solver;
                auto result = solver.solve(params, cstrSys);

                if (result.status == cstr::SolveStatus::Success ||
                    result.status == cstr::SolveStatus::UnderConstrained) {
                    params.applyToEntities(doc.entities());

                    // Snapshot after-solve state.
                    for (auto& snap : snapshots) {
                        for (const auto& entity : doc.entities()) {
                            if (entity->id() == snap.entityId) {
                                snap.afterState = entity->clone();
                                break;
                            }
                        }
                    }

                    // Push solve command. Entities are already in afterState;
                    // execute() applies afterState (no-op), undo() restores beforeState.
                    auto solveCmd = std::make_unique<doc::ApplyConstraintSolveCommand>(
                        doc, std::move(snapshots));
                    m_viewport->document()->undoStack().push(std::move(solveCmd));
                }
            }
        }
    }

    m_dragging = false;
    m_viewport->setLastSnapResult({});
    return true;
}

bool MoveTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape && m_dragging) {
        cancel();
        return true;
    }
    return false;
}

void MoveTool::cancel() {
    if (m_dragging && m_viewport && m_viewport->document()) {
        // Undo the real-time translation (skip locked/hidden layers).
        math::Vec2 neg{-m_totalDelta.x, -m_totalDelta.y};
        auto& doc = m_viewport->document()->draftDocument();
        auto& sel = m_viewport->selectionManager();
        const auto& layerMgr = m_viewport->document()->layerManager();
        for (const auto& entity : doc.entities()) {
            if (!sel.isSelected(entity->id())) continue;
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;
            entity->translate(neg);
        }
    }
    m_dragging = false;
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

}  // namespace hz::ui
