#include "horizon/ui/SelectTool.h"

#include <QInputDialog>
#include <QKeyEvent>
#include <QMouseEvent>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <set>

#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/document/Commands.h"
#include "horizon/document/ConstraintCommands.h"
#include "horizon/document/ConstraintSolveHelper.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/ui/GripManager.h"
#include "horizon/ui/ViewportWidget.h"

namespace hz::ui {

// Expand the current selection to include all group mates of selected entities.
// Respects layer visibility/lock — hidden/locked entities are NOT added.
static void expandSelectionToGroups(render::SelectionManager& sel, const draft::DraftDocument& doc,
                                    const draft::LayerManager& layerMgr) {
    std::set<uint64_t> groupIds;
    for (uint64_t id : sel.selectedIds()) {
        const draft::DraftEntity* e = doc.findEntity(id);
        if (e != nullptr && e->groupId() != 0) groupIds.insert(e->groupId());
    }
    if (groupIds.empty()) return;

    for (const auto& e : doc.entities()) {
        if (e->groupId() == 0) continue;
        if (groupIds.count(e->groupId()) == 0) continue;
        const auto* lp = layerMgr.getLayer(e->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        sel.select(e->id());
    }
}

bool SelectTool::isWindowSelection() const {
    return m_dragCurrent.x >= m_dragStart.x;
}

bool SelectTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    // Handle double-click: try to edit a dimensional constraint on the clicked entity.
    if (event->type() == QEvent::MouseButtonDblClick) {
        if (handleConstraintDoubleClick(worldPos)) return true;
    }

    auto& doc = m_viewport->document()->activeDrawing();
    auto& sel = m_viewport->selectionManager();

    // --- Check for grip hit first (only when entities are selected) ---
    auto selectedIds = sel.selectedIds();
    if (!selectedIds.empty()) {
        double gripTol = m_viewport->pickTolerance(8.0);

        for (uint64_t id : selectedIds) {
            const draft::DraftEntity* e = doc.findEntity(id);
            if (e == nullptr) continue;
            auto grips = GripManager::gripPoints(*e);
            for (int gi = 0; gi < static_cast<int>(grips.size()); ++gi) {
                if (worldPos.distanceTo(grips[gi]) <= gripTol) {
                    // Start grip drag.
                    m_draggingGrip = true;
                    m_gripEntityId = id;
                    m_gripIndex = gi;
                    m_gripOrigPos = grips[gi];
                    m_gripCurrentPos = worldPos;
                    m_gripBeforeClone = e->clone();
                    m_gripBeforeClone->setId(e->id());
                    m_gripBeforeClone->setLayer(e->layer());
                    m_gripBeforeClone->setColor(e->color());
                    m_gripBeforeClone->setLineWidth(e->lineWidth());
                    return true;
                }
            }
        }
    }

    // Record press position for potential box selection drag.
    m_leftButtonDown = true;
    m_draggingBox = false;
    m_dragStart = worldPos;
    m_dragCurrent = worldPos;
    m_dragStartScreen = event->pos();

    return true;
}

bool SelectTool::mouseMoveEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    // --- Grip drag takes priority ---
    if (m_draggingGrip) {
        if (!m_viewport || !m_viewport->document()) return false;

        math::Vec2 snappedPos = worldPos;
        auto result = m_viewport->snap(worldPos);
        snappedPos = result.point;
        m_viewport->setLastSnapResult(result);

        m_gripCurrentPos = snappedPos;

        auto& doc = m_viewport->document()->activeDrawing();
        std::shared_ptr<draft::DraftEntity> fresh = m_gripBeforeClone->clone();
        fresh->setId(m_gripEntityId);
        fresh->setLayer(m_gripBeforeClone->layer());
        fresh->setColor(m_gripBeforeClone->color());
        fresh->setLineWidth(m_gripBeforeClone->lineWidth());
        GripManager::moveGrip(*fresh, m_gripIndex, snappedPos);
        doc.replaceEntity(m_gripEntityId, std::move(fresh));

        m_viewport->update();
        return true;
    }

    // --- Box selection drag ---
    if (m_leftButtonDown && (event->buttons() & Qt::LeftButton)) {
        QPoint screenPos = event->pos();
        int dx = screenPos.x() - m_dragStartScreen.x();
        int dy = screenPos.y() - m_dragStartScreen.y();
        int dist2 = dx * dx + dy * dy;

        if (!m_draggingBox && dist2 >= kDragThreshold * kDragThreshold) {
            m_draggingBox = true;
        }

        if (m_draggingBox) {
            m_dragCurrent = worldPos;
            m_viewport->update();
            return true;
        }
    }

    // Over a solid, the face or edge under the cursor is shown, as what a
    // click there would choose.
    if (!(event->buttons() & Qt::LeftButton)) {
        m_viewport->setModelHover(m_viewport->pickModel(event->position()));
    }
    return false;
}

bool SelectTool::mouseReleaseEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    // --- Grip drag release ---
    if (m_draggingGrip) {
        auto& doc = m_viewport->document()->activeDrawing();

        std::shared_ptr<draft::DraftEntity> afterClone;
        if (const auto e = doc.sharedEntity(m_gripEntityId)) {
            afterClone = e->clone();
            afterClone->setId(e->id());
            afterClone->setLayer(e->layer());
            afterClone->setColor(e->color());
            afterClone->setLineWidth(e->lineWidth());
        }

        if (afterClone && m_gripBeforeClone) {
            auto& cstrSys = m_viewport->document()->activeConstraints();
            auto& pReg = m_viewport->document()->parameterRegistry();
            auto varResolver = [&pReg](const std::string& n) { return pReg.get(n); };
            auto cmd = std::make_unique<doc::GripMoveCommand>(
                doc, m_gripEntityId, m_gripBeforeClone, afterClone, cstrSys, varResolver);
            m_viewport->document()->undoStack().push(std::move(cmd));
        }

        m_draggingGrip = false;
        m_gripEntityId = 0;
        m_gripIndex = -1;
        m_gripBeforeClone = nullptr;
        m_viewport->setLastSnapResult({});
        return true;
    }

    // --- Box selection release ---
    if (m_draggingBox) {
        m_draggingBox = false;
        m_leftButtonDown = false;

        auto& doc = m_viewport->document()->activeDrawing();
        auto& sel = m_viewport->selectionManager();
        const auto& layerMgr = m_viewport->document()->layerManager();
        bool shiftHeld = (event->modifiers() & Qt::ShiftModifier);

        // Build selection rectangle.
        double minX = std::min(m_dragStart.x, m_dragCurrent.x);
        double minY = std::min(m_dragStart.y, m_dragCurrent.y);
        double maxX = std::max(m_dragStart.x, m_dragCurrent.x);
        double maxY = std::max(m_dragStart.y, m_dragCurrent.y);
        math::BoundingBox selectRect(math::Vec3(minX, minY, -1e9), math::Vec3(maxX, maxY, 1e9));

        bool windowMode = isWindowSelection();

        if (!shiftHeld) {
            sel.clearSelection();
        }

        auto candidateIds = doc.spatialIndex().query(selectRect);

        for (uint64_t candId : candidateIds) {
            const draft::DraftEntity* entity = doc.findEntity(candId);
            if (entity == nullptr) continue;
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;

            if (windowMode) {
                math::BoundingBox ebb = entity->boundingBox();
                if (!ebb.isValid()) continue;
                // Window: entity must be fully inside the selection rectangle.
                if (selectRect.contains(ebb)) {
                    sel.select(entity->id());
                }
            } else {
                // Already confirmed intersects via R*-tree query.
                sel.select(entity->id());
            }
        }

        expandSelectionToGroups(sel, doc, layerMgr);
        m_viewport->update();
        return true;
    }

    // --- Normal click selection (no drag occurred) ---
    m_leftButtonDown = false;

    auto& doc = m_viewport->document()->activeDrawing();
    auto& sel = m_viewport->selectionManager();
    const auto& layerMgr = m_viewport->document()->layerManager();

    const double tolerance = m_viewport->pickTolerance(10.0);

    uint64_t hitId = 0;
    {
        math::BoundingBox searchBox(
            math::Vec3(worldPos.x - tolerance, worldPos.y - tolerance, -1e9),
            math::Vec3(worldPos.x + tolerance, worldPos.y + tolerance, 1e9));
        auto candidateIds = doc.spatialIndex().query(searchBox);

        for (uint64_t candId : candidateIds) {
            for (const auto& entity : doc.entities()) {
                if (entity->id() != candId) continue;
                const auto* lp = layerMgr.getLayer(entity->layer());
                if (!lp || !lp->visible || lp->locked) break;
                if (entity->hitTest(worldPos, tolerance)) {
                    hitId = entity->id();
                }
                break;
            }
            if (hitId != 0) break;
        }
    }

    bool shiftHeld = (event->modifiers() & Qt::ShiftModifier);

    if (hitId != 0) {
        // Find the groupId of the hit entity.
        uint64_t hitGroupId = 0;
        if (const auto entity = doc.sharedEntity(hitId)) {
            hitGroupId = entity->groupId();
        }

        if (shiftHeld) {
            if (hitGroupId != 0 && sel.isSelected(hitId)) {
                // Deselect entire group.
                for (const auto& entity : doc.entities()) {
                    if (entity->groupId() == hitGroupId) sel.deselect(entity->id());
                }
            } else {
                sel.select(hitId);
                expandSelectionToGroups(sel, doc, layerMgr);
            }
        } else {
            sel.clearSelection();
            sel.select(hitId);
            expandSelectionToGroups(sel, doc, layerMgr);
        }
    } else {
        if (!shiftHeld) {
            sel.clearSelection();
        }
    }

    // A click on nothing in the drawing chooses the face or edge of a solid
    // under it (Shift adds or takes one away), for the commands that work
    // on faces and edges; a click on the drawing, or on nothing, clears it.
    if (hitId == 0) {
        m_viewport->chooseModel(m_viewport->pickModel(event->position()), shiftHeld);
    } else if (!shiftHeld) {
        m_viewport->clearModelSelection();
    }

    return true;
}

bool SelectTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }

    if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace) {
        if (!m_viewport || !m_viewport->document()) return false;

        auto& sel = m_viewport->selectionManager();
        auto ids = sel.selectedIds();
        if (ids.empty()) return false;

        const auto& layerMgr = m_viewport->document()->layerManager();
        auto& doc = m_viewport->document()->activeDrawing();

        auto composite = std::make_unique<doc::CompositeCommand>("Delete");

        auto& cstrSys = m_viewport->document()->activeConstraints();
        std::set<uint64_t> removedConstraints;
        for (uint64_t id : ids) {
            auto constrs = cstrSys.constraintsForEntity(id);
            for (const auto* c : constrs) {
                if (removedConstraints.insert(c->id()).second) {
                    composite->addCommand(
                        std::make_unique<doc::RemoveConstraintCommand>(cstrSys, c->id()));
                }
            }
        }

        // One command for all of them: removing entities one command at a
        // time scans the drawing once per entity.
        std::vector<uint64_t> deletable;
        deletable.reserve(ids.size());
        for (uint64_t id : ids) {
            const draft::DraftEntity* e = doc.findEntity(id);
            if (e == nullptr) continue;
            const auto* lp = layerMgr.getLayer(e->layer());
            if (!lp || !lp->visible || lp->locked) continue;
            deletable.push_back(id);
        }
        if (!deletable.empty()) {
            composite->addCommand(
                std::make_unique<doc::RemoveEntitiesCommand>(doc, std::move(deletable)));
        }
        if (!composite->empty()) {
            m_viewport->document()->undoStack().push(std::move(composite));
        }
        sel.clearSelection();
        return true;
    }
    return false;
}

bool SelectTool::handleConstraintDoubleClick(const math::Vec2& worldPos) {
    if (!m_viewport || !m_viewport->document()) return false;

    auto& draftDoc = m_viewport->document()->activeDrawing();
    auto& cstrSys = m_viewport->document()->activeConstraints();

    double tolerance = m_viewport->pickTolerance(15.0);

    for (const auto& constraint : cstrSys.constraints()) {
        if (!constraint->hasDimensionalValue()) continue;

        const auto entityIds = constraint->referencedEntityIds();
        bool hit = false;
        for (uint64_t eid : entityIds) {
            for (const auto& e : draftDoc.entities()) {
                if (e->id() != eid) continue;
                if (e->hitTest(worldPos, tolerance)) {
                    hit = true;
                }
                break;
            }
            if (hit) break;
        }

        if (hit) {
            bool isAngle = (constraint->type() == cstr::ConstraintType::Angle);
            return editConstraintDimension(constraint->id(), constraint->dimensionalValue(),
                                           isAngle);
        }
    }
    return false;
}

bool SelectTool::editConstraintDimension(uint64_t constraintId, double currentValue, bool isAngle) {
    if (!m_viewport || !m_viewport->document()) return false;

    auto& cstrSys = m_viewport->document()->activeConstraints();
    auto& draftDoc = m_viewport->document()->activeDrawing();

    const double pi = std::numbers::pi;

    // Convert to display units (degrees for angles).
    double displayValue = isAngle ? (currentValue * 180.0 / pi) : currentValue;
    QString label = isAngle ? QStringLiteral("Angle (degrees):") : QStringLiteral("Distance:");

    bool ok = false;
    double newDisplay =
        QInputDialog::getDouble(m_viewport, QStringLiteral("Edit Constraint"), label, displayValue,
                                isAngle ? 0.001 : 0.001,  // min
                                isAngle ? 359.999 : 1e9,  // max
                                4,                        // decimals
                                &ok);

    if (!ok) return false;

    double newValue = isAngle ? (newDisplay * pi / 180.0) : newDisplay;
    if (std::abs(newValue - currentValue) < 1e-12) return false;

    // Build composite: modify value + apply solve result.
    auto composite = std::make_unique<doc::CompositeCommand>("Edit Constraint Value");
    composite->addCommand(
        std::make_unique<doc::ModifyConstraintValueCommand>(cstrSys, constraintId, newValue));

    // Temporarily set the new value so the solver can use it.
    cstr::Constraint* c = cstrSys.getConstraint(constraintId);
    if (!c) return false;
    c->setDimensionalValue(newValue);

    auto& paramReg = m_viewport->document()->parameterRegistry();
    auto resolver = [&paramReg](const std::string& name) { return paramReg.get(name); };
    auto solveCmd = doc::ConstraintSolveHelper::solveAndCreateCommand(draftDoc, cstrSys, resolver);

    // Restore old value — ModifyConstraintValueCommand::execute() will re-apply it.
    c->setDimensionalValue(currentValue);

    if (solveCmd) {
        // Undo the entity changes that solveAndCreateCommand applied.
        solveCmd->undo();
        composite->addCommand(std::move(solveCmd));
    }

    m_viewport->document()->undoStack().push(std::move(composite));
    return true;
}

void SelectTool::cancel() {
    if (m_draggingBox) {
        m_draggingBox = false;
        m_leftButtonDown = false;
        if (m_viewport) m_viewport->update();
        return;
    }

    if (m_draggingGrip && m_gripBeforeClone && m_viewport && m_viewport->document()) {
        auto& doc = m_viewport->document()->activeDrawing();
        std::shared_ptr<draft::DraftEntity> restored = m_gripBeforeClone->clone();
        restored->setId(m_gripEntityId);
        restored->setLayer(m_gripBeforeClone->layer());
        restored->setColor(m_gripBeforeClone->color());
        restored->setLineWidth(m_gripBeforeClone->lineWidth());
        doc.replaceEntity(m_gripEntityId, std::move(restored));
        m_viewport->update();
    }
    m_draggingGrip = false;
    m_gripEntityId = 0;
    m_gripIndex = -1;
    m_gripBeforeClone = nullptr;
}

std::vector<std::pair<math::Vec2, math::Vec2>> SelectTool::getPreviewLines() const {
    if (m_draggingGrip) {
        return {{m_gripOrigPos, m_gripCurrentPos}};
    }

    if (m_draggingBox) {
        math::Vec2 p1 = m_dragStart;
        math::Vec2 p2 = m_dragCurrent;
        math::Vec2 tl{std::min(p1.x, p2.x), std::max(p1.y, p2.y)};
        math::Vec2 tr{std::max(p1.x, p2.x), std::max(p1.y, p2.y)};
        math::Vec2 br{std::max(p1.x, p2.x), std::min(p1.y, p2.y)};
        math::Vec2 bl{std::min(p1.x, p2.x), std::min(p1.y, p2.y)};
        return {{tl, tr}, {tr, br}, {br, bl}, {bl, tl}};
    }

    return {};
}

std::string SelectTool::promptText() const {
    if (m_draggingBox) {
        return isWindowSelection()
                   ? "Window selection \xe2\x80\x94 only fully enclosed entities"
                   : "Crossing selection \xe2\x80\x94 overlapping entities included";
    }
    return "Click to select, drag for box selection. Shift to add.";
}

bool SelectTool::wantsCrosshair() const {
    return false;
}

math::Vec3 SelectTool::previewColor() const {
    if (m_draggingBox) {
        return isWindowSelection() ? math::Vec3{0.3, 0.5, 1.0}   // Blue for window
                                   : math::Vec3{0.3, 1.0, 0.5};  // Green for crossing
    }
    return {0.0, 0.8, 1.0};  // Default cyan
}

}  // namespace hz::ui
