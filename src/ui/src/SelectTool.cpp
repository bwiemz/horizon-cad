#include "horizon/ui/SelectTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/ui/GripManager.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/ConstraintCommands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/constraint/ConstraintSystem.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <algorithm>
#include <cmath>
#include <set>

namespace hz::ui {

bool SelectTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->draftDocument();
    auto& sel = m_viewport->selectionManager();

    double pixelScale = m_viewport->pixelToWorldScale();
    const double tolerance = std::max(10.0 * pixelScale, 0.15);

    // --- Check for grip hit first (only when entities are selected) ---
    auto selectedIds = sel.selectedIds();
    if (!selectedIds.empty()) {
        double gripTol = std::max(8.0 * pixelScale, 0.12);

        for (uint64_t id : selectedIds) {
            for (const auto& e : doc.entities()) {
                if (e->id() != id) continue;
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
                break;
            }
        }
    }

    // --- Normal entity selection ---
    const auto& layerMgr = m_viewport->document()->layerManager();
    uint64_t hitId = 0;
    for (const auto& entity : doc.entities()) {
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        if (entity->hitTest(worldPos, tolerance)) {
            hitId = entity->id();
            break;
        }
    }

    bool shiftHeld = (event->modifiers() & Qt::ShiftModifier);

    if (hitId != 0) {
        if (shiftHeld) {
            sel.toggle(hitId);
        } else {
            sel.clearSelection();
            sel.select(hitId);
        }
    } else {
        if (!shiftHeld) {
            sel.clearSelection();
        }
    }

    return true;
}

bool SelectTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (!m_draggingGrip) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    // Apply snapping.
    math::Vec2 snappedPos = worldPos;
    auto result = m_viewport->snapEngine().snap(
        worldPos, m_viewport->document()->draftDocument().entities());
    snappedPos = result.point;
    m_viewport->setLastSnapResult(result);

    m_gripCurrentPos = snappedPos;

    // Live preview: restore entity to before-state, then apply new grip position.
    auto& doc = m_viewport->document()->draftDocument();
    for (auto& e : doc.entities()) {
        if (e->id() == m_gripEntityId) {
            auto fresh = m_gripBeforeClone->clone();
            fresh->setId(m_gripEntityId);
            fresh->setLayer(m_gripBeforeClone->layer());
            fresh->setColor(m_gripBeforeClone->color());
            fresh->setLineWidth(m_gripBeforeClone->lineWidth());
            e = fresh;
            GripManager::moveGrip(*e, m_gripIndex, snappedPos);
            break;
        }
    }

    m_viewport->update();
    return true;
}

bool SelectTool::mouseReleaseEvent(QMouseEvent* event, const math::Vec2& /*worldPos*/) {
    if (!m_draggingGrip) return false;
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->draftDocument();

    // The entity is already in the after-state (from live preview in mouseMoveEvent).
    std::shared_ptr<draft::DraftEntity> afterClone;
    for (const auto& e : doc.entities()) {
        if (e->id() == m_gripEntityId) {
            afterClone = e->clone();
            afterClone->setId(e->id());
            afterClone->setLayer(e->layer());
            afterClone->setColor(e->color());
            afterClone->setLineWidth(e->lineWidth());
            break;
        }
    }

    if (afterClone && m_gripBeforeClone) {
        auto cmd = std::make_unique<doc::GripMoveCommand>(
            doc, m_gripEntityId, m_gripBeforeClone, afterClone);
        m_viewport->document()->undoStack().push(std::move(cmd));
    }

    m_draggingGrip = false;
    m_gripEntityId = 0;
    m_gripIndex = -1;
    m_gripBeforeClone = nullptr;
    m_viewport->setLastSnapResult({});

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
        auto& doc = m_viewport->document()->draftDocument();

        auto composite = std::make_unique<doc::CompositeCommand>("Delete");

        auto& cstrSys = m_viewport->document()->constraintSystem();
        std::set<uint64_t> removedConstraints;
        for (uint64_t id : ids) {
            auto constrs = cstrSys.constraintsForEntity(id);
            for (const auto* c : constrs) {
                if (removedConstraints.insert(c->id()).second) {
                    composite->addCommand(std::make_unique<doc::RemoveConstraintCommand>(
                        cstrSys, c->id()));
                }
            }
        }

        for (uint64_t id : ids) {
            bool canDelete = true;
            for (const auto& e : doc.entities()) {
                if (e->id() == id) {
                    const auto* lp = layerMgr.getLayer(e->layer());
                    if (!lp || !lp->visible || lp->locked) canDelete = false;
                    break;
                }
            }
            if (!canDelete) continue;
            composite->addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, id));
        }
        if (!composite->empty()) {
            m_viewport->document()->undoStack().push(std::move(composite));
        }
        sel.clearSelection();
        return true;
    }
    return false;
}

void SelectTool::cancel() {
    if (m_draggingGrip && m_gripBeforeClone && m_viewport && m_viewport->document()) {
        auto& doc = m_viewport->document()->draftDocument();
        for (auto& e : doc.entities()) {
            if (e->id() == m_gripEntityId) {
                auto restored = m_gripBeforeClone->clone();
                restored->setId(m_gripEntityId);
                restored->setLayer(m_gripBeforeClone->layer());
                restored->setColor(m_gripBeforeClone->color());
                restored->setLineWidth(m_gripBeforeClone->lineWidth());
                e = restored;
                break;
            }
        }
        m_viewport->update();
    }
    m_draggingGrip = false;
    m_gripEntityId = 0;
    m_gripIndex = -1;
    m_gripBeforeClone = nullptr;
}

std::vector<std::pair<math::Vec2, math::Vec2>> SelectTool::getPreviewLines() const {
    if (!m_draggingGrip) return {};
    return {{m_gripOrigPos, m_gripCurrentPos}};
}

}  // namespace hz::ui
