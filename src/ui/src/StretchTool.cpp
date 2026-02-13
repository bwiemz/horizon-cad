#include "horizon/ui/StretchTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/MathUtils.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <algorithm>
#include <cmath>
#include <set>

namespace hz::ui {

// ---------------------------------------------------------------------------
// Helper: test if a 2D point is inside an axis-aligned rectangle
// ---------------------------------------------------------------------------

static bool pointInRect(const math::Vec2& pt,
                        const math::Vec2& rectMin,
                        const math::Vec2& rectMax) {
    return pt.x >= rectMin.x && pt.x <= rectMax.x &&
           pt.y >= rectMin.y && pt.y <= rectMax.y;
}

// ---------------------------------------------------------------------------
// Helper: get "stretch points" for an entity
// ---------------------------------------------------------------------------
// These are the key geometry vertices that can be displaced independently.
// Different from GripManager for some types (e.g. rectangle uses 4 corners,
// ellipse/circle only use center).

static std::vector<math::Vec2> getStretchPoints(const draft::DraftEntity& entity) {
    if (auto* e = dynamic_cast<const draft::DraftLine*>(&entity)) {
        return {e->start(), e->end()};
    }
    if (auto* e = dynamic_cast<const draft::DraftArc*>(&entity)) {
        return {e->center(), e->startPoint(), e->endPoint()};
    }
    if (auto* e = dynamic_cast<const draft::DraftCircle*>(&entity)) {
        return {e->center()};
    }
    if (auto* e = dynamic_cast<const draft::DraftRectangle*>(&entity)) {
        auto c1 = e->corner1();
        auto c2 = e->corner2();
        return {{c1.x, c1.y}, {c2.x, c1.y}, {c2.x, c2.y}, {c1.x, c2.y}};
    }
    if (auto* e = dynamic_cast<const draft::DraftPolyline*>(&entity)) {
        return e->points();
    }
    if (auto* e = dynamic_cast<const draft::DraftSpline*>(&entity)) {
        return e->controlPoints();
    }
    if (auto* e = dynamic_cast<const draft::DraftEllipse*>(&entity)) {
        return {e->center()};
    }
    if (auto* e = dynamic_cast<const draft::DraftText*>(&entity)) {
        return {e->position()};
    }
    if (auto* e = dynamic_cast<const draft::DraftHatch*>(&entity)) {
        return e->boundary();
    }
    if (auto* e = dynamic_cast<const draft::DraftBlockRef*>(&entity)) {
        return {e->insertPos()};
    }
    return {};
}

// ---------------------------------------------------------------------------
// Helper: restore entity geometry from a clone (undo in-place modifications)
// ---------------------------------------------------------------------------

static void restoreEntityState(const draft::DraftEntity& src, draft::DraftEntity& dst) {
    if (auto* s = dynamic_cast<const draft::DraftLine*>(&src)) {
        auto* d = dynamic_cast<draft::DraftLine*>(&dst);
        d->setStart(s->start());
        d->setEnd(s->end());
    } else if (auto* s = dynamic_cast<const draft::DraftArc*>(&src)) {
        auto* d = dynamic_cast<draft::DraftArc*>(&dst);
        d->setCenter(s->center());
        d->setRadius(s->radius());
        d->setStartAngle(s->startAngle());
        d->setEndAngle(s->endAngle());
    } else if (auto* s = dynamic_cast<const draft::DraftCircle*>(&src)) {
        auto* d = dynamic_cast<draft::DraftCircle*>(&dst);
        d->setCenter(s->center());
        d->setRadius(s->radius());
    } else if (auto* s = dynamic_cast<const draft::DraftRectangle*>(&src)) {
        auto* d = dynamic_cast<draft::DraftRectangle*>(&dst);
        d->setCorner1(s->corner1());
        d->setCorner2(s->corner2());
    } else if (auto* s = dynamic_cast<const draft::DraftPolyline*>(&src)) {
        auto* d = dynamic_cast<draft::DraftPolyline*>(&dst);
        d->setPoints(s->points());
    } else if (auto* s = dynamic_cast<const draft::DraftSpline*>(&src)) {
        auto* d = dynamic_cast<draft::DraftSpline*>(&dst);
        d->setControlPoints(s->controlPoints());
    } else if (auto* s = dynamic_cast<const draft::DraftEllipse*>(&src)) {
        auto* d = dynamic_cast<draft::DraftEllipse*>(&dst);
        d->setCenter(s->center());
        d->setSemiMajor(s->semiMajor());
        d->setSemiMinor(s->semiMinor());
        d->setRotation(s->rotation());
    } else if (auto* s = dynamic_cast<const draft::DraftText*>(&src)) {
        auto* d = dynamic_cast<draft::DraftText*>(&dst);
        d->setPosition(s->position());
    } else if (auto* s = dynamic_cast<const draft::DraftHatch*>(&src)) {
        auto* d = dynamic_cast<draft::DraftHatch*>(&dst);
        d->setBoundary(s->boundary());
    } else if (auto* s = dynamic_cast<const draft::DraftBlockRef*>(&src)) {
        auto* d = dynamic_cast<draft::DraftBlockRef*>(&dst);
        d->setInsertPos(s->insertPos());
    }
}

// ---------------------------------------------------------------------------
// Helper: apply a stretch displacement to specific points of an entity
// ---------------------------------------------------------------------------

static void applyStretch(draft::DraftEntity& entity,
                         const std::vector<int>& insideIndices,
                         int totalPoints,
                         const math::Vec2& disp) {
    if (insideIndices.empty()) return;

    // If ALL points are inside, just translate.
    if (static_cast<int>(insideIndices.size()) == totalPoints) {
        entity.translate(disp);
        return;
    }

    // Build a set for fast lookup.
    std::set<int> inside(insideIndices.begin(), insideIndices.end());

    // --- Line ---
    if (auto* e = dynamic_cast<draft::DraftLine*>(&entity)) {
        if (inside.count(0)) e->setStart(e->start() + disp);
        if (inside.count(1)) e->setEnd(e->end() + disp);
        return;
    }

    // --- Arc ---
    if (auto* e = dynamic_cast<draft::DraftArc*>(&entity)) {
        bool centerIn = inside.count(0);
        bool startIn = inside.count(1);
        bool endIn = inside.count(2);

        if (centerIn) {
            // Move center — angles stay the same, so endpoints follow.
            e->setCenter(e->center() + disp);
        }
        if (startIn && !centerIn) {
            // Stretch start endpoint: recalculate start angle (keep radius).
            // The endpoint is projected onto the arc's circle — it follows
            // a circular path rather than the exact cursor displacement.
            math::Vec2 newPt = e->startPoint() + disp;
            double angle = std::atan2(newPt.y - e->center().y, newPt.x - e->center().x);
            e->setStartAngle(math::normalizeAngle(angle));
        }
        if (endIn && !centerIn) {
            // Stretch end endpoint: recalculate end angle (keep radius).
            // Same projection logic as start endpoint above.
            math::Vec2 newPt = e->endPoint() + disp;
            double angle = std::atan2(newPt.y - e->center().y, newPt.x - e->center().x);
            e->setEndAngle(math::normalizeAngle(angle));
        }
        return;
    }

    // --- Circle ---
    if (auto* e = dynamic_cast<draft::DraftCircle*>(&entity)) {
        if (inside.count(0)) e->setCenter(e->center() + disp);
        return;
    }

    // --- Rectangle (4-corner mapping) ---
    if (auto* e = dynamic_cast<draft::DraftRectangle*>(&entity)) {
        // Stretch points: [0]=(c1.x,c1.y), [1]=(c2.x,c1.y), [2]=(c2.x,c2.y), [3]=(c1.x,c2.y)
        bool moveX1 = inside.count(0) || inside.count(3);
        bool moveY1 = inside.count(0) || inside.count(1);
        bool moveX2 = inside.count(1) || inside.count(2);
        bool moveY2 = inside.count(2) || inside.count(3);

        auto c1 = e->corner1();
        auto c2 = e->corner2();
        e->setCorner1({c1.x + (moveX1 ? disp.x : 0.0), c1.y + (moveY1 ? disp.y : 0.0)});
        e->setCorner2({c2.x + (moveX2 ? disp.x : 0.0), c2.y + (moveY2 ? disp.y : 0.0)});
        return;
    }

    // --- Polyline ---
    if (auto* e = dynamic_cast<draft::DraftPolyline*>(&entity)) {
        auto pts = e->points();
        for (int i : insideIndices) {
            if (i >= 0 && i < static_cast<int>(pts.size()))
                pts[i] = pts[i] + disp;
        }
        e->setPoints(pts);
        return;
    }

    // --- Spline ---
    if (auto* e = dynamic_cast<draft::DraftSpline*>(&entity)) {
        auto pts = e->controlPoints();
        for (int i : insideIndices) {
            if (i >= 0 && i < static_cast<int>(pts.size()))
                pts[i] = pts[i] + disp;
        }
        e->setControlPoints(pts);
        return;
    }

    // --- Ellipse ---
    if (auto* e = dynamic_cast<draft::DraftEllipse*>(&entity)) {
        if (inside.count(0)) e->setCenter(e->center() + disp);
        return;
    }

    // --- Text ---
    if (auto* e = dynamic_cast<draft::DraftText*>(&entity)) {
        if (inside.count(0)) e->setPosition(e->position() + disp);
        return;
    }

    // --- Hatch ---
    if (auto* e = dynamic_cast<draft::DraftHatch*>(&entity)) {
        auto bnd = e->boundary();
        for (int i : insideIndices) {
            if (i >= 0 && i < static_cast<int>(bnd.size()))
                bnd[i] = bnd[i] + disp;
        }
        e->setBoundary(bnd);
        return;
    }

    // --- Block ref ---
    if (auto* e = dynamic_cast<draft::DraftBlockRef*>(&entity)) {
        if (inside.count(0)) e->setInsertPos(e->insertPos() + disp);
        return;
    }
}

// ---------------------------------------------------------------------------
// StretchTool: collect entities and their stretch points from crossing window
// ---------------------------------------------------------------------------

void StretchTool::collectStretchEntities() {
    m_stretchEntities.clear();
    if (!m_viewport || !m_viewport->document()) return;

    auto& doc = m_viewport->document()->draftDocument();
    const auto& layerMgr = m_viewport->document()->layerManager();

    // Normalize window to min/max.
    math::Vec2 wMin{std::min(m_windowStart.x, m_windowEnd.x),
                    std::min(m_windowStart.y, m_windowEnd.y)};
    math::Vec2 wMax{std::max(m_windowStart.x, m_windowEnd.x),
                    std::max(m_windowStart.y, m_windowEnd.y)};

    // Build BoundingBox for entity-level intersection check.
    math::BoundingBox windowBB(math::Vec3(wMin.x, wMin.y, -1e9),
                               math::Vec3(wMax.x, wMax.y, 1e9));

    for (const auto& entity : doc.entities()) {
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;

        // Quick reject: entity must intersect the crossing window.
        auto ebb = entity->boundingBox();
        if (!ebb.isValid() || !windowBB.intersects(ebb)) continue;

        // Check which stretch points are inside the window.
        auto pts = getStretchPoints(*entity);
        if (pts.empty()) continue;

        std::vector<int> inside;
        for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
            if (pointInRect(pts[i], wMin, wMax)) {
                inside.push_back(i);
            }
        }
        if (inside.empty()) continue;

        StretchEntity se;
        se.entityId = entity->id();
        se.beforeClone = entity->clone();
        se.insideIndices = std::move(inside);
        se.totalPoints = static_cast<int>(pts.size());
        m_stretchEntities.push_back(std::move(se));
    }
}

// ---------------------------------------------------------------------------
// StretchTool: apply current displacement to all stretch entities
// ---------------------------------------------------------------------------

void StretchTool::applyCurrentStretch() {
    if (!m_viewport || !m_viewport->document()) return;
    auto& doc = m_viewport->document()->draftDocument();

    math::Vec2 disp{m_currentPos.x - m_basePoint.x, m_currentPos.y - m_basePoint.y};

    for (const auto& se : m_stretchEntities) {
        for (const auto& entity : doc.entities()) {
            if (entity->id() != se.entityId) continue;

            // Restore to before-state first.
            restoreEntityState(*se.beforeClone, *entity);

            // Then apply stretch with current displacement.
            applyStretch(*entity, se.insideIndices, se.totalPoints, disp);
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// StretchTool: restore all entities from before-clones
// ---------------------------------------------------------------------------

void StretchTool::restoreAllEntities() {
    if (!m_viewport || !m_viewport->document()) return;
    auto& doc = m_viewport->document()->draftDocument();

    for (const auto& se : m_stretchEntities) {
        for (const auto& entity : doc.entities()) {
            if (entity->id() != se.entityId) continue;
            restoreEntityState(*se.beforeClone, *entity);
            break;
        }
    }
}

void StretchTool::resetState() {
    m_state = State::SelectingWindow;
    m_stretchEntities.clear();
    if (m_viewport) {
        m_viewport->setLastSnapResult({});
    }
}

// ---------------------------------------------------------------------------
// Event handlers
// ---------------------------------------------------------------------------

bool StretchTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    switch (m_state) {
        case State::SelectingWindow:
            m_windowStart = worldPos;
            m_windowEnd = worldPos;
            m_state = State::DraggingWindow;
            return true;

        case State::DraggingWindow: {
            m_windowEnd = worldPos;
            collectStretchEntities();
            if (m_stretchEntities.empty()) {
                // No entities found — restart.
                m_state = State::SelectingWindow;
                return true;
            }
            m_state = State::WaitingBasePoint;
            return true;
        }

        case State::WaitingBasePoint: {
            // Apply snapping for precise base point.
            auto& doc = m_viewport->document()->draftDocument();
            auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
            m_basePoint = result.point;
            m_currentPos = result.point;
            m_viewport->setLastSnapResult(result);
            m_state = State::Dragging;
            return true;
        }

        case State::Dragging: {
            // Finalize the stretch.
            auto& doc = m_viewport->document()->draftDocument();
            auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
            m_currentPos = result.point;
            m_viewport->setLastSnapResult(result);

            math::Vec2 disp{m_currentPos.x - m_basePoint.x,
                            m_currentPos.y - m_basePoint.y};

            if (std::abs(disp.x) < 1e-10 && std::abs(disp.y) < 1e-10) {
                // Zero displacement — just restore and cancel.
                restoreAllEntities();
                resetState();
                return true;
            }

            // Entities are already in stretched state from mouseMoveEvent.
            // Clone after-states, then restore before-states, then push command.
            auto composite = std::make_unique<doc::CompositeCommand>("Stretch");

            for (auto& se : m_stretchEntities) {
                for (const auto& entity : doc.entities()) {
                    if (entity->id() != se.entityId) continue;

                    auto afterClone = entity->clone();
                    restoreEntityState(*se.beforeClone, *entity);

                    composite->addCommand(std::make_unique<doc::GripMoveCommand>(
                        doc, se.entityId, se.beforeClone, afterClone));
                    break;
                }
            }

            if (!composite->empty()) {
                m_viewport->document()->undoStack().push(std::move(composite));
            }

            resetState();
            return true;
        }
    }
    return false;
}

bool StretchTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    if (!m_viewport || !m_viewport->document()) return false;

    switch (m_state) {
        case State::DraggingWindow:
            m_windowEnd = worldPos;
            return true;

        case State::Dragging: {
            auto& doc = m_viewport->document()->draftDocument();
            auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
            m_currentPos = result.point;
            m_viewport->setLastSnapResult(result);

            // Apply stretch in real-time (restore from clones first).
            applyCurrentStretch();
            return true;
        }

        default:
            return false;
    }
}

bool StretchTool::mouseReleaseEvent(QMouseEvent* /*event*/,
                                     const math::Vec2& /*worldPos*/) {
    return false;
}

bool StretchTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void StretchTool::deactivate() {
    cancel();
    Tool::deactivate();
}

void StretchTool::cancel() {
    if (m_state == State::Dragging) {
        restoreAllEntities();
    }
    resetState();
}

// ---------------------------------------------------------------------------
// Preview and UI
// ---------------------------------------------------------------------------

std::vector<std::pair<math::Vec2, math::Vec2>> StretchTool::getPreviewLines() const {
    std::vector<std::pair<math::Vec2, math::Vec2>> lines;

    if (m_state == State::DraggingWindow) {
        // Draw the crossing window rectangle.
        math::Vec2 a = m_windowStart;
        math::Vec2 b = {m_windowEnd.x, m_windowStart.y};
        math::Vec2 c = m_windowEnd;
        math::Vec2 d = {m_windowStart.x, m_windowEnd.y};
        lines.push_back({a, b});
        lines.push_back({b, c});
        lines.push_back({c, d});
        lines.push_back({d, a});
    }

    return lines;
}

math::Vec3 StretchTool::previewColor() const {
    // Green for crossing-style window.
    if (m_state == State::DraggingWindow) return {0.0, 0.8, 0.0};
    return {0.0, 0.8, 1.0};
}

std::string StretchTool::promptText() const {
    switch (m_state) {
        case State::SelectingWindow:
            return "Specify first corner of crossing window";
        case State::DraggingWindow:
            return "Specify opposite corner of crossing window";
        case State::WaitingBasePoint:
            return "Specify base point";
        case State::Dragging:
            return "Specify destination point";
    }
    return "";
}

bool StretchTool::wantsCrosshair() const { return true; }

}  // namespace hz::ui
