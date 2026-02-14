#include "horizon/ui/PolylineEditTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/math/MathUtils.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <algorithm>
#include <cmath>

namespace hz::ui {

// ---------------------------------------------------------------------------
// Activation / deactivation
// ---------------------------------------------------------------------------

void PolylineEditTool::activate(ViewportWidget* viewport) {
    Tool::activate(viewport);
    m_editEntityId = 0;
    m_mode = Mode::MoveVertex;
    m_dragging = false;
    m_beforeClone = nullptr;
}

void PolylineEditTool::deactivate() {
    if (m_dragging && m_beforeClone) {
        // Restore if mid-drag.
        finishEditing();
    }
    m_editEntityId = 0;
    m_dragging = false;
    m_beforeClone = nullptr;
    Tool::deactivate();
}

void PolylineEditTool::cancel() {
    if (m_dragging && m_beforeClone && m_viewport && m_viewport->document()) {
        // Restore entity state in-place from before-clone.
        auto& doc = m_viewport->document()->draftDocument();
        for (auto& entity : doc.entities()) {
            if (entity->id() == m_editEntityId) {
                auto* poly = dynamic_cast<draft::DraftPolyline*>(entity.get());
                auto* beforePoly = dynamic_cast<const draft::DraftPolyline*>(m_beforeClone.get());
                if (poly && beforePoly) {
                    poly->setPoints(beforePoly->points());
                    poly->setClosed(beforePoly->closed());
                }
                break;
            }
        }
        m_dragging = false;
        m_beforeClone = nullptr;
    }
    if (m_editEntityId != 0) {
        m_editEntityId = 0;
        m_mode = Mode::MoveVertex;
    }
    if (m_viewport) m_viewport->update();
}

void PolylineEditTool::finishEditing() {
    m_editEntityId = 0;
    m_mode = Mode::MoveVertex;
    m_dragging = false;
    m_beforeClone = nullptr;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void PolylineEditTool::pushSnapshot(const std::string& desc) {
    if (!m_viewport || !m_viewport->document() || !m_beforeClone) return;
    auto& doc = m_viewport->document()->draftDocument();

    // Find current state of the entity.
    for (const auto& entity : doc.entities()) {
        if (entity->id() == m_editEntityId) {
            auto afterClone = entity->clone();
            auto cmd = std::make_unique<doc::GripMoveCommand>(
                doc, m_editEntityId, m_beforeClone, afterClone);
            m_viewport->document()->undoStack().push(std::move(cmd));
            m_beforeClone = nullptr;
            return;
        }
    }
}

int PolylineEditTool::findNearestVertex(const math::Vec2& worldPos, double tolerance) const {
    if (!m_viewport || !m_viewport->document()) return -1;
    auto& doc = m_viewport->document()->draftDocument();

    for (const auto& entity : doc.entities()) {
        if (entity->id() != m_editEntityId) continue;
        auto* poly = dynamic_cast<const draft::DraftPolyline*>(entity.get());
        if (!poly) return -1;

        const auto& pts = poly->points();
        double bestDist = tolerance;
        int bestIdx = -1;
        for (int i = 0; i < static_cast<int>(pts.size()); ++i) {
            double d = worldPos.distanceTo(pts[i]);
            if (d < bestDist) {
                bestDist = d;
                bestIdx = i;
            }
        }
        return bestIdx;
    }
    return -1;
}

int PolylineEditTool::findNearestSegment(const math::Vec2& worldPos, math::Vec2& closestPt) const {
    if (!m_viewport || !m_viewport->document()) return -1;
    auto& doc = m_viewport->document()->draftDocument();

    for (const auto& entity : doc.entities()) {
        if (entity->id() != m_editEntityId) continue;
        auto* poly = dynamic_cast<const draft::DraftPolyline*>(entity.get());
        if (!poly) return -1;

        const auto& pts = poly->points();
        int segCount = static_cast<int>(pts.size()) - 1;
        if (poly->closed() && pts.size() >= 2) segCount += 1;

        double bestDist = 1e18;
        int bestIdx = -1;

        for (int i = 0; i < segCount; ++i) {
            const math::Vec2& a = pts[i];
            const math::Vec2& b = pts[(i + 1) % pts.size()];
            math::Vec2 ab = b - a;
            math::Vec2 ap = worldPos - a;
            double lenSq = ab.lengthSquared();
            double t = (lenSq < 1e-14) ? 0.0 : math::clamp(ap.dot(ab) / lenSq, 0.0, 1.0);
            math::Vec2 proj = a + ab * t;
            double d = worldPos.distanceTo(proj);
            if (d < bestDist) {
                bestDist = d;
                bestIdx = i;
                closestPt = proj;
            }
        }
        return bestIdx;
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Event handlers
// ---------------------------------------------------------------------------

bool PolylineEditTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->draftDocument();
    double pixelScale = m_viewport->pixelToWorldScale();
    double tolerance = std::max(10.0 * pixelScale, 0.15);
    double gripTol = std::max(8.0 * pixelScale, 0.12);

    // If no polyline selected yet, pick one.
    if (m_editEntityId == 0) {
        const auto& layerMgr = m_viewport->document()->layerManager();
        for (const auto& entity : doc.entities()) {
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;
            if (!dynamic_cast<const draft::DraftPolyline*>(entity.get())) continue;
            if (entity->hitTest(worldPos, tolerance)) {
                m_editEntityId = entity->id();
                m_mode = Mode::MoveVertex;
                return true;
            }
        }
        return false;
    }

    // Already editing a polyline — handle sub-modes.
    if (m_mode == Mode::MoveVertex) {
        int idx = findNearestVertex(worldPos, gripTol);
        if (idx >= 0) {
            // Start dragging this vertex.
            for (const auto& entity : doc.entities()) {
                if (entity->id() == m_editEntityId) {
                    m_beforeClone = entity->clone();
                    break;
                }
            }
            m_dragging = true;
            m_dragVertexIndex = idx;
            m_currentPos = worldPos;
            return true;
        }
        // Click elsewhere while in move mode: check if clicking another polyline.
        const auto& layerMgr = m_viewport->document()->layerManager();
        for (const auto& entity : doc.entities()) {
            if (entity->id() == m_editEntityId) continue;
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;
            if (!dynamic_cast<const draft::DraftPolyline*>(entity.get())) continue;
            if (entity->hitTest(worldPos, tolerance)) {
                // Switch to editing this polyline.
                m_editEntityId = entity->id();
                m_mode = Mode::MoveVertex;
                return true;
            }
        }
        // Click on empty space: deselect.
        finishEditing();
        return true;
    }

    if (m_mode == Mode::AddVertex) {
        math::Vec2 closestPt;
        int segIdx = findNearestSegment(worldPos, closestPt);
        if (segIdx >= 0) {
            for (auto& entity : doc.entities()) {
                if (entity->id() != m_editEntityId) continue;
                auto* poly = dynamic_cast<draft::DraftPolyline*>(entity.get());
                if (!poly) break;

                m_beforeClone = entity->clone();
                auto pts = poly->points();
                pts.insert(pts.begin() + segIdx + 1, closestPt);
                poly->setPoints(pts);
                pushSnapshot("Add vertex");
                break;
            }
        }
        m_mode = Mode::MoveVertex;
        return true;
    }

    if (m_mode == Mode::RemoveVertex) {
        int idx = findNearestVertex(worldPos, gripTol);
        if (idx >= 0) {
            for (auto& entity : doc.entities()) {
                if (entity->id() != m_editEntityId) continue;
                auto* poly = dynamic_cast<draft::DraftPolyline*>(entity.get());
                if (!poly) break;

                const auto& pts = poly->points();
                if (pts.size() <= 2) break;  // Min 2 points.

                m_beforeClone = entity->clone();
                auto newPts = pts;
                newPts.erase(newPts.begin() + idx);
                poly->setPoints(newPts);
                pushSnapshot("Remove vertex");
                break;
            }
        }
        m_mode = Mode::MoveVertex;
        return true;
    }

    if (m_mode == Mode::JoinPolyline) {
        const auto& layerMgr = m_viewport->document()->layerManager();
        for (const auto& entity : doc.entities()) {
            if (entity->id() == m_editEntityId) continue;
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;
            auto* otherPoly = dynamic_cast<const draft::DraftPolyline*>(entity.get());
            if (!otherPoly) continue;
            if (!entity->hitTest(worldPos, tolerance)) continue;

            // Find current polyline.
            draft::DraftPolyline* myPoly = nullptr;
            for (auto& e : doc.entities()) {
                if (e->id() == m_editEntityId)
                    myPoly = dynamic_cast<draft::DraftPolyline*>(e.get());
            }
            if (!myPoly) break;

            m_beforeClone = myPoly->clone();
            auto myPts = myPoly->points();
            auto otherPts = otherPoly->points();

            // Determine best connection: compare distances between all 4 endpoint pairs.
            double d00 = myPts.front().distanceTo(otherPts.front());
            double d01 = myPts.front().distanceTo(otherPts.back());
            double d10 = myPts.back().distanceTo(otherPts.front());
            double d11 = myPts.back().distanceTo(otherPts.back());
            double minD = std::min({d00, d01, d10, d11});

            std::vector<math::Vec2> joined;
            if (minD == d10) {
                // my.back → other.front: append other after my
                joined = myPts;
                joined.insert(joined.end(), otherPts.begin(), otherPts.end());
            } else if (minD == d11) {
                // my.back → other.back: append reversed other
                joined = myPts;
                joined.insert(joined.end(), otherPts.rbegin(), otherPts.rend());
            } else if (minD == d00) {
                // my.front → other.front: prepend reversed other
                joined.assign(otherPts.rbegin(), otherPts.rend());
                joined.insert(joined.end(), myPts.begin(), myPts.end());
            } else {
                // my.front → other.back: prepend other
                joined = otherPts;
                joined.insert(joined.end(), myPts.begin(), myPts.end());
            }

            // Apply join as composite: modify current + remove other.
            myPoly->setPoints(joined);
            myPoly->setClosed(false);
            auto afterClone = myPoly->clone();

            auto composite = std::make_unique<doc::CompositeCommand>("Join polylines");
            composite->addCommand(std::make_unique<doc::GripMoveCommand>(
                doc, m_editEntityId, m_beforeClone, afterClone));
            composite->addCommand(std::make_unique<doc::RemoveEntityCommand>(
                doc, entity->id()));
            m_viewport->document()->undoStack().push(std::move(composite));
            m_beforeClone = nullptr;

            m_mode = Mode::MoveVertex;
            return true;
        }
        m_mode = Mode::MoveVertex;
        return true;
    }

    return false;
}

bool PolylineEditTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    m_currentPos = worldPos;

    if (m_dragging && m_editEntityId != 0 && m_viewport && m_viewport->document()) {
        auto& doc = m_viewport->document()->draftDocument();
        for (auto& entity : doc.entities()) {
            if (entity->id() != m_editEntityId) continue;
            auto* poly = dynamic_cast<draft::DraftPolyline*>(entity.get());
            if (!poly) break;

            // Snap.
            auto result = m_viewport->snapEngine().snap(worldPos, doc.entities());
            math::Vec2 snappedPos = result.point;

            auto pts = poly->points();
            if (m_dragVertexIndex >= 0 && m_dragVertexIndex < static_cast<int>(pts.size())) {
                pts[m_dragVertexIndex] = snappedPos;
                poly->setPoints(pts);
            }
            break;
        }
        return true;
    }
    return false;
}

bool PolylineEditTool::mouseReleaseEvent(QMouseEvent* event, const math::Vec2& /*worldPos*/) {
    if (event->button() != Qt::LeftButton) return false;

    if (m_dragging) {
        pushSnapshot("Move vertex");
        m_dragging = false;
        m_dragVertexIndex = -1;
        return true;
    }
    return false;
}

bool PolylineEditTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
        finishEditing();
        return true;
    }

    // Sub-mode keys (only when a polyline is selected).
    if (m_editEntityId != 0) {
        if (event->key() == Qt::Key_A) {
            m_mode = Mode::AddVertex;
            return true;
        }
        if (event->key() == Qt::Key_D) {
            m_mode = Mode::RemoveVertex;
            return true;
        }
        if (event->key() == Qt::Key_J) {
            m_mode = Mode::JoinPolyline;
            return true;
        }
        if (event->key() == Qt::Key_C) {
            // Toggle closed/open.
            if (!m_viewport || !m_viewport->document()) return true;
            auto& doc = m_viewport->document()->draftDocument();
            for (auto& entity : doc.entities()) {
                if (entity->id() != m_editEntityId) continue;
                auto* poly = dynamic_cast<draft::DraftPolyline*>(entity.get());
                if (!poly) break;

                m_beforeClone = entity->clone();
                poly->setClosed(!poly->closed());
                pushSnapshot("Toggle closed");
                break;
            }
            return true;
        }
    }

    return false;
}

// ---------------------------------------------------------------------------
// Preview
// ---------------------------------------------------------------------------

std::vector<std::pair<math::Vec2, math::Vec2>> PolylineEditTool::getPreviewLines() const {
    // No line preview needed — the entity itself is rendered normally.
    return {};
}

std::vector<std::pair<math::Vec2, double>> PolylineEditTool::getPreviewCircles() const {
    // Show vertex dots as small circles when editing.
    if (m_editEntityId == 0) return {};
    if (!m_viewport || !m_viewport->document()) return {};

    auto& doc = m_viewport->document()->draftDocument();
    double pixelScale = m_viewport->pixelToWorldScale();
    double dotRadius = 4.0 * pixelScale;

    for (const auto& entity : doc.entities()) {
        if (entity->id() != m_editEntityId) continue;
        auto* poly = dynamic_cast<const draft::DraftPolyline*>(entity.get());
        if (!poly) break;

        std::vector<std::pair<math::Vec2, double>> circles;
        for (const auto& pt : poly->points()) {
            circles.push_back({pt, dotRadius});
        }
        return circles;
    }
    return {};
}

std::string PolylineEditTool::promptText() const {
    if (m_editEntityId == 0) return "Select polyline to edit";

    switch (m_mode) {
        case Mode::MoveVertex:
            return "Drag vertex | A=Add  D=Remove  C=Close/Open  J=Join  Esc=Done";
        case Mode::AddVertex:
            return "Click on segment to add vertex | Esc=Done";
        case Mode::RemoveVertex:
            return "Click on vertex to remove | Esc=Done";
        case Mode::JoinPolyline:
            return "Click another polyline to join | Esc=Done";
    }
    return "";
}

bool PolylineEditTool::wantsCrosshair() const { return false; }

}  // namespace hz::ui
