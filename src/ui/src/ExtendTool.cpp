#include "horizon/ui/ExtendTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/Intersection.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/Constants.h"
#include "horizon/math/MathUtils.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <algorithm>
#include <cmath>

namespace hz::ui {

// Helper: copy visual properties from source entity to target.
static void copyProps(const draft::DraftEntity* src, draft::DraftEntity* dst) {
    dst->setLayer(src->layer());
    dst->setColor(src->color());
    dst->setLineWidth(src->lineWidth());
    dst->setLineType(src->lineType());
}

// ---------------------------------------------------------------------------
// Extend a line endpoint to the nearest boundary intersection
// ---------------------------------------------------------------------------

static bool extendLine(const draft::DraftLine* line,
                       const math::Vec2& clickPos,
                       const std::vector<std::shared_ptr<draft::DraftEntity>>& entities,
                       const draft::LayerManager& layerMgr,
                       doc::CompositeCommand& composite,
                       draft::DraftDocument& doc) {
    // Determine which endpoint to extend (closest to click).
    double distToStart = (clickPos - line->start()).length();
    double distToEnd = (clickPos - line->end()).length();
    bool extendStart = (distToStart < distToEnd);

    // Compute ray: from the line body, extending beyond the chosen endpoint.
    math::Vec2 rayOrigin, rayDir;
    if (extendStart) {
        // Extend start: ray goes from end() toward start() and beyond.
        rayOrigin = line->start();
        rayDir = (line->start() - line->end()).normalized();
    } else {
        // Extend end: ray goes from start() toward end() and beyond.
        rayOrigin = line->end();
        rayDir = (line->end() - line->start()).normalized();
    }

    if (rayDir.lengthSquared() < 1e-14) return false;

    // Find nearest boundary intersection along the ray.
    math::Vec2 bestPt;
    double bestDist = 1e30;
    for (const auto& other : entities) {
        if (other->id() == line->id()) continue;
        const auto* lp = layerMgr.getLayer(other->layer());
        if (!lp || !lp->visible) continue;

        auto pts = draft::intersectRayEntity(rayOrigin, rayDir, *other);
        for (const auto& pt : pts) {
            double d = (pt - rayOrigin).length();
            if (d > 1e-6 && d < bestDist) {
                bestDist = d;
                bestPt = pt;
            }
        }
    }

    if (bestDist > 1e29) return false;  // No boundary found.

    // Create extended line.
    composite.addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, line->id()));

    math::Vec2 newStart = extendStart ? bestPt : line->start();
    math::Vec2 newEnd = extendStart ? line->end() : bestPt;
    auto newLine = std::make_shared<draft::DraftLine>(newStart, newEnd);
    copyProps(line, newLine.get());
    composite.addCommand(std::make_unique<doc::AddEntityCommand>(doc, newLine));

    return true;
}

// ---------------------------------------------------------------------------
// Extend an arc endpoint to the nearest boundary intersection
// ---------------------------------------------------------------------------

static bool extendArc(const draft::DraftArc* arc,
                      const math::Vec2& clickPos,
                      const std::vector<std::shared_ptr<draft::DraftEntity>>& entities,
                      const draft::LayerManager& layerMgr,
                      doc::CompositeCommand& composite,
                      draft::DraftDocument& doc) {
    // Determine which endpoint to extend.
    double distToStart = (clickPos - arc->startPoint()).length();
    double distToEnd = (clickPos - arc->endPoint()).length();
    bool extendStart = (distToStart < distToEnd);

    // Find all intersections of the full circle with boundary entities.
    // Then pick the nearest one in the extension direction.
    double bestAngleDist = 1e30;
    double bestAngle = 0.0;
    bool found = false;

    double arcStart = arc->startAngle();
    double arcEnd = math::normalizeAngle(arcStart + arc->sweepAngle());

    for (const auto& other : entities) {
        if (other->id() == arc->id()) continue;
        const auto* lp = layerMgr.getLayer(other->layer());
        if (!lp || !lp->visible) continue;

        // Intersect other entity with the full circle of this arc.
        // Use ray from center in all directions — simpler: use extractSegments + intersectLineCircle.
        auto segs = draft::extractSegments(*other);
        for (const auto& [s, e] : segs) {
            auto pts = draft::intersectLineCircle(s, e, arc->center(), arc->radius());
            for (const auto& pt : pts) {
                double angle = math::normalizeAngle(
                    std::atan2(pt.y - arc->center().y, pt.x - arc->center().x));

                // Check if this angle is in the extension direction (outside current arc).
                double offset = math::normalizeAngle(angle - arcStart);
                double arcSweep = arc->sweepAngle();
                if (offset >= -1e-6 && offset <= arcSweep + 1e-6) continue;  // Inside arc, skip.

                // Compute angular distance from the endpoint being extended.
                double angleDist;
                if (extendStart) {
                    // Extending start backward: angle should be just before arcStart (CCW).
                    angleDist = math::normalizeAngle(arcStart - angle);
                } else {
                    // Extending end forward: angle should be just after arcEnd (CCW).
                    angleDist = math::normalizeAngle(angle - arcEnd);
                }

                if (angleDist > 1e-6 && angleDist < bestAngleDist) {
                    bestAngleDist = angleDist;
                    bestAngle = angle;
                    found = true;
                }
            }
        }

        // Also check circle/arc entities.
        if (auto* circ = dynamic_cast<const draft::DraftCircle*>(other.get())) {
            auto pts = draft::intersectCircleCircle(
                arc->center(), arc->radius(), circ->center(), circ->radius());
            for (const auto& pt : pts) {
                double angle = math::normalizeAngle(
                    std::atan2(pt.y - arc->center().y, pt.x - arc->center().x));
                double offset = math::normalizeAngle(angle - arcStart);
                double arcSweep = arc->sweepAngle();
                if (offset >= -1e-6 && offset <= arcSweep + 1e-6) continue;

                double angleDist;
                if (extendStart) {
                    angleDist = math::normalizeAngle(arcStart - angle);
                } else {
                    angleDist = math::normalizeAngle(angle - arcEnd);
                }

                if (angleDist > 1e-6 && angleDist < bestAngleDist) {
                    bestAngleDist = angleDist;
                    bestAngle = angle;
                    found = true;
                }
            }
        } else if (auto* otherArc = dynamic_cast<const draft::DraftArc*>(other.get())) {
            auto pts = draft::intersectCircleCircle(
                arc->center(), arc->radius(), otherArc->center(), otherArc->radius());
            for (const auto& pt : pts) {
                // Check the point is on the other arc.
                double otherAngle = math::normalizeAngle(
                    std::atan2(pt.y - otherArc->center().y, pt.x - otherArc->center().x));
                double otherOffset = math::normalizeAngle(otherAngle - otherArc->startAngle());
                if (otherOffset > otherArc->sweepAngle() + 1e-6) continue;

                double angle = math::normalizeAngle(
                    std::atan2(pt.y - arc->center().y, pt.x - arc->center().x));
                double offset = math::normalizeAngle(angle - arcStart);
                double arcSweep = arc->sweepAngle();
                if (offset >= -1e-6 && offset <= arcSweep + 1e-6) continue;

                double angleDist;
                if (extendStart) {
                    angleDist = math::normalizeAngle(arcStart - angle);
                } else {
                    angleDist = math::normalizeAngle(angle - arcEnd);
                }

                if (angleDist > 1e-6 && angleDist < bestAngleDist) {
                    bestAngleDist = angleDist;
                    bestAngle = angle;
                    found = true;
                }
            }
        }
    }

    if (!found) return false;

    composite.addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, arc->id()));

    double newStart, newEnd;
    if (extendStart) {
        newStart = bestAngle;
        newEnd = math::normalizeAngle(arcStart + arc->sweepAngle());
    } else {
        newStart = arcStart;
        newEnd = bestAngle;
    }

    auto newArc = std::make_shared<draft::DraftArc>(
        arc->center(), arc->radius(), newStart, newEnd);
    copyProps(arc, newArc.get());
    composite.addCommand(std::make_unique<doc::AddEntityCommand>(doc, newArc));

    return true;
}

// ---------------------------------------------------------------------------
// ExtendTool event handlers
// ---------------------------------------------------------------------------

bool ExtendTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->draftDocument();
    double pixelScale = m_viewport->pixelToWorldScale();
    double tolerance = std::max(10.0 * pixelScale, 0.15);

    // Find the entity under the cursor (skip hidden/locked layers).
    const auto& layerMgr = m_viewport->document()->layerManager();
    std::shared_ptr<draft::DraftEntity> target;
    for (const auto& entity : doc.entities()) {
        const auto* lp = layerMgr.getLayer(entity->layer());
        if (!lp || !lp->visible || lp->locked) continue;
        if (entity->hitTest(worldPos, tolerance)) {
            target = entity;
            break;
        }
    }
    if (!target) return false;

    auto composite = std::make_unique<doc::CompositeCommand>("Extend");

    bool success = false;
    if (auto* line = dynamic_cast<const draft::DraftLine*>(target.get())) {
        success = extendLine(line, worldPos, doc.entities(), layerMgr, *composite, doc);
    } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(target.get())) {
        success = extendArc(arc, worldPos, doc.entities(), layerMgr, *composite, doc);
    }

    if (success && !composite->empty()) {
        m_viewport->document()->undoStack().push(std::move(composite));
        m_viewport->selectionManager().deselect(target->id());
    }
    return true;
}

bool ExtendTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool ExtendTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool ExtendTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void ExtendTool::cancel() {
    // Nothing to cancel — extend is a single-click operation.
}

std::string ExtendTool::promptText() const {
    return "Select entity near endpoint to extend";
}

bool ExtendTool::wantsCrosshair() const { return false; }

}  // namespace hz::ui
