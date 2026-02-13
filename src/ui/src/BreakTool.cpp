#include "horizon/ui/BreakTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/Intersection.h"
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
// Break line at a single point
// ---------------------------------------------------------------------------

static void breakLine(const draft::DraftLine* line,
                      const math::Vec2& breakPt,
                      doc::CompositeCommand& composite,
                      draft::DraftDocument& doc) {
    math::Vec2 dir = line->end() - line->start();
    double lenSq = dir.lengthSquared();
    if (lenSq < 1e-14) return;

    // Parameterize break point along the line.
    double t = (breakPt - line->start()).dot(dir) / lenSq;
    t = math::clamp(t, 0.0, 1.0);
    if (t < 1e-6 || t > 1.0 - 1e-6) return;  // Break point at endpoint — nothing to split.

    math::Vec2 splitPt = line->start() + dir * t;

    composite.addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, line->id()));

    auto line1 = std::make_shared<draft::DraftLine>(line->start(), splitPt);
    copyProps(line, line1.get());
    composite.addCommand(std::make_unique<doc::AddEntityCommand>(doc, line1));

    auto line2 = std::make_shared<draft::DraftLine>(splitPt, line->end());
    copyProps(line, line2.get());
    composite.addCommand(std::make_unique<doc::AddEntityCommand>(doc, line2));
}

// ---------------------------------------------------------------------------
// Break arc at a single point
// ---------------------------------------------------------------------------

static void breakArc(const draft::DraftArc* arc,
                     const math::Vec2& breakPt,
                     doc::CompositeCommand& composite,
                     draft::DraftDocument& doc) {
    double breakAngle = math::normalizeAngle(
        std::atan2(breakPt.y - arc->center().y, breakPt.x - arc->center().x));

    double arcStart = arc->startAngle();
    double arcEnd = arc->endAngle();
    double arcSweep = arc->sweepAngle();
    if (std::abs(arcSweep) < 1e-10) return;

    // Parameterize the break angle within the arc's CCW sweep.
    double offset = math::normalizeAngle(breakAngle - arcStart);
    if (offset > arcSweep + 1e-6) return;  // Break point not on arc.
    double t = offset / arcSweep;
    if (t < 1e-6 || t > 1.0 - 1e-6) return;  // At endpoint — nothing to split.

    composite.addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, arc->id()));

    auto arc1 = std::make_shared<draft::DraftArc>(
        arc->center(), arc->radius(), arcStart, breakAngle);
    copyProps(arc, arc1.get());
    composite.addCommand(std::make_unique<doc::AddEntityCommand>(doc, arc1));

    auto arc2 = std::make_shared<draft::DraftArc>(
        arc->center(), arc->radius(), breakAngle, arcEnd);
    copyProps(arc, arc2.get());
    composite.addCommand(std::make_unique<doc::AddEntityCommand>(doc, arc2));
}

// ---------------------------------------------------------------------------
// Break circle at intersection points → two arcs
// ---------------------------------------------------------------------------

static void breakCircle(const draft::DraftCircle* circle,
                        const math::Vec2& clickPos,
                        const std::vector<math::Vec2>& isectPts,
                        doc::CompositeCommand& composite,
                        draft::DraftDocument& doc) {
    if (isectPts.size() < 2) return;  // Need at least 2 points to split a circle.

    // Convert intersection points to angles, sort.
    std::vector<double> angles;
    for (const auto& pt : isectPts) {
        double a = math::normalizeAngle(
            std::atan2(pt.y - circle->center().y, pt.x - circle->center().x));
        angles.push_back(a);
    }
    std::sort(angles.begin(), angles.end());
    angles.erase(std::unique(angles.begin(), angles.end(),
        [](double a, double b) { return std::abs(a - b) < 1e-8; }), angles.end());

    if (angles.size() < 2) return;

    // Find the two nearest intersections to the click position.
    double clickAngle = math::normalizeAngle(
        std::atan2(clickPos.y - circle->center().y, clickPos.x - circle->center().x));

    // Sort angles by angular distance from click.
    std::vector<size_t> idx(angles.size());
    for (size_t i = 0; i < idx.size(); ++i) idx[i] = i;
    std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b) {
        double da = std::min(math::normalizeAngle(angles[a] - clickAngle),
                             math::normalizeAngle(clickAngle - angles[a]));
        double db = std::min(math::normalizeAngle(angles[b] - clickAngle),
                             math::normalizeAngle(clickAngle - angles[b]));
        return da < db;
    });

    double a1 = angles[idx[0]];
    double a2 = angles[idx[1]];
    if (a1 > a2) std::swap(a1, a2);

    // Create two arcs covering the full circle.
    composite.addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, circle->id()));

    auto arc1 = std::make_shared<draft::DraftArc>(circle->center(), circle->radius(), a1, a2);
    copyProps(circle, arc1.get());
    composite.addCommand(std::make_unique<doc::AddEntityCommand>(doc, arc1));

    auto arc2 = std::make_shared<draft::DraftArc>(circle->center(), circle->radius(), a2, a1);
    copyProps(circle, arc2.get());
    composite.addCommand(std::make_unique<doc::AddEntityCommand>(doc, arc2));
}

// ---------------------------------------------------------------------------
// Find nearest point on entity to click (for break without intersections)
// ---------------------------------------------------------------------------

static math::Vec2 nearestPointOnLine(const draft::DraftLine* line,
                                     const math::Vec2& pt) {
    math::Vec2 dir = line->end() - line->start();
    double lenSq = dir.lengthSquared();
    if (lenSq < 1e-14) return line->start();
    double t = math::clamp((pt - line->start()).dot(dir) / lenSq, 0.0, 1.0);
    return line->start() + dir * t;
}

static math::Vec2 nearestPointOnArc(const draft::DraftArc* arc,
                                    const math::Vec2& pt) {
    double angle = std::atan2(pt.y - arc->center().y, pt.x - arc->center().x);
    // Clamp angle to arc range.
    double normAngle = math::normalizeAngle(angle);
    double arcStart = arc->startAngle();
    double arcSweep = arc->sweepAngle();
    double offset = math::normalizeAngle(normAngle - arcStart);
    if (offset > arcSweep) {
        // Outside arc — snap to nearest endpoint.
        double arcEnd = arc->endAngle();
        double distToStart = math::normalizeAngle(arcStart - normAngle);
        double distToEnd = math::normalizeAngle(normAngle - arcEnd);
        angle = (distToStart < distToEnd) ? arcStart : arcEnd;
    }
    return arc->center() + math::Vec2(std::cos(angle), std::sin(angle)) * arc->radius();
}

// ---------------------------------------------------------------------------
// BreakTool event handlers
// ---------------------------------------------------------------------------

bool BreakTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
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

    // Find all intersection points on the target with other entities.
    std::vector<math::Vec2> allIsects;
    for (const auto& other : doc.entities()) {
        if (other->id() == target->id()) continue;
        const auto* lp = layerMgr.getLayer(other->layer());
        if (!lp || !lp->visible) continue;
        auto result = draft::intersect(*target, *other);
        allIsects.insert(allIsects.end(), result.points.begin(), result.points.end());
    }

    auto composite = std::make_unique<doc::CompositeCommand>("Break");

    if (auto* circle = dynamic_cast<const draft::DraftCircle*>(target.get())) {
        // Circle requires intersection points (need at least 2 to split).
        breakCircle(circle, worldPos, allIsects, *composite, doc);
    } else {
        // For line/arc: find the nearest intersection or nearest point on entity.
        math::Vec2 breakPt;
        if (!allIsects.empty()) {
            // Find nearest intersection to click.
            double bestDist = 1e30;
            for (const auto& pt : allIsects) {
                double d = (pt - worldPos).length();
                if (d < bestDist) {
                    bestDist = d;
                    breakPt = pt;
                }
            }
        } else {
            // No intersections — use nearest point on entity.
            if (auto* line = dynamic_cast<const draft::DraftLine*>(target.get())) {
                breakPt = nearestPointOnLine(line, worldPos);
            } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(target.get())) {
                breakPt = nearestPointOnArc(arc, worldPos);
            } else {
                return false;  // Unsupported entity type.
            }
        }

        if (auto* line = dynamic_cast<const draft::DraftLine*>(target.get())) {
            breakLine(line, breakPt, *composite, doc);
        } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(target.get())) {
            breakArc(arc, breakPt, *composite, doc);
        }
    }

    if (!composite->empty()) {
        m_viewport->document()->undoStack().push(std::move(composite));
        m_viewport->selectionManager().deselect(target->id());
    }
    return true;
}

bool BreakTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool BreakTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool BreakTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void BreakTool::cancel() {
    // Nothing to cancel — break is a single-click operation.
}

std::string BreakTool::promptText() const {
    return "Select entity to break";
}

bool BreakTool::wantsCrosshair() const { return false; }

}  // namespace hz::ui
