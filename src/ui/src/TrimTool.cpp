#include "horizon/ui/TrimTool.h"
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

// ---------------------------------------------------------------------------
// Trim line at intersection points
// ---------------------------------------------------------------------------

static void trimLine(const draft::DraftLine* line,
                     const math::Vec2& clickPos,
                     const std::vector<math::Vec2>& isectPts,
                     doc::CompositeCommand& composite,
                     draft::DraftDocument& doc) {
    math::Vec2 dir = line->end() - line->start();
    double lenSq = dir.lengthSquared();
    if (lenSq < 1e-14) return;

    // Parameterize intersection points along the line.
    std::vector<double> params;
    params.push_back(0.0);
    params.push_back(1.0);
    for (const auto& pt : isectPts) {
        double t = (pt - line->start()).dot(dir) / lenSq;
        if (t > 1e-6 && t < 1.0 - 1e-6) {
            params.push_back(t);
        }
    }
    std::sort(params.begin(), params.end());
    // Remove duplicates.
    params.erase(std::unique(params.begin(), params.end(),
        [](double a, double b) { return std::abs(a - b) < 1e-8; }), params.end());

    if (params.size() < 3) return;  // No intersection found on the line.

    // Find which segment the click falls in.
    double clickT = (clickPos - line->start()).dot(dir) / lenSq;
    clickT = math::clamp(clickT, 0.0, 1.0);

    int clickSegIdx = -1;
    for (size_t i = 0; i + 1 < params.size(); ++i) {
        if (clickT >= params[i] - 1e-6 && clickT <= params[i + 1] + 1e-6) {
            clickSegIdx = static_cast<int>(i);
            break;
        }
    }
    if (clickSegIdx < 0) return;

    // Remove original, add remaining segments.
    composite.addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, line->id()));
    for (size_t i = 0; i + 1 < params.size(); ++i) {
        if (static_cast<int>(i) == clickSegIdx) continue;  // Skip the clicked segment.
        math::Vec2 segStart = line->start() + dir * params[i];
        math::Vec2 segEnd = line->start() + dir * params[i + 1];
        auto newLine = std::make_shared<draft::DraftLine>(segStart, segEnd);
        newLine->setLayer(line->layer());
        newLine->setColor(line->color());
        newLine->setLineWidth(line->lineWidth());
        composite.addCommand(std::make_unique<doc::AddEntityCommand>(doc, newLine));
    }
}

// ---------------------------------------------------------------------------
// Trim circle at intersection points → arcs
// ---------------------------------------------------------------------------

static void trimCircle(const draft::DraftCircle* circle,
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
    // Remove duplicates.
    angles.erase(std::unique(angles.begin(), angles.end(),
        [](double a, double b) { return std::abs(a - b) < 1e-8; }), angles.end());

    if (angles.size() < 2) return;

    // Determine which arc the click falls in.
    double clickAngle = math::normalizeAngle(
        std::atan2(clickPos.y - circle->center().y, clickPos.x - circle->center().x));

    int clickArcIdx = -1;
    size_t n = angles.size();
    for (size_t i = 0; i < n; ++i) {
        double sa = angles[i];
        double ea = angles[(i + 1) % n];
        // Check if clickAngle is in this arc.
        double norm = math::normalizeAngle(clickAngle - sa);
        double span = math::normalizeAngle(ea - sa);
        if (span < 1e-8) span = math::kTwoPi;
        if (norm <= span + 1e-6) {
            clickArcIdx = static_cast<int>(i);
            break;
        }
    }
    if (clickArcIdx < 0) return;

    // Remove circle, add arcs for each segment except the clicked one.
    composite.addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, circle->id()));
    for (size_t i = 0; i < n; ++i) {
        if (static_cast<int>(i) == clickArcIdx) continue;
        double sa = angles[i];
        double ea = angles[(i + 1) % n];
        auto arc = std::make_shared<draft::DraftArc>(circle->center(), circle->radius(), sa, ea);
        arc->setLayer(circle->layer());
        arc->setColor(circle->color());
        arc->setLineWidth(circle->lineWidth());
        composite.addCommand(std::make_unique<doc::AddEntityCommand>(doc, arc));
    }
}

// ---------------------------------------------------------------------------
// Trim arc at intersection points
// ---------------------------------------------------------------------------

static void trimArc(const draft::DraftArc* arc,
                    const math::Vec2& clickPos,
                    const std::vector<math::Vec2>& isectPts,
                    doc::CompositeCommand& composite,
                    draft::DraftDocument& doc) {
    // Convert intersection points to angles within the arc's range.
    double arcStart = arc->startAngle();
    double arcSweep = arc->sweepAngle();
    if (std::abs(arcSweep) < 1e-10) return;  // Degenerate arc.

    std::vector<double> params;  // Parameterized as fraction of sweep [0, 1].
    params.push_back(0.0);
    params.push_back(1.0);
    for (const auto& pt : isectPts) {
        double angle = math::normalizeAngle(
            std::atan2(pt.y - arc->center().y, pt.x - arc->center().x));
        double offset = math::normalizeAngle(angle - arcStart);
        double t = offset / arcSweep;
        if (t > 1e-6 && t < 1.0 - 1e-6) {
            params.push_back(t);
        }
    }
    std::sort(params.begin(), params.end());
    params.erase(std::unique(params.begin(), params.end(),
        [](double a, double b) { return std::abs(a - b) < 1e-8; }), params.end());

    if (params.size() < 3) return;

    // Find click segment.
    double clickAngle = math::normalizeAngle(
        std::atan2(clickPos.y - arc->center().y, clickPos.x - arc->center().x));
    double clickOffset = math::normalizeAngle(clickAngle - arcStart);
    double clickT = clickOffset / arcSweep;
    clickT = math::clamp(clickT, 0.0, 1.0);

    int clickSegIdx = -1;
    for (size_t i = 0; i + 1 < params.size(); ++i) {
        if (clickT >= params[i] - 1e-6 && clickT <= params[i + 1] + 1e-6) {
            clickSegIdx = static_cast<int>(i);
            break;
        }
    }
    if (clickSegIdx < 0) return;

    composite.addCommand(std::make_unique<doc::RemoveEntityCommand>(doc, arc->id()));
    for (size_t i = 0; i + 1 < params.size(); ++i) {
        if (static_cast<int>(i) == clickSegIdx) continue;
        double sa = math::normalizeAngle(arcStart + params[i] * arcSweep);
        double ea = math::normalizeAngle(arcStart + params[i + 1] * arcSweep);
        auto newArc = std::make_shared<draft::DraftArc>(arc->center(), arc->radius(), sa, ea);
        newArc->setLayer(arc->layer());
        newArc->setColor(arc->color());
        newArc->setLineWidth(arc->lineWidth());
        composite.addCommand(std::make_unique<doc::AddEntityCommand>(doc, newArc));
    }
}

// ---------------------------------------------------------------------------
// TrimTool event handlers
// ---------------------------------------------------------------------------

bool TrimTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
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
        auto result = draft::intersect(*target, *other);
        allIsects.insert(allIsects.end(), result.points.begin(), result.points.end());
    }
    if (allIsects.empty()) return false;  // Nothing to trim against.

    auto composite = std::make_unique<doc::CompositeCommand>("Trim");

    if (auto* line = dynamic_cast<const draft::DraftLine*>(target.get())) {
        trimLine(line, worldPos, allIsects, *composite, doc);
    } else if (auto* circle = dynamic_cast<const draft::DraftCircle*>(target.get())) {
        trimCircle(circle, worldPos, allIsects, *composite, doc);
    } else if (auto* arc = dynamic_cast<const draft::DraftArc*>(target.get())) {
        trimArc(arc, worldPos, allIsects, *composite, doc);
    }
    // Rectangles and polylines: skip for now (complex decomposition).

    if (!composite->empty()) {
        m_viewport->document()->undoStack().push(std::move(composite));
        m_viewport->selectionManager().deselect(target->id());
    }
    return true;
}

bool TrimTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool TrimTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool TrimTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void TrimTool::cancel() {
    // Nothing to cancel — trim is a single-click operation.
}

std::string TrimTool::promptText() const {
    return "Select entity to trim";
}

bool TrimTool::wantsCrosshair() const { return false; }

}  // namespace hz::ui
