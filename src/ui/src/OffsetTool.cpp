#include "horizon/ui/OffsetTool.h"
#include "horizon/ui/ViewportWidget.h"
#include "horizon/document/Document.h"
#include "horizon/document/Commands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/Intersection.h"
#include "horizon/math/MathUtils.h"

#include <QMouseEvent>
#include <QKeyEvent>
#include <algorithm>
#include <cmath>

namespace hz::ui {

void OffsetTool::deactivate() {
    cancel();
    Tool::deactivate();
}

double OffsetTool::computeDistanceAndSide(int& side) const {
    if (!m_sourceEntity) { side = 1; return 0.0; }

    if (auto* line = dynamic_cast<const draft::DraftLine*>(m_sourceEntity.get())) {
        math::Vec2 dir = (line->end() - line->start()).normalized();
        math::Vec2 normal = dir.perpendicular();  // {-dy, dx}
        math::Vec2 toMouse = m_currentPos - line->start();
        side = (toMouse.dot(normal) >= 0.0) ? 1 : -1;

        // Distance from cursor to the line.
        math::Vec2 ab = line->end() - line->start();
        double lenSq = ab.lengthSquared();
        if (lenSq < 1e-14) { return m_currentPos.distanceTo(line->start()); }
        double t = math::clamp(toMouse.dot(ab) / lenSq, 0.0, 1.0);
        math::Vec2 closest = line->start() + ab * t;
        return m_currentPos.distanceTo(closest);
    }

    if (auto* circle = dynamic_cast<const draft::DraftCircle*>(m_sourceEntity.get())) {
        double dist = m_currentPos.distanceTo(circle->center());
        side = (dist >= circle->radius()) ? 1 : -1;
        return std::abs(dist - circle->radius());
    }

    if (auto* arc = dynamic_cast<const draft::DraftArc*>(m_sourceEntity.get())) {
        double dist = m_currentPos.distanceTo(arc->center());
        side = (dist >= arc->radius()) ? 1 : -1;
        return std::abs(dist - arc->radius());
    }

    if (auto* rect = dynamic_cast<const draft::DraftRectangle*>(m_sourceEntity.get())) {
        math::Vec2 center = (rect->corner1() + rect->corner2()) * 0.5;
        // Simple: use distance from cursor to nearest edge.
        auto segs = draft::extractSegments(*rect);
        double minDist = 1e18;
        for (const auto& [s, e] : segs) {
            math::Vec2 ab = e - s;
            math::Vec2 ap = m_currentPos - s;
            double lenSq = ab.lengthSquared();
            double t = (lenSq < 1e-14) ? 0.0 : math::clamp(ap.dot(ab) / lenSq, 0.0, 1.0);
            double d = m_currentPos.distanceTo(s + ab * t);
            if (d < minDist) minDist = d;
        }
        // Inside or outside: check if cursor is inside the rect bounds.
        double minX = std::min(rect->corner1().x, rect->corner2().x);
        double maxX = std::max(rect->corner1().x, rect->corner2().x);
        double minY = std::min(rect->corner1().y, rect->corner2().y);
        double maxY = std::max(rect->corner1().y, rect->corner2().y);
        bool inside = (m_currentPos.x >= minX && m_currentPos.x <= maxX &&
                       m_currentPos.y >= minY && m_currentPos.y <= maxY);
        side = inside ? -1 : 1;
        return minDist;
    }

    // Polyline: distance to nearest segment.
    if (auto* poly = dynamic_cast<const draft::DraftPolyline*>(m_sourceEntity.get())) {
        auto segs = draft::extractSegments(*poly);
        double minDist = 1e18;
        int bestIdx = 0;
        for (int i = 0; i < static_cast<int>(segs.size()); ++i) {
            const auto& [s, e] = segs[i];
            math::Vec2 ab = e - s;
            math::Vec2 ap = m_currentPos - s;
            double lenSq = ab.lengthSquared();
            double t = (lenSq < 1e-14) ? 0.0 : math::clamp(ap.dot(ab) / lenSq, 0.0, 1.0);
            double d = m_currentPos.distanceTo(s + ab * t);
            if (d < minDist) { minDist = d; bestIdx = i; }
        }
        // Side relative to nearest segment.
        if (!segs.empty()) {
            const auto& [s, e] = segs[bestIdx];
            math::Vec2 dir = (e - s).normalized();
            math::Vec2 normal = dir.perpendicular();
            side = ((m_currentPos - s).dot(normal) >= 0.0) ? 1 : -1;
        } else {
            side = 1;
        }
        return minDist;
    }

    if (auto* ellipse = dynamic_cast<const draft::DraftEllipse*>(m_sourceEntity.get())) {
        auto pts = ellipse->evaluate(64);
        double minDist = 1e18;
        for (const auto& pt : pts) {
            double d = m_currentPos.distanceTo(pt);
            if (d < minDist) minDist = d;
        }
        // Inside/outside: compare cursor distance to center vs average radius.
        double distToCenter = m_currentPos.distanceTo(ellipse->center());
        double avgRadius = (ellipse->semiMajor() + ellipse->semiMinor()) * 0.5;
        side = (distToCenter >= avgRadius) ? 1 : -1;
        return minDist;
    }

    side = 1;
    return 0.0;
}

std::shared_ptr<draft::DraftEntity> OffsetTool::computeOffset() const {
    if (!m_sourceEntity) return nullptr;

    int side = 1;
    double dist = computeDistanceAndSide(side);
    if (dist < 1e-6) return nullptr;

    if (auto* line = dynamic_cast<const draft::DraftLine*>(m_sourceEntity.get())) {
        math::Vec2 dir = (line->end() - line->start()).normalized();
        math::Vec2 normal = dir.perpendicular() * static_cast<double>(side) * dist;
        return std::make_shared<draft::DraftLine>(
            line->start() + normal, line->end() + normal);
    }

    if (auto* circle = dynamic_cast<const draft::DraftCircle*>(m_sourceEntity.get())) {
        double newR = circle->radius() + side * dist;
        if (newR < 0.01) newR = 0.01;
        return std::make_shared<draft::DraftCircle>(circle->center(), newR);
    }

    if (auto* arc = dynamic_cast<const draft::DraftArc*>(m_sourceEntity.get())) {
        double newR = arc->radius() + side * dist;
        if (newR < 0.01) newR = 0.01;
        return std::make_shared<draft::DraftArc>(
            arc->center(), newR, arc->startAngle(), arc->endAngle());
    }

    if (auto* rect = dynamic_cast<const draft::DraftRectangle*>(m_sourceEntity.get())) {
        double d = side * dist;
        double minX = std::min(rect->corner1().x, rect->corner2().x);
        double maxX = std::max(rect->corner1().x, rect->corner2().x);
        double minY = std::min(rect->corner1().y, rect->corner2().y);
        double maxY = std::max(rect->corner1().y, rect->corner2().y);
        math::Vec2 c1(minX - d, minY - d);
        math::Vec2 c2(maxX + d, maxY + d);
        if (c2.x - c1.x < 0.01 || c2.y - c1.y < 0.01) return nullptr;
        return std::make_shared<draft::DraftRectangle>(c1, c2);
    }

    if (auto* poly = dynamic_cast<const draft::DraftPolyline*>(m_sourceEntity.get())) {
        const auto& pts = poly->points();
        if (pts.size() < 2) return nullptr;

        std::vector<math::Vec2> offsetPts;
        offsetPts.reserve(pts.size());

        for (size_t i = 0; i < pts.size(); ++i) {
            // Average the normals of adjacent segments.
            math::Vec2 nPrev, nNext;
            bool hasPrev = false, hasNext = false;

            if (i > 0) {
                math::Vec2 d = (pts[i] - pts[i - 1]).normalized();
                nPrev = d.perpendicular();
                hasPrev = true;
            } else if (poly->closed() && pts.size() >= 2) {
                math::Vec2 d = (pts[0] - pts.back()).normalized();
                nPrev = d.perpendicular();
                hasPrev = true;
            }

            if (i + 1 < pts.size()) {
                math::Vec2 d = (pts[i + 1] - pts[i]).normalized();
                nNext = d.perpendicular();
                hasNext = true;
            } else if (poly->closed() && pts.size() >= 2) {
                math::Vec2 d = (pts[0] - pts.back()).normalized();
                nNext = d.perpendicular();
                hasNext = true;
            }

            math::Vec2 avgNormal;
            if (hasPrev && hasNext) {
                avgNormal = (nPrev + nNext).normalized();
                // Scale to maintain distance.
                double cosHalf = avgNormal.dot(nPrev);
                if (std::abs(cosHalf) > 0.1) {
                    avgNormal = avgNormal * (1.0 / cosHalf);
                }
            } else if (hasPrev) {
                avgNormal = nPrev;
            } else if (hasNext) {
                avgNormal = nNext;
            }

            offsetPts.push_back(pts[i] + avgNormal * (side * dist));
        }

        return std::make_shared<draft::DraftPolyline>(offsetPts, poly->closed());
    }

    if (auto* ellipse = dynamic_cast<const draft::DraftEllipse*>(m_sourceEntity.get())) {
        double newMajor = ellipse->semiMajor() + side * dist;
        double newMinor = ellipse->semiMinor() + side * dist;
        if (newMajor < 0.01) newMajor = 0.01;
        if (newMinor < 0.01) newMinor = 0.01;
        return std::make_shared<draft::DraftEllipse>(
            ellipse->center(), newMajor, newMinor, ellipse->rotation());
    }

    return nullptr;
}

bool OffsetTool::mousePressEvent(QMouseEvent* event, const math::Vec2& worldPos) {
    if (event->button() != Qt::LeftButton) return false;
    if (!m_viewport || !m_viewport->document()) return false;

    auto& doc = m_viewport->document()->draftDocument();
    double pixelScale = m_viewport->pixelToWorldScale();
    double tolerance = std::max(10.0 * pixelScale, 0.15);

    if (m_state == State::SelectEntity) {
        // Hit-test to find entity under cursor (skip hidden/locked layers).
        const auto& layerMgr = m_viewport->document()->layerManager();
        for (const auto& entity : doc.entities()) {
            const auto* lp = layerMgr.getLayer(entity->layer());
            if (!lp || !lp->visible || lp->locked) continue;
            if (entity->hitTest(worldPos, tolerance)) {
                m_sourceEntity = entity;
                m_currentPos = worldPos;
                m_state = State::SpecifyDistance;
                return true;
            }
        }
        return false;
    }

    if (m_state == State::SpecifyDistance) {
        auto offset = computeOffset();
        if (offset) {
            offset->setLayer(m_sourceEntity->layer());
            offset->setColor(m_sourceEntity->color());
            offset->setLineWidth(m_sourceEntity->lineWidth());
            offset->setLineType(m_sourceEntity->lineType());
            auto cmd = std::make_unique<doc::AddEntityCommand>(doc, offset);
            m_viewport->document()->undoStack().push(std::move(cmd));
        }
        m_state = State::SelectEntity;
        m_sourceEntity = nullptr;
        return true;
    }

    return false;
}

bool OffsetTool::mouseMoveEvent(QMouseEvent* /*event*/, const math::Vec2& worldPos) {
    m_currentPos = worldPos;
    return (m_state == State::SpecifyDistance);
}

bool OffsetTool::mouseReleaseEvent(QMouseEvent* /*event*/, const math::Vec2& /*worldPos*/) {
    return false;
}

bool OffsetTool::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        cancel();
        return true;
    }
    return false;
}

void OffsetTool::cancel() {
    m_state = State::SelectEntity;
    m_sourceEntity = nullptr;
}

std::vector<std::pair<math::Vec2, math::Vec2>> OffsetTool::getPreviewLines() const {
    if (m_state != State::SpecifyDistance) return {};
    auto offset = computeOffset();
    if (!offset) return {};
    return draft::extractSegments(*offset);
}

std::vector<std::pair<math::Vec2, double>> OffsetTool::getPreviewCircles() const {
    if (m_state != State::SpecifyDistance) return {};
    auto offset = computeOffset();
    if (!offset) return {};
    if (auto* c = dynamic_cast<const draft::DraftCircle*>(offset.get())) {
        return {{c->center(), c->radius()}};
    }
    return {};
}

std::vector<Tool::ArcPreview> OffsetTool::getPreviewArcs() const {
    if (m_state != State::SpecifyDistance) return {};
    auto offset = computeOffset();
    if (!offset) return {};
    if (auto* a = dynamic_cast<const draft::DraftArc*>(offset.get())) {
        return {{a->center(), a->radius(), a->startAngle(), a->endAngle()}};
    }
    return {};
}

std::string OffsetTool::promptText() const {
    switch (m_state) {
        case State::SelectEntity: return "Select entity to offset";
        case State::SpecifyDistance: return "Specify offset distance and side";
    }
    return "";
}

bool OffsetTool::wantsCrosshair() const { return false; }

}  // namespace hz::ui
