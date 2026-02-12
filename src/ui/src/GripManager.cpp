#include "horizon/ui/GripManager.h"

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
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftRadialDimension.h"
#include "horizon/drafting/DraftAngularDimension.h"
#include "horizon/drafting/DraftLeader.h"

#include <cmath>

namespace hz::ui {

// ---------------------------------------------------------------------------
// gripPoints() — extract editable control points per entity type
// ---------------------------------------------------------------------------

std::vector<math::Vec2> GripManager::gripPoints(const draft::DraftEntity& entity) {
    // -- Line: start, end --
    if (auto* e = dynamic_cast<const draft::DraftLine*>(&entity)) {
        return {e->start(), e->end()};
    }

    // -- Circle: center, right, top, left, bottom (quadrants) --
    if (auto* e = dynamic_cast<const draft::DraftCircle*>(&entity)) {
        auto c = e->center();
        double r = e->radius();
        return {c,
                {c.x + r, c.y},
                {c.x, c.y + r},
                {c.x - r, c.y},
                {c.x, c.y - r}};
    }

    // -- Arc: center, start point, end point --
    if (auto* e = dynamic_cast<const draft::DraftArc*>(&entity)) {
        return {e->center(), e->startPoint(), e->endPoint()};
    }

    // -- Rectangle: corner1, corner2 --
    if (auto* e = dynamic_cast<const draft::DraftRectangle*>(&entity)) {
        return {e->corner1(), e->corner2()};
    }

    // -- Polyline: all vertices --
    if (auto* e = dynamic_cast<const draft::DraftPolyline*>(&entity)) {
        return e->points();
    }

    // -- Spline: all control points --
    if (auto* e = dynamic_cast<const draft::DraftSpline*>(&entity)) {
        return e->controlPoints();
    }

    // -- Ellipse: center, major+, major-, minor+, minor- --
    if (auto* e = dynamic_cast<const draft::DraftEllipse*>(&entity)) {
        auto c = e->center();
        double cosR = std::cos(e->rotation());
        double sinR = std::sin(e->rotation());
        double a = e->semiMajor();
        double b = e->semiMinor();
        return {c,
                {c.x + a * cosR, c.y + a * sinR},
                {c.x - a * cosR, c.y - a * sinR},
                {c.x - b * sinR, c.y + b * cosR},
                {c.x + b * sinR, c.y - b * cosR}};
    }

    // -- Text: position --
    if (auto* e = dynamic_cast<const draft::DraftText*>(&entity)) {
        return {e->position()};
    }

    // -- Hatch: boundary vertices --
    if (auto* e = dynamic_cast<const draft::DraftHatch*>(&entity)) {
        return e->boundary();
    }

    // -- Block ref: insertion point --
    if (auto* e = dynamic_cast<const draft::DraftBlockRef*>(&entity)) {
        return {e->insertPos()};
    }

    // -- Linear dimension: defPoint1, defPoint2, dimLinePoint --
    if (auto* e = dynamic_cast<const draft::DraftLinearDimension*>(&entity)) {
        return {e->defPoint1(), e->defPoint2(), e->dimLinePoint()};
    }

    // -- Radial dimension: center, text point --
    if (auto* e = dynamic_cast<const draft::DraftRadialDimension*>(&entity)) {
        return {e->center(), e->textPoint()};
    }

    // -- Angular dimension: vertex, line1Point, line2Point --
    if (auto* e = dynamic_cast<const draft::DraftAngularDimension*>(&entity)) {
        return {e->vertex(), e->line1Point(), e->line2Point()};
    }

    // -- Leader: all polyline points --
    if (auto* e = dynamic_cast<const draft::DraftLeader*>(&entity)) {
        return e->points();
    }

    // Fallback: no grips.
    return {};
}

// ---------------------------------------------------------------------------
// moveGrip() — apply a grip move for a specific index
// ---------------------------------------------------------------------------

bool GripManager::moveGrip(draft::DraftEntity& entity, int gripIndex,
                            const math::Vec2& newPos) {
    // -- Line --
    if (auto* e = dynamic_cast<draft::DraftLine*>(&entity)) {
        if (gripIndex == 0) { e->setStart(newPos); return true; }
        if (gripIndex == 1) { e->setEnd(newPos); return true; }
        return false;
    }

    // -- Circle --
    if (auto* e = dynamic_cast<draft::DraftCircle*>(&entity)) {
        if (gripIndex == 0) { e->setCenter(newPos); return true; }
        // Grips 1-4 are quadrant points — change radius.
        if (gripIndex >= 1 && gripIndex <= 4) {
            double r = newPos.distanceTo(e->center());
            if (r > 1e-6) { e->setRadius(r); return true; }
        }
        return false;
    }

    // -- Arc --
    if (auto* e = dynamic_cast<draft::DraftArc*>(&entity)) {
        if (gripIndex == 0) { e->setCenter(newPos); return true; }
        if (gripIndex == 1) {
            // Move start point: adjust start angle and radius.
            double dx = newPos.x - e->center().x;
            double dy = newPos.y - e->center().y;
            double r = std::sqrt(dx * dx + dy * dy);
            if (r > 1e-6) {
                e->setRadius(r);
                double angle = std::atan2(dy, dx);
                if (angle < 0) angle += 2.0 * 3.14159265358979323846;
                e->setStartAngle(angle);
            }
            return true;
        }
        if (gripIndex == 2) {
            // Move end point: adjust end angle and radius.
            double dx = newPos.x - e->center().x;
            double dy = newPos.y - e->center().y;
            double r = std::sqrt(dx * dx + dy * dy);
            if (r > 1e-6) {
                e->setRadius(r);
                double angle = std::atan2(dy, dx);
                if (angle < 0) angle += 2.0 * 3.14159265358979323846;
                e->setEndAngle(angle);
            }
            return true;
        }
        return false;
    }

    // -- Rectangle --
    if (auto* e = dynamic_cast<draft::DraftRectangle*>(&entity)) {
        if (gripIndex == 0) { e->setCorner1(newPos); return true; }
        if (gripIndex == 1) { e->setCorner2(newPos); return true; }
        return false;
    }

    // -- Polyline --
    if (auto* e = dynamic_cast<draft::DraftPolyline*>(&entity)) {
        auto pts = e->points();
        if (gripIndex >= 0 && gripIndex < static_cast<int>(pts.size())) {
            pts[gripIndex] = newPos;
            e->setPoints(pts);
            return true;
        }
        return false;
    }

    // -- Spline --
    if (auto* e = dynamic_cast<draft::DraftSpline*>(&entity)) {
        auto pts = e->controlPoints();
        if (gripIndex >= 0 && gripIndex < static_cast<int>(pts.size())) {
            pts[gripIndex] = newPos;
            e->setControlPoints(pts);
            return true;
        }
        return false;
    }

    // -- Ellipse --
    if (auto* e = dynamic_cast<draft::DraftEllipse*>(&entity)) {
        if (gripIndex == 0) { e->setCenter(newPos); return true; }
        // Major axis endpoints: grips 1,2.
        if (gripIndex == 1 || gripIndex == 2) {
            double dx = newPos.x - e->center().x;
            double dy = newPos.y - e->center().y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist > 1e-6) {
                e->setSemiMajor(dist);
                double angle = std::atan2(dy, dx);
                if (gripIndex == 2) angle += 3.14159265358979323846;
                e->setRotation(angle);
            }
            return true;
        }
        // Minor axis endpoints: grips 3,4.
        if (gripIndex == 3 || gripIndex == 4) {
            double dx = newPos.x - e->center().x;
            double dy = newPos.y - e->center().y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist > 1e-6) { e->setSemiMinor(dist); }
            return true;
        }
        return false;
    }

    // -- Text --
    if (auto* e = dynamic_cast<draft::DraftText*>(&entity)) {
        if (gripIndex == 0) { e->setPosition(newPos); return true; }
        return false;
    }

    // -- Hatch --
    if (auto* e = dynamic_cast<draft::DraftHatch*>(&entity)) {
        auto bnd = e->boundary();
        if (gripIndex >= 0 && gripIndex < static_cast<int>(bnd.size())) {
            bnd[gripIndex] = newPos;
            e->setBoundary(bnd);
            return true;
        }
        return false;
    }

    // -- Block ref --
    if (auto* e = dynamic_cast<draft::DraftBlockRef*>(&entity)) {
        if (gripIndex == 0) { e->setInsertPos(newPos); return true; }
        return false;
    }

    // -- Linear dimension: we don't support grip editing for dimensions yet
    // (would require non-trivial recalculation). Just allow moving the text point.
    if (auto* e = dynamic_cast<draft::DraftLinearDimension*>(&entity)) {
        // Only allow moving the dim line point (grip 2).
        // defPoint1/defPoint2 are definition points that typically shouldn't be moved.
        (void)e; (void)gripIndex; (void)newPos;
        return false;  // Not supported for now.
    }

    // -- Leader: move polyline points --
    if (auto* e = dynamic_cast<draft::DraftLeader*>(&entity)) {
        // Leader doesn't expose setPoints(), so we can't move individual points.
        (void)e; (void)gripIndex; (void)newPos;
        return false;
    }

    return false;
}

}  // namespace hz::ui
