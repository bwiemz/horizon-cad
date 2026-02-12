#include "horizon/drafting/Intersection.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/math/Constants.h"
#include "horizon/math/MathUtils.h"

#include <algorithm>
#include <cmath>

namespace hz::draft {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static constexpr double kTol = 1e-10;

/// Check whether angle is within [startAngle, endAngle] CCW arc range.
static bool angleInArcRange(double angle, double startAngle, double endAngle) {
    angle = math::normalizeAngle(angle);
    startAngle = math::normalizeAngle(startAngle);
    endAngle = math::normalizeAngle(endAngle);
    if (startAngle <= endAngle) {
        return angle >= startAngle - kTol && angle <= endAngle + kTol;
    }
    // Wraps around zero.
    return angle >= startAngle - kTol || angle <= endAngle + kTol;
}

// ---------------------------------------------------------------------------
// Line-Line
// ---------------------------------------------------------------------------

std::vector<math::Vec2> intersectLineLine(
    const math::Vec2& p1, const math::Vec2& p2,
    const math::Vec2& p3, const math::Vec2& p4) {

    math::Vec2 d1 = p2 - p1;
    math::Vec2 d2 = p4 - p3;

    double denom = d1.cross(d2);
    if (std::abs(denom) < kTol) return {};  // Parallel or coincident.

    math::Vec2 d3 = p3 - p1;
    double t = d3.cross(d2) / denom;
    double s = d3.cross(d1) / denom;

    if (t >= -kTol && t <= 1.0 + kTol && s >= -kTol && s <= 1.0 + kTol) {
        return {p1 + d1 * t};
    }
    return {};
}

// ---------------------------------------------------------------------------
// Line-Circle
// ---------------------------------------------------------------------------

std::vector<math::Vec2> intersectLineCircle(
    const math::Vec2& p1, const math::Vec2& p2,
    const math::Vec2& center, double radius) {

    math::Vec2 d = p2 - p1;
    math::Vec2 f = p1 - center;

    double a = d.dot(d);
    double b = 2.0 * f.dot(d);
    double c = f.dot(f) - radius * radius;

    double discriminant = b * b - 4.0 * a * c;
    if (discriminant < -kTol) return {};

    std::vector<math::Vec2> result;
    if (discriminant < kTol) {
        // Tangent.
        double t = -b / (2.0 * a);
        if (t >= -kTol && t <= 1.0 + kTol) {
            result.push_back(p1 + d * t);
        }
    } else {
        double sqrtDisc = std::sqrt(discriminant);
        double t1 = (-b - sqrtDisc) / (2.0 * a);
        double t2 = (-b + sqrtDisc) / (2.0 * a);
        if (t1 >= -kTol && t1 <= 1.0 + kTol) {
            result.push_back(p1 + d * t1);
        }
        if (t2 >= -kTol && t2 <= 1.0 + kTol) {
            result.push_back(p1 + d * t2);
        }
    }
    return result;
}

// ---------------------------------------------------------------------------
// Circle-Circle
// ---------------------------------------------------------------------------

std::vector<math::Vec2> intersectCircleCircle(
    const math::Vec2& c1, double r1,
    const math::Vec2& c2, double r2) {

    math::Vec2 delta = c2 - c1;
    double d = delta.length();

    if (d < kTol) return {};  // Concentric.
    if (d > r1 + r2 + kTol) return {};  // Too far apart.
    if (d < std::abs(r1 - r2) - kTol) return {};  // One inside the other.

    double a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    double hSq = r1 * r1 - a * a;
    if (hSq < 0.0) hSq = 0.0;
    double h = std::sqrt(hSq);

    math::Vec2 dir = delta / d;
    math::Vec2 perp = dir.perpendicular();
    math::Vec2 mid = c1 + dir * a;

    if (h < kTol) {
        return {mid};  // Single tangent point.
    }
    return {mid + perp * h, mid - perp * h};
}

// ---------------------------------------------------------------------------
// Line-Arc
// ---------------------------------------------------------------------------

std::vector<math::Vec2> intersectLineArc(
    const math::Vec2& p1, const math::Vec2& p2,
    const math::Vec2& center, double radius,
    double startAngle, double endAngle) {

    auto pts = intersectLineCircle(p1, p2, center, radius);
    std::vector<math::Vec2> result;
    for (const auto& pt : pts) {
        double angle = std::atan2(pt.y - center.y, pt.x - center.x);
        if (angleInArcRange(angle, startAngle, endAngle)) {
            result.push_back(pt);
        }
    }
    return result;
}

// ---------------------------------------------------------------------------
// Extract segments
// ---------------------------------------------------------------------------

std::vector<std::pair<math::Vec2, math::Vec2>> extractSegments(const DraftEntity& entity) {
    std::vector<std::pair<math::Vec2, math::Vec2>> segs;

    if (auto* line = dynamic_cast<const DraftLine*>(&entity)) {
        segs.emplace_back(line->start(), line->end());
    } else if (auto* rect = dynamic_cast<const DraftRectangle*>(&entity)) {
        auto c = rect->corners();
        for (int i = 0; i < 4; ++i) {
            segs.emplace_back(c[i], c[(i + 1) % 4]);
        }
    } else if (auto* poly = dynamic_cast<const DraftPolyline*>(&entity)) {
        const auto& pts = poly->points();
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            segs.emplace_back(pts[i], pts[i + 1]);
        }
        if (poly->closed() && pts.size() >= 2) {
            segs.emplace_back(pts.back(), pts.front());
        }
    } else if (auto* spline = dynamic_cast<const DraftSpline*>(&entity)) {
        auto pts = spline->evaluate();
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            segs.emplace_back(pts[i], pts[i + 1]);
        }
    } else if (auto* hatch = dynamic_cast<const DraftHatch*>(&entity)) {
        const auto& boundary = hatch->boundary();
        for (size_t i = 0; i + 1 < boundary.size(); ++i) {
            segs.emplace_back(boundary[i], boundary[i + 1]);
        }
        if (boundary.size() >= 2) {
            segs.emplace_back(boundary.back(), boundary.front());
        }
    } else if (auto* ellipse = dynamic_cast<const DraftEllipse*>(&entity)) {
        auto pts = ellipse->evaluate();
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            segs.emplace_back(pts[i], pts[i + 1]);
        }
    }
    // Text and dimension entities have no geometric segments for intersection.
    // Block reference: extract and transform sub-entity segments.
    if (auto* ref = dynamic_cast<const DraftBlockRef*>(&entity)) {
        for (const auto& subEnt : ref->definition()->entities) {
            auto subSegs = extractSegments(*subEnt);
            for (const auto& [s, e] : subSegs) {
                segs.emplace_back(ref->transformPoint(s), ref->transformPoint(e));
            }
        }
    }
    // Circles and arcs have no line segments.
    return segs;
}

// ---------------------------------------------------------------------------
// Top-level intersect
// ---------------------------------------------------------------------------

/// Intersect segments from entity A with a circle.
static void intersectSegmentsVsCircle(
    const std::vector<std::pair<math::Vec2, math::Vec2>>& segs,
    const math::Vec2& center, double radius,
    std::vector<math::Vec2>& out) {
    for (const auto& [s, e] : segs) {
        auto pts = intersectLineCircle(s, e, center, radius);
        out.insert(out.end(), pts.begin(), pts.end());
    }
}

/// Intersect segments from entity A with an arc.
static void intersectSegmentsVsArc(
    const std::vector<std::pair<math::Vec2, math::Vec2>>& segs,
    const math::Vec2& center, double radius,
    double startAngle, double endAngle,
    std::vector<math::Vec2>& out) {
    for (const auto& [s, e] : segs) {
        auto pts = intersectLineArc(s, e, center, radius, startAngle, endAngle);
        out.insert(out.end(), pts.begin(), pts.end());
    }
}

/// Intersect all segments from A against all segments from B.
static void intersectSegmentsVsSegments(
    const std::vector<std::pair<math::Vec2, math::Vec2>>& segsA,
    const std::vector<std::pair<math::Vec2, math::Vec2>>& segsB,
    std::vector<math::Vec2>& out) {
    for (const auto& [a1, a2] : segsA) {
        for (const auto& [b1, b2] : segsB) {
            auto pts = intersectLineLine(a1, a2, b1, b2);
            out.insert(out.end(), pts.begin(), pts.end());
        }
    }
}

/// Intersect circle/arc A vs circle/arc B, filtering by both angle ranges.
static void intersectCircularVsCircular(
    const math::Vec2& cA, double rA, double saA, double eaA, bool fullCircleA,
    const math::Vec2& cB, double rB, double saB, double eaB, bool fullCircleB,
    std::vector<math::Vec2>& out) {
    auto pts = intersectCircleCircle(cA, rA, cB, rB);
    for (const auto& pt : pts) {
        if (!fullCircleA) {
            double angle = std::atan2(pt.y - cA.y, pt.x - cA.x);
            if (!angleInArcRange(angle, saA, eaA)) continue;
        }
        if (!fullCircleB) {
            double angle = std::atan2(pt.y - cB.y, pt.x - cB.x);
            if (!angleInArcRange(angle, saB, eaB)) continue;
        }
        out.push_back(pt);
    }
}

IntersectionResult intersect(const DraftEntity& a, const DraftEntity& b) {
    IntersectionResult result;

    // Classify each entity.
    auto* circA = dynamic_cast<const DraftCircle*>(&a);
    auto* arcA  = dynamic_cast<const DraftArc*>(&a);
    auto* circB = dynamic_cast<const DraftCircle*>(&b);
    auto* arcB  = dynamic_cast<const DraftArc*>(&b);

    bool aIsCircular = (circA || arcA);
    bool bIsCircular = (circB || arcB);

    auto segsA = extractSegments(a);
    auto segsB = extractSegments(b);

    // Case 1: Both have segments (line, rect, polyline vs line, rect, polyline).
    if (!segsA.empty() && !segsB.empty()) {
        intersectSegmentsVsSegments(segsA, segsB, result.points);
    }

    // Case 2: A has segments, B is circular.
    if (!segsA.empty() && bIsCircular) {
        if (circB) {
            intersectSegmentsVsCircle(segsA, circB->center(), circB->radius(), result.points);
        } else if (arcB) {
            intersectSegmentsVsArc(segsA, arcB->center(), arcB->radius(),
                                   arcB->startAngle(), arcB->endAngle(), result.points);
        }
    }

    // Case 3: B has segments, A is circular.
    if (!segsB.empty() && aIsCircular) {
        if (circA) {
            intersectSegmentsVsCircle(segsB, circA->center(), circA->radius(), result.points);
        } else if (arcA) {
            intersectSegmentsVsArc(segsB, arcA->center(), arcA->radius(),
                                   arcA->startAngle(), arcA->endAngle(), result.points);
        }
    }

    // Case 4: Both circular.
    if (aIsCircular && bIsCircular) {
        math::Vec2 cA = circA ? circA->center() : arcA->center();
        double rA     = circA ? circA->radius()  : arcA->radius();
        double saA = 0, eaA = 0;
        bool fullA = (circA != nullptr);
        if (arcA) { saA = arcA->startAngle(); eaA = arcA->endAngle(); }

        math::Vec2 cB = circB ? circB->center() : arcB->center();
        double rB     = circB ? circB->radius()  : arcB->radius();
        double saB = 0, eaB = 0;
        bool fullB = (circB != nullptr);
        if (arcB) { saB = arcB->startAngle(); eaB = arcB->endAngle(); }

        intersectCircularVsCircular(cA, rA, saA, eaA, fullA,
                                     cB, rB, saB, eaB, fullB,
                                     result.points);
    }

    // Case 5: Mixed segment + circular entities (e.g. rectangle vs circle).
    // Segments from A vs circular B and vice versa are already handled above.

    return result;
}

}  // namespace hz::draft
