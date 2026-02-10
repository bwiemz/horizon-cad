#pragma once

#include "horizon/math/Vec2.h"
#include <memory>
#include <utility>
#include <vector>

namespace hz::draft {

class DraftEntity;

/// Result of an intersection computation between two entities.
struct IntersectionResult {
    std::vector<math::Vec2> points;
};

/// Compute intersections between two entities.
/// Supports all entity type combinations (line, circle, arc, rectangle, polyline).
IntersectionResult intersect(const DraftEntity& a, const DraftEntity& b);

// Low-level intersection primitives:

/// Line segment (p1->p2) vs line segment (p3->p4). Returns 0 or 1 points.
std::vector<math::Vec2> intersectLineLine(
    const math::Vec2& p1, const math::Vec2& p2,
    const math::Vec2& p3, const math::Vec2& p4);

/// Line segment (p1->p2) vs circle (center, radius). Returns 0-2 points on the segment.
std::vector<math::Vec2> intersectLineCircle(
    const math::Vec2& p1, const math::Vec2& p2,
    const math::Vec2& center, double radius);

/// Circle (c1,r1) vs circle (c2,r2). Returns 0-2 points.
std::vector<math::Vec2> intersectCircleCircle(
    const math::Vec2& c1, double r1,
    const math::Vec2& c2, double r2);

/// Line segment vs arc: intersectLineCircle filtered by arc angle range.
std::vector<math::Vec2> intersectLineArc(
    const math::Vec2& p1, const math::Vec2& p2,
    const math::Vec2& center, double radius,
    double startAngle, double endAngle);

/// Extract line segments from a line-based entity (line, rectangle, polyline).
/// Returns empty for circles/arcs.
std::vector<std::pair<math::Vec2, math::Vec2>> extractSegments(const DraftEntity& entity);

}  // namespace hz::draft
