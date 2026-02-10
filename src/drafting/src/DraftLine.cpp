#include "horizon/drafting/DraftLine.h"
#include "horizon/math/MathUtils.h"
#include <algorithm>
#include <cmath>

namespace hz::draft {

DraftLine::DraftLine(const math::Vec2& start, const math::Vec2& end)
    : m_start(start), m_end(end) {}

math::BoundingBox DraftLine::boundingBox() const {
    math::Vec3 lo(std::min(m_start.x, m_end.x),
                  std::min(m_start.y, m_end.y),
                  0.0);
    math::Vec3 hi(std::max(m_start.x, m_end.x),
                  std::max(m_start.y, m_end.y),
                  0.0);
    return math::BoundingBox(lo, hi);
}

bool DraftLine::hitTest(const math::Vec2& point, double tolerance) const {
    // Distance from point to line segment [m_start, m_end]
    math::Vec2 ab = m_end - m_start;
    math::Vec2 ap = point - m_start;

    double lenSq = ab.lengthSquared();
    if (lenSq < 1e-14) {
        // Degenerate line (start == end)
        return point.distanceTo(m_start) <= tolerance;
    }

    // Parameter t clamped to [0,1] for segment projection
    double t = math::clamp(ap.dot(ab) / lenSq, 0.0, 1.0);
    math::Vec2 closest = m_start + ab * t;
    return point.distanceTo(closest) <= tolerance;
}

std::vector<math::Vec2> DraftLine::snapPoints() const {
    return { m_start, m_end };
}

}  // namespace hz::draft
