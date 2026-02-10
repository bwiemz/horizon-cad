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

void DraftLine::translate(const math::Vec2& delta) {
    m_start += delta;
    m_end += delta;
}

static math::Vec2 mirrorPoint(const math::Vec2& p,
                               const math::Vec2& axisP1,
                               const math::Vec2& axisP2) {
    math::Vec2 d = (axisP2 - axisP1).normalized();
    math::Vec2 v = p - axisP1;
    return axisP1 + d * (2.0 * v.dot(d)) - v;
}

std::shared_ptr<DraftEntity> DraftLine::clone() const {
    auto copy = std::make_shared<DraftLine>(m_start, m_end);
    copy->setLayer(layer());
    copy->setColor(color());
    return copy;
}

void DraftLine::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    m_start = mirrorPoint(m_start, axisP1, axisP2);
    m_end = mirrorPoint(m_end, axisP1, axisP2);
}

static math::Vec2 rotatePoint(const math::Vec2& p, const math::Vec2& center, double angle) {
    double c = std::cos(angle), s = std::sin(angle);
    math::Vec2 v = p - center;
    return {center.x + v.x * c - v.y * s, center.y + v.x * s + v.y * c};
}

static math::Vec2 scalePoint(const math::Vec2& p, const math::Vec2& center, double factor) {
    return center + (p - center) * factor;
}

void DraftLine::rotate(const math::Vec2& center, double angle) {
    m_start = rotatePoint(m_start, center, angle);
    m_end = rotatePoint(m_end, center, angle);
}

void DraftLine::scale(const math::Vec2& center, double factor) {
    m_start = scalePoint(m_start, center, factor);
    m_end = scalePoint(m_end, center, factor);
}

}  // namespace hz::draft
