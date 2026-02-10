#include "horizon/drafting/DraftRectangle.h"
#include "horizon/math/MathUtils.h"
#include <algorithm>
#include <cmath>

namespace hz::draft {

DraftRectangle::DraftRectangle(const math::Vec2& corner1, const math::Vec2& corner2)
    : m_corner1(corner1), m_corner2(corner2) {}

std::array<math::Vec2, 4> DraftRectangle::corners() const {
    double minX = std::min(m_corner1.x, m_corner2.x);
    double minY = std::min(m_corner1.y, m_corner2.y);
    double maxX = std::max(m_corner1.x, m_corner2.x);
    double maxY = std::max(m_corner1.y, m_corner2.y);
    return {math::Vec2(minX, minY), math::Vec2(maxX, minY),
            math::Vec2(maxX, maxY), math::Vec2(minX, maxY)};
}

math::BoundingBox DraftRectangle::boundingBox() const {
    double minX = std::min(m_corner1.x, m_corner2.x);
    double minY = std::min(m_corner1.y, m_corner2.y);
    double maxX = std::max(m_corner1.x, m_corner2.x);
    double maxY = std::max(m_corner1.y, m_corner2.y);
    return math::BoundingBox(math::Vec3(minX, minY, 0.0),
                             math::Vec3(maxX, maxY, 0.0));
}

bool DraftRectangle::hitTest(const math::Vec2& point, double tolerance) const {
    auto c = corners();
    for (int i = 0; i < 4; ++i) {
        int j = (i + 1) % 4;
        // Point-to-segment distance.
        math::Vec2 ab = c[j] - c[i];
        math::Vec2 ap = point - c[i];
        double lenSq = ab.lengthSquared();
        if (lenSq < 1e-14) {
            if (point.distanceTo(c[i]) <= tolerance) return true;
            continue;
        }
        double t = math::clamp(ap.dot(ab) / lenSq, 0.0, 1.0);
        math::Vec2 closest = c[i] + ab * t;
        if (point.distanceTo(closest) <= tolerance) return true;
    }
    return false;
}

std::vector<math::Vec2> DraftRectangle::snapPoints() const {
    auto c = corners();
    return {
        c[0], c[1], c[2], c[3],                                     // corners
        (c[0] + c[1]) * 0.5, (c[1] + c[2]) * 0.5,                  // edge midpoints
        (c[2] + c[3]) * 0.5, (c[3] + c[0]) * 0.5,
        (m_corner1 + m_corner2) * 0.5                                 // center
    };
}

void DraftRectangle::translate(const math::Vec2& delta) {
    m_corner1 += delta;
    m_corner2 += delta;
}

static math::Vec2 mirrorPoint(const math::Vec2& p,
                               const math::Vec2& axisP1,
                               const math::Vec2& axisP2) {
    math::Vec2 d = (axisP2 - axisP1).normalized();
    math::Vec2 v = p - axisP1;
    return axisP1 + d * (2.0 * v.dot(d)) - v;
}

std::shared_ptr<DraftEntity> DraftRectangle::clone() const {
    auto copy = std::make_shared<DraftRectangle>(m_corner1, m_corner2);
    copy->setLayer(layer());
    copy->setColor(color());
    return copy;
}

void DraftRectangle::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    m_corner1 = mirrorPoint(m_corner1, axisP1, axisP2);
    m_corner2 = mirrorPoint(m_corner2, axisP1, axisP2);
}

}  // namespace hz::draft
