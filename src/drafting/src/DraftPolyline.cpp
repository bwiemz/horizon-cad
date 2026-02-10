#include "horizon/drafting/DraftPolyline.h"
#include "horizon/math/MathUtils.h"
#include <algorithm>
#include <cmath>

namespace hz::draft {

DraftPolyline::DraftPolyline(const std::vector<math::Vec2>& points, bool closed)
    : m_points(points), m_closed(closed) {}

void DraftPolyline::addPoint(const math::Vec2& point) {
    m_points.push_back(point);
}

math::BoundingBox DraftPolyline::boundingBox() const {
    if (m_points.empty()) return {};

    double minX = m_points[0].x, maxX = m_points[0].x;
    double minY = m_points[0].y, maxY = m_points[0].y;
    for (size_t i = 1; i < m_points.size(); ++i) {
        minX = std::min(minX, m_points[i].x);
        minY = std::min(minY, m_points[i].y);
        maxX = std::max(maxX, m_points[i].x);
        maxY = std::max(maxY, m_points[i].y);
    }
    return math::BoundingBox(math::Vec3(minX, minY, 0.0),
                             math::Vec3(maxX, maxY, 0.0));
}

bool DraftPolyline::hitTest(const math::Vec2& point, double tolerance) const {
    if (m_points.size() < 2) return false;

    auto segmentDist = [](const math::Vec2& p, const math::Vec2& a, const math::Vec2& b) -> double {
        math::Vec2 ab = b - a;
        math::Vec2 ap = p - a;
        double lenSq = ab.lengthSquared();
        if (lenSq < 1e-14) return p.distanceTo(a);
        double t = math::clamp(ap.dot(ab) / lenSq, 0.0, 1.0);
        math::Vec2 closest = a + ab * t;
        return p.distanceTo(closest);
    };

    for (size_t i = 0; i + 1 < m_points.size(); ++i) {
        if (segmentDist(point, m_points[i], m_points[i + 1]) <= tolerance)
            return true;
    }
    if (m_closed && m_points.size() >= 2) {
        if (segmentDist(point, m_points.back(), m_points[0]) <= tolerance)
            return true;
    }
    return false;
}

std::vector<math::Vec2> DraftPolyline::snapPoints() const {
    std::vector<math::Vec2> result;
    result.reserve(m_points.size() * 2);

    // Vertices.
    for (const auto& pt : m_points) {
        result.push_back(pt);
    }
    // Segment midpoints.
    for (size_t i = 0; i + 1 < m_points.size(); ++i) {
        result.push_back((m_points[i] + m_points[i + 1]) * 0.5);
    }
    if (m_closed && m_points.size() >= 2) {
        result.push_back((m_points.back() + m_points[0]) * 0.5);
    }
    return result;
}

void DraftPolyline::translate(const math::Vec2& delta) {
    for (auto& pt : m_points) {
        pt += delta;
    }
}

static math::Vec2 mirrorPoint(const math::Vec2& p,
                               const math::Vec2& axisP1,
                               const math::Vec2& axisP2) {
    math::Vec2 d = (axisP2 - axisP1).normalized();
    math::Vec2 v = p - axisP1;
    return axisP1 + d * (2.0 * v.dot(d)) - v;
}

std::shared_ptr<DraftEntity> DraftPolyline::clone() const {
    auto copy = std::make_shared<DraftPolyline>(m_points, m_closed);
    copy->setLayer(layer());
    copy->setColor(color());
    copy->setLineWidth(lineWidth());
    return copy;
}

void DraftPolyline::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    for (auto& pt : m_points) {
        pt = mirrorPoint(pt, axisP1, axisP2);
    }
}

static math::Vec2 rotatePoint(const math::Vec2& p, const math::Vec2& center, double angle) {
    double c = std::cos(angle), s = std::sin(angle);
    math::Vec2 v = p - center;
    return {center.x + v.x * c - v.y * s, center.y + v.x * s + v.y * c};
}

static math::Vec2 scalePoint(const math::Vec2& p, const math::Vec2& center, double factor) {
    return center + (p - center) * factor;
}

void DraftPolyline::rotate(const math::Vec2& center, double angle) {
    for (auto& pt : m_points) {
        pt = rotatePoint(pt, center, angle);
    }
}

void DraftPolyline::scale(const math::Vec2& center, double factor) {
    for (auto& pt : m_points) {
        pt = scalePoint(pt, center, factor);
    }
}

}  // namespace hz::draft
