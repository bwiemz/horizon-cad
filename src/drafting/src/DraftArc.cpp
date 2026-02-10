#include "horizon/drafting/DraftArc.h"
#include "horizon/math/Constants.h"
#include "horizon/math/MathUtils.h"
#include <algorithm>
#include <cmath>

namespace hz::draft {

DraftArc::DraftArc(const math::Vec2& center, double radius,
                   double startAngle, double endAngle)
    : m_center(center)
    , m_radius(radius)
    , m_startAngle(math::normalizeAngle(startAngle))
    , m_endAngle(math::normalizeAngle(endAngle)) {}

math::Vec2 DraftArc::startPoint() const {
    return {m_center.x + m_radius * std::cos(m_startAngle),
            m_center.y + m_radius * std::sin(m_startAngle)};
}

math::Vec2 DraftArc::endPoint() const {
    return {m_center.x + m_radius * std::cos(m_endAngle),
            m_center.y + m_radius * std::sin(m_endAngle)};
}

double DraftArc::sweepAngle() const {
    double sweep = m_endAngle - m_startAngle;
    if (sweep <= 0.0) sweep += math::kTwoPi;
    return sweep;
}

math::Vec2 DraftArc::midPoint() const {
    double midAngle = m_startAngle + sweepAngle() * 0.5;
    return {m_center.x + m_radius * std::cos(midAngle),
            m_center.y + m_radius * std::sin(midAngle)};
}

bool DraftArc::containsAngle(double angle) const {
    angle = math::normalizeAngle(angle);
    if (m_startAngle <= m_endAngle) {
        return angle >= m_startAngle && angle <= m_endAngle;
    }
    // Wraps around zero.
    return angle >= m_startAngle || angle <= m_endAngle;
}

math::BoundingBox DraftArc::boundingBox() const {
    math::Vec2 sp = startPoint();
    math::Vec2 ep = endPoint();

    double minX = std::min(sp.x, ep.x);
    double minY = std::min(sp.y, ep.y);
    double maxX = std::max(sp.x, ep.x);
    double maxY = std::max(sp.y, ep.y);

    // Expand if arc crosses a quadrant boundary.
    if (containsAngle(0.0))              maxX = m_center.x + m_radius;
    if (containsAngle(math::kHalfPi))    maxY = m_center.y + m_radius;
    if (containsAngle(math::kPi))        minX = m_center.x - m_radius;
    if (containsAngle(math::kPi * 1.5))  minY = m_center.y - m_radius;

    return math::BoundingBox(math::Vec3(minX, minY, 0.0),
                             math::Vec3(maxX, maxY, 0.0));
}

bool DraftArc::hitTest(const math::Vec2& point, double tolerance) const {
    double distToCenter = point.distanceTo(m_center);
    double distToEdge = std::abs(distToCenter - m_radius);
    if (distToEdge > tolerance) return false;

    double angle = std::atan2(point.y - m_center.y, point.x - m_center.x);
    return containsAngle(angle);
}

std::vector<math::Vec2> DraftArc::snapPoints() const {
    return {startPoint(), endPoint(), m_center, midPoint()};
}

void DraftArc::translate(const math::Vec2& delta) {
    m_center += delta;
}

static math::Vec2 mirrorPoint(const math::Vec2& p,
                               const math::Vec2& axisP1,
                               const math::Vec2& axisP2) {
    math::Vec2 d = (axisP2 - axisP1).normalized();
    math::Vec2 v = p - axisP1;
    return axisP1 + d * (2.0 * v.dot(d)) - v;
}

std::shared_ptr<DraftEntity> DraftArc::clone() const {
    auto copy = std::make_shared<DraftArc>(m_center, m_radius, m_startAngle, m_endAngle);
    copy->setLayer(layer());
    copy->setColor(color());
    copy->setLineWidth(lineWidth());
    return copy;
}

void DraftArc::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    // Mirror center.
    m_center = mirrorPoint(m_center, axisP1, axisP2);

    // Mirror the start and end points, then recompute angles.
    // Mirroring reverses the winding, so swap start/end.
    math::Vec2 sp = mirrorPoint(startPoint(), axisP1, axisP2);
    math::Vec2 ep = mirrorPoint(endPoint(), axisP1, axisP2);

    // After mirror, the old start becomes new end and vice versa.
    m_startAngle = math::normalizeAngle(std::atan2(ep.y - m_center.y, ep.x - m_center.x));
    m_endAngle = math::normalizeAngle(std::atan2(sp.y - m_center.y, sp.x - m_center.x));
}

static math::Vec2 rotatePoint(const math::Vec2& p, const math::Vec2& center, double angle) {
    double c = std::cos(angle), s = std::sin(angle);
    math::Vec2 v = p - center;
    return {center.x + v.x * c - v.y * s, center.y + v.x * s + v.y * c};
}

static math::Vec2 scalePoint(const math::Vec2& p, const math::Vec2& center, double factor) {
    return center + (p - center) * factor;
}

void DraftArc::rotate(const math::Vec2& center, double angle) {
    m_center = rotatePoint(m_center, center, angle);
    m_startAngle = math::normalizeAngle(m_startAngle + angle);
    m_endAngle = math::normalizeAngle(m_endAngle + angle);
}

void DraftArc::scale(const math::Vec2& center, double factor) {
    m_center = scalePoint(m_center, center, factor);
    m_radius *= std::abs(factor);
}

}  // namespace hz::draft
