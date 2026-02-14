#include "horizon/drafting/DraftRadialDimension.h"
#include "horizon/math/Constants.h"
#include "horizon/math/MathUtils.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace hz::draft {

// ---- Helpers (file-local) ----

static math::Vec2 mirrorPoint(const math::Vec2& p,
                               const math::Vec2& axisP1,
                               const math::Vec2& axisP2) {
    math::Vec2 d = (axisP2 - axisP1).normalized();
    math::Vec2 v = p - axisP1;
    return axisP1 + d * (2.0 * v.dot(d)) - v;
}

static math::Vec2 rotatePoint(const math::Vec2& p,
                               const math::Vec2& center, double angle) {
    double c = std::cos(angle), s = std::sin(angle);
    math::Vec2 v = p - center;
    return {center.x + v.x * c - v.y * s, center.y + v.x * s + v.y * c};
}

static math::Vec2 scalePoint(const math::Vec2& p,
                              const math::Vec2& center, double factor) {
    return center + (p - center) * factor;
}

// ---- Construction ----

DraftRadialDimension::DraftRadialDimension(
    const math::Vec2& center, double radius,
    const math::Vec2& textPoint, bool isDiameter)
    : m_center(center)
    , m_radius(radius)
    , m_textPoint(textPoint)
    , m_isDiameter(isDiameter) {}

// ---- Measurement ----

double DraftRadialDimension::computedValue() const {
    return m_isDiameter ? m_radius * 2.0 : m_radius;
}

std::string DraftRadialDimension::displayText(const DimensionStyle& style) const {
    if (hasTextOverride()) return m_textOverride;

    std::string prefix = m_isDiameter ? "\xE2\x8C\x80" : "R";  // UTF-8 ⌀ (U+2300) or R
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(style.precision) << computedValue();
    return prefix + oss.str();
}

// ---- Geometry helpers ----

math::Vec2 DraftRadialDimension::boundaryPoint() const {
    math::Vec2 dir = (m_textPoint - m_center).normalized();
    if (dir.length() < 1e-12) dir = {1.0, 0.0};
    return m_center + dir * m_radius;
}

math::Vec2 DraftRadialDimension::textPosition() const {
    return m_textPoint;
}

// ---- Rendering geometry ----

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftRadialDimension::extensionLines(const DimensionStyle& /*style*/) const {
    // Radial dimensions don't have extension lines.
    return {};
}

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftRadialDimension::dimensionLines(const DimensionStyle& /*style*/) const {
    math::Vec2 bp = boundaryPoint();

    if (m_isDiameter) {
        // Line through center from one boundary point to the opposite.
        math::Vec2 dir = (bp - m_center).normalized();
        math::Vec2 opposite = m_center - dir * m_radius;
        return {{opposite, m_textPoint}};
    }

    // Radius: line from center to text point (through boundary).
    return {{m_center, m_textPoint}};
}

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftRadialDimension::arrowheadLines(const DimensionStyle& style) const {
    math::Vec2 bp = boundaryPoint();
    math::Vec2 dir = (m_textPoint - m_center).normalized();

    // Arrow at boundary, pointing inward (toward center).
    auto arrows = makeArrowhead(bp, -dir, style.arrowSize, style.arrowAngle);

    if (m_isDiameter) {
        // Second arrow at opposite boundary, pointing inward.
        math::Vec2 opposite = m_center - dir * m_radius;
        auto arrows2 = makeArrowhead(opposite, dir, style.arrowSize, style.arrowAngle);
        arrows.insert(arrows.end(), arrows2.begin(), arrows2.end());
    }

    return arrows;
}

// ---- DraftEntity overrides ----

math::BoundingBox DraftRadialDimension::boundingBox() const {
    math::Vec2 bp = boundaryPoint();
    double minX = std::min({m_center.x, bp.x, m_textPoint.x});
    double minY = std::min({m_center.y, bp.y, m_textPoint.y});
    double maxX = std::max({m_center.x, bp.x, m_textPoint.x});
    double maxY = std::max({m_center.y, bp.y, m_textPoint.y});
    return math::BoundingBox({minX, minY, 0.0}, {maxX, maxY, 0.0});
}

bool DraftRadialDimension::hitTest(const math::Vec2& point, double tolerance) const {
    // Test against dimension line.
    math::Vec2 a, b;
    if (m_isDiameter) {
        math::Vec2 dir = (boundaryPoint() - m_center).normalized();
        a = m_center - dir * m_radius;
        b = m_textPoint;
    } else {
        a = m_center;
        b = m_textPoint;
    }

    math::Vec2 ab = b - a;
    double lenSq = ab.lengthSquared();
    if (lenSq > 1e-14) {
        double t = math::clamp((point - a).dot(ab) / lenSq, 0.0, 1.0);
        if (point.distanceTo(a + ab * t) <= tolerance) return true;
    }

    return false;
}

std::vector<math::Vec2> DraftRadialDimension::snapPoints() const {
    return {m_center, boundaryPoint(), m_textPoint};
}

void DraftRadialDimension::translate(const math::Vec2& delta) {
    m_center += delta;
    m_textPoint += delta;
}

std::shared_ptr<DraftEntity> DraftRadialDimension::clone() const {
    auto copy = std::make_shared<DraftRadialDimension>(
        m_center, m_radius, m_textPoint, m_isDiameter);
    copy->setLayer(layer());
    copy->setColor(color());
    copy->setLineWidth(lineWidth());
    copy->setLineType(lineType());
    copy->setGroupId(groupId());
    copy->setTextOverride(m_textOverride);
    return copy;
}

void DraftRadialDimension::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    m_center = mirrorPoint(m_center, axisP1, axisP2);
    m_textPoint = mirrorPoint(m_textPoint, axisP1, axisP2);
}

void DraftRadialDimension::rotate(const math::Vec2& center, double angle) {
    m_center = rotatePoint(m_center, center, angle);
    m_textPoint = rotatePoint(m_textPoint, center, angle);
}

void DraftRadialDimension::scale(const math::Vec2& center, double factor) {
    m_center = scalePoint(m_center, center, factor);
    m_textPoint = scalePoint(m_textPoint, center, factor);
    m_radius *= std::abs(factor);
}

}  // namespace hz::draft
