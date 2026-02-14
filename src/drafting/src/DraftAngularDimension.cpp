#include "horizon/drafting/DraftAngularDimension.h"
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

/// Normalize angle to [0, 2pi).
static double normalizeAngle(double a) {
    a = std::fmod(a, math::kTwoPi);
    if (a < 0.0) a += math::kTwoPi;
    return a;
}

// ---- Construction ----

DraftAngularDimension::DraftAngularDimension(
    const math::Vec2& vertex, const math::Vec2& line1Point,
    const math::Vec2& line2Point, double arcRadius)
    : m_vertex(vertex)
    , m_line1Point(line1Point)
    , m_line2Point(line2Point)
    , m_arcRadius(arcRadius) {}

// ---- Angle helpers ----

double DraftAngularDimension::startAngle() const {
    return std::atan2(m_line1Point.y - m_vertex.y, m_line1Point.x - m_vertex.x);
}

double DraftAngularDimension::endAngle() const {
    return std::atan2(m_line2Point.y - m_vertex.y, m_line2Point.x - m_vertex.x);
}

// ---- Measurement ----

double DraftAngularDimension::computedValue() const {
    double a1 = normalizeAngle(startAngle());
    double a2 = normalizeAngle(endAngle());
    double sweep = a2 - a1;
    if (sweep < 0.0) sweep += math::kTwoPi;
    // Always take the smaller angle.
    if (sweep > math::kPi) sweep = math::kTwoPi - sweep;
    return sweep * math::kRadToDeg;
}

std::string DraftAngularDimension::displayText(const DimensionStyle& style) const {
    if (hasTextOverride()) return m_textOverride;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(style.precision) << computedValue();
    oss << "\xC2\xB0";  // UTF-8 degree symbol °
    return oss.str();
}

math::Vec2 DraftAngularDimension::textPosition() const {
    double a1 = normalizeAngle(startAngle());
    double a2 = normalizeAngle(endAngle());
    double sweep = a2 - a1;
    if (sweep < 0.0) sweep += math::kTwoPi;
    if (sweep > math::kPi) {
        // Take the shorter arc the other way.
        a1 = a2;
        sweep = math::kTwoPi - sweep;
    }
    double midAngle = a1 + sweep * 0.5;
    return m_vertex + math::Vec2{std::cos(midAngle), std::sin(midAngle)} * m_arcRadius;
}

// ---- Rendering geometry ----

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftAngularDimension::extensionLines(const DimensionStyle& style) const {
    // Extension lines from vertex outward along each line direction.
    math::Vec2 dir1 = (m_line1Point - m_vertex).normalized();
    math::Vec2 dir2 = (m_line2Point - m_vertex).normalized();

    math::Vec2 ext1Start = m_vertex + dir1 * style.extensionGap;
    math::Vec2 ext1End   = m_vertex + dir1 * (m_arcRadius + style.extensionOvershoot);
    math::Vec2 ext2Start = m_vertex + dir2 * style.extensionGap;
    math::Vec2 ext2End   = m_vertex + dir2 * (m_arcRadius + style.extensionOvershoot);

    return {{ext1Start, ext1End}, {ext2Start, ext2End}};
}

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftAngularDimension::dimensionLines(const DimensionStyle& /*style*/) const {
    // Arc approximated as line segments.
    double a1 = normalizeAngle(startAngle());
    double a2 = normalizeAngle(endAngle());
    double sweep = a2 - a1;
    if (sweep < 0.0) sweep += math::kTwoPi;
    if (sweep > math::kPi) {
        double tmp = a1;
        a1 = a2;
        a2 = tmp;
        sweep = math::kTwoPi - sweep;
    }

    int segments = std::max(8, static_cast<int>(32 * sweep / math::kTwoPi));
    double step = sweep / static_cast<double>(segments);

    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    lines.reserve(static_cast<size_t>(segments));

    for (int i = 0; i < segments; ++i) {
        double ang0 = a1 + step * static_cast<double>(i);
        double ang1 = a1 + step * static_cast<double>(i + 1);
        math::Vec2 p0 = m_vertex + math::Vec2{std::cos(ang0), std::sin(ang0)} * m_arcRadius;
        math::Vec2 p1 = m_vertex + math::Vec2{std::cos(ang1), std::sin(ang1)} * m_arcRadius;
        lines.push_back({p0, p1});
    }
    return lines;
}

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftAngularDimension::arrowheadLines(const DimensionStyle& style) const {
    double a1 = normalizeAngle(startAngle());
    double a2 = normalizeAngle(endAngle());
    double sweep = a2 - a1;
    if (sweep < 0.0) sweep += math::kTwoPi;
    bool flipped = false;
    if (sweep > math::kPi) {
        double tmp = a1;
        a1 = a2;
        a2 = tmp;
        sweep = math::kTwoPi - sweep;
        flipped = true;
    }

    // Arrow at arc start: tangent direction (perpendicular to radial, CCW).
    math::Vec2 arcStart = m_vertex + math::Vec2{std::cos(a1), std::sin(a1)} * m_arcRadius;
    math::Vec2 tangent1{-std::sin(a1), std::cos(a1)};  // CCW tangent

    // Arrow at arc end: tangent direction (perpendicular to radial, CW = inward).
    math::Vec2 arcEnd = m_vertex + math::Vec2{std::cos(a2), std::sin(a2)} * m_arcRadius;
    math::Vec2 tangent2{std::sin(a2), -std::cos(a2)};  // CW tangent

    auto arrows = makeArrowhead(arcStart, tangent1, style.arrowSize, style.arrowAngle);
    auto arrows2 = makeArrowhead(arcEnd, tangent2, style.arrowSize, style.arrowAngle);
    arrows.insert(arrows.end(), arrows2.begin(), arrows2.end());
    return arrows;
}

// ---- DraftEntity overrides ----

math::BoundingBox DraftAngularDimension::boundingBox() const {
    double a1 = normalizeAngle(startAngle());
    double a2 = normalizeAngle(endAngle());
    double sweep = a2 - a1;
    if (sweep < 0.0) sweep += math::kTwoPi;
    if (sweep > math::kPi) {
        a1 = a2;
        sweep = math::kTwoPi - sweep;
    }

    double minX = m_vertex.x, minY = m_vertex.y;
    double maxX = m_vertex.x, maxY = m_vertex.y;

    // Sample arc endpoints and some intermediates.
    int samples = 8;
    for (int i = 0; i <= samples; ++i) {
        double a = a1 + sweep * static_cast<double>(i) / static_cast<double>(samples);
        double px = m_vertex.x + m_arcRadius * std::cos(a);
        double py = m_vertex.y + m_arcRadius * std::sin(a);
        minX = std::min(minX, px);
        minY = std::min(minY, py);
        maxX = std::max(maxX, px);
        maxY = std::max(maxY, py);
    }

    return math::BoundingBox({minX, minY, 0.0}, {maxX, maxY, 0.0});
}

bool DraftAngularDimension::hitTest(const math::Vec2& point, double tolerance) const {
    // Test against the dimension arc.
    double dist = m_vertex.distanceTo(point);
    if (std::abs(dist - m_arcRadius) > tolerance) return false;

    // Check if angle is within the swept range.
    double angle = normalizeAngle(std::atan2(point.y - m_vertex.y, point.x - m_vertex.x));
    double a1 = normalizeAngle(startAngle());
    double a2 = normalizeAngle(endAngle());
    double sweep = a2 - a1;
    if (sweep < 0.0) sweep += math::kTwoPi;
    if (sweep > math::kPi) {
        a1 = a2;
        sweep = math::kTwoPi - sweep;
    }

    double rel = normalizeAngle(angle - a1);
    if (rel <= sweep) return true;

    // Also test extension lines with default style.
    DimensionStyle defaultStyle;
    for (const auto& [a, b] : extensionLines(defaultStyle)) {
        math::Vec2 seg = b - a;
        double sLenSq = seg.lengthSquared();
        if (sLenSq < 1e-14) continue;
        double t = math::clamp((point - a).dot(seg) / sLenSq, 0.0, 1.0);
        if (point.distanceTo(a + seg * t) <= tolerance) return true;
    }

    return false;
}

std::vector<math::Vec2> DraftAngularDimension::snapPoints() const {
    return {m_vertex, textPosition()};
}

void DraftAngularDimension::translate(const math::Vec2& delta) {
    m_vertex += delta;
    m_line1Point += delta;
    m_line2Point += delta;
}

std::shared_ptr<DraftEntity> DraftAngularDimension::clone() const {
    auto copy = std::make_shared<DraftAngularDimension>(
        m_vertex, m_line1Point, m_line2Point, m_arcRadius);
    copy->setLayer(layer());
    copy->setColor(color());
    copy->setLineWidth(lineWidth());
    copy->setLineType(lineType());
    copy->setGroupId(groupId());
    copy->setTextOverride(m_textOverride);
    return copy;
}

void DraftAngularDimension::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    m_vertex = mirrorPoint(m_vertex, axisP1, axisP2);
    m_line1Point = mirrorPoint(m_line1Point, axisP1, axisP2);
    m_line2Point = mirrorPoint(m_line2Point, axisP1, axisP2);
}

void DraftAngularDimension::rotate(const math::Vec2& center, double angle) {
    m_vertex = rotatePoint(m_vertex, center, angle);
    m_line1Point = rotatePoint(m_line1Point, center, angle);
    m_line2Point = rotatePoint(m_line2Point, center, angle);
}

void DraftAngularDimension::scale(const math::Vec2& center, double factor) {
    m_vertex = scalePoint(m_vertex, center, factor);
    m_line1Point = scalePoint(m_line1Point, center, factor);
    m_line2Point = scalePoint(m_line2Point, center, factor);
    m_arcRadius *= std::abs(factor);
}

}  // namespace hz::draft
