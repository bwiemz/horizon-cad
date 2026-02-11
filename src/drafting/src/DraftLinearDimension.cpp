#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/math/Constants.h"
#include "horizon/math/MathUtils.h"

#include <algorithm>
#include <cmath>

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

DraftLinearDimension::DraftLinearDimension(
    const math::Vec2& defPoint1, const math::Vec2& defPoint2,
    const math::Vec2& dimLinePoint, Orientation orientation)
    : m_defPoint1(defPoint1)
    , m_defPoint2(defPoint2)
    , m_dimLinePoint(dimLinePoint)
    , m_orientation(orientation) {}

// ---- Measurement ----

double DraftLinearDimension::computedValue() const {
    switch (m_orientation) {
        case Orientation::Horizontal:
            return std::abs(m_defPoint2.x - m_defPoint1.x);
        case Orientation::Vertical:
            return std::abs(m_defPoint2.y - m_defPoint1.y);
        case Orientation::Aligned:
            return m_defPoint1.distanceTo(m_defPoint2);
    }
    return 0.0;
}

// ---- Geometry helpers ----

std::pair<math::Vec2, math::Vec2> DraftLinearDimension::dimLineEndpoints() const {
    // The dimension line endpoints are the projections of defPoint1/defPoint2
    // onto the dimension line.
    switch (m_orientation) {
        case Orientation::Horizontal: {
            double y = m_dimLinePoint.y;
            return {{m_defPoint1.x, y}, {m_defPoint2.x, y}};
        }
        case Orientation::Vertical: {
            double x = m_dimLinePoint.x;
            return {{x, m_defPoint1.y}, {x, m_defPoint2.y}};
        }
        case Orientation::Aligned: {
            // Dimension line is parallel to defPoint1→defPoint2.
            math::Vec2 dir = (m_defPoint2 - m_defPoint1).normalized();
            math::Vec2 perp = dir.perpendicular();

            // Offset = signed distance from defPoint1→defPoint2 line to dimLinePoint.
            math::Vec2 v = m_dimLinePoint - m_defPoint1;
            double offset = v.dot(perp);

            return {m_defPoint1 + perp * offset, m_defPoint2 + perp * offset};
        }
    }
    return {{}, {}};
}

math::Vec2 DraftLinearDimension::textPosition() const {
    auto [a, b] = dimLineEndpoints();
    return (a + b) * 0.5;
}

// ---- Rendering geometry ----

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftLinearDimension::extensionLines(const DimensionStyle& style) const {
    auto [dlA, dlB] = dimLineEndpoints();

    // Perpendicular direction from defPoint to its dim line endpoint.
    auto makeExtLine = [&](const math::Vec2& defPt, const math::Vec2& dlPt) {
        math::Vec2 dir = (dlPt - defPt);
        double len = dir.length();
        if (len < 1e-12) return std::make_pair(defPt, defPt);

        math::Vec2 d = dir / len;
        math::Vec2 start = defPt + d * style.extensionGap;
        math::Vec2 end   = dlPt  + d * style.extensionOvershoot;
        return std::make_pair(start, end);
    };

    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    auto ext1 = makeExtLine(m_defPoint1, dlA);
    auto ext2 = makeExtLine(m_defPoint2, dlB);
    if (ext1.first.distanceTo(ext1.second) > 1e-12)
        lines.push_back(ext1);
    if (ext2.first.distanceTo(ext2.second) > 1e-12)
        lines.push_back(ext2);
    return lines;
}

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftLinearDimension::dimensionLines(const DimensionStyle& /*style*/) const {
    auto [a, b] = dimLineEndpoints();
    return {{a, b}};
}

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftLinearDimension::arrowheadLines(const DimensionStyle& style) const {
    auto [a, b] = dimLineEndpoints();
    math::Vec2 dir = (b - a).normalized();

    auto arrows = makeArrowhead(a, dir, style.arrowSize, style.arrowAngle);
    auto arrows2 = makeArrowhead(b, -dir, style.arrowSize, style.arrowAngle);
    arrows.insert(arrows.end(), arrows2.begin(), arrows2.end());
    return arrows;
}

// ---- DraftEntity overrides ----

math::BoundingBox DraftLinearDimension::boundingBox() const {
    auto [dlA, dlB] = dimLineEndpoints();

    double minX = std::min({m_defPoint1.x, m_defPoint2.x, dlA.x, dlB.x});
    double minY = std::min({m_defPoint1.y, m_defPoint2.y, dlA.y, dlB.y});
    double maxX = std::max({m_defPoint1.x, m_defPoint2.x, dlA.x, dlB.x});
    double maxY = std::max({m_defPoint1.y, m_defPoint2.y, dlA.y, dlB.y});

    return math::BoundingBox({minX, minY, 0.0}, {maxX, maxY, 0.0});
}

bool DraftLinearDimension::hitTest(const math::Vec2& point, double tolerance) const {
    auto [dlA, dlB] = dimLineEndpoints();

    // Test against dimension line segment.
    math::Vec2 ab = dlB - dlA;
    double lenSq = ab.lengthSquared();
    if (lenSq > 1e-14) {
        double t = math::clamp((point - dlA).dot(ab) / lenSq, 0.0, 1.0);
        math::Vec2 closest = dlA + ab * t;
        if (point.distanceTo(closest) <= tolerance) return true;
    }

    // Test against extension lines.
    DimensionStyle defaultStyle;  // use defaults for hit test geometry
    for (const auto& [a, b] : extensionLines(defaultStyle)) {
        math::Vec2 seg = b - a;
        double sLenSq = seg.lengthSquared();
        if (sLenSq < 1e-14) continue;
        double t = math::clamp((point - a).dot(seg) / sLenSq, 0.0, 1.0);
        if (point.distanceTo(a + seg * t) <= tolerance) return true;
    }

    return false;
}

std::vector<math::Vec2> DraftLinearDimension::snapPoints() const {
    auto [dlA, dlB] = dimLineEndpoints();
    return {m_defPoint1, m_defPoint2, dlA, dlB, textPosition()};
}

void DraftLinearDimension::translate(const math::Vec2& delta) {
    m_defPoint1 += delta;
    m_defPoint2 += delta;
    m_dimLinePoint += delta;
}

std::shared_ptr<DraftEntity> DraftLinearDimension::clone() const {
    auto copy = std::make_shared<DraftLinearDimension>(
        m_defPoint1, m_defPoint2, m_dimLinePoint, m_orientation);
    copy->setLayer(layer());
    copy->setColor(color());
    copy->setLineWidth(lineWidth());
    copy->setTextOverride(m_textOverride);
    return copy;
}

void DraftLinearDimension::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    m_defPoint1 = mirrorPoint(m_defPoint1, axisP1, axisP2);
    m_defPoint2 = mirrorPoint(m_defPoint2, axisP1, axisP2);
    m_dimLinePoint = mirrorPoint(m_dimLinePoint, axisP1, axisP2);
}

void DraftLinearDimension::rotate(const math::Vec2& center, double angle) {
    m_defPoint1 = rotatePoint(m_defPoint1, center, angle);
    m_defPoint2 = rotatePoint(m_defPoint2, center, angle);
    m_dimLinePoint = rotatePoint(m_dimLinePoint, center, angle);
}

void DraftLinearDimension::scale(const math::Vec2& center, double factor) {
    m_defPoint1 = scalePoint(m_defPoint1, center, factor);
    m_defPoint2 = scalePoint(m_defPoint2, center, factor);
    m_dimLinePoint = scalePoint(m_dimLinePoint, center, factor);
}

}  // namespace hz::draft
