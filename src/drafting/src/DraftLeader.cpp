#include "horizon/drafting/DraftLeader.h"
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

DraftLeader::DraftLeader(const std::vector<math::Vec2>& points, const std::string& text)
    : m_points(points), m_text(text) {}

// ---- Measurement ----

double DraftLeader::computedValue() const { return 0.0; }

std::string DraftLeader::displayText(const DimensionStyle& /*style*/) const {
    return m_text;
}

math::Vec2 DraftLeader::textPosition() const {
    if (m_points.empty()) return {};
    return m_points.back();
}

// ---- Rendering geometry ----

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftLeader::extensionLines(const DimensionStyle& /*style*/) const {
    return {};  // Leaders don't have extension lines.
}

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftLeader::dimensionLines(const DimensionStyle& /*style*/) const {
    std::vector<std::pair<math::Vec2, math::Vec2>> lines;
    for (size_t i = 0; i + 1 < m_points.size(); ++i) {
        lines.push_back({m_points[i], m_points[i + 1]});
    }
    return lines;
}

std::vector<std::pair<math::Vec2, math::Vec2>>
DraftLeader::arrowheadLines(const DimensionStyle& style) const {
    if (m_points.size() < 2) return {};

    // Arrow at the first point, pointing from second toward first.
    math::Vec2 dir = (m_points[1] - m_points[0]).normalized();
    return makeArrowhead(m_points[0], dir, style.arrowSize, style.arrowAngle);
}

// ---- DraftEntity overrides ----

math::BoundingBox DraftLeader::boundingBox() const {
    if (m_points.empty()) return {};

    double minX = m_points[0].x, minY = m_points[0].y;
    double maxX = minX, maxY = minY;
    for (const auto& p : m_points) {
        minX = std::min(minX, p.x);
        minY = std::min(minY, p.y);
        maxX = std::max(maxX, p.x);
        maxY = std::max(maxY, p.y);
    }
    return math::BoundingBox({minX, minY, 0.0}, {maxX, maxY, 0.0});
}

bool DraftLeader::hitTest(const math::Vec2& point, double tolerance) const {
    for (size_t i = 0; i + 1 < m_points.size(); ++i) {
        math::Vec2 ab = m_points[i + 1] - m_points[i];
        double lenSq = ab.lengthSquared();
        if (lenSq < 1e-14) continue;
        double t = math::clamp((point - m_points[i]).dot(ab) / lenSq, 0.0, 1.0);
        if (point.distanceTo(m_points[i] + ab * t) <= tolerance) return true;
    }
    return false;
}

std::vector<math::Vec2> DraftLeader::snapPoints() const {
    return m_points;
}

void DraftLeader::translate(const math::Vec2& delta) {
    for (auto& p : m_points) p += delta;
}

std::shared_ptr<DraftEntity> DraftLeader::clone() const {
    auto copy = std::make_shared<DraftLeader>(m_points, m_text);
    copy->setLayer(layer());
    copy->setColor(color());
    copy->setLineWidth(lineWidth());
    copy->setLineType(lineType());
    copy->setGroupId(groupId());
    copy->setTextOverride(m_textOverride);
    return copy;
}

void DraftLeader::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    for (auto& p : m_points) p = mirrorPoint(p, axisP1, axisP2);
}

void DraftLeader::rotate(const math::Vec2& center, double angle) {
    for (auto& p : m_points) p = rotatePoint(p, center, angle);
}

void DraftLeader::scale(const math::Vec2& center, double factor) {
    for (auto& p : m_points) p = scalePoint(p, center, factor);
}

}  // namespace hz::draft
