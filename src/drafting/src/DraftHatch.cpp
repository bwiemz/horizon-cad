#include "horizon/drafting/DraftHatch.h"
#include <algorithm>
#include <cmath>

namespace hz::draft {

DraftHatch::DraftHatch(const std::vector<math::Vec2>& boundary,
                       HatchPattern pattern, double angle, double spacing)
    : m_boundary(boundary)
    , m_pattern(pattern)
    , m_angle(angle)
    , m_spacing(std::max(spacing, 0.01)) {}

// ---------------------------------------------------------------------------
// DraftEntity virtuals
// ---------------------------------------------------------------------------

math::BoundingBox DraftHatch::boundingBox() const {
    if (m_boundary.empty()) return {};

    double minX = m_boundary[0].x, maxX = m_boundary[0].x;
    double minY = m_boundary[0].y, maxY = m_boundary[0].y;
    for (size_t i = 1; i < m_boundary.size(); ++i) {
        minX = std::min(minX, m_boundary[i].x);
        minY = std::min(minY, m_boundary[i].y);
        maxX = std::max(maxX, m_boundary[i].x);
        maxY = std::max(maxY, m_boundary[i].y);
    }
    return math::BoundingBox(math::Vec3(minX, minY, 0.0),
                             math::Vec3(maxX, maxY, 0.0));
}

bool DraftHatch::hitTest(const math::Vec2& point, double tolerance) const {
    if (m_boundary.size() < 3) return false;

    // Hit if inside polygon.
    if (pointInPolygon(point)) return true;

    // Hit if near any boundary edge.
    auto segmentDist = [](const math::Vec2& p, const math::Vec2& a,
                          const math::Vec2& b) -> double {
        math::Vec2 ab = b - a;
        math::Vec2 ap = p - a;
        double lenSq = ab.lengthSquared();
        if (lenSq < 1e-14) return p.distanceTo(a);
        double t = std::clamp(ap.dot(ab) / lenSq, 0.0, 1.0);
        math::Vec2 closest = a + ab * t;
        return p.distanceTo(closest);
    };

    for (size_t i = 0; i < m_boundary.size(); ++i) {
        size_t j = (i + 1) % m_boundary.size();
        if (segmentDist(point, m_boundary[i], m_boundary[j]) <= tolerance)
            return true;
    }
    return false;
}

std::vector<math::Vec2> DraftHatch::snapPoints() const {
    std::vector<math::Vec2> result;
    result.reserve(m_boundary.size() * 2);
    for (const auto& pt : m_boundary) {
        result.push_back(pt);
    }
    for (size_t i = 0; i < m_boundary.size(); ++i) {
        size_t j = (i + 1) % m_boundary.size();
        result.push_back((m_boundary[i] + m_boundary[j]) * 0.5);
    }
    return result;
}

void DraftHatch::translate(const math::Vec2& delta) {
    for (auto& pt : m_boundary) {
        pt += delta;
    }
}

std::shared_ptr<DraftEntity> DraftHatch::clone() const {
    auto copy = std::make_shared<DraftHatch>(m_boundary, m_pattern, m_angle, m_spacing);
    copy->setLayer(layer());
    copy->setColor(color());
    copy->setLineWidth(lineWidth());
    copy->setLineType(lineType());
    copy->setGroupId(groupId());
    return copy;
}

static math::Vec2 mirrorPoint(const math::Vec2& p,
                               const math::Vec2& axisP1,
                               const math::Vec2& axisP2) {
    math::Vec2 d = (axisP2 - axisP1).normalized();
    math::Vec2 v = p - axisP1;
    return axisP1 + d * (2.0 * v.dot(d)) - v;
}

void DraftHatch::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    for (auto& pt : m_boundary) {
        pt = mirrorPoint(pt, axisP1, axisP2);
    }
    double axisAngle = std::atan2((axisP2 - axisP1).y, (axisP2 - axisP1).x);
    m_angle = 2.0 * axisAngle - m_angle;
}

static math::Vec2 rotatePoint(const math::Vec2& p, const math::Vec2& center, double angle) {
    double c = std::cos(angle), s = std::sin(angle);
    math::Vec2 v = p - center;
    return {center.x + v.x * c - v.y * s, center.y + v.x * s + v.y * c};
}

static math::Vec2 scalePoint(const math::Vec2& p, const math::Vec2& center, double factor) {
    return center + (p - center) * factor;
}

void DraftHatch::rotate(const math::Vec2& center, double angle) {
    for (auto& pt : m_boundary) {
        pt = rotatePoint(pt, center, angle);
    }
    m_angle += angle;
}

void DraftHatch::scale(const math::Vec2& center, double factor) {
    for (auto& pt : m_boundary) {
        pt = scalePoint(pt, center, factor);
    }
    m_spacing *= std::abs(factor);
}

// ---------------------------------------------------------------------------
// Hatch line generation
// ---------------------------------------------------------------------------

bool DraftHatch::pointInPolygon(const math::Vec2& point) const {
    // Ray-casting algorithm.
    bool inside = false;
    size_t n = m_boundary.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double yi = m_boundary[i].y, yj = m_boundary[j].y;
        double xi = m_boundary[i].x, xj = m_boundary[j].x;
        if (((yi > point.y) != (yj > point.y)) &&
            (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

std::vector<std::pair<math::Vec2, math::Vec2>> DraftHatch::scanLines(
    double scanAngle, double scanSpacing) const {
    if (m_boundary.size() < 3 || scanSpacing < 0.001) return {};

    // Direction along the hatch lines and perpendicular (scan direction).
    math::Vec2 dir = {std::cos(scanAngle), std::sin(scanAngle)};
    math::Vec2 perp = {-dir.y, dir.x};

    // Project all boundary points onto the perpendicular to find scan range.
    double minProj = perp.dot(m_boundary[0]);
    double maxProj = minProj;
    for (size_t i = 1; i < m_boundary.size(); ++i) {
        double proj = perp.dot(m_boundary[i]);
        minProj = std::min(minProj, proj);
        maxProj = std::max(maxProj, proj);
    }

    // Also find extent along the direction for scan line length.
    double minDir = dir.dot(m_boundary[0]);
    double maxDir = minDir;
    for (size_t i = 1; i < m_boundary.size(); ++i) {
        double d = dir.dot(m_boundary[i]);
        minDir = std::min(minDir, d);
        maxDir = std::max(maxDir, d);
    }

    // Limit number of scan lines to prevent performance issues.
    double range = maxProj - minProj;
    int maxLines = 2000;
    if (range / scanSpacing > maxLines) return {};

    std::vector<std::pair<math::Vec2, math::Vec2>> result;
    size_t n = m_boundary.size();

    for (double offset = minProj + scanSpacing * 0.5; offset < maxProj; offset += scanSpacing) {
        // Origin of the scan line: a point at distance 'offset' along perp.
        // The scan line runs in direction 'dir'.
        // lineP = perp * offset (relative to origin)
        // Points on line: lineP + t * dir

        // Find intersections of this scan line with all boundary edges.
        std::vector<double> intersections;
        for (size_t i = 0; i < n; ++i) {
            size_t j = (i + 1) % n;
            const auto& a = m_boundary[i];
            const auto& b = m_boundary[j];

            // Projection of edge endpoints onto perp.
            double pa = perp.dot(a);
            double pb = perp.dot(b);

            // Check if scan line crosses this edge.
            if ((pa - offset) * (pb - offset) > 0.0) continue;  // both on same side
            if (std::abs(pb - pa) < 1e-14) continue;  // edge parallel to scan line

            // Parameter along edge where intersection occurs.
            double t = (offset - pa) / (pb - pa);
            // Intersection point projected onto dir.
            math::Vec2 pt = a + (b - a) * t;
            double d = dir.dot(pt);
            intersections.push_back(d);
        }

        // Sort intersections and pair them up.
        std::sort(intersections.begin(), intersections.end());
        for (size_t k = 0; k + 1 < intersections.size(); k += 2) {
            math::Vec2 p1 = perp * offset + dir * intersections[k];
            math::Vec2 p2 = perp * offset + dir * intersections[k + 1];
            result.emplace_back(p1, p2);
        }
    }

    return result;
}

std::vector<std::pair<math::Vec2, math::Vec2>> DraftHatch::generateHatchLines() const {
    if (m_boundary.size() < 3) return {};

    double effectiveSpacing = m_spacing;
    if (m_pattern == HatchPattern::Solid) {
        // Dense lines for solid fill.
        effectiveSpacing = std::max(m_spacing * 0.1, 0.05);
    }

    auto lines = scanLines(m_angle, effectiveSpacing);

    if (m_pattern == HatchPattern::CrossHatch) {
        // Add perpendicular set.
        auto cross = scanLines(m_angle + 3.14159265358979323846 * 0.5, effectiveSpacing);
        lines.insert(lines.end(), cross.begin(), cross.end());
    }

    return lines;
}

}  // namespace hz::draft
