#include "horizon/drafting/DraftHatch.h"

#include <algorithm>
#include <cmath>

namespace hz::draft {

DraftHatch::DraftHatch(const std::vector<math::Vec2>& boundary, HatchPattern pattern, double angle,
                       double spacing)
    : m_boundary(boundary),
      m_pattern(pattern),
      m_angle(angle),
      m_spacing(std::max(spacing, 0.01)) {}

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
    return math::BoundingBox(math::Vec3(minX, minY, 0.0), math::Vec3(maxX, maxY, 0.0));
}

bool DraftHatch::hitTest(const math::Vec2& point, double tolerance) const {
    if (m_boundary.size() < 3) return false;

    // Hit if inside polygon.
    if (pointInPolygon(point)) return true;

    // Hit if near any boundary edge.
    auto segmentDist = [](const math::Vec2& p, const math::Vec2& a, const math::Vec2& b) -> double {
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
        if (segmentDist(point, m_boundary[i], m_boundary[j]) <= tolerance) return true;
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

std::vector<SnapPoint> DraftHatch::typedSnapPoints() const {
    std::vector<SnapPoint> out;
    out.reserve(m_boundary.size() * 2);
    for (const auto& pt : m_boundary) out.push_back({pt, SnapType::Endpoint});
    for (size_t i = 0; i < m_boundary.size(); ++i) {
        out.push_back(
            {(m_boundary[i] + m_boundary[(i + 1) % m_boundary.size()]) * 0.5, SnapType::Midpoint});
    }
    return out;
}

void DraftHatch::translate(const math::Vec2& delta) {
    for (auto& pt : m_boundary) {
        pt += delta;
    }
}

std::shared_ptr<DraftEntity> DraftHatch::clone() const {
    auto copy = std::make_shared<DraftHatch>(m_boundary, m_pattern, m_angle, m_spacing);
    copyInto(*copy);
    return copy;
}

static math::Vec2 mirrorPoint(const math::Vec2& p, const math::Vec2& axisP1,
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

std::vector<std::pair<math::Vec2, math::Vec2>> DraftHatch::scanLines(double scanAngle,
                                                                     double scanSpacing) const {
    if (m_boundary.size() < 3 || !(scanSpacing > 0.0) || !std::isfinite(scanSpacing)) return {};

    // Direction along the hatch lines and perpendicular (scan direction).
    math::Vec2 dir = {std::cos(scanAngle), std::sin(scanAngle)};
    math::Vec2 perp = {-dir.y, dir.x};

    // Measured from the boundary's first point: far from the origin (a
    // survey's coordinates) an offset there kept what a step of the spacing
    // added, and at 1e17 a step of 0.001 added nothing and never ended.
    const math::Vec2 origin = m_boundary[0];
    const size_t n = m_boundary.size();
    std::vector<math::Vec2> local(n);
    for (size_t i = 0; i < n; ++i) local[i] = m_boundary[i] - origin;

    // Project all boundary points onto the perpendicular to find scan range.
    double minProj = 0.0;
    double maxProj = 0.0;
    for (const math::Vec2& p : local) {
        minProj = std::min(minProj, perp.dot(p));
        maxProj = std::max(maxProj, perp.dot(p));
    }
    const double range = maxProj - minProj;
    if (!std::isfinite(range) || range <= 0.0) return {};

    // At most kMaxLines across: a spacing too fine for that is drawn at the
    // finest that keeps to it. A hatch past it drew nothing at all, in the
    // view or a plot: a solid fill 300 wide, at its tenth of a unit apart.
    constexpr double kMaxLines = 2000.0;
    const double spacing = std::max(scanSpacing, range / kMaxLines);
    const auto count = static_cast<long>(std::ceil(range / spacing));

    // The edges by the lowest of their ends' offsets, swept: a line looks
    // only at the edges that span it, not at all of them (a boundary of
    // 100,000 points by 2,000 lines).
    struct Edge {
        double low;
        double high;
        size_t from;
    };
    std::vector<Edge> edges(n);
    for (size_t i = 0; i < n; ++i) {
        const double pa = perp.dot(local[i]);
        const double pb = perp.dot(local[(i + 1) % n]);
        edges[i] = {std::min(pa, pb), std::max(pa, pb), i};
    }
    std::sort(edges.begin(), edges.end(),
              [](const Edge& a, const Edge& b) { return a.low < b.low; });
    std::vector<Edge> active;
    size_t next = 0;

    std::vector<std::pair<math::Vec2, math::Vec2>> result;
    std::vector<double> intersections;
    for (long line = 0; line < count; ++line) {
        const double offset = minProj + (static_cast<double>(line) + 0.5) * spacing;
        if (offset >= maxProj) break;
        // Origin of the scan line: a point at distance 'offset' along perp.
        // The scan line runs in direction 'dir'.
        while (next < edges.size() && edges[next].low <= offset) active.push_back(edges[next++]);
        active.erase(std::remove_if(active.begin(), active.end(),
                                    [offset](const Edge& e) { return e.high < offset; }),
                     active.end());

        // Find intersections of this scan line with the edges spanning it.
        intersections.clear();
        for (const Edge& edge : active) {
            const size_t i = edge.from;
            const size_t j = (i + 1) % n;
            const auto& a = local[i];
            const auto& b = local[j];

            // Projection of edge endpoints onto perp.
            double pa = perp.dot(a);
            double pb = perp.dot(b);

            // Check if scan line crosses this edge. Each end of an edge is on
            // one side of the line or the other, one on it counting as below
            // (as in pointInPolygon): a line through a corner of the boundary
            // then crosses it once where the boundary passes on, and twice or
            // not at all where it turns back, so the crossings pair up into
            // the pieces inside. An edge along the line is not crossed.
            if ((pa <= offset) == (pb <= offset)) continue;

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
            math::Vec2 p1 = origin + perp * offset + dir * intersections[k];
            math::Vec2 p2 = origin + perp * offset + dir * intersections[k + 1];
            result.emplace_back(p1, p2);
        }
    }

    return result;
}

std::vector<std::pair<math::Vec2, math::Vec2>> DraftHatch::generateHatchLines() const {
    if (m_boundary.size() < 3) return {};

    double effectiveSpacing = m_spacing;
    if (m_pattern == HatchPattern::Solid) {
        // Dense lines for a fill: a tenth of the spacing, and at least fifty
        // across, however small it is (it was a line or two across a hatch
        // a unit wide). scanLines() keeps a large one to its most.
        const math::BoundingBox box = boundingBox();
        const double across = std::max(box.max().x - box.min().x, box.max().y - box.min().y);
        effectiveSpacing = std::min(m_spacing * 0.1, across / 50.0);
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
