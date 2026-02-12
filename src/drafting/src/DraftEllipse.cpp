#include "horizon/drafting/DraftEllipse.h"
#include "horizon/math/Constants.h"

#include <algorithm>
#include <cmath>

namespace hz::draft {

// ---------------------------------------------------------------------------
// Local helpers
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

DraftEllipse::DraftEllipse(const math::Vec2& center, double semiMajor,
                           double semiMinor, double rotation)
    : m_center(center)
    , m_semiMajor(semiMajor)
    , m_semiMinor(semiMinor)
    , m_rotation(rotation) {}

// ---------------------------------------------------------------------------
// Evaluate curve points
// ---------------------------------------------------------------------------

std::vector<math::Vec2> DraftEllipse::evaluate(int segments) const {
    std::vector<math::Vec2> pts;
    pts.reserve(segments + 1);
    double cosR = std::cos(m_rotation);
    double sinR = std::sin(m_rotation);
    for (int i = 0; i <= segments; ++i) {
        double t = math::kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
        double lx = m_semiMajor * std::cos(t);
        double ly = m_semiMinor * std::sin(t);
        pts.push_back({m_center.x + lx * cosR - ly * sinR,
                        m_center.y + lx * sinR + ly * cosR});
    }
    return pts;
}

// ---------------------------------------------------------------------------
// DraftEntity overrides
// ---------------------------------------------------------------------------

math::BoundingBox DraftEllipse::boundingBox() const {
    // Exact rotated-ellipse bounding box via parametric extrema.
    double cosR = std::cos(m_rotation);
    double sinR = std::sin(m_rotation);
    double dx = std::sqrt(m_semiMajor * m_semiMajor * cosR * cosR +
                          m_semiMinor * m_semiMinor * sinR * sinR);
    double dy = std::sqrt(m_semiMajor * m_semiMajor * sinR * sinR +
                          m_semiMinor * m_semiMinor * cosR * cosR);
    return math::BoundingBox({m_center.x - dx, m_center.y - dy, 0.0},
                              {m_center.x + dx, m_center.y + dy, 0.0});
}

bool DraftEllipse::hitTest(const math::Vec2& point, double tolerance) const {
    // Transform point into ellipse-local space (un-rotate), then test
    // distance to the unit-circle-scaled ellipse.
    double cosR = std::cos(-m_rotation);
    double sinR = std::sin(-m_rotation);
    math::Vec2 v = point - m_center;
    double lx = v.x * cosR - v.y * sinR;
    double ly = v.x * sinR + v.y * cosR;

    // Avoid division by zero for degenerate ellipses.
    if (m_semiMajor < 1e-12 || m_semiMinor < 1e-12) return false;

    // Approximate distance to ellipse using the implicit equation.
    // For a point on the ellipse, (lx/a)^2 + (ly/b)^2 = 1.
    double nx = lx / m_semiMajor;
    double ny = ly / m_semiMinor;
    double d = std::sqrt(nx * nx + ny * ny);
    if (d < 1e-12) return tolerance >= std::min(m_semiMajor, m_semiMinor);

    // Approximate distance to the nearest point on the ellipse.
    double px = lx / d;  // project onto approximate closest point
    double py = ly / d;
    double ex = m_semiMajor * nx / d;
    double ey = m_semiMinor * ny / d;
    double dist = std::sqrt((lx - ex) * (lx - ex) + (ly - ey) * (ly - ey));
    return dist <= tolerance;
}

std::vector<math::Vec2> DraftEllipse::snapPoints() const {
    double cosR = std::cos(m_rotation);
    double sinR = std::sin(m_rotation);
    // Center + endpoints of major and minor axes (4 quadrant points).
    return {
        m_center,
        {m_center.x + m_semiMajor * cosR, m_center.y + m_semiMajor * sinR},
        {m_center.x - m_semiMajor * cosR, m_center.y - m_semiMajor * sinR},
        {m_center.x - m_semiMinor * sinR, m_center.y + m_semiMinor * cosR},
        {m_center.x + m_semiMinor * sinR, m_center.y - m_semiMinor * cosR},
    };
}

void DraftEllipse::translate(const math::Vec2& delta) {
    m_center += delta;
}

std::shared_ptr<DraftEntity> DraftEllipse::clone() const {
    auto copy = std::make_shared<DraftEllipse>(m_center, m_semiMajor, m_semiMinor, m_rotation);
    copy->setLayer(layer());
    copy->setColor(color());
    copy->setLineWidth(lineWidth());
    return copy;
}

void DraftEllipse::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    m_center = mirrorPoint(m_center, axisP1, axisP2);
    // Mirror flips the rotation: reflect across the axis.
    double axisAngle = std::atan2(axisP2.y - axisP1.y, axisP2.x - axisP1.x);
    m_rotation = 2.0 * axisAngle - m_rotation;
}

void DraftEllipse::rotate(const math::Vec2& center, double angle) {
    m_center = rotatePoint(m_center, center, angle);
    m_rotation += angle;
}

void DraftEllipse::scale(const math::Vec2& center, double factor) {
    m_center = scalePoint(m_center, center, factor);
    m_semiMajor *= std::abs(factor);
    m_semiMinor *= std::abs(factor);
}

}  // namespace hz::draft
