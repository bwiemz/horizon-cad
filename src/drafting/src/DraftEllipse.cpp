#include "horizon/drafting/DraftEllipse.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "horizon/math/Constants.h"

namespace hz::draft {

// ---------------------------------------------------------------------------
// Local helpers
// ---------------------------------------------------------------------------

static math::Vec2 mirrorPoint(const math::Vec2& p, const math::Vec2& axisP1,
                              const math::Vec2& axisP2) {
    math::Vec2 d = (axisP2 - axisP1).normalized();
    math::Vec2 v = p - axisP1;
    return axisP1 + d * (2.0 * v.dot(d)) - v;
}

static math::Vec2 rotatePoint(const math::Vec2& p, const math::Vec2& center, double angle) {
    double c = std::cos(angle), s = std::sin(angle);
    math::Vec2 v = p - center;
    return {center.x + v.x * c - v.y * s, center.y + v.x * s + v.y * c};
}

static math::Vec2 scalePoint(const math::Vec2& p, const math::Vec2& center, double factor) {
    return center + (p - center) * factor;
}

/// The s >= 0 at which (r0 z0 / (s + r0))^2 + (z1 / (s + 1))^2 = 1, found by
/// halving the interval it lies in until it cannot be halved: the nearest
/// point of an ellipse, as D. Eberly finds it ("Distance from a Point to an
/// Ellipse, an Ellipsoid, or a Hyperellipsoid").
static double ellipseRoot(double r0, double z0, double z1, double g) {
    const double n0 = r0 * z0;
    double s0 = z1 - 1.0;
    double s1 = g < 0.0 ? 0.0 : std::hypot(n0, z1) - 1.0;
    double s = 0.0;
    // Each pass halves the interval; no more passes than a double has bits
    // of mantissa and exponent.
    for (int i = 0; i < 1100; ++i) {
        s = (s0 + s1) * 0.5;
        if (s == s0 || s == s1) break;
        const double ratio0 = n0 / (s + r0);
        const double ratio1 = z1 / (s + 1.0);
        g = ratio0 * ratio0 + ratio1 * ratio1 - 1.0;
        if (g > 0.0) {
            s0 = s;
        } else if (g < 0.0) {
            s1 = s;
        } else {
            break;
        }
    }
    return s;
}

/// The distance from (y0, y1), both >= 0, to the ellipse with semi-axes
/// e0 >= e1 > 0 along x and y. The nearest point is where the normal to the
/// ellipse passes through (y0, y1): not along the line to the centre, which
/// on a long thin ellipse is several times as far.
static double distanceInQuarter(double e0, double e1, double y0, double y1) {
    if (y1 > 0.0) {
        if (y0 > 0.0) {
            const double z0 = y0 / e0;
            const double z1 = y1 / e1;
            const double g = z0 * z0 + z1 * z1 - 1.0;
            if (g == 0.0) return 0.0;
            const double r0 = (e0 / e1) * (e0 / e1);
            const double s = ellipseRoot(r0, z0, z1, g);
            const double x0 = r0 * y0 / (s + r0);
            const double x1 = y1 / (s + 1.0);
            return std::hypot(x0 - y0, x1 - y1);
        }
        return std::abs(y1 - e1);  // on the short axis: its end
    }
    // On the long axis: inside, near enough the centre, the nearest point is
    // off the axis; else the axis's end.
    const double numer0 = e0 * y0;
    const double denom0 = e0 * e0 - e1 * e1;
    if (numer0 < denom0) {
        const double xde0 = numer0 / denom0;
        const double x0 = e0 * xde0;
        const double x1 = e1 * std::sqrt(1.0 - xde0 * xde0);
        return std::hypot(x0 - y0, x1);
    }
    return std::abs(y0 - e0);
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

DraftEllipse::DraftEllipse(const math::Vec2& center, double semiMajor, double semiMinor,
                           double rotation)
    : m_center(center), m_semiMajor(semiMajor), m_semiMinor(semiMinor), m_rotation(rotation) {}

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
        pts.push_back({m_center.x + lx * cosR - ly * sinR, m_center.y + lx * sinR + ly * cosR});
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
    // Avoid division by zero for degenerate ellipses.
    if (m_semiMajor < 1e-12 || m_semiMinor < 1e-12) return false;
    return distanceTo(point) <= tolerance;
}

double DraftEllipse::distanceTo(const math::Vec2& point) const {
    // In the ellipse's own frame (un-rotated), folded into the quarter where
    // both coordinates are positive: the nearest point is in the same
    // quarter as the point.
    const double cosR = std::cos(m_rotation);
    const double sinR = std::sin(m_rotation);
    const math::Vec2 v = point - m_center;
    double x = std::abs(v.x * cosR + v.y * sinR);
    double y = std::abs(-v.x * sinR + v.y * cosR);
    double a = std::abs(m_semiMajor);
    double b = std::abs(m_semiMinor);
    if (a < b) {
        std::swap(a, b);
        std::swap(x, y);
    }
    if (b < 1e-12) return std::hypot(std::max(x - a, 0.0), y);  // flat: a segment
    return distanceInQuarter(a, b, x, y);
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

std::vector<SnapPoint> DraftEllipse::typedSnapPoints() const {
    const auto points = snapPoints();
    std::vector<SnapPoint> out;
    out.reserve(points.size());
    for (const auto& p : points) out.push_back({p, SnapType::Quadrant});
    out.front().type = SnapType::Center;  // snapPoints() starts with the centre
    return out;
}

void DraftEllipse::translate(const math::Vec2& delta) {
    m_center += delta;
}

std::shared_ptr<DraftEntity> DraftEllipse::clone() const {
    auto copy = std::make_shared<DraftEllipse>(m_center, m_semiMajor, m_semiMinor, m_rotation);
    copyInto(*copy);
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
