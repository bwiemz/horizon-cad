#include "horizon/drafting/DraftSpline.h"
#include <algorithm>
#include <cmath>

namespace hz::draft {

DraftSpline::DraftSpline(const std::vector<math::Vec2>& controlPoints, bool closed)
    : m_controlPoints(controlPoints),
      m_weights(controlPoints.size(), 1.0),
      m_closed(closed) {}

void DraftSpline::syncWeights() {
    if (m_weights.size() != m_controlPoints.size()) {
        m_weights.resize(m_controlPoints.size(), 1.0);
    }
}

void DraftSpline::setControlPoints(const std::vector<math::Vec2>& points) {
    m_controlPoints = points;
    syncWeights();
}

void DraftSpline::setWeights(const std::vector<double>& weights) {
    m_weights = weights;
    // Ensure same size as control points; pad or truncate as needed.
    m_weights.resize(m_controlPoints.size(), 1.0);
}

bool DraftSpline::hasNonUniformWeights() const {
    for (double w : m_weights) {
        if (std::abs(w - 1.0) > 1e-12) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Uniform cubic B-spline evaluation
// ---------------------------------------------------------------------------

/// Evaluate one point on a uniform cubic B-spline span (non-rational).
/// Given 4 control points P0..P3 and parameter t in [0,1]:
///   B(t) = (1/6)[ (1-t)^3 P0 + (3t^3 - 6t^2 + 4) P1
///                + (-3t^3 + 3t^2 + 3t + 1) P2 + t^3 P3 ]
static math::Vec2 bsplinePt(const math::Vec2& p0, const math::Vec2& p1,
                              const math::Vec2& p2, const math::Vec2& p3, double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    double omt = 1.0 - t;
    double b0 = omt * omt * omt;
    double b1 = 3.0 * t3 - 6.0 * t2 + 4.0;
    double b2 = -3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0;
    double b3 = t3;
    double inv6 = 1.0 / 6.0;
    return {inv6 * (b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x),
            inv6 * (b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y)};
}

/// Evaluate one point on a rational cubic B-spline span (NURBS weights).
/// Uses homogeneous coordinates: each control point is lifted to (w*x, w*y, w),
/// the standard basis blend is applied, then projected back to 2D.
static math::Vec2 bsplinePtRational(const math::Vec2& p0, double w0,
                                     const math::Vec2& p1, double w1,
                                     const math::Vec2& p2, double w2,
                                     const math::Vec2& p3, double w3, double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    double omt = 1.0 - t;
    double b0 = omt * omt * omt;
    double b1 = 3.0 * t3 - 6.0 * t2 + 4.0;
    double b2 = -3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0;
    double b3 = t3;
    double inv6 = 1.0 / 6.0;
    // Homogeneous sum
    double hx = inv6 * (b0 * w0 * p0.x + b1 * w1 * p1.x + b2 * w2 * p2.x + b3 * w3 * p3.x);
    double hy = inv6 * (b0 * w0 * p0.y + b1 * w1 * p1.y + b2 * w2 * p2.y + b3 * w3 * p3.y);
    double hw = inv6 * (b0 * w0 + b1 * w1 + b2 * w2 + b3 * w3);
    if (std::abs(hw) < 1e-15) return {hx, hy};  // degenerate: return as-is
    return {hx / hw, hy / hw};
}

std::vector<math::Vec2> DraftSpline::evaluate(int segmentsPerSpan) const {
    const size_t n = m_controlPoints.size();
    if (n == 0) return {};
    if (n == 1) return {m_controlPoints[0]};

    // Fewer than 4 control points: degenerate to straight segments.
    if (n < 4 && !m_closed) {
        return m_controlPoints;
    }
    if (n < 3 && m_closed) {
        return m_controlPoints;
    }

    // Ensure weight vector is the right size (treat missing weights as 1.0).
    auto w = [this, n](size_t i) -> double {
        return (i < m_weights.size()) ? m_weights[i] : 1.0;
    };

    const bool rational = hasNonUniformWeights();
    std::vector<math::Vec2> pts;
    const int sps = std::max(segmentsPerSpan, 2);

    if (m_closed) {
        // Closed: n spans, indices wrap with modular arithmetic.
        pts.reserve(n * sps + 1);
        for (size_t span = 0; span < n; ++span) {
            size_t i0 = span % n;
            size_t i1 = (span + 1) % n;
            size_t i2 = (span + 2) % n;
            size_t i3 = (span + 3) % n;
            const auto& cp0 = m_controlPoints[i0];
            const auto& cp1 = m_controlPoints[i1];
            const auto& cp2 = m_controlPoints[i2];
            const auto& cp3 = m_controlPoints[i3];
            int count = (span + 1 < n) ? sps : sps + 1;  // last span includes endpoint
            for (int j = 0; j < count; ++j) {
                double t = static_cast<double>(j) / sps;
                if (rational) {
                    pts.push_back(bsplinePtRational(cp0, w(i0), cp1, w(i1),
                                                    cp2, w(i2), cp3, w(i3), t));
                } else {
                    pts.push_back(bsplinePt(cp0, cp1, cp2, cp3, t));
                }
            }
        }
    } else {
        // Open: n-3 spans.
        size_t spans = n - 3;
        pts.reserve(spans * sps + 1);
        for (size_t span = 0; span < spans; ++span) {
            const auto& cp0 = m_controlPoints[span];
            const auto& cp1 = m_controlPoints[span + 1];
            const auto& cp2 = m_controlPoints[span + 2];
            const auto& cp3 = m_controlPoints[span + 3];
            int count = (span + 1 < spans) ? sps : sps + 1;  // last span includes endpoint
            for (int j = 0; j < count; ++j) {
                double t = static_cast<double>(j) / sps;
                if (rational) {
                    pts.push_back(bsplinePtRational(cp0, w(span), cp1, w(span + 1),
                                                    cp2, w(span + 2), cp3, w(span + 3), t));
                } else {
                    pts.push_back(bsplinePt(cp0, cp1, cp2, cp3, t));
                }
            }
        }
    }

    return pts;
}

// ---------------------------------------------------------------------------
// DraftEntity virtuals
// ---------------------------------------------------------------------------

math::BoundingBox DraftSpline::boundingBox() const {
    auto pts = evaluate();
    if (pts.empty()) return {};

    double minX = pts[0].x, maxX = pts[0].x;
    double minY = pts[0].y, maxY = pts[0].y;
    for (size_t i = 1; i < pts.size(); ++i) {
        minX = std::min(minX, pts[i].x);
        minY = std::min(minY, pts[i].y);
        maxX = std::max(maxX, pts[i].x);
        maxY = std::max(maxY, pts[i].y);
    }
    return math::BoundingBox(math::Vec3(minX, minY, 0.0),
                             math::Vec3(maxX, maxY, 0.0));
}

bool DraftSpline::hitTest(const math::Vec2& point, double tolerance) const {
    auto pts = evaluate();
    if (pts.size() < 2) return false;

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

    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        if (segmentDist(point, pts[i], pts[i + 1]) <= tolerance)
            return true;
    }
    return false;
}

std::vector<math::Vec2> DraftSpline::snapPoints() const {
    // Snap to control points and evaluated start/end points.
    std::vector<math::Vec2> result;
    result.reserve(m_controlPoints.size() + 2);
    for (const auto& cp : m_controlPoints) {
        result.push_back(cp);
    }
    auto pts = evaluate();
    if (!pts.empty()) {
        result.push_back(pts.front());
        if (pts.size() > 1) {
            result.push_back(pts.back());
        }
    }
    return result;
}

void DraftSpline::translate(const math::Vec2& delta) {
    for (auto& cp : m_controlPoints) {
        cp += delta;
    }
}

std::shared_ptr<DraftEntity> DraftSpline::clone() const {
    auto copy = std::make_shared<DraftSpline>(m_controlPoints, m_closed);
    copy->setWeights(m_weights);
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

void DraftSpline::mirror(const math::Vec2& axisP1, const math::Vec2& axisP2) {
    for (auto& cp : m_controlPoints) {
        cp = mirrorPoint(cp, axisP1, axisP2);
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

void DraftSpline::rotate(const math::Vec2& center, double angle) {
    for (auto& cp : m_controlPoints) {
        cp = rotatePoint(cp, center, angle);
    }
}

void DraftSpline::scale(const math::Vec2& center, double factor) {
    for (auto& cp : m_controlPoints) {
        cp = scalePoint(cp, center, factor);
    }
}

}  // namespace hz::draft
