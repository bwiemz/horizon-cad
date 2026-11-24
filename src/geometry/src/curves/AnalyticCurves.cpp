#include "horizon/geometry/curves/AnalyticCurves.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hz::geo {

namespace {
constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kInf = std::numeric_limits<double>::infinity();

/// An angle within one turn of [0, 2 pi) normalised into it; a tiny negative angle that rounds
/// up to 2 pi gives 0.
double wrapAngle(double a) {
    if (a < 0.0) {
        a += kTwoPi;
    }
    if (a >= kTwoPi) {
        a -= kTwoPi;
    }
    return a >= kTwoPi ? 0.0 : a;
}
}  // namespace

// ---- Line ----

LineCurve::LineCurve(const math::Vec3& origin, const math::Vec3& direction, double scale)
    : origin_(origin), direction_(direction.normalized()), scale_(scale) {}

CurveKind LineCurve::kind() const {
    return CurveKind::Line;
}

math::Vec3 LineCurve::evaluate(double t) const {
    return origin_ + direction_ * (t * scale_);
}

CurveDerivatives LineCurve::derivatives(double t) const {
    CurveDerivatives d;
    d.point = evaluate(t);
    d.d1 = direction_ * scale_;
    return d;  // the second derivative is zero
}

Interval LineCurve::range() const {
    return {-kInf, kInf};
}

bool LineCurve::periodic() const {
    return false;
}

double LineCurve::closestParameter(const math::Vec3& point) const {
    return (point - origin_).dot(direction_) / scale_;
}

std::unique_ptr<Curve3> LineCurve::transformed(const math::Mat4& m) const {
    const math::Vec3 d = m.transformDirection(direction_);
    return std::make_unique<LineCurve>(m.transformPoint(origin_), d, scale_ * d.length());
}

// ---- Circle ----

CircleCurve::CircleCurve(const Frame& frame, double radius) : frame_(frame), radius_(radius) {}

CurveKind CircleCurve::kind() const {
    return CurveKind::Circle;
}

math::Vec3 CircleCurve::evaluate(double t) const {
    return frame_.origin + (frame_.x * std::cos(t) + frame_.y * std::sin(t)) * radius_;
}

CurveDerivatives CircleCurve::derivatives(double t) const {
    CurveDerivatives d;
    d.point = evaluate(t);
    d.d1 = (frame_.y * std::cos(t) - frame_.x * std::sin(t)) * radius_;
    d.d2 = (frame_.x * std::cos(t) + frame_.y * std::sin(t)) * -radius_;
    return d;
}

Interval CircleCurve::range() const {
    return {0.0, kTwoPi};
}

bool CircleCurve::periodic() const {
    return true;
}

double CircleCurve::closestParameter(const math::Vec3& point) const {
    const math::Vec3 p = point - frame_.origin;
    const double px = p.dot(frame_.x);
    const double py = p.dot(frame_.y);
    if (px == 0.0 && py == 0.0) {
        return 0.0;
    }
    return wrapAngle(std::atan2(py, px));
}

std::unique_ptr<Curve3> CircleCurve::transformed(const math::Mat4& m) const {
    return std::make_unique<CircleCurve>(frame_.transformed(m), radius_ * Frame::scaleOf(m));
}

// ---- Ellipse ----

EllipseCurve::EllipseCurve(const Frame& frame, double majorRadius, double minorRadius)
    : frame_(frame), major_(majorRadius), minor_(minorRadius) {}

CurveKind EllipseCurve::kind() const {
    return CurveKind::Ellipse;
}

math::Vec3 EllipseCurve::evaluate(double t) const {
    return frame_.origin + frame_.x * (major_ * std::cos(t)) + frame_.y * (minor_ * std::sin(t));
}

CurveDerivatives EllipseCurve::derivatives(double t) const {
    CurveDerivatives d;
    d.point = evaluate(t);
    d.d1 = frame_.y * (minor_ * std::cos(t)) - frame_.x * (major_ * std::sin(t));
    d.d2 = (frame_.x * (major_ * std::cos(t)) + frame_.y * (minor_ * std::sin(t))) * -1.0;
    return d;
}

Interval EllipseCurve::range() const {
    return {0.0, kTwoPi};
}

bool EllipseCurve::periodic() const {
    return true;
}

double EllipseCurve::closestParameter(const math::Vec3& point) const {
    const math::Vec3 p = point - frame_.origin;
    const double px = p.dot(frame_.x);
    const double py = p.dot(frame_.y);
    if (px == 0.0 && py == 0.0) {
        return 0.0;
    }
    // In-plane squared distance (the out-of-plane part is the same for every t) as a function
    // of the eccentric angle: g(t) = (a cos t - px)^2 + (b sin t - py)^2.
    const auto dist2 = [&](double t) {
        const double dx = major_ * std::cos(t) - px;
        const double dy = minor_ * std::sin(t) - py;
        return dx * dx + dy * dy;
    };
    // Newton on f(t) = (C - P) . C', from a start; the step is halved while it does not
    // reduce the distance, so it cannot run off near the ends of the axes.
    const auto refine = [&](double t) {
        for (int iter = 0; iter < 60; ++iter) {
            const double c = std::cos(t);
            const double s = std::sin(t);
            const double dx = major_ * c - px;
            const double dy = minor_ * s - py;
            const double f = -dx * major_ * s + dy * minor_ * c;
            const double df = (major_ * major_ - minor_ * minor_) * (s * s - c * c) +
                              dx * -major_ * c - dy * minor_ * s;
            if (df == 0.0) {
                break;
            }
            double step = f / df;
            const double base = dist2(t);
            while (std::abs(step) > 1e-300 && dist2(t - step) > base) {
                step *= 0.5;
            }
            t -= step;
            if (std::abs(step) < 1e-15) {
                break;
            }
        }
        return t;
    };
    // The circle's estimate (exact for a point on the ellipse), and the best of a few
    // uniform samples, in case the estimate is far from the true foot.
    double starts[2] = {std::atan2(py / minor_, px / major_), 0.0};
    double best = kInf;
    for (int i = 0; i < 32; ++i) {
        const double t = kTwoPi * i / 32.0;
        if (dist2(t) < best) {
            best = dist2(t);
            starts[1] = t;
        }
    }
    double result = 0.0;
    double resultDist = kInf;
    for (double start : starts) {
        const double t = refine(start);
        if (dist2(t) < resultDist) {
            resultDist = dist2(t);
            result = t;
        }
    }
    return wrapAngle(std::fmod(result, kTwoPi));
}

std::unique_ptr<Curve3> EllipseCurve::transformed(const math::Mat4& m) const {
    const double k = Frame::scaleOf(m);
    return std::make_unique<EllipseCurve>(frame_.transformed(m), major_ * k, minor_ * k);
}

}  // namespace hz::geo
