#include "horizon/geometry/surfaces/AnalyticSurfaces.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hz::geo {

namespace {
constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kHalfPi = 1.5707963267948966192313216916398;

/// An angle normalised into [0, 2 pi); a tiny negative angle that rounds up to 2 pi gives 0.
double wrapAngle(double a) {
    if (a < 0.0) {
        a += kTwoPi;
    }
    return a >= kTwoPi ? 0.0 : a;
}

/// The unit radial direction cos u x + sin u y and its u derivative.
math::Vec3 radial(const Frame& f, double u) {
    return f.x * std::cos(u) + f.y * std::sin(u);
}
math::Vec3 tangent(const Frame& f, double u) {
    return f.y * std::cos(u) - f.x * std::sin(u);
}

/// The longitude of @p p (relative to the origin) in the frame; 0 on the axis.
double longitude(const Frame& f, const math::Vec3& p) {
    return wrapAngle(std::atan2(p.dot(f.y), p.dot(f.x)));
}

/// The distance of @p p from the frame's axis.
double radialDistance(const Frame& f, const math::Vec3& p) {
    return std::hypot(p.dot(f.x), p.dot(f.y));
}
}  // namespace

// ---- Plane ----

Plane::Plane(const Frame& frame, double scale) : frame_(frame), scale_(scale) {}

SurfaceKind Plane::kind() const {
    return SurfaceKind::Plane;
}

math::Vec3 Plane::evaluate(double u, double v) const {
    return frame_.origin + (frame_.x * u + frame_.y * v) * scale_;
}

SurfaceDerivatives Plane::derivatives(double u, double v) const {
    SurfaceDerivatives d;
    d.point = evaluate(u, v);
    d.du = frame_.x * scale_;
    d.dv = frame_.y * scale_;
    return d;  // second derivatives are zero (default-initialised Vec3)
}

Interval Plane::uRange() const {
    return {-kInf, kInf};
}
Interval Plane::vRange() const {
    return {-kInf, kInf};
}
bool Plane::periodicU() const {
    return false;
}
bool Plane::periodicV() const {
    return false;
}

math::Vec2 Plane::invert(const math::Vec3& point) const {
    const math::Vec3 p = point - frame_.origin;
    return {p.dot(frame_.x) / scale_, p.dot(frame_.y) / scale_};
}

std::unique_ptr<Surface> Plane::transformed(const math::Mat4& m) const {
    return std::make_unique<Plane>(frame_.transformed(m), scale_ * Frame::scaleOf(m));
}

// ---- Cylinder ----

Cylinder::Cylinder(const Frame& frame, double radius, double axialScale)
    : frame_(frame), radius_(radius), axialScale_(axialScale) {}

SurfaceKind Cylinder::kind() const {
    return SurfaceKind::Cylinder;
}

math::Vec3 Cylinder::evaluate(double u, double v) const {
    return frame_.origin + (frame_.x * std::cos(u) + frame_.y * std::sin(u)) * radius_ +
           frame_.z * (v * axialScale_);
}

SurfaceDerivatives Cylinder::derivatives(double u, double v) const {
    const double c = std::cos(u);
    const double s = std::sin(u);
    SurfaceDerivatives d;
    d.point = evaluate(u, v);
    d.du = (frame_.y * c - frame_.x * s) * radius_;
    d.dv = frame_.z * axialScale_;
    d.duu = (frame_.x * c + frame_.y * s) * -radius_;
    return d;  // duv and dvv are zero
}

Interval Cylinder::uRange() const {
    return {0.0, kTwoPi};
}
Interval Cylinder::vRange() const {
    return {-kInf, kInf};
}
bool Cylinder::periodicU() const {
    return true;
}
bool Cylinder::periodicV() const {
    return false;
}

math::Vec2 Cylinder::invert(const math::Vec3& point) const {
    const math::Vec3 p = point - frame_.origin;
    return {longitude(frame_, p), p.dot(frame_.z) / axialScale_};
}

std::unique_ptr<Surface> Cylinder::transformed(const math::Mat4& m) const {
    return std::make_unique<Cylinder>(frame_.transformed(m), radius_ * Frame::scaleOf(m),
                                      axialScale_ * Frame::scaleOf(m));
}

// ---- Cone ----

Cone::Cone(const Frame& frame, double radius, double halfAngle, double axialScale)
    : frame_(frame), radius_(radius), halfAngle_(halfAngle), axialScale_(axialScale) {}

SurfaceKind Cone::kind() const {
    return SurfaceKind::Cone;
}

math::Vec3 Cone::evaluate(double u, double v) const {
    const double rho = radius_ + v * axialScale_ * std::tan(halfAngle_);
    return frame_.origin + radial(frame_, u) * rho + frame_.z * (v * axialScale_);
}

SurfaceDerivatives Cone::derivatives(double u, double v) const {
    const double slope = axialScale_ * std::tan(halfAngle_);
    const double rho = radius_ + v * slope;
    const math::Vec3 e = radial(frame_, u);
    const math::Vec3 t = tangent(frame_, u);
    SurfaceDerivatives d;
    d.point = evaluate(u, v);
    d.du = t * rho;
    d.dv = e * slope + frame_.z * axialScale_;
    d.duu = e * -rho;
    d.duv = t * slope;
    return d;  // dvv is zero
}

math::Vec3 Cone::normal(double u, double v) const {
    const double slopeTerm = v * axialScale_ * std::tan(halfAngle_);
    const double rho = radius_ + slopeTerm;
    const double scale = std::max(std::abs(radius_), std::abs(slopeTerm));
    if (std::abs(rho) <= 1e-12 * scale) {
        return {0.0, 0.0, 0.0};  // the apex
    }
    return Surface::normal(u, v);
}

Interval Cone::uRange() const {
    return {0.0, kTwoPi};
}
Interval Cone::vRange() const {
    return {-kInf, kInf};
}
bool Cone::periodicU() const {
    return true;
}
bool Cone::periodicV() const {
    return false;
}

math::Vec2 Cone::invert(const math::Vec3& point) const {
    const math::Vec3 p = point - frame_.origin;
    const double q = radialDistance(frame_, p);
    const double h = p.dot(frame_.z);
    // Project (q, h) onto the generator line (r + w tan a, w), w = s v.
    const double t = std::tan(halfAngle_);
    const double w = ((q - radius_) * t + h) / (1.0 + t * t);
    return {longitude(frame_, p), w / axialScale_};
}

std::unique_ptr<Surface> Cone::transformed(const math::Mat4& m) const {
    const double k = Frame::scaleOf(m);
    return std::make_unique<Cone>(frame_.transformed(m), radius_ * k, halfAngle_, axialScale_ * k);
}

// ---- Sphere ----

Sphere::Sphere(const Frame& frame, double radius) : frame_(frame), radius_(radius) {}

SurfaceKind Sphere::kind() const {
    return SurfaceKind::Sphere;
}

math::Vec3 Sphere::evaluate(double u, double v) const {
    return frame_.origin + radial(frame_, u) * (radius_ * std::cos(v)) +
           frame_.z * (radius_ * std::sin(v));
}

SurfaceDerivatives Sphere::derivatives(double u, double v) const {
    const double cv = std::cos(v);
    const double sv = std::sin(v);
    const math::Vec3 e = radial(frame_, u);
    const math::Vec3 t = tangent(frame_, u);
    SurfaceDerivatives d;
    d.point = evaluate(u, v);
    d.du = t * (radius_ * cv);
    d.dv = (frame_.z * cv - e * sv) * radius_;
    d.duu = e * (-radius_ * cv);
    d.duv = t * (-radius_ * sv);
    d.dvv = (e * cv + frame_.z * sv) * -radius_;
    return d;
}

math::Vec3 Sphere::normal(double u, double v) const {
    const math::Vec3 n = (evaluate(u, v) - frame_.origin) / radius_;
    // du x dv points outward for a right-handed frame and inward for a mirrored one.
    return frame_.x.cross(frame_.y).dot(frame_.z) < 0.0 ? n * -1.0 : n;
}

Interval Sphere::uRange() const {
    return {0.0, kTwoPi};
}
Interval Sphere::vRange() const {
    return {-kHalfPi, kHalfPi};
}
bool Sphere::periodicU() const {
    return true;
}
bool Sphere::periodicV() const {
    return false;
}

math::Vec2 Sphere::invert(const math::Vec3& point) const {
    const math::Vec3 p = point - frame_.origin;
    return {longitude(frame_, p), std::atan2(p.dot(frame_.z), radialDistance(frame_, p))};
}

std::unique_ptr<Surface> Sphere::transformed(const math::Mat4& m) const {
    return std::make_unique<Sphere>(frame_.transformed(m), radius_ * Frame::scaleOf(m));
}

// ---- Torus ----

Torus::Torus(const Frame& frame, double majorRadius, double minorRadius)
    : frame_(frame), majorRadius_(majorRadius), minorRadius_(minorRadius) {}

SurfaceKind Torus::kind() const {
    return SurfaceKind::Torus;
}

math::Vec3 Torus::evaluate(double u, double v) const {
    const double rho = majorRadius_ + minorRadius_ * std::cos(v);
    return frame_.origin + radial(frame_, u) * rho + frame_.z * (minorRadius_ * std::sin(v));
}

SurfaceDerivatives Torus::derivatives(double u, double v) const {
    const double cv = std::cos(v);
    const double sv = std::sin(v);
    const double rho = majorRadius_ + minorRadius_ * cv;
    const math::Vec3 e = radial(frame_, u);
    const math::Vec3 t = tangent(frame_, u);
    SurfaceDerivatives d;
    d.point = evaluate(u, v);
    d.du = t * rho;
    d.dv = (frame_.z * cv - e * sv) * minorRadius_;
    d.duu = e * -rho;
    d.duv = t * (-minorRadius_ * sv);
    d.dvv = (e * cv + frame_.z * sv) * -minorRadius_;
    return d;
}

Interval Torus::uRange() const {
    return {0.0, kTwoPi};
}
Interval Torus::vRange() const {
    return {0.0, kTwoPi};
}
bool Torus::periodicU() const {
    return true;
}
bool Torus::periodicV() const {
    return true;
}

math::Vec2 Torus::invert(const math::Vec3& point) const {
    const math::Vec3 p = point - frame_.origin;
    const double v = std::atan2(p.dot(frame_.z), radialDistance(frame_, p) - majorRadius_);
    return {longitude(frame_, p), wrapAngle(v)};
}

std::unique_ptr<Surface> Torus::transformed(const math::Mat4& m) const {
    const double k = Frame::scaleOf(m);
    return std::make_unique<Torus>(frame_.transformed(m), majorRadius_ * k, minorRadius_ * k);
}

}  // namespace hz::geo
