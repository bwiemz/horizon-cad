#include "horizon/geometry/surfaces/AnalyticSurfaces.h"

#include <cmath>
#include <limits>

namespace hz::geo {

namespace {
constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kInf = std::numeric_limits<double>::infinity();
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
    double u = std::atan2(p.dot(frame_.y), p.dot(frame_.x));  // 0 on the axis
    if (u < 0.0) {
        u += kTwoPi;
    }
    if (u >= kTwoPi) {
        u = 0.0;  // a tiny negative angle rounds up to 2 pi
    }
    return {u, p.dot(frame_.z) / axialScale_};
}

std::unique_ptr<Surface> Cylinder::transformed(const math::Mat4& m) const {
    return std::make_unique<Cylinder>(frame_.transformed(m), radius_ * Frame::scaleOf(m),
                                      axialScale_ * Frame::scaleOf(m));
}

}  // namespace hz::geo
