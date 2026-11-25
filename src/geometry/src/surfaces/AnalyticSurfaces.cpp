#include "horizon/geometry/surfaces/AnalyticSurfaces.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "../AngleWrap.h"
#include "../RationalArc.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"

namespace hz::geo {

namespace {
using detail::wrapAngle;
using math::kHalfPi;
using math::kTwoPi;
constexpr double kInf = std::numeric_limits<double>::infinity();

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

/// A meridian of a surface turned about a frame's z axis: a rational B-spline in (radial, axial)
/// coordinates of the half-plane at each longitude.
struct Meridian {
    std::vector<math::Vec2> points;  ///< (distance from the axis, height along it)
    std::vector<double> weights;
    std::vector<double> knots;
    int degree = 1;
};

/// The straight meridian from (r0, h0) at v0 to (r1, h1) at v1, at one speed.
Meridian straightMeridian(double r0, double h0, double r1, double h1, double v0, double v1) {
    return {{{r0, h0}, {r1, h1}}, {1.0, 1.0}, {v0, v0, v1, v1}, 1};
}

/// The circular meridian centre + radius (cos v, sin v), v from v0 to v1 (rational arcs).
Meridian circularMeridian(const math::Vec2& centre, double radius, double v0, double v1) {
    detail::UnitArc arc = detail::unitArc(v0, v1, v0, v1);
    Meridian m;
    m.points.reserve(arc.points.size());
    for (const math::Vec2& p : arc.points) m.points.push_back(centre + p * radius);
    m.weights = std::move(arc.weights);
    m.knots = std::move(arc.knots);
    m.degree = 2;
    return m;
}

/// The surface origin + m.x (cos u x + sin u y) + m.y z of @p f, for u from @p u0 to @p u1 and
/// m the meridian, as a NURBS, exactly: the unit arc's rows round the axis (built from the frame's
/// own x and y, so a left-handed frame turns as its surface does) times the meridian's columns,
/// each weight the product of theirs. Its domain is [u0, u1] by the meridian's.
std::shared_ptr<NurbsSurface> revolvedMeridian(const Frame& f, double u0, double u1,
                                               const Meridian& m) {
    const detail::UnitArc arc = detail::unitArc(u0, u1, u0, u1);
    std::vector<std::vector<math::Vec3>> points(arc.points.size());
    std::vector<std::vector<double>> weights(arc.points.size());
    for (std::size_t i = 0; i < arc.points.size(); ++i) {
        const math::Vec3 out = f.x * arc.points[i].x + f.y * arc.points[i].y;
        for (std::size_t j = 0; j < m.points.size(); ++j) {
            points[i].push_back(f.origin + out * m.points[j].x + f.z * m.points[j].y);
            weights[i].push_back(arc.weights[i] * m.weights[j]);
        }
    }
    return std::make_shared<NurbsSurface>(std::move(points), std::move(weights), arc.knots, m.knots,
                                          2, m.degree);
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

std::shared_ptr<NurbsSurface> Plane::toNurbs(double u0, double u1, double v0, double v1) const {
    u1 = detail::checkedRangeEnd(u0, u1, 0.0, "Plane::toNurbs (u)");
    v1 = detail::checkedRangeEnd(v0, v1, 0.0, "Plane::toNurbs (v)");
    // A bilinear patch on a parallelogram: the plane's own parameters.
    std::vector<std::vector<math::Vec3>> points = {{evaluate(u0, v0), evaluate(u0, v1)},
                                                   {evaluate(u1, v0), evaluate(u1, v1)}};
    std::vector<std::vector<double>> weights(2, std::vector<double>(2, 1.0));
    return std::make_shared<NurbsSurface>(std::move(points), std::move(weights),
                                          std::vector<double>{u0, u0, u1, u1},
                                          std::vector<double>{v0, v0, v1, v1}, 1, 1);
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

std::shared_ptr<NurbsSurface> Cylinder::toNurbs(double u0, double u1, double v0, double v1) const {
    u1 = detail::checkedRangeEnd(u0, u1, kTwoPi, "Cylinder::toNurbs (u)");
    v1 = detail::checkedRangeEnd(v0, v1, 0.0, "Cylinder::toNurbs (v)");
    return revolvedMeridian(
        frame_, u0, u1,
        straightMeridian(radius_, v0 * axialScale_, radius_, v1 * axialScale_, v0, v1));
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
    const double u1 = longitude(frame_, p);
    const double t = std::tan(halfAngle_);
    // The generator line (r + w tan a, w), w = s v, runs through the apex into the other
    // nappe, which lies at longitude u + pi. Project (q, h) onto it in the meridian plane
    // at u1 (signed radius q) and at u1 + pi (signed radius -q); the nearer one wins.
    const auto project = [&](double signedQ, double& dist2) {
        const double w = ((signedQ - radius_) * t + h) / (1.0 + t * t);
        const double dr = signedQ - radius_ - w * t;
        const double dh = h - w;
        dist2 = dr * dr + dh * dh;
        return w;
    };
    double d1 = 0.0;
    double d2 = 0.0;
    const double w1 = project(q, d1);
    const double w2 = project(-q, d2);
    if (d2 < d1) {
        return {wrapAngle(u1 + kTwoPi / 2.0), w2 / axialScale_};
    }
    return {u1, w1 / axialScale_};
}

std::unique_ptr<Surface> Cone::transformed(const math::Mat4& m) const {
    const double k = Frame::scaleOf(m);
    return std::make_unique<Cone>(frame_.transformed(m), radius_ * k, halfAngle_, axialScale_ * k);
}

std::shared_ptr<NurbsSurface> Cone::toNurbs(double u0, double u1, double v0, double v1) const {
    u1 = detail::checkedRangeEnd(u0, u1, kTwoPi, "Cone::toNurbs (u)");
    v1 = detail::checkedRangeEnd(v0, v1, 0.0, "Cone::toNurbs (v)");
    // A generator is straight: the radius and the height both run with v.
    const double slope = axialScale_ * std::tan(halfAngle_);
    return revolvedMeridian(frame_, u0, u1,
                            straightMeridian(radius_ + v0 * slope, v0 * axialScale_,
                                             radius_ + v1 * slope, v1 * axialScale_, v0, v1));
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

std::shared_ptr<NurbsSurface> Sphere::toNurbs(double u0, double u1, double v0, double v1) const {
    u1 = detail::checkedRangeEnd(u0, u1, kTwoPi, "Sphere::toNurbs (u)");
    // v is a latitude: past a pole the meridian goes on round, so at most a turn of it.
    v1 = detail::checkedRangeEnd(v0, v1, kTwoPi, "Sphere::toNurbs (v)");
    return revolvedMeridian(frame_, u0, u1, circularMeridian({0.0, 0.0}, radius_, v0, v1));
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

std::shared_ptr<NurbsSurface> Torus::toNurbs(double u0, double u1, double v0, double v1) const {
    u1 = detail::checkedRangeEnd(u0, u1, kTwoPi, "Torus::toNurbs (u)");
    v1 = detail::checkedRangeEnd(v0, v1, kTwoPi, "Torus::toNurbs (v)");
    return revolvedMeridian(frame_, u0, u1,
                            circularMeridian({majorRadius_, 0.0}, minorRadius_, v0, v1));
}

}  // namespace hz::geo
