#pragma once

#include "horizon/geometry/curves/Curve3.h"

namespace hz::geo {

/// A line C(t) = origin + t * scale * direction, direction a unit vector. t is a distance
/// until a uniform scale is applied: transformed() then multiplies @c scale, so the
/// parameters stay put.
class LineCurve : public Curve3 {
public:
    /// @p direction is divided by its length (at any length) and must be non-zero.
    LineCurve(const math::Vec3& origin, const math::Vec3& direction, double scale = 1.0);

    CurveKind kind() const override;
    math::Vec3 evaluate(double t) const override;
    CurveDerivatives derivatives(double t) const override;
    Interval range() const override;
    bool periodic() const override;
    double closestParameter(const math::Vec3& point) const override;
    std::unique_ptr<Curve3> transformed(const math::Mat4& m) const override;
    std::shared_ptr<NurbsCurve> toNurbs(double t0, double t1) const override;

    const math::Vec3& origin() const { return origin_; }
    const math::Vec3& direction() const { return direction_; }
    double scale() const { return scale_; }

private:
    math::Vec3 origin_;
    math::Vec3 direction_;
    double scale_;
};

/// A circle C(t) = origin + r (cos t x + sin t y), t in [0, 2 pi), periodic.
class CircleCurve : public Curve3 {
public:
    CircleCurve(const Frame& frame, double radius);

    CurveKind kind() const override;
    math::Vec3 evaluate(double t) const override;
    CurveDerivatives derivatives(double t) const override;
    Interval range() const override;
    bool periodic() const override;
    double closestParameter(const math::Vec3& point) const override;
    std::unique_ptr<Curve3> transformed(const math::Mat4& m) const override;
    std::shared_ptr<NurbsCurve> toNurbs(double t0, double t1) const override;

    const Frame& frame() const { return frame_; }
    double radius() const { return radius_; }

private:
    Frame frame_;
    double radius_;
};

/// An ellipse C(t) = origin + a cos t x + b sin t y, t in [0, 2 pi), periodic. t is the
/// eccentric angle, not the polar one. Both radii must be positive.
class EllipseCurve : public Curve3 {
public:
    EllipseCurve(const Frame& frame, double majorRadius, double minorRadius);

    CurveKind kind() const override;
    math::Vec3 evaluate(double t) const override;
    CurveDerivatives derivatives(double t) const override;
    Interval range() const override;
    bool periodic() const override;
    /// The foot of the perpendicular from @p point; a point at the ellipse's centre has
    /// arbitrary feet, and gets 0.
    double closestParameter(const math::Vec3& point) const override;
    std::unique_ptr<Curve3> transformed(const math::Mat4& m) const override;
    std::shared_ptr<NurbsCurve> toNurbs(double t0, double t1) const override;

    const Frame& frame() const { return frame_; }
    double majorRadius() const { return major_; }
    double minorRadius() const { return minor_; }

private:
    Frame frame_;
    double major_;
    double minor_;
};

}  // namespace hz::geo
