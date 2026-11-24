#pragma once

#include "horizon/geometry/surfaces/Surface.h"

namespace hz::geo {

/// A plane: S(u, v) = origin + scale (u x + v y), with u and v unbounded. The
/// scale is 1 unless the plane came from a scaling transform, which keeps the
/// parameters and so stretches the parameter step instead of the unit axes.
class Plane : public Surface {
public:
    /// A plane through the frame's origin spanned by its x and y axes.
    explicit Plane(const Frame& frame, double scale = 1.0);

    /// The length of one parameter step along the frame's axes.
    double scale() const { return scale_; }

    /// The frame the plane is parametrised by.
    const Frame& frame() const { return frame_; }

    SurfaceKind kind() const override;
    math::Vec3 evaluate(double u, double v) const override;
    SurfaceDerivatives derivatives(double u, double v) const override;
    Interval uRange() const override;
    Interval vRange() const override;
    bool periodicU() const override;
    bool periodicV() const override;
    math::Vec2 invert(const math::Vec3& point) const override;
    std::unique_ptr<Surface> transformed(const math::Mat4& m) const override;

private:
    Frame frame_;
    double scale_;
};

/// A cylinder of radius r about the frame's z axis:
/// S(u, v) = origin + r (cos u x + sin u y) + scale v z, with u in [0, 2 pi)
/// periodic and v unbounded. The axial scale is 1 unless the cylinder came from
/// a scaling transform, which keeps the parameters (the radius scales too).
class Cylinder : public Surface {
public:
    /// A cylinder about the frame's z axis.
    Cylinder(const Frame& frame, double radius, double axialScale = 1.0);

    /// The radius.
    double radius() const { return radius_; }
    /// The frame the cylinder is parametrised by.
    const Frame& frame() const { return frame_; }
    /// The length of one v step along the axis.
    double axialScale() const { return axialScale_; }

    SurfaceKind kind() const override;
    math::Vec3 evaluate(double u, double v) const override;
    SurfaceDerivatives derivatives(double u, double v) const override;
    Interval uRange() const override;
    Interval vRange() const override;
    bool periodicU() const override;
    bool periodicV() const override;
    /// A point on the axis gives u = 0.
    math::Vec2 invert(const math::Vec3& point) const override;
    std::unique_ptr<Surface> transformed(const math::Mat4& m) const override;

private:
    Frame frame_;
    double radius_;
    double axialScale_;
};

}  // namespace hz::geo
