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

/// A cone about the frame's z axis, as STEP's CONICAL_SURFACE:
/// S(u, v) = origin + (r + s v tan a)(cos u x + sin u y) + s v z, with r the
/// radius at v = 0, a the half-angle, s the axial scale; u in [0, 2 pi) periodic,
/// v unbounded (the apex is at v = -r / (s tan a)). The axial scale is 1 unless
/// the cone came from a scaling transform, which keeps the parameters.
class Cone : public Surface {
public:
    /// A cone about the frame's z axis.
    Cone(const Frame& frame, double radius, double halfAngle, double axialScale = 1.0);

    /// The radius at v = 0.
    double radius() const { return radius_; }
    /// The half-angle between the axis and a generator.
    double halfAngle() const { return halfAngle_; }
    /// The frame the cone is parametrised by.
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
    /// The zero vector at the apex, where rounding leaves a radius of a few ulps
    /// instead of zero.
    math::Vec3 normal(double u, double v) const override;
    /// The nearest point of the generator line in the point's meridian plane;
    /// a point on the axis gives u = 0.
    math::Vec2 invert(const math::Vec3& point) const override;
    std::unique_ptr<Surface> transformed(const math::Mat4& m) const override;

private:
    Frame frame_;
    double radius_;
    double halfAngle_;
    double axialScale_;
};

/// A sphere: S(u, v) = origin + r cos v (cos u x + sin u y) + r sin v z, with u in
/// [0, 2 pi) periodic and v (the latitude) in [-pi/2, pi/2].
class Sphere : public Surface {
public:
    /// A sphere centred on the frame's origin.
    Sphere(const Frame& frame, double radius);

    /// The radius.
    double radius() const { return radius_; }
    /// The frame the sphere is parametrised by.
    const Frame& frame() const { return frame_; }

    SurfaceKind kind() const override;
    math::Vec3 evaluate(double u, double v) const override;
    SurfaceDerivatives derivatives(double u, double v) const override;
    Interval uRange() const override;
    Interval vRange() const override;
    bool periodicU() const override;
    bool periodicV() const override;
    /// The closed form (point - centre) / r, signed by the frame's handedness;
    /// defined at the poles too.
    math::Vec3 normal(double u, double v) const override;
    /// A point on the axis gives u = 0, the centre v = 0.
    math::Vec2 invert(const math::Vec3& point) const override;
    std::unique_ptr<Surface> transformed(const math::Mat4& m) const override;

private:
    Frame frame_;
    double radius_;
};

/// A torus: S(u, v) = origin + (R + r cos v)(cos u x + sin u y) + r sin v z, with
/// major radius R about the frame's z axis, minor radius r, and u and v both
/// periodic in [0, 2 pi).
class Torus : public Surface {
public:
    /// A torus about the frame's z axis.
    Torus(const Frame& frame, double majorRadius, double minorRadius);

    /// The radius of the circle the tube's centre follows.
    double majorRadius() const { return majorRadius_; }
    /// The radius of the tube.
    double minorRadius() const { return minorRadius_; }
    /// The frame the torus is parametrised by.
    const Frame& frame() const { return frame_; }

    SurfaceKind kind() const override;
    math::Vec3 evaluate(double u, double v) const override;
    SurfaceDerivatives derivatives(double u, double v) const override;
    Interval uRange() const override;
    Interval vRange() const override;
    bool periodicU() const override;
    bool periodicV() const override;
    /// The nearest point of the minor circle in the point's meridian plane; a
    /// point on the axis gives u = 0.
    math::Vec2 invert(const math::Vec3& point) const override;
    std::unique_ptr<Surface> transformed(const math::Mat4& m) const override;

private:
    Frame frame_;
    double majorRadius_;
    double minorRadius_;
};

}  // namespace hz::geo
