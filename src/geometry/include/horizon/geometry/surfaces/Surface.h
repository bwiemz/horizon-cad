#pragma once

#include <memory>

#include "horizon/geometry/Frame.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"

namespace hz::geo {

class NurbsSurface;

/// The kind of a Surface, for code that must treat the analytic ones specially.
enum class SurfaceKind { Plane, Cylinder, Cone, Sphere, Torus, Nurbs };

/// A surface point with its first and second partial derivatives.
struct SurfaceDerivatives {
    math::Vec3 point;
    math::Vec3 du, dv;
    math::Vec3 duu, duv, dvv;
};

/// A parametric surface S(u, v). A face's outward normal is du x dv (negated when
/// the face is reversed).
class Surface {
public:
    virtual ~Surface() = default;

    /// The kind of this surface.
    virtual SurfaceKind kind() const = 0;

    /// The point S(u, v).
    virtual math::Vec3 evaluate(double u, double v) const = 0;

    /// The point and its first and second partial derivatives at (u, v).
    virtual SurfaceDerivatives derivatives(double u, double v) const = 0;

    /// Unit du x dv. Where du x dv vanishes (a pole, an apex), the limit of the unit normal
    /// where one exists (a sphere's pole, a NURBS surface's degenerate edge), otherwise the zero
    /// vector (a cone's apex; this default, for a surface with no closed form).
    virtual math::Vec3 normal(double u, double v) const;

    /// The range of u.
    virtual Interval uRange() const = 0;
    /// The range of v.
    virtual Interval vRange() const = 0;

    /// Whether u wraps around; the period is the width of uRange().
    virtual bool periodicU() const = 0;
    /// Whether v wraps around; the period is the width of vRange().
    virtual bool periodicV() const = 0;

    /// Parameters of the nearest point of the surface; a periodic u lies in [min, max).
    virtual math::Vec2 invert(const math::Vec3& point) const = 0;

    /// This surface moved by the similarity @p m, keeping its parameters:
    /// result->evaluate(u, v) == m.transformPoint(evaluate(u, v)). A reflecting
    /// @p m flips the direction of du x dv.
    virtual std::unique_ptr<Surface> transformed(const math::Mat4& m) const = 0;

    /// This surface over the parameter box [u0, u1] x [v0, v1] as a rational NURBS, exactly: its
    /// points are this surface's over the box, its domain is the box, its corners are this
    /// surface's corners of the box, and its du x dv faces as this surface's does, so a face
    /// keeps its sense on it. Between its corners its parameters may differ from this surface's
    /// (a circle's rational arcs run at their own pace, meeting the angle at their knots), so a
    /// pcurve on this surface is not one on it. Unlike transformed(), the result moves exactly
    /// under any affine matrix (NurbsSurface::transformed).
    /// @throws std::invalid_argument for a box that is empty or not finite, or that runs more
    ///         than one period round a periodic direction.
    virtual std::shared_ptr<NurbsSurface> toNurbs(double u0, double u1, double v0,
                                                  double v1) const = 0;
};

}  // namespace hz::geo
