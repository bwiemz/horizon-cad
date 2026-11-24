#pragma once

#include <memory>

#include "horizon/geometry/Frame.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"

namespace hz::geo {

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

    /// Unit du x dv; the zero vector where it is undefined (a pole, an apex).
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
};

}  // namespace hz::geo
