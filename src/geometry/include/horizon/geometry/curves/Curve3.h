#pragma once

#include <memory>

#include "horizon/geometry/Frame.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"

namespace hz::geo {

/// The kind of a Curve3, for code that must treat the analytic ones specially.
enum class CurveKind { Line, Circle, Ellipse, Nurbs };

/// A curve point with its first and second derivatives.
struct CurveDerivatives {
    math::Vec3 point, d1, d2;
};

/// A parametric 3D curve C(t): the 3D edge curve of an exact B-rep.
class Curve3 {
public:
    virtual ~Curve3() = default;

    /// The kind of this curve.
    virtual CurveKind kind() const = 0;

    /// The point C(t).
    virtual math::Vec3 evaluate(double t) const = 0;

    /// The point and its first and second derivatives at @p t.
    virtual CurveDerivatives derivatives(double t) const = 0;

    /// The range of t; either end may be infinite.
    virtual Interval range() const = 0;

    /// Whether t wraps around; the period is the width of range().
    virtual bool periodic() const = 0;

    /// The parameter of the nearest point of the curve; a periodic t lies in [min, max).
    virtual double closestParameter(const math::Vec3& point) const = 0;

    /// This curve moved by the similarity @p m, keeping its parameters:
    /// result->evaluate(t) == m.transformPoint(evaluate(t)).
    virtual std::unique_ptr<Curve3> transformed(const math::Mat4& m) const = 0;
};

}  // namespace hz::geo
