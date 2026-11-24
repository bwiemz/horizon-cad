#pragma once

#include <vector>

#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/math/Vec2.h"

namespace hz::geo {

/// A pcurve that is a NURBS: a 2D rational B-spline, held as a NurbsCurve with z = 0.
class Nurbs2D : public Curve2D {
public:
    Nurbs2D(std::vector<math::Vec2> controlPoints, std::vector<double> weights,
            std::vector<double> knots, int degree);

    math::Vec2 evaluate(double t) const override;
    math::Vec2 derivative(double t, int order = 1) const override;
    double tMin() const override;
    double tMax() const override;
    /// Whether the first and last points of the curve coincide.
    bool isClosed() const override;

    /// The 3D curve inside (z = 0 throughout).
    const NurbsCurve& curve() const { return curve_; }

private:
    NurbsCurve curve_;
};

}  // namespace hz::geo
