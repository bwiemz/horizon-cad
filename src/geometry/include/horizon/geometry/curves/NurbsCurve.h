#pragma once

#include <limits>
#include <vector>

#include "horizon/math/Vec3.h"

namespace hz::geo {

/// Non-uniform rational B-spline (NURBS) curve.
///
/// Stores control points in 3D, per-point weights, a knot vector, and the
/// polynomial degree.  Evaluation uses the De Boor algorithm in homogeneous
/// coordinates so that rational weights are handled correctly.
class NurbsCurve {
public:
    NurbsCurve(std::vector<math::Vec3> controlPoints, std::vector<double> weights,
               std::vector<double> knots, int degree);

    // -- Accessors -----------------------------------------------------------

    int degree() const;
    int controlPointCount() const;
    const std::vector<math::Vec3>& controlPoints() const;
    const std::vector<double>& weights() const;
    const std::vector<double>& knots() const;

    /// Start of the useful parameter domain: knots[degree].
    double tMin() const;

    /// End of the useful parameter domain: knots[n] where n = controlPointCount().
    double tMax() const;

    // -- Evaluation ----------------------------------------------------------

    /// Evaluate the curve at parameter @p t using De Boor's algorithm.
    math::Vec3 evaluate(double t) const;

    // -- Derivatives & Tessellation (Task 2) ---------------------------------

    /// The n-th derivative at parameter @p t, exactly: from the basis
    /// functions' derivatives, and the quotient rule for the weights. At an
    /// end of the domain, the derivative from inside it. Order 0 or less is
    /// the point itself.
    math::Vec3 derivative(double t, int order = 1) const;

    /// Tessellate the curve to a polyline within the given chord tolerance.
    std::vector<math::Vec3> tessellate(double tolerance = 0.01) const;

    // -- Knot Insertion & Degree Elevation (Task 3) -------------------------

    /// Return a new curve with a knot inserted at parameter @p t (Boehm's algorithm).
    NurbsCurve insertKnot(double t) const;

    /// The part of the curve between @p t0 and @p t1 (either order; each
    /// clamped to the domain), exactly: the same points, on [t0, t1], each
    /// end a knot of full multiplicity.
    NurbsCurve segment(double t0, double t1) const;

    /// Return a new curve with polynomial degree raised by one.
    NurbsCurve elevateDegree() const;

    // -- Closest-Point & Arc-Length (Task 4) ----------------------------------

    /// Find the parameter of the closest point on the curve to @p point.
    /// Uses Newton iteration on f(t) = (C(t) - P) . C'(t) = 0.
    double closestPoint(const math::Vec3& point, double tol = 1e-8) const;

    /// Compute arc length between two parameter values using Simpson's rule.
    double arcLength(double tStart, double tEnd, int segments = 64) const;

    /// Return the parameter at a given arc-length from @p tStart (default, or
    /// NaN: tMin). A negative @p tStart is a parameter like any other: -1 as
    /// the default was, a curve on [-1, 1] measured from its start whatever
    /// was asked.
    double parameterAtLength(double length,
                             double tStart = std::numeric_limits<double>::quiet_NaN()) const;

    // -- Conic Factory Functions (Task 5) -----------------------------------

    /// Factory: create a full NURBS circle (degree-2 rational, 9 control points).
    static NurbsCurve makeCircle(const math::Vec3& center, double radius,
                                 const math::Vec3& normal = math::Vec3::UnitZ);

    /// Factory: create a NURBS circular arc.
    static NurbsCurve makeArc(const math::Vec3& center, double radius, double startAngle,
                              double endAngle, const math::Vec3& normal = math::Vec3::UnitZ);

    /// Factory: create a NURBS ellipse.
    static NurbsCurve makeEllipse(const math::Vec3& center, double semiMajor, double semiMinor,
                                  double rotation = 0.0,
                                  const math::Vec3& normal = math::Vec3::UnitZ);

private:
    std::vector<math::Vec3> m_controlPoints;
    std::vector<double> m_weights;
    std::vector<double> m_knots;
    int m_degree;

    /// Find the knot span index k such that knots[k] <= t < knots[k+1].
    int findKnotSpan(double t) const;
};

}  // namespace hz::geo
