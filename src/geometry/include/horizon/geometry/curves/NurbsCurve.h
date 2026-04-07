#pragma once

#include "horizon/math/Vec3.h"

#include <vector>

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

    /// Compute the n-th derivative at parameter @p t via numerical differentiation.
    math::Vec3 derivative(double t, int order = 1) const;

    /// Tessellate the curve to a polyline within the given chord tolerance.
    std::vector<math::Vec3> tessellate(double tolerance = 0.01) const;

    // -- Knot Insertion & Degree Elevation (Task 3) -------------------------

    /// Return a new curve with a knot inserted at parameter @p t (Boehm's algorithm).
    NurbsCurve insertKnot(double t) const;

    /// Return a new curve with polynomial degree raised by one.
    NurbsCurve elevateDegree() const;

    // -- Stubs for future tasks (declarations only) --------------------------

    /// Find the parameter of the closest point on the curve.  (Task 4)
    // double closestPoint(const math::Vec3& point) const;

    /// Compute arc length between two parameter values.  (Task 4)
    // double arcLength(double tStart, double tEnd) const;

    /// Return the parameter at a given arc-length from tMin.  (Task 4)
    // double parameterAtLength(double length) const;

    /// Factory: create a NURBS circle.  (Task 5)
    // static NurbsCurve makeCircle(const math::Vec3& center, double radius);

    /// Factory: create a NURBS circular arc.  (Task 5)
    // static NurbsCurve makeArc(const math::Vec3& center, double radius,
    //                           double startAngle, double endAngle);

    /// Factory: create a NURBS ellipse.  (Task 5)
    // static NurbsCurve makeEllipse(const math::Vec3& center,
    //                               double majorRadius, double minorRadius);

private:
    std::vector<math::Vec3> m_controlPoints;
    std::vector<double> m_weights;
    std::vector<double> m_knots;
    int m_degree;

    /// Find the knot span index k such that knots[k] <= t < knots[k+1].
    int findKnotSpan(double t) const;
};

}  // namespace hz::geo
