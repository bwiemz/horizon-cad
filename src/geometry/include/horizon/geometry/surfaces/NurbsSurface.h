#pragma once

#include "horizon/math/Vec3.h"

#include <vector>

namespace hz::geo {

/// Non-uniform rational B-spline (NURBS) tensor-product surface.
///
/// Stores a 2D grid of control points with per-point weights, knot vectors in
/// U and V directions, and polynomial degrees in U and V.  Evaluation uses a
/// two-pass De Boor algorithm: evaluate each row in V, then combine the results
/// in U.
class NurbsSurface {
public:
    /// Construct a NURBS surface.
    /// @param controlPoints  2D grid [row_u][col_v] of control points.
    /// @param weights        2D grid of weights matching the control point layout.
    /// @param knotsU         Knot vector in the U direction (length = numU + degreeU + 1).
    /// @param knotsV         Knot vector in the V direction (length = numV + degreeV + 1).
    /// @param degreeU        Polynomial degree in U.
    /// @param degreeV        Polynomial degree in V.
    NurbsSurface(std::vector<std::vector<math::Vec3>> controlPoints,
                 std::vector<std::vector<double>> weights, std::vector<double> knotsU,
                 std::vector<double> knotsV, int degreeU, int degreeV);

    // -- Accessors -----------------------------------------------------------

    int degreeU() const;
    int degreeV() const;
    int controlPointCountU() const;
    int controlPointCountV() const;

    const std::vector<std::vector<math::Vec3>>& controlPoints() const;
    const std::vector<std::vector<double>>& weights() const;
    const std::vector<double>& knotsU() const;
    const std::vector<double>& knotsV() const;

    /// Start of the parameter domain in U: knotsU[degreeU].
    double uMin() const;

    /// End of the parameter domain in U: knotsU[numU].
    double uMax() const;

    /// Start of the parameter domain in V: knotsV[degreeV].
    double vMin() const;

    /// End of the parameter domain in V: knotsV[numV].
    double vMax() const;

    // -- Evaluation ----------------------------------------------------------

    /// Evaluate the surface at parameters (u, v) using tensor-product De Boor.
    math::Vec3 evaluate(double u, double v) const;

    // -- Derivatives & Normal (Task 2) ----------------------------------------

    /// Partial derivative with respect to U at (u, v) via numerical differentiation.
    math::Vec3 derivativeU(double u, double v) const;

    /// Partial derivative with respect to V at (u, v) via numerical differentiation.
    math::Vec3 derivativeV(double u, double v) const;

    /// Unit surface normal at (u, v): normalize(dS/du x dS/dv).
    math::Vec3 normal(double u, double v) const;

    // -- Stubs for future tasks -----------------------------------------------

    /// Find the parameter pair (u, v) of the closest point on the surface to @p point.
    // std::pair<double, double> closestPoint(const math::Vec3& point, double tol = 1e-8) const;

    /// Extract an iso-curve at constant U.
    // NurbsCurve isoCurveU(double u) const;

    /// Extract an iso-curve at constant V.
    // NurbsCurve isoCurveV(double v) const;

    /// Tessellate the surface to a triangle mesh within the given tolerance.
    // std::vector<math::Vec3> tessellate(double tolerance = 0.01) const;

    // -- Factory stubs -------------------------------------------------------

    // static NurbsSurface makePlane(...);
    // static NurbsSurface makeCylinder(...);
    // static NurbsSurface makeSphere(...);
    // static NurbsSurface makeTorus(...);
    // static NurbsSurface makeRevolution(...);
    // static NurbsSurface makeExtrusion(...);

private:
    std::vector<std::vector<math::Vec3>> m_controlPoints;  // [row_u][col_v]
    std::vector<std::vector<double>> m_weights;             // same layout
    std::vector<double> m_knotsU;
    std::vector<double> m_knotsV;
    int m_degreeU;
    int m_degreeV;
};

}  // namespace hz::geo
