#include "horizon/geometry/surfaces/NurbsSurface.h"

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/math/Constants.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hz::geo {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

NurbsSurface::NurbsSurface(std::vector<std::vector<math::Vec3>> controlPoints,
                           std::vector<std::vector<double>> weights,
                           std::vector<double> knotsU, std::vector<double> knotsV,
                           int degreeU, int degreeV)
    : m_controlPoints(std::move(controlPoints)),
      m_weights(std::move(weights)),
      m_knotsU(std::move(knotsU)),
      m_knotsV(std::move(knotsV)),
      m_degreeU(degreeU),
      m_degreeV(degreeV) {
    const int numU = controlPointCountU();
    const int numV = controlPointCountV();

    if (numU < 2 || numV < 2) {
        throw std::invalid_argument("NurbsSurface requires at least 2x2 control points");
    }

    if (m_degreeU < 1 || m_degreeU >= numU) {
        throw std::invalid_argument("NurbsSurface degreeU must be in [1, numU-1]");
    }
    if (m_degreeV < 1 || m_degreeV >= numV) {
        throw std::invalid_argument("NurbsSurface degreeV must be in [1, numV-1]");
    }

    // Validate that all rows have the same number of columns.
    for (int i = 0; i < numU; ++i) {
        if (static_cast<int>(m_controlPoints[i].size()) != numV) {
            throw std::invalid_argument(
                "NurbsSurface control point rows must all have the same size");
        }
        if (static_cast<int>(m_weights[i].size()) != numV) {
            throw std::invalid_argument("NurbsSurface weight rows must all have the same size");
        }
    }

    // Validate knot vector lengths.
    const int expectedKnotsU = numU + m_degreeU + 1;
    const int expectedKnotsV = numV + m_degreeV + 1;

    if (static_cast<int>(m_knotsU.size()) != expectedKnotsU) {
        throw std::invalid_argument(
            "NurbsSurface knotsU length must be numU + degreeU + 1");
    }
    if (static_cast<int>(m_knotsV.size()) != expectedKnotsV) {
        throw std::invalid_argument(
            "NurbsSurface knotsV length must be numV + degreeV + 1");
    }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

int NurbsSurface::degreeU() const {
    return m_degreeU;
}

int NurbsSurface::degreeV() const {
    return m_degreeV;
}

int NurbsSurface::controlPointCountU() const {
    return static_cast<int>(m_controlPoints.size());
}

int NurbsSurface::controlPointCountV() const {
    if (m_controlPoints.empty()) return 0;
    return static_cast<int>(m_controlPoints[0].size());
}

const std::vector<std::vector<math::Vec3>>& NurbsSurface::controlPoints() const {
    return m_controlPoints;
}

const std::vector<std::vector<double>>& NurbsSurface::weights() const {
    return m_weights;
}

const std::vector<double>& NurbsSurface::knotsU() const {
    return m_knotsU;
}

const std::vector<double>& NurbsSurface::knotsV() const {
    return m_knotsV;
}

double NurbsSurface::uMin() const {
    return m_knotsU[m_degreeU];
}

double NurbsSurface::uMax() const {
    return m_knotsU[controlPointCountU()];
}

double NurbsSurface::vMin() const {
    return m_knotsV[m_degreeV];
}

double NurbsSurface::vMax() const {
    return m_knotsV[controlPointCountV()];
}

// ---------------------------------------------------------------------------
// evaluate — tensor-product De Boor (two-pass)
// ---------------------------------------------------------------------------

math::Vec3 NurbsSurface::evaluate(double u, double v) const {
    const int numU = controlPointCountU();

    // Pass 1: For each row (U index), evaluate the V-direction NURBS curve at v.
    std::vector<math::Vec3> tempPts(numU);
    std::vector<double> tempWeights(numU, 1.0);

    for (int i = 0; i < numU; ++i) {
        NurbsCurve rowCurve(m_controlPoints[i], m_weights[i], m_knotsV, m_degreeV);
        tempPts[i] = rowCurve.evaluate(v);
    }

    // Pass 2: Evaluate the U-direction NURBS curve using the temporary points.
    NurbsCurve uCurve(tempPts, tempWeights, m_knotsU, m_degreeU);
    return uCurve.evaluate(u);
}

// ---------------------------------------------------------------------------
// Derivatives — numerical differentiation
// ---------------------------------------------------------------------------

math::Vec3 NurbsSurface::derivativeU(double u, double v) const {
    const double h = 1e-7;
    const double u0 = std::max(u - h, uMin());
    const double u1 = std::min(u + h, uMax());
    const double actualH = u1 - u0;
    if (actualH < 1e-15) {
        return {0.0, 0.0, 0.0};
    }
    return (evaluate(u1, v) - evaluate(u0, v)) * (1.0 / actualH);
}

math::Vec3 NurbsSurface::derivativeV(double u, double v) const {
    const double h = 1e-7;
    const double v0 = std::max(v - h, vMin());
    const double v1 = std::min(v + h, vMax());
    const double actualH = v1 - v0;
    if (actualH < 1e-15) {
        return {0.0, 0.0, 0.0};
    }
    return (evaluate(u, v1) - evaluate(u, v0)) * (1.0 / actualH);
}

// ---------------------------------------------------------------------------
// normal — cross product of partial derivatives
// ---------------------------------------------------------------------------

math::Vec3 NurbsSurface::normal(double u, double v) const {
    const math::Vec3 du = derivativeU(u, v);
    const math::Vec3 dv = derivativeV(u, v);
    const math::Vec3 n = du.cross(dv);
    const double len = n.length();
    if (len < 1e-12) {
        return math::Vec3::UnitZ;
    }
    return n * (1.0 / len);
}

// ---------------------------------------------------------------------------
// closestPoint — 8x8 grid search + 2D Newton iteration
// ---------------------------------------------------------------------------

std::pair<double, double> NurbsSurface::closestPoint(const math::Vec3& point,
                                                      double tol) const {
    const double u0 = uMin();
    const double u1 = uMax();
    const double v0 = vMin();
    const double v1 = vMax();

    // Grid search for initial guess.
    constexpr int kGridRes = 8;
    double bestU = u0;
    double bestV = v0;
    double bestDistSq = (evaluate(u0, v0) - point).lengthSquared();

    for (int i = 0; i <= kGridRes; ++i) {
        const double u = u0 + (u1 - u0) * static_cast<double>(i) / kGridRes;
        for (int j = 0; j <= kGridRes; ++j) {
            const double v = v0 + (v1 - v0) * static_cast<double>(j) / kGridRes;
            const double distSq = (evaluate(u, v) - point).lengthSquared();
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                bestU = u;
                bestV = v;
            }
        }
    }

    // 2D Newton iteration on the gradient of distance-squared.
    double u = bestU;
    double v = bestV;

    for (int iter = 0; iter < 50; ++iter) {
        const math::Vec3 S = evaluate(u, v);
        const math::Vec3 diff = S - point;
        const math::Vec3 Su = derivativeU(u, v);
        const math::Vec3 Sv = derivativeV(u, v);

        // Gradient of distance-squared.
        const double fu = diff.dot(Su);
        const double fv = diff.dot(Sv);

        // Approximate Hessian (Gauss-Newton: ignore second derivatives).
        const double J00 = Su.dot(Su);
        const double J01 = Su.dot(Sv);
        const double J11 = Sv.dot(Sv);

        const double det = J00 * J11 - J01 * J01;
        if (std::abs(det) < 1e-15) {
            break;
        }

        // Solve 2x2 system: J * [du; dv] = -[fu; fv] via Cramer's rule.
        const double du = -(J11 * fu - J01 * fv) / det;
        const double dv = -(-J01 * fu + J00 * fv) / det;

        u = std::clamp(u + du, u0, u1);
        v = std::clamp(v + dv, v0, v1);

        if (std::abs(du) + std::abs(dv) < tol) {
            break;
        }
    }

    return {u, v};
}

// ---------------------------------------------------------------------------
// isoCurveU — fix u, return a degree-1 polyline along V
// ---------------------------------------------------------------------------

NurbsCurve NurbsSurface::isoCurveU(double u, int numSamples) const {
    const double v0 = vMin();
    const double v1 = vMax();
    const int n = std::max(numSamples, 2);

    std::vector<math::Vec3> pts(n);
    std::vector<double> wts(n, 1.0);

    for (int j = 0; j < n; ++j) {
        const double v = v0 + (v1 - v0) * static_cast<double>(j) / (n - 1);
        pts[j] = evaluate(u, v);
    }

    // Build clamped uniform knot vector for degree-1.
    std::vector<double> knots(n + 2);
    knots[0] = 0.0;
    knots[1] = 0.0;
    for (int i = 1; i < n - 1; ++i) {
        knots[i + 1] = static_cast<double>(i) / (n - 1);
    }
    knots[n] = 1.0;
    knots[n + 1] = 1.0;

    return NurbsCurve(std::move(pts), std::move(wts), std::move(knots), 1);
}

// ---------------------------------------------------------------------------
// isoCurveV — fix v, return a degree-1 polyline along U
// ---------------------------------------------------------------------------

NurbsCurve NurbsSurface::isoCurveV(double v, int numSamples) const {
    const double u0 = uMin();
    const double u1 = uMax();
    const int n = std::max(numSamples, 2);

    std::vector<math::Vec3> pts(n);
    std::vector<double> wts(n, 1.0);

    for (int i = 0; i < n; ++i) {
        const double u = u0 + (u1 - u0) * static_cast<double>(i) / (n - 1);
        pts[i] = evaluate(u, v);
    }

    // Build clamped uniform knot vector for degree-1.
    std::vector<double> knots(n + 2);
    knots[0] = 0.0;
    knots[1] = 0.0;
    for (int i = 1; i < n - 1; ++i) {
        knots[i + 1] = static_cast<double>(i) / (n - 1);
    }
    knots[n] = 1.0;
    knots[n + 1] = 1.0;

    return NurbsCurve(std::move(pts), std::move(wts), std::move(knots), 1);
}

// ---------------------------------------------------------------------------
// tessellate — fixed-resolution grid to triangle mesh
// ---------------------------------------------------------------------------

TessellationResult NurbsSurface::tessellate(double tolerance) const {
    int res = std::max(4, static_cast<int>(10.0 / tolerance));
    res = std::min(res, 200);

    const double u0 = uMin();
    const double u1 = uMax();
    const double v0 = vMin();
    const double v1 = vMax();

    const int numVerts = (res + 1) * (res + 1);

    TessellationResult result;
    result.positions.reserve(numVerts * 3);
    result.normals.reserve(numVerts * 3);
    result.indices.reserve(res * res * 6);

    // Generate vertices.
    for (int i = 0; i <= res; ++i) {
        const double u = u0 + (u1 - u0) * static_cast<double>(i) / res;
        for (int j = 0; j <= res; ++j) {
            const double v = v0 + (v1 - v0) * static_cast<double>(j) / res;

            const math::Vec3 pt = evaluate(u, v);
            result.positions.push_back(static_cast<float>(pt.x));
            result.positions.push_back(static_cast<float>(pt.y));
            result.positions.push_back(static_cast<float>(pt.z));

            const math::Vec3 n = normal(u, v);
            result.normals.push_back(static_cast<float>(n.x));
            result.normals.push_back(static_cast<float>(n.y));
            result.normals.push_back(static_cast<float>(n.z));
        }
    }

    // Generate triangle indices (two triangles per quad).
    for (int i = 0; i < res; ++i) {
        for (int j = 0; j < res; ++j) {
            const uint32_t idx00 = static_cast<uint32_t>(i * (res + 1) + j);
            const uint32_t idx10 = static_cast<uint32_t>((i + 1) * (res + 1) + j);
            const uint32_t idx01 = static_cast<uint32_t>(i * (res + 1) + (j + 1));
            const uint32_t idx11 = static_cast<uint32_t>((i + 1) * (res + 1) + (j + 1));

            // Triangle 1.
            result.indices.push_back(idx00);
            result.indices.push_back(idx10);
            result.indices.push_back(idx11);

            // Triangle 2.
            result.indices.push_back(idx00);
            result.indices.push_back(idx11);
            result.indices.push_back(idx01);
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
// Helpers for factory surfaces
// ---------------------------------------------------------------------------

namespace {

/// Rotate a point from the XY plane to an arbitrary plane defined by its normal.
math::Vec3 rotateToPlane(const math::Vec3& pt, const math::Vec3& planeNormal) {
    const math::Vec3 unitZ{0.0, 0.0, 1.0};
    const math::Vec3 n = planeNormal.normalized();
    const double d = n.dot(unitZ);

    if (d > 1.0 - 1e-12) {
        return pt;
    }
    if (d < -1.0 + 1e-12) {
        return {pt.x, -pt.y, -pt.z};
    }

    // Rodrigues' rotation formula.
    const math::Vec3 axis = unitZ.cross(n).normalized();
    const double cosA = d;
    const double sinA = std::sqrt(1.0 - d * d);
    return pt * cosA + axis.cross(pt) * sinA + axis * axis.dot(pt) * (1.0 - cosA);
}

/// Build a clamped uniform knot vector for n control points and given degree.
std::vector<double> clampedKnots(int n, int degree) {
    std::vector<double> knots(n + degree + 1);
    for (int i = 0; i <= degree; ++i) knots[i] = 0.0;
    for (int i = degree + 1; i < n; ++i)
        knots[i] = static_cast<double>(i - degree) / (n - degree);
    for (int i = n; i < n + degree + 1; ++i) knots[i] = 1.0;
    return knots;
}

}  // namespace

// ---------------------------------------------------------------------------
// makePlane — bilinear 2x2 surface
// ---------------------------------------------------------------------------

NurbsSurface NurbsSurface::makePlane(const math::Vec3& origin, const math::Vec3& uDir,
                                     const math::Vec3& vDir, double uSize, double vSize) {
    const math::Vec3 p00 = origin;
    const math::Vec3 p10 = origin + uDir * uSize;
    const math::Vec3 p01 = origin + vDir * vSize;
    const math::Vec3 p11 = origin + uDir * uSize + vDir * vSize;

    std::vector<std::vector<math::Vec3>> ctrlPts = {
        {p00, p01},
        {p10, p11},
    };
    std::vector<std::vector<double>> wts = {
        {1.0, 1.0},
        {1.0, 1.0},
    };

    return NurbsSurface(std::move(ctrlPts), std::move(wts),
                        clampedKnots(2, 1), clampedKnots(2, 1), 1, 1);
}

// ---------------------------------------------------------------------------
// makeCylinder — circular cross-section (U) extruded along axis (V)
// ---------------------------------------------------------------------------

NurbsSurface NurbsSurface::makeCylinder(const math::Vec3& center, const math::Vec3& axis,
                                         double radius, double height) {
    // Create a full circle in the plane perpendicular to the axis at the center.
    NurbsCurve circle = NurbsCurve::makeCircle(center, radius, axis.normalized());

    const auto& circlePts = circle.controlPoints();
    const auto& circleWts = circle.weights();
    const int numU = circle.controlPointCount();  // 9 for full circle

    const math::Vec3 axisDir = axis.normalized();
    const math::Vec3 offset = axisDir * height;

    // Surface: U = circular (degree 2), V = linear (degree 1).
    // Control grid: [numU][2] (bottom and top).
    std::vector<std::vector<math::Vec3>> ctrlPts(numU, std::vector<math::Vec3>(2));
    std::vector<std::vector<double>> wts(numU, std::vector<double>(2));

    for (int i = 0; i < numU; ++i) {
        ctrlPts[i][0] = circlePts[i];           // bottom
        ctrlPts[i][1] = circlePts[i] + offset;  // top
        wts[i][0] = circleWts[i];
        wts[i][1] = circleWts[i];
    }

    return NurbsSurface(std::move(ctrlPts), std::move(wts), circle.knots(),
                        clampedKnots(2, 1), circle.degree(), 1);
}

// ---------------------------------------------------------------------------
// makeSphere — 9x5 rational NURBS (revolve a semicircle around Z)
// ---------------------------------------------------------------------------

NurbsSurface NurbsSurface::makeSphere(const math::Vec3& center, double radius) {
    // The sphere is a surface of revolution: revolve a semicircular arc (V direction)
    // around the Z axis (U direction).
    //
    // Semicircle in the XZ plane from (0,0,r) down to (0,0,-r):
    // 5 control points (two 90-degree arcs, degree 2).
    const double w = std::cos(math::kPi / 4.0);  // sqrt(2)/2
    const double r = radius;

    // Semicircle profile control points in the XZ plane.
    // Arc 1: (0,0,r) -> (r,0,r) -> (r,0,0)     [top to equator]
    // Arc 2: (r,0,0) -> (r,0,-r) -> (0,0,-r)    [equator to bottom]
    const math::Vec3 profilePts[5] = {
        {0.0, 0.0, r},      // north pole
        {r,   0.0, r},      // weighted point
        {r,   0.0, 0.0},    // equator
        {r,   0.0, -r},     // weighted point
        {0.0, 0.0, -r},     // south pole
    };
    const double profileWts[5] = {1.0, w, 1.0, w, 1.0};

    // Full revolution around Z (U direction): 9 control points, degree 2.
    // Rotation angles at 0, 45, 90, 135, 180, 225, 270, 315, 360 degrees.
    const double cosAngles[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    const double sinAngles[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    const double revWts[9] = {1, w, 1, w, 1, w, 1, w, 1};

    const int numU = 9;
    const int numV = 5;

    std::vector<std::vector<math::Vec3>> ctrlPts(numU, std::vector<math::Vec3>(numV));
    std::vector<std::vector<double>> wts(numU, std::vector<double>(numV));

    for (int i = 0; i < numU; ++i) {
        for (int j = 0; j < numV; ++j) {
            // Revolve profile point around Z axis.
            const double px = profilePts[j].x;
            const double pz = profilePts[j].z;

            const double x = px * cosAngles[i];
            const double y = px * sinAngles[i];
            const double z = pz;

            ctrlPts[i][j] = math::Vec3{x, y, z} + center;
            wts[i][j] = revWts[i] * profileWts[j];
        }
    }

    // Knot vectors: same as a full circle for U (degree 2, 9 pts).
    std::vector<double> knotsU = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};
    // Semicircle for V: degree 2, 5 pts (two 90-degree arcs).
    std::vector<double> knotsV = {0, 0, 0, 0.5, 0.5, 1, 1, 1};

    return NurbsSurface(std::move(ctrlPts), std::move(wts), std::move(knotsU), std::move(knotsV),
                        2, 2);
}

// ---------------------------------------------------------------------------
// makeTorus — 9x9 rational NURBS (revolve a circle around Z)
// ---------------------------------------------------------------------------

NurbsSurface NurbsSurface::makeTorus(const math::Vec3& center, const math::Vec3& axis,
                                      double majorRadius, double minorRadius) {
    const double w = std::cos(math::kPi / 4.0);  // sqrt(2)/2

    // Cross-section circle in XZ plane, centered at (majorRadius, 0, 0).
    // This is a full circle with 9 control points (degree 2).
    const math::Vec3 minorPts[9] = {
        {1, 0, 0},  {1, 0, 1},  {0, 0, 1},  {-1, 0, 1},
        {-1, 0, 0}, {-1, 0, -1}, {0, 0, -1}, {1, 0, -1}, {1, 0, 0},
    };
    const double minorWts[9] = {1, w, 1, w, 1, w, 1, w, 1};

    // Revolution around Z: same 9-point structure.
    const double cosAngles[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
    const double sinAngles[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
    const double revWts[9] = {1, w, 1, w, 1, w, 1, w, 1};

    const int numU = 9;
    const int numV = 9;

    const math::Vec3 axisDir = axis.normalized();

    std::vector<std::vector<math::Vec3>> ctrlPts(numU, std::vector<math::Vec3>(numV));
    std::vector<std::vector<double>> wts(numU, std::vector<double>(numV));

    for (int i = 0; i < numU; ++i) {
        for (int j = 0; j < numV; ++j) {
            // Minor circle control point in local XZ frame, offset by majorRadius in X.
            const double localX = majorRadius + minorPts[j].x * minorRadius;
            const double localZ = minorPts[j].z * minorRadius;

            // Revolve around Z axis.
            const double x = localX * cosAngles[i];
            const double y = localX * sinAngles[i];
            const double z = localZ;

            // Rotate from Z-up to the desired axis, then translate.
            ctrlPts[i][j] = rotateToPlane({x, y, z}, axisDir) + center;
            wts[i][j] = revWts[i] * minorWts[j];
        }
    }

    std::vector<double> knotsU = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};
    std::vector<double> knotsV = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};

    return NurbsSurface(std::move(ctrlPts), std::move(wts), std::move(knotsU), std::move(knotsV),
                        2, 2);
}

// ---------------------------------------------------------------------------
// makeCone — circular cross-section (U) tapered linearly along axis (V)
// ---------------------------------------------------------------------------

NurbsSurface NurbsSurface::makeCone(const math::Vec3& apex, const math::Vec3& axis,
                                     double halfAngle, double height) {
    const math::Vec3 axisDir = axis.normalized();
    const double baseRadius = height * std::tan(halfAngle);

    // Create a full circle at the base.
    const math::Vec3 baseCenter = apex + axisDir * height;
    NurbsCurve baseCircle = NurbsCurve::makeCircle(baseCenter, baseRadius, axisDir);

    const auto& basePts = baseCircle.controlPoints();
    const auto& baseWts = baseCircle.weights();
    const int numU = baseCircle.controlPointCount();  // 9

    // Surface: U = circular (degree 2), V = linear (degree 1).
    // V=0 is the apex (all points collapse to apex), V=1 is the base circle.
    std::vector<std::vector<math::Vec3>> ctrlPts(numU, std::vector<math::Vec3>(2));
    std::vector<std::vector<double>> wts(numU, std::vector<double>(2));

    for (int i = 0; i < numU; ++i) {
        ctrlPts[i][0] = apex;      // all apex points collapse
        ctrlPts[i][1] = basePts[i];
        wts[i][0] = baseWts[i];    // same weight pattern as base
        wts[i][1] = baseWts[i];
    }

    return NurbsSurface(std::move(ctrlPts), std::move(wts), baseCircle.knots(),
                        clampedKnots(2, 1), baseCircle.degree(), 1);
}

}  // namespace hz::geo
