#include "horizon/geometry/surfaces/NurbsSurface.h"

#include "horizon/geometry/curves/NurbsCurve.h"

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
// Derivatives & Normal — stubs (implemented in Task 2)
// ---------------------------------------------------------------------------

math::Vec3 NurbsSurface::derivativeU(double /*u*/, double /*v*/) const {
    return {0.0, 0.0, 0.0};
}

math::Vec3 NurbsSurface::derivativeV(double /*u*/, double /*v*/) const {
    return {0.0, 0.0, 0.0};
}

math::Vec3 NurbsSurface::normal(double /*u*/, double /*v*/) const {
    return math::Vec3::UnitZ;
}

}  // namespace hz::geo
