#include "horizon/geometry/curves/NurbsCurve.h"

#include "horizon/math/Tolerance.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <stdexcept>

namespace hz::geo {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

NurbsCurve::NurbsCurve(std::vector<math::Vec3> controlPoints, std::vector<double> weights,
                       std::vector<double> knots, int degree)
    : m_controlPoints(std::move(controlPoints)),
      m_weights(std::move(weights)),
      m_knots(std::move(knots)),
      m_degree(degree) {
    const int n = static_cast<int>(m_controlPoints.size());
    const int k = static_cast<int>(m_knots.size());

    if (n < 2) {
        throw std::invalid_argument("NurbsCurve requires at least 2 control points");
    }
    if (m_degree < 1 || m_degree >= n) {
        throw std::invalid_argument("NurbsCurve degree must be in [1, controlPointCount-1]");
    }
    if (static_cast<int>(m_weights.size()) != n) {
        throw std::invalid_argument("NurbsCurve weights size must match control point count");
    }
    // Knot vector length must be n + degree + 1.
    if (k != n + m_degree + 1) {
        throw std::invalid_argument("NurbsCurve knot vector length must be n + degree + 1");
    }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

int NurbsCurve::degree() const {
    return m_degree;
}

int NurbsCurve::controlPointCount() const {
    return static_cast<int>(m_controlPoints.size());
}

const std::vector<math::Vec3>& NurbsCurve::controlPoints() const {
    return m_controlPoints;
}

const std::vector<double>& NurbsCurve::weights() const {
    return m_weights;
}

const std::vector<double>& NurbsCurve::knots() const {
    return m_knots;
}

double NurbsCurve::tMin() const {
    return m_knots[m_degree];
}

double NurbsCurve::tMax() const {
    return m_knots[static_cast<int>(m_controlPoints.size())];
}

// ---------------------------------------------------------------------------
// findKnotSpan
// ---------------------------------------------------------------------------

int NurbsCurve::findKnotSpan(double t) const {
    const int n = static_cast<int>(m_controlPoints.size());

    // Special case: t at or beyond the end of the domain.
    if (t >= m_knots[n]) {
        return n - 1;
    }

    // Linear search from degree to n-1 for the span [knots[i], knots[i+1]).
    // (A binary search would be an optimization for very large knot vectors,
    // but for typical CAD curves the knot vector is small.)
    for (int i = m_degree; i < n; ++i) {
        if (t < m_knots[i + 1]) {
            return i;
        }
    }

    // Fallback — should not be reached for valid input.
    return n - 1;
}

// ---------------------------------------------------------------------------
// evaluate — De Boor's algorithm in homogeneous coordinates
// ---------------------------------------------------------------------------

math::Vec3 NurbsCurve::evaluate(double t) const {
    // Clamp t to the valid domain.
    t = std::clamp(t, tMin(), tMax());

    const int p = m_degree;
    const int k = findKnotSpan(t);

    // Build (p+1) homogeneous control points for the affected span.
    // Homogeneous point = (P.x * w, P.y * w, P.z * w, w).
    // We store them as 4-component arrays: [wx, wy, wz, w].
    const int count = p + 1;
    std::vector<std::array<double, 4>> d(count);

    for (int j = 0; j < count; ++j) {
        const int idx = k - p + j;
        const double w = m_weights[idx];
        const auto& pt = m_controlPoints[idx];
        d[j] = {pt.x * w, pt.y * w, pt.z * w, w};
    }

    // De Boor triangular computation.
    for (int r = 1; r <= p; ++r) {
        for (int j = count - 1; j >= r; --j) {
            const int i = k - p + j;  // original control-point index
            const double denom = m_knots[i + p + 1 - r] - m_knots[i];
            double alpha = 0.0;
            if (std::abs(denom) > math::Tolerance::kLinear) {
                alpha = (t - m_knots[i]) / denom;
            }
            const double oneMinusAlpha = 1.0 - alpha;
            for (int c = 0; c < 4; ++c) {
                d[j][c] = oneMinusAlpha * d[j - 1][c] + alpha * d[j][c];
            }
        }
    }

    // The result is in d[count-1].  Dehomogenise.
    const auto& h = d[count - 1];
    const double wInv = 1.0 / h[3];
    return {h[0] * wInv, h[1] * wInv, h[2] * wInv};
}

}  // namespace hz::geo
