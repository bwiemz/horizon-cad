#include "horizon/geometry/surfaces/NurbsSurface.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/math/Constants.h"

namespace hz::geo {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

NurbsSurface::NurbsSurface(std::vector<std::vector<math::Vec3>> controlPoints,
                           std::vector<std::vector<double>> weights, std::vector<double> knotsU,
                           std::vector<double> knotsV, int degreeU, int degreeV)
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

    // A weight for each control point, in as many rows: with fewer rows the
    // checks below read past the end.
    if (static_cast<int>(m_weights.size()) != numU) {
        throw std::invalid_argument("NurbsSurface needs a row of weights for each row of points");
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
        // As for a curve: a weight of zero or less makes NaNs, or a pole.
        for (const double w : m_weights[i]) {
            if (!std::isfinite(w) || !(w > 0.0)) {
                throw std::invalid_argument("NurbsSurface weights must be finite and positive");
            }
        }
    }

    // Validate knot vector lengths.
    const int expectedKnotsU = numU + m_degreeU + 1;
    const int expectedKnotsV = numV + m_degreeV + 1;

    if (static_cast<int>(m_knotsU.size()) != expectedKnotsU) {
        throw std::invalid_argument("NurbsSurface knotsU length must be numU + degreeU + 1");
    }
    if (static_cast<int>(m_knotsV.size()) != expectedKnotsV) {
        throw std::invalid_argument("NurbsSurface knotsV length must be numV + degreeV + 1");
    }
    for (const auto* knots : {&m_knotsU, &m_knotsV}) {
        for (size_t i = 0; i < knots->size(); ++i) {
            if (!std::isfinite((*knots)[i]) || (i > 0 && (*knots)[i] < (*knots)[i - 1])) {
                throw std::invalid_argument("NurbsSurface knots must be finite and non-decreasing");
            }
        }
    }
    if (!(uMin() < uMax()) || !(vMin() < vMax())) {
        throw std::invalid_argument("NurbsSurface parameter domain must not be empty");
    }
    for (const auto& row : m_controlPoints) {
        for (const auto& p : row) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                throw std::invalid_argument("NurbsSurface control points must be finite");
            }
        }
    }

    // Closed where the two edges coincide: sampled along them, to the
    // control net's size.
    double scale = 0.0;
    for (const auto& row : m_controlPoints) {
        for (const auto& p : row) scale = std::max(scale, (p - m_controlPoints[0][0]).length());
    }
    const double tol = 1e-12 * std::max(scale, 1.0);
    constexpr int kSamples = 7;
    m_closedU = true;
    m_closedV = true;
    for (int k = 0; k <= kSamples; ++k) {
        const double s = static_cast<double>(k) / kSamples;
        const double v = vMin() + (vMax() - vMin()) * s;
        const double u = uMin() + (uMax() - uMin()) * s;
        m_closedU = m_closedU && (evaluateWithDerivatives(uMin(), v).point -
                                  evaluateWithDerivatives(uMax(), v).point)
                                         .length() <= tol;
        m_closedV = m_closedV && (evaluateWithDerivatives(u, vMin()).point -
                                  evaluateWithDerivatives(u, vMax()).point)
                                         .length() <= tol;
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

    // Pass 1: for each row (U index), evaluate the V-direction curve at v.
    // A rational row curve returns the projected point
    //     C_i = sum_j N_j(v) w_ij P_ij / W_i,  W_i = sum_j N_j(v) w_ij,
    // so the second pass must carry W_i as the row's weight: then
    //     sum_i N_i(u) W_i C_i / sum_i N_i(u) W_i
    // is exactly the tensor-product rational surface.  Passing unit weights
    // instead drops the U-direction rationality, which put every point of a
    // cylinder, sphere or torus off its surface by up to a few percent.
    std::vector<math::Vec3> tempPts(numU);
    std::vector<double> tempWeights(numU, 1.0);

    const int numV = controlPointCountV();
    const std::vector<double> unitWeights(static_cast<size_t>(numV), 1.0);
    for (int i = 0; i < numU; ++i) {
        NurbsCurve rowCurve(m_controlPoints[i], m_weights[i], m_knotsV, m_degreeV);
        tempPts[i] = rowCurve.evaluate(v);

        // W_i is the plain B-spline of the row's weights, evaluated at v.
        std::vector<math::Vec3> weightPts;
        weightPts.reserve(static_cast<size_t>(numV));
        for (double w : m_weights[i]) weightPts.emplace_back(w, 0.0, 0.0);
        NurbsCurve weightCurve(std::move(weightPts), unitWeights, m_knotsV, m_degreeV);
        tempWeights[i] = weightCurve.evaluate(v).x;
    }

    // Pass 2: evaluate the U-direction rational curve through the row points.
    NurbsCurve uCurve(tempPts, tempWeights, m_knotsU, m_degreeU);
    return uCurve.evaluate(u);
}

// ---------------------------------------------------------------------------
// Derivatives — exact, from evaluateWithDerivatives
// ---------------------------------------------------------------------------

// They differenced evaluate() 1e-7 apart, so a normal was good to about 1e-7
// in direction, and at an edge of the domain taken from one side: what a flat
// face's normal is compared with to decide it is flat (1e-7) was the size of
// the error in the normals compared.
math::Vec3 NurbsSurface::derivativeU(double u, double v) const {
    return evaluateWithDerivatives(u, v).du;
}

math::Vec3 NurbsSurface::derivativeV(double u, double v) const {
    return evaluateWithDerivatives(u, v).dv;
}

// ---------------------------------------------------------------------------
// normal — cross product of partial derivatives
// ---------------------------------------------------------------------------

math::Vec3 NurbsSurface::normal(double u, double v) const {
    // The unit normal at (uu, vv), unless the surface comes to a point there
    // (a pole, an apex): its tangents vanish or lie along each other, measured
    // against the longer, so a small surface is judged as a large one.
    const auto unitAt = [this](double uu, double vv, math::Vec3& n) {
        const math::Vec3 du = derivativeU(uu, vv);
        const math::Vec3 dv = derivativeV(uu, vv);
        const math::Vec3 cross = du.cross(dv);
        const double len = cross.length();
        const double scale = std::max(du.length(), dv.length());
        if (!(len > 1e-9 * scale * scale) || !(len > 0.0)) return false;
        n = cross * (1.0 / len);
        return true;
    };
    math::Vec3 n;
    if (unitAt(u, v, n)) return n;
    // Where it comes to a point, its normal is the one it comes to: the
    // normal a step into the surface (a fixed +Z was the wrong way at a south
    // pole, and anywhere at a turned one).
    const double su = 1e-6 * (uMax() - uMin());
    const double sv = 1e-6 * (vMax() - vMin());
    const double uIn = u > 0.5 * (uMin() + uMax()) ? u - su : u + su;
    const double vIn = v > 0.5 * (vMin() + vMax()) ? v - sv : v + sv;
    if (unitAt(u, vIn, n) || unitAt(uIn, v, n) || unitAt(uIn, vIn, n)) return n;
    return math::Vec3::UnitZ;  // nothing near it is a surface either
}

// ---------------------------------------------------------------------------
// evaluateWithDerivatives — the rational basis and its first derivatives
// ---------------------------------------------------------------------------

namespace {

constexpr int kMaxOrder = 16;  // degree 15

/// The span [knots[s], knots[s + 1]) holding @p t, for @p count control
/// points of degree @p p (The NURBS Book, A2.1); the last span at the end.
int findSpan(const std::vector<double>& knots, int count, int p, double t) {
    if (t >= knots[static_cast<size_t>(count)]) {
        // The last non-empty span.
        int s = count - 1;
        while (s > p && knots[static_cast<size_t>(s)] >= knots[static_cast<size_t>(s) + 1]) --s;
        return s;
    }
    if (t <= knots[static_cast<size_t>(p)]) {
        int s = p;
        while (s < count - 1 && knots[static_cast<size_t>(s) + 1] <= t) ++s;
        return s;
    }
    int low = p;
    int high = count;
    int mid = (low + high) / 2;
    while (t < knots[static_cast<size_t>(mid)] || t >= knots[static_cast<size_t>(mid) + 1]) {
        if (t < knots[static_cast<size_t>(mid)]) {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    return mid;
}

/// The p + 1 basis functions non-zero at @p t in @p span, and their first
/// derivatives (The NURBS Book, A2.2 and A2.3 to the first derivative).
void basisAndDerivatives(const std::vector<double>& knots, int span, int p, double t, double* n,
                         double* dn) {
    double ndu[kMaxOrder][kMaxOrder];
    double left[kMaxOrder];
    double right[kMaxOrder];
    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = t - knots[static_cast<size_t>(span + 1 - j)];
        right[j] = knots[static_cast<size_t>(span + j)] - t;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];  // knot differences, below
            const double temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;  // basis functions, above
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    for (int r = 0; r <= p; ++r) {
        n[r] = ndu[r][p];
        double d = 0.0;
        if (r >= 1) d += ndu[r - 1][p - 1] / ndu[p][r - 1];
        if (r <= p - 1) d -= ndu[r][p - 1] / ndu[p][r];
        dn[r] = d * p;
    }
}

}  // namespace

SurfacePoint NurbsSurface::evaluateWithDerivatives(double u, double v) const {
    if (m_degreeU >= kMaxOrder || m_degreeV >= kMaxOrder) {
        // Past the fixed arrays: differences of evaluate(), one-sided at an
        // edge of the domain.
        constexpr double h = 1e-7;
        const auto difference = [&](double t, double lo, double hi, auto at) {
            const double t0 = std::max(t - h, lo);
            const double t1 = std::min(t + h, hi);
            return t1 - t0 < 1e-15 ? math::Vec3(0.0, 0.0, 0.0)
                                   : (at(t1) - at(t0)) * (1.0 / (t1 - t0));
        };
        const auto alongU = [&](double t) { return evaluate(t, v); };
        const auto alongV = [&](double t) { return evaluate(u, t); };
        return {evaluate(u, v), difference(u, uMin(), uMax(), alongU),
                difference(v, vMin(), vMax(), alongV)};
    }
    u = std::clamp(u, uMin(), uMax());
    v = std::clamp(v, vMin(), vMax());
    const int p = m_degreeU;
    const int q = m_degreeV;
    const int spanU = findSpan(m_knotsU, controlPointCountU(), p, u);
    const int spanV = findSpan(m_knotsV, controlPointCountV(), q, v);
    double nu[kMaxOrder];
    double dnu[kMaxOrder];
    double nv[kMaxOrder];
    double dnv[kMaxOrder];
    basisAndDerivatives(m_knotsU, spanU, p, u, nu, dnu);
    basisAndDerivatives(m_knotsV, spanV, q, v, nv, dnv);

    // The homogeneous point A / W and its derivatives.
    math::Vec3 a, au, av;
    double w = 0.0, wu = 0.0, wv = 0.0;
    for (int i = 0; i <= p; ++i) {
        const auto& row = m_controlPoints[static_cast<size_t>(spanU - p + i)];
        const auto& rowW = m_weights[static_cast<size_t>(spanU - p + i)];
        for (int j = 0; j <= q; ++j) {
            const size_t col = static_cast<size_t>(spanV - q + j);
            const double weight = rowW[col];
            const math::Vec3 wp = row[col] * weight;
            const double b = nu[i] * nv[j];
            const double bu = dnu[i] * nv[j];
            const double bv = nu[i] * dnv[j];
            a += wp * b;
            au += wp * bu;
            av += wp * bv;
            w += weight * b;
            wu += weight * bu;
            wv += weight * bv;
        }
    }
    const math::Vec3 point = a * (1.0 / w);
    return {point, (au - point * wu) * (1.0 / w), (av - point * wv) * (1.0 / w)};
}

std::pair<double, double> NurbsSurface::project(const math::Vec3& point, double u, double v) const {
    const double u0 = uMin(), u1 = uMax();
    const double v0 = vMin(), v1 = vMax();
    // Round a seam, or stopped at an edge.
    const auto keep = [](double t, double lo, double hi, bool closed) {
        if (!closed) return std::clamp(t, lo, hi);
        const double period = hi - lo;
        t = lo + std::fmod(t - lo, period);
        return t < lo ? t + period : t;
    };
    u = keep(u, u0, u1, m_closedU);
    v = keep(v, v0, v1, m_closedV);
    // A step that would leave the domain goes halfway to its edge instead:
    // clamped onto an edge where the surface degenerates (a pole, an apex),
    // the search stuck there, u no longer turning the point.
    const auto shorten = [](double t, double d, double lo, double hi, bool closed, double& alpha) {
        if (closed || d == 0.0) return;
        if (t + d < lo && t > lo) alpha = std::min(alpha, 0.5 * (t - lo) / -d);
        if (t + d > hi && t < hi) alpha = std::min(alpha, 0.5 * (hi - t) / d);
    };
    for (int iter = 0; iter < 100; ++iter) {
        const SurfacePoint s = evaluateWithDerivatives(u, v);
        const math::Vec3 r = s.point - point;
        // Gauss-Newton on |S - point|^2, damped a little so a pole (where
        // dS/du vanishes) does not make the system singular.
        const double a = s.du.dot(s.du);
        const double b = s.du.dot(s.dv);
        const double c = s.dv.dot(s.dv);
        const double damping = 1e-14 * (a + c) + 1e-300;
        const double det = (a + damping) * (c + damping) - b * b;
        if (!(det > 0.0)) break;
        const double gu = r.dot(s.du);
        const double gv = r.dot(s.dv);
        double du = -((c + damping) * gu - b * gv) / det;
        double dv = -((a + damping) * gv - b * gu) / det;
        double alpha = 1.0;
        shorten(u, du, u0, u1, m_closedU, alpha);
        shorten(v, dv, v0, v1, m_closedV, alpha);
        du *= alpha;
        dv *= alpha;
        // Still by the step, not by where it lands: on a seam a step of
        // 1e-17 lands a period away.
        const bool still = std::abs(du) <= 1e-15 * (u1 - u0) && std::abs(dv) <= 1e-15 * (v1 - v0);
        u = keep(u + du, u0, u1, m_closedU);
        v = keep(v + dv, v0, v1, m_closedV);
        if (still) break;
    }
    return {u, v};
}

std::pair<double, double> NurbsSurface::locate(const math::Vec3& point) const {
    constexpr int kGrid = 16;
    constexpr size_t kStarts = 4;
    struct Start {
        double distance;
        double u;
        double v;
    };
    std::vector<Start> starts;
    starts.reserve(static_cast<size_t>(kGrid * kGrid));
    const double du = (uMax() - uMin()) / kGrid;
    const double dv = (vMax() - vMin()) / kGrid;
    for (int i = 0; i < kGrid; ++i) {
        for (int j = 0; j < kGrid; ++j) {
            const double u = uMin() + du * (i + 0.5);
            const double v = vMin() + dv * (j + 0.5);
            starts.push_back({(evaluateWithDerivatives(u, v).point - point).lengthSquared(), u, v});
        }
    }
    std::partial_sort(starts.begin(), starts.begin() + static_cast<std::ptrdiff_t>(kStarts),
                      starts.end(),
                      [](const Start& a, const Start& b) { return a.distance < b.distance; });
    std::pair<double, double> best{starts.front().u, starts.front().v};
    double bestDistance = std::numeric_limits<double>::infinity();
    for (size_t k = 0; k < kStarts; ++k) {
        const auto uv = project(point, starts[k].u, starts[k].v);
        const double d =
            (evaluateWithDerivatives(uv.first, uv.second).point - point).lengthSquared();
        if (d < bestDistance) {
            bestDistance = d;
            best = uv;
        }
    }
    return best;
}

// ---------------------------------------------------------------------------
// Surface interface: second derivatives, invert, transformed
// ---------------------------------------------------------------------------

namespace {

/// The p + 1 basis functions non-zero at @p t in @p span and their first two
/// derivatives (The NURBS Book, A2.3 with n = 2): n[r], d1[r], d2[r].
void basisAndSecondDerivatives(const std::vector<double>& knots, int span, int p, double t,
                               double* n, double* d1, double* d2) {
    double ndu[kMaxOrder][kMaxOrder];
    double left[kMaxOrder];
    double right[kMaxOrder];
    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = t - knots[static_cast<size_t>(span + 1 - j)];
        right[j] = knots[static_cast<size_t>(span + j)] - t;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];
            const double temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    double* const ders[3] = {n, d1, d2};
    for (int j = 0; j <= p; ++j) {
        n[j] = ndu[j][p];
        d1[j] = 0.0;
        d2[j] = 0.0;
    }
    const int top = std::min(2, p);
    for (int r = 0; r <= p; ++r) {
        double a[2][3];
        int s1 = 0;
        int s2 = 1;
        a[0][0] = 1.0;
        for (int k = 1; k <= top; ++k) {
            double d = 0.0;
            const int rk = r - k;
            const int pk = p - k;
            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }
            const int j1 = rk >= -1 ? 1 : -rk;
            const int j2 = (r - 1 <= pk) ? k - 1 : p - r;
            for (int j = j1; j <= j2; ++j) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }
            if (r <= pk) {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }
            ders[k][r] = d;
            std::swap(s1, s2);
        }
    }
    double factor = p;
    for (int k = 1; k <= top; ++k) {
        for (int j = 0; j <= p; ++j) ders[k][j] *= factor;
        factor *= (p - k);
    }
}

}  // namespace

SurfaceDerivatives NurbsSurface::derivatives(double u, double v) const {
    if (m_degreeU >= kMaxOrder || m_degreeV >= kMaxOrder) {
        // Past the fixed arrays: differences of the first derivatives.
        constexpr double h = 1e-6;
        const auto first = [this](double a, double b) { return evaluateWithDerivatives(a, b); };
        const double u0 = std::max(u - h, uMin());
        const double u1 = std::min(u + h, uMax());
        const double v0 = std::max(v - h, vMin());
        const double v1 = std::min(v + h, vMax());
        const SurfacePoint c = first(u, v);
        SurfaceDerivatives d{c.point, c.du, c.dv, {}, {}, {}};
        if (u1 > u0) d.duu = (first(u1, v).du - first(u0, v).du) * (1.0 / (u1 - u0));
        if (v1 > v0) {
            d.dvv = (first(u, v1).dv - first(u, v0).dv) * (1.0 / (v1 - v0));
            d.duv = (first(u, v1).du - first(u, v0).du) * (1.0 / (v1 - v0));
        }
        return d;
    }
    u = std::clamp(u, uMin(), uMax());
    v = std::clamp(v, vMin(), vMax());
    const int p = m_degreeU;
    const int q = m_degreeV;
    const int spanU = findSpan(m_knotsU, controlPointCountU(), p, u);
    const int spanV = findSpan(m_knotsV, controlPointCountV(), q, v);
    double nu[kMaxOrder], dnu[kMaxOrder], ddnu[kMaxOrder];
    double nv[kMaxOrder], dnv[kMaxOrder], ddnv[kMaxOrder];
    basisAndSecondDerivatives(m_knotsU, spanU, p, u, nu, dnu, ddnu);
    basisAndSecondDerivatives(m_knotsV, spanV, q, v, nv, dnv, ddnv);

    // The weighted point A(u, v) and weight w(u, v), with their derivatives.
    math::Vec3 a, au, av, auu, auv, avv;
    double w = 0.0, wu = 0.0, wv = 0.0, wuu = 0.0, wuv = 0.0, wvv = 0.0;
    for (int i = 0; i <= p; ++i) {
        const auto& row = m_controlPoints[static_cast<size_t>(spanU - p + i)];
        const auto& rowW = m_weights[static_cast<size_t>(spanU - p + i)];
        for (int j = 0; j <= q; ++j) {
            const size_t col = static_cast<size_t>(spanV - q + j);
            const double weight = rowW[col];
            const math::Vec3 wp = row[col] * weight;
            const double b = nu[i] * nv[j];
            const double bu = dnu[i] * nv[j];
            const double bv = nu[i] * dnv[j];
            const double buu = ddnu[i] * nv[j];
            const double buv = dnu[i] * dnv[j];
            const double bvv = nu[i] * ddnv[j];
            a += wp * b;
            au += wp * bu;
            av += wp * bv;
            auu += wp * buu;
            auv += wp * buv;
            avv += wp * bvv;
            w += weight * b;
            wu += weight * bu;
            wv += weight * bv;
            wuu += weight * buu;
            wuv += weight * buv;
            wvv += weight * bvv;
        }
    }
    const double inv = 1.0 / w;
    const math::Vec3 s = a * inv;
    const math::Vec3 su = (au - s * wu) * inv;
    const math::Vec3 sv = (av - s * wv) * inv;
    const math::Vec3 suu = (auu - su * (2.0 * wu) - s * wuu) * inv;
    const math::Vec3 suv = (auv - su * wv - sv * wu - s * wuv) * inv;
    const math::Vec3 svv = (avv - sv * (2.0 * wv) - s * wvv) * inv;
    return {s, su, sv, suu, suv, svv};
}

math::Vec2 NurbsSurface::invert(const math::Vec3& point) const {
    auto uv = locate(point);
    // A periodic parameter lies in [min, max): the search may land on the
    // far edge of the seam, which is the start of the range.
    if (periodicU() && !(uv.first < uMax())) uv.first = uMin();
    if (periodicV() && !(uv.second < vMax())) uv.second = vMin();
    return {uv.first, uv.second};
}

std::unique_ptr<Surface> NurbsSurface::transformed(const math::Mat4& m) const {
    auto points = m_controlPoints;
    for (auto& row : points) {
        for (auto& p : row) p = m.transformPoint(p);
    }
    return std::make_unique<NurbsSurface>(std::move(points), m_weights, m_knotsU, m_knotsV,
                                          m_degreeU, m_degreeV);
}

// ---------------------------------------------------------------------------
// Knot insertion and splitting
// ---------------------------------------------------------------------------

namespace {

using Hom = std::array<double, 4>;  // (w x, w y, w z, w)
using HomGrid = std::vector<std::vector<Hom>>;

HomGrid toHomogeneous(const std::vector<std::vector<math::Vec3>>& points,
                      const std::vector<std::vector<double>>& weights, bool transpose) {
    const size_t nu = points.size();
    const size_t nv = points[0].size();
    HomGrid g(transpose ? nv : nu, std::vector<Hom>(transpose ? nu : nv));
    for (size_t i = 0; i < nu; ++i) {
        for (size_t j = 0; j < nv; ++j) {
            const double w = weights[i][j];
            const Hom h{points[i][j].x * w, points[i][j].y * w, points[i][j].z * w, w};
            (transpose ? g[j][i] : g[i][j]) = h;
        }
    }
    return g;
}

void fromHomogeneous(const HomGrid& g, bool transpose, std::vector<std::vector<math::Vec3>>& points,
                     std::vector<std::vector<double>>& weights) {
    const size_t nu = transpose ? g[0].size() : g.size();
    const size_t nv = transpose ? g.size() : g[0].size();
    points.assign(nu, std::vector<math::Vec3>(nv));
    weights.assign(nu, std::vector<double>(nv));
    for (size_t i = 0; i < nu; ++i) {
        for (size_t j = 0; j < nv; ++j) {
            const Hom& h = transpose ? g[j][i] : g[i][j];
            points[i][j] = {h[0] / h[3], h[1] / h[3], h[2] / h[3]};
            weights[i][j] = h[3];
        }
    }
}

/// Boehm's insertion of the knot @p t into @p knots, applied to each column of
/// the control net @p g (rows are along the parameter): the net gains a row.
/// @p t must lie strictly inside the domain.
void insertKnotRows(HomGrid& g, std::vector<double>& knots, int p, double t) {
    const int n = static_cast<int>(g.size());
    const int k = findSpan(knots, n, p, t);  // knots[k] <= t < knots[k + 1]
    HomGrid q(g.size() + 1);
    for (int i = 0; i <= n; ++i) {
        if (i <= k - p) {
            q[i] = g[i];
        } else if (i > k) {
            q[i] = g[i - 1];
        } else {
            const double denom = knots[i + p] - knots[i];
            const double alpha = denom > 0.0 ? (t - knots[i]) / denom : 0.0;
            q[i].resize(g[i].size());

            for (size_t c = 0; c < g[i].size(); ++c) {
                for (int d = 0; d < 4; ++d) {
                    q[i][c][d] = (1.0 - alpha) * g[i - 1][c][d] + alpha * g[i][c][d];
                }
            }
        }
    }
    knots.insert(knots.begin() + k + 1, t);
    g = std::move(q);
}

void requireInside(double t, double lo, double hi, const char* what) {
    if (!(t > lo && t < hi)) {
        throw std::invalid_argument(std::string("NurbsSurface ") + what +
                                    " must be strictly inside the parameter domain");
    }
}

}  // namespace

NurbsSurface NurbsSurface::insertKnotU(double u) const {
    requireInside(u, uMin(), uMax(), "knot");
    HomGrid g = toHomogeneous(m_controlPoints, m_weights, false);
    auto knots = m_knotsU;
    insertKnotRows(g, knots, m_degreeU, u);
    std::vector<std::vector<math::Vec3>> pts;
    std::vector<std::vector<double>> wts;
    fromHomogeneous(g, false, pts, wts);
    return {std::move(pts), std::move(wts), std::move(knots), m_knotsV, m_degreeU, m_degreeV};
}

NurbsSurface NurbsSurface::insertKnotV(double v) const {
    requireInside(v, vMin(), vMax(), "knot");
    HomGrid g = toHomogeneous(m_controlPoints, m_weights, true);  // rows along V
    auto knots = m_knotsV;
    insertKnotRows(g, knots, m_degreeV, v);
    std::vector<std::vector<math::Vec3>> pts;
    std::vector<std::vector<double>> wts;
    fromHomogeneous(g, true, pts, wts);
    return {std::move(pts), std::move(wts), m_knotsU, std::move(knots), m_degreeU, m_degreeV};
}

namespace {

/// Cut @p g (rows along the parameter, knots @p knots, degree @p p) at @p t:
/// insert it to multiplicity p, so the surface passes through a row of the net,
/// then take the rows and knots either side of it.
struct CutNet {
    HomGrid lowRows, highRows;
    std::vector<double> lowKnots, highKnots;
};

CutNet cutAt(HomGrid g, std::vector<double> knots, int p, double t) {
    int mult = static_cast<int>(std::count(knots.begin(), knots.end(), t));
    for (; mult < p; ++mult) insertKnotRows(g, knots, p, t);
    // t's run of mult knots starts at f; the surface at t is the row before
    // the run (a run of p + 1 leaves the surface discontinuous: the part
    // above starts at the next row).
    const int f = static_cast<int>(std::find(knots.begin(), knots.end(), t) - knots.begin());
    CutNet cut;
    cut.lowRows.assign(g.begin(), g.begin() + f);
    cut.highRows.assign(g.begin() + (f - 1 + (mult - p)), g.end());
    cut.lowKnots.assign(knots.begin(), knots.begin() + f);
    cut.lowKnots.insert(cut.lowKnots.end(), static_cast<size_t>(p + 1), t);
    cut.highKnots.assign(static_cast<size_t>(p + 1), t);
    cut.highKnots.insert(cut.highKnots.end(), knots.begin() + (f + mult), knots.end());
    return cut;
}

}  // namespace

std::pair<NurbsSurface, NurbsSurface> NurbsSurface::splitU(double u) const {
    requireInside(u, uMin(), uMax(), "split parameter");
    CutNet cut = cutAt(toHomogeneous(m_controlPoints, m_weights, false), m_knotsU, m_degreeU, u);
    std::vector<std::vector<math::Vec3>> lp, hp;
    std::vector<std::vector<double>> lw, hw;
    fromHomogeneous(cut.lowRows, false, lp, lw);
    fromHomogeneous(cut.highRows, false, hp, hw);
    return {NurbsSurface(std::move(lp), std::move(lw), std::move(cut.lowKnots), m_knotsV, m_degreeU,
                         m_degreeV),
            NurbsSurface(std::move(hp), std::move(hw), std::move(cut.highKnots), m_knotsV,
                         m_degreeU, m_degreeV)};
}

std::pair<NurbsSurface, NurbsSurface> NurbsSurface::splitV(double v) const {
    requireInside(v, vMin(), vMax(), "split parameter");
    CutNet cut = cutAt(toHomogeneous(m_controlPoints, m_weights, true), m_knotsV, m_degreeV, v);
    std::vector<std::vector<math::Vec3>> lp, hp;
    std::vector<std::vector<double>> lw, hw;
    fromHomogeneous(cut.lowRows, true, lp, lw);
    fromHomogeneous(cut.highRows, true, hp, hw);
    return {NurbsSurface(std::move(lp), std::move(lw), m_knotsU, std::move(cut.lowKnots), m_degreeU,
                         m_degreeV),
            NurbsSurface(std::move(hp), std::move(hw), m_knotsU, std::move(cut.highKnots),
                         m_degreeU, m_degreeV)};
}

// ---------------------------------------------------------------------------
// closestPoint — 8x8 grid search + 2D Newton iteration
// ---------------------------------------------------------------------------

std::pair<double, double> NurbsSurface::closestPoint(const math::Vec3& point, double tol) const {
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
    // Worked out as a double and clamped before it is an int: 10 / tolerance
    // for a tolerance of 0, or under about 5e-9, is past what an int holds,
    // and converting it was undefined (on x86 INT_MIN, so the finest
    // tolerance gave the coarsest mesh).
    const double wanted = tolerance > 0.0 ? 10.0 / tolerance : 200.0;
    const int res =
        static_cast<int>(std::clamp(std::isfinite(wanted) ? wanted : 200.0, 4.0, 200.0));

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
    for (int i = degree + 1; i < n; ++i) knots[i] = static_cast<double>(i - degree) / (n - degree);
    for (int i = n; i < n + degree + 1; ++i) knots[i] = 1.0;
    return knots;
}

}  // namespace

// ---------------------------------------------------------------------------
// makePlane — bilinear 2x2 surface
// ---------------------------------------------------------------------------

NurbsSurface NurbsSurface::reversedU() const {
    auto cps = controlPoints();
    auto wts = weights();
    std::reverse(cps.begin(), cps.end());
    std::reverse(wts.begin(), wts.end());
    const auto& k = knotsU();
    const double lo = k.front();
    const double hi = k.back();
    std::vector<double> rk(k.rbegin(), k.rend());
    for (double& v : rk) v = lo + hi - v;
    return {std::move(cps), std::move(wts), std::move(rk), knotsV(), degreeU(), degreeV()};
}

namespace {

/// Closed curves @p lines, the lines of a surface's net one way, each cut at
/// @p t and joined again where it closed: each starting at @p t, and all
/// with the same knots. Nothing where a line's two ends differ in weight.
std::optional<std::vector<NurbsCurve>> startedAt(const std::vector<NurbsCurve>& lines, double t) {
    std::vector<NurbsCurve> started;
    started.reserve(lines.size());
    for (const NurbsCurve& line : lines) {
        const double lo = line.tMin();
        const double hi = line.tMax();
        const int p = line.degree();
        const NurbsCurve after = line.segment(t, hi);
        const NurbsCurve before = line.segment(lo, t);
        if (!(after.tMin() > lo) || !(before.tMax() < hi)) return std::nullopt;  // not cut
        // One curve through the join: its point the same from both sides,
        // in weight too.
        const double wa = after.weights().back();
        const double wb = before.weights().front();
        if (std::abs(wa - wb) > 1e-12 * std::max(std::abs(wa), std::abs(wb))) return std::nullopt;
        const auto& ka = after.knots();
        const auto& kb = before.knots();
        const auto end = static_cast<std::ptrdiff_t>(p + 1);
        std::vector<double> knots(ka.begin(), ka.end() - end);
        knots.insert(knots.end(), static_cast<std::size_t>(p), hi);
        for (auto k = kb.begin() + end; k != kb.end(); ++k) knots.push_back(*k + (hi - lo));
        std::vector<math::Vec3> points = after.controlPoints();
        points.insert(points.end(), before.controlPoints().begin() + 1,
                      before.controlPoints().end());
        std::vector<double> weights = after.weights();
        weights.insert(weights.end(), before.weights().begin() + 1, before.weights().end());
        started.emplace_back(std::move(points), std::move(weights), std::move(knots), p);
        if (started.back().knots() != started.front().knots()) return std::nullopt;
    }
    return started;
}

}  // namespace

std::optional<NurbsSurface> NurbsSurface::startingAtU(double u) const {
    if (!m_closedU) return std::nullopt;
    const double eps = 1e-12 * std::max(1.0, uMax() - uMin());
    if (u <= uMin() + eps || u >= uMax() - eps) return *this;  // it closes there already
    const int numU = controlPointCountU();
    const int numV = controlPointCountV();
    std::vector<NurbsCurve> columns;
    columns.reserve(static_cast<std::size_t>(numV));
    for (int j = 0; j < numV; ++j) {
        std::vector<math::Vec3> points;
        std::vector<double> weights;
        for (int i = 0; i < numU; ++i) {
            points.push_back(m_controlPoints[i][j]);
            weights.push_back(m_weights[i][j]);
        }
        columns.emplace_back(std::move(points), std::move(weights), m_knotsU, m_degreeU);
    }
    const auto started = startedAt(columns, u);
    if (!started) return std::nullopt;
    const std::size_t rows = started->front().controlPoints().size();
    std::vector<std::vector<math::Vec3>> points(rows);
    std::vector<std::vector<double>> weights(rows);
    for (const NurbsCurve& column : *started) {
        for (std::size_t i = 0; i < rows; ++i) {
            points[i].push_back(column.controlPoints()[i]);
            weights[i].push_back(column.weights()[i]);
        }
    }
    return NurbsSurface(std::move(points), std::move(weights), started->front().knots(), m_knotsV,
                        m_degreeU, m_degreeV);
}

std::optional<NurbsSurface> NurbsSurface::startingAtV(double v) const {
    if (!m_closedV) return std::nullopt;
    const double eps = 1e-12 * std::max(1.0, vMax() - vMin());
    if (v <= vMin() + eps || v >= vMax() - eps) return *this;  // it closes there already
    std::vector<NurbsCurve> rows;
    rows.reserve(m_controlPoints.size());
    for (std::size_t i = 0; i < m_controlPoints.size(); ++i) {
        rows.emplace_back(m_controlPoints[i], m_weights[i], m_knotsV, m_degreeV);
    }
    const auto started = startedAt(rows, v);
    if (!started) return std::nullopt;
    std::vector<std::vector<math::Vec3>> points;
    std::vector<std::vector<double>> weights;
    for (const NurbsCurve& row : *started) {
        points.push_back(row.controlPoints());
        weights.push_back(row.weights());
    }
    return NurbsSurface(std::move(points), std::move(weights), m_knotsU, started->front().knots(),
                        m_degreeU, m_degreeV);
}

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

    return NurbsSurface(std::move(ctrlPts), std::move(wts), clampedKnots(2, 1), clampedKnots(2, 1),
                        1, 1);
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

    return NurbsSurface(std::move(ctrlPts), std::move(wts), circle.knots(), clampedKnots(2, 1),
                        circle.degree(), 1);
}

// ---------------------------------------------------------------------------
// makeSphere — 9x5 rational NURBS (revolve a semicircle around Z)
// ---------------------------------------------------------------------------

NurbsSurface NurbsSurface::makeSphereOctant(const math::Vec3& center, double radius,
                                            const math::Vec3& e1, const math::Vec3& e2,
                                            const math::Vec3& e3) {
    const double w = std::cos(math::kPi / 4.0);  // sqrt(2)/2, the weight of a quarter arc
    const double r = radius;
    // The profile, from the equator (along e1) to the pole (along e3): radial
    // distance from the e3 axis, height along it, weight.
    const double rho[3] = {r, r, 0.0};
    const double height[3] = {0.0, r, r};
    const double profileWts[3] = {1.0, w, 1.0};
    // The quarter turn from e1 to e2; its middle control point sits on the
    // corner of the square, as in any rational quarter circle.
    const math::Vec3 turn[3] = {e1, e1 + e2, e2};
    const double turnWts[3] = {1.0, w, 1.0};

    std::vector<std::vector<math::Vec3>> ctrlPts(3, std::vector<math::Vec3>(3));
    std::vector<std::vector<double>> wts(3, std::vector<double>(3));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            ctrlPts[i][j] = center + turn[i] * rho[j] + e3 * height[j];
            wts[i][j] = turnWts[i] * profileWts[j];
        }
    }
    const std::vector<double> knots = {0, 0, 0, 1, 1, 1};
    return NurbsSurface(std::move(ctrlPts), std::move(wts), knots, knots, 2, 2);
}

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
        {0.0, 0.0, r},   // north pole
        {r, 0.0, r},     // weighted point
        {r, 0.0, 0.0},   // equator
        {r, 0.0, -r},    // weighted point
        {0.0, 0.0, -r},  // south pole
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

    return NurbsSurface(std::move(ctrlPts), std::move(wts), std::move(knotsU), std::move(knotsV), 2,
                        2);
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
        {1, 0, 0},   {1, 0, 1},  {0, 0, 1},  {-1, 0, 1}, {-1, 0, 0},
        {-1, 0, -1}, {0, 0, -1}, {1, 0, -1}, {1, 0, 0},
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

    return NurbsSurface(std::move(ctrlPts), std::move(wts), std::move(knotsU), std::move(knotsV), 2,
                        2);
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
        ctrlPts[i][0] = apex;  // all apex points collapse
        ctrlPts[i][1] = basePts[i];
        wts[i][0] = baseWts[i];  // same weight pattern as base
        wts[i][1] = baseWts[i];
    }

    return NurbsSurface(std::move(ctrlPts), std::move(wts), baseCircle.knots(), clampedKnots(2, 1),
                        baseCircle.degree(), 1);
}

}  // namespace hz::geo
