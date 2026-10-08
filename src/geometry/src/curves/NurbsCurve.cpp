#include "horizon/geometry/curves/NurbsCurve.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "horizon/math/Constants.h"
#include "horizon/math/Tolerance.h"

namespace hz::geo {

namespace {

/// The p + 1 basis functions that are not zero at @p t, in knot span @p k,
/// and their derivatives up to order @p n <= p (The NURBS Book, A2.3):
/// result[r][j] is the r-th derivative of N_{k-p+j, p}(t). Exact, where the
/// finite differences that stood here were not: at an end of the domain the
/// second difference's sample beyond it was clamped onto the end, and the
/// second derivative of a straight line came out a million.
std::vector<std::vector<double>> basisDerivatives(const std::vector<double>& U, int k, int p,
                                                  double t, int n) {
    const auto sz = [](int i) { return static_cast<size_t>(i); };
    std::vector<std::vector<double>> ndu(sz(p + 1), std::vector<double>(sz(p + 1), 0.0));
    std::vector<double> left(sz(p + 1), 0.0);
    std::vector<double> right(sz(p + 1), 0.0);
    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[sz(j)] = t - U[sz(k + 1 - j)];
        right[sz(j)] = U[sz(k + j)] - t;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            ndu[sz(j)][sz(r)] = right[sz(r + 1)] + left[sz(j - r)];  // knot differences
            const double temp = ndu[sz(r)][sz(j - 1)] / ndu[sz(j)][sz(r)];
            ndu[sz(r)][sz(j)] = saved + right[sz(r + 1)] * temp;  // basis functions
            saved = left[sz(j - r)] * temp;
        }
        ndu[sz(j)][sz(j)] = saved;
    }

    std::vector<std::vector<double>> ders(sz(n + 1), std::vector<double>(sz(p + 1), 0.0));
    for (int j = 0; j <= p; ++j) ders[0][sz(j)] = ndu[sz(j)][sz(p)];
    std::vector<std::vector<double>> a(2, std::vector<double>(sz(p + 1), 0.0));
    for (int r = 0; r <= p; ++r) {
        int s1 = 0;
        int s2 = 1;
        a[0][0] = 1.0;
        for (int d = 1; d <= n; ++d) {
            double value = 0.0;
            const int rk = r - d;
            const int pk = p - d;
            if (r >= d) {
                a[sz(s2)][0] = a[sz(s1)][0] / ndu[sz(pk + 1)][sz(rk)];
                value = a[sz(s2)][0] * ndu[sz(rk)][sz(pk)];
            }
            const int j1 = rk >= -1 ? 1 : -rk;
            const int j2 = r - 1 <= pk ? d - 1 : p - r;
            for (int j = j1; j <= j2; ++j) {
                a[sz(s2)][sz(j)] =
                    (a[sz(s1)][sz(j)] - a[sz(s1)][sz(j - 1)]) / ndu[sz(pk + 1)][sz(rk + j)];
                value += a[sz(s2)][sz(j)] * ndu[sz(rk + j)][sz(pk)];
            }
            if (r <= pk) {
                a[sz(s2)][sz(d)] = -a[sz(s1)][sz(d - 1)] / ndu[sz(pk + 1)][sz(r)];
                value += a[sz(s2)][sz(d)] * ndu[sz(r)][sz(pk)];
            }
            ders[sz(d)][sz(r)] = value;
            std::swap(s1, s2);
        }
    }
    double factor = p;
    for (int d = 1; d <= n; ++d) {
        for (int j = 0; j <= p; ++j) ders[sz(d)][sz(j)] *= factor;
        factor *= p - d;
    }
    return ders;
}

}  // namespace

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
    // A weight of zero or less puts a pole on the curve, or divides by zero
    // where it is evaluated: a NaN that spreads silently through everything
    // built on it. STEP allows only positive weights too.
    for (const double w : m_weights) {
        if (!std::isfinite(w) || !(w > 0.0)) {
            throw std::invalid_argument("NurbsCurve weights must be finite and positive");
        }
    }
    for (const auto& p : m_controlPoints) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            throw std::invalid_argument("NurbsCurve control points must be finite");
        }
    }
    for (int i = 0; i < k; ++i) {
        if (!std::isfinite(m_knots[i]) || (i > 0 && m_knots[i] < m_knots[i - 1])) {
            throw std::invalid_argument("NurbsCurve knots must be finite and non-decreasing");
        }
    }
    if (!(tMin() < tMax())) {
        throw std::invalid_argument("NurbsCurve parameter domain must not be empty");
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

    // Special case: t at or beyond the end of the domain: the last span that
    // is not empty, so a curve whose end knot is repeated more than its
    // degree still has one to evaluate in.
    if (t >= m_knots[n]) {
        int k = n - 1;
        while (k > m_degree && !(m_knots[k] < m_knots[k + 1])) --k;
        return k;
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
            // Positive for every i of a span that is not empty, however close
            // its knots: against a length tolerance (1e-7) as it was, knots
            // 1e-8 apart read as one and the curve evaluated to its first
            // control point between them.
            const double denom = m_knots[i + p + 1 - r] - m_knots[i];
            double alpha = 0.0;
            if (denom > 0.0) {
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

// ---------------------------------------------------------------------------
// derivative — exact, from the basis functions' derivatives
// ---------------------------------------------------------------------------

math::Vec3 NurbsCurve::derivative(double t, int order) const {
    if (order <= 0) {
        return evaluate(t);
    }
    t = std::clamp(t, tMin(), tMax());
    const int p = m_degree;
    const int k = findKnotSpan(t);

    // Derivatives of the curve in homogeneous space, A(t) = sum N_j w_j P_j and
    // w(t) = sum N_j w_j, from the basis functions' derivatives; a
    // polynomial's derivatives past its degree are zero.
    const auto ders = basisDerivatives(m_knots, k, p, t, std::min(order, p));
    std::vector<math::Vec3> a(static_cast<size_t>(order) + 1, math::Vec3(0.0, 0.0, 0.0));
    std::vector<double> w(static_cast<size_t>(order) + 1, 0.0);
    for (size_t r = 0; r < ders.size(); ++r) {
        for (int j = 0; j <= p; ++j) {
            const size_t idx = static_cast<size_t>(k - p + j);
            const double nw = ders[r][static_cast<size_t>(j)] * m_weights[idx];
            a[r] = a[r] + m_controlPoints[idx] * nw;
            w[r] += nw;
        }
    }

    // The quotient rule for C = A / w, order by order (The NURBS Book, A4.2):
    // C^(r) = (A^(r) - sum_{i=1..r} binom(r, i) w^(i) C^(r-i)) / w.
    std::vector<math::Vec3> c(static_cast<size_t>(order) + 1);
    for (int r = 0; r <= order; ++r) {
        math::Vec3 v = a[static_cast<size_t>(r)];
        double binom = 1.0;
        for (int i = 1; i <= r; ++i) {
            binom = binom * (r - i + 1) / i;
            v = v - c[static_cast<size_t>(r - i)] * (binom * w[static_cast<size_t>(i)]);
        }
        c[static_cast<size_t>(r)] = v * (1.0 / w[0]);
    }
    return c[static_cast<size_t>(order)];
}

namespace {

void tessellateRecursive(const NurbsCurve& crv, double t0, const math::Vec3& p0, double t1,
                         const math::Vec3& p1, double tolerance, int depth, int maxDepth,
                         std::vector<math::Vec3>& result) {
    const double tMid = (t0 + t1) * 0.5;
    const math::Vec3 pMid = crv.evaluate(tMid);
    const math::Vec3 chordMid = (p0 + p1) * 0.5;

    const bool needsSubdivision =
        depth < 3 || (pMid.distanceTo(chordMid) > tolerance && depth < maxDepth);

    if (needsSubdivision) {
        tessellateRecursive(crv, t0, p0, tMid, pMid, tolerance, depth + 1, maxDepth, result);
        tessellateRecursive(crv, tMid, pMid, t1, p1, tolerance, depth + 1, maxDepth, result);
    } else {
        result.push_back(p1);
    }
}

}  // namespace

std::vector<math::Vec3> NurbsCurve::tessellate(double tolerance) const {
    std::vector<math::Vec3> result;
    const math::Vec3 pStart = evaluate(tMin());
    const math::Vec3 pEnd = evaluate(tMax());
    result.push_back(pStart);
    tessellateRecursive(*this, tMin(), pStart, tMax(), pEnd, tolerance, 0, 16, result);
    return result;
}

// ---------------------------------------------------------------------------
// insertKnot — Boehm's algorithm
// ---------------------------------------------------------------------------

NurbsCurve NurbsCurve::insertKnot(double t) const {
    t = std::clamp(t, tMin(), tMax());

    const int p = m_degree;
    const int n = static_cast<int>(m_controlPoints.size());  // number of control points
    const int k = findKnotSpan(t);                           // knots[k] <= t < knots[k+1]

    // Build new knot vector (one extra knot).
    std::vector<double> newKnots;
    newKnots.reserve(m_knots.size() + 1);
    for (int i = 0; i <= k; ++i) newKnots.push_back(m_knots[i]);
    newKnots.push_back(t);
    for (int i = k + 1; i < static_cast<int>(m_knots.size()); ++i) newKnots.push_back(m_knots[i]);

    // Convert to homogeneous coordinates: (wx, wy, wz, w).
    std::vector<std::array<double, 4>> Pw(n);
    for (int i = 0; i < n; ++i) {
        const double w = m_weights[i];
        Pw[i] = {m_controlPoints[i].x * w, m_controlPoints[i].y * w, m_controlPoints[i].z * w, w};
    }

    // Compute new control points in homogeneous coords.
    std::vector<std::array<double, 4>> Qw(n + 1);

    // Points that are unchanged at the beginning.
    for (int i = 0; i <= k - p; ++i) {
        Qw[i] = Pw[i];
    }

    // Points in the affected region.
    for (int i = k - p + 1; i <= k; ++i) {
        const double denom = m_knots[i + p] - m_knots[i];
        double alpha = 0.0;
        if (std::abs(denom) > 1e-15) {
            alpha = (t - m_knots[i]) / denom;
        }
        for (int c = 0; c < 4; ++c) {
            Qw[i][c] = (1.0 - alpha) * Pw[i - 1][c] + alpha * Pw[i][c];
        }
    }

    // Points that are shifted at the end.
    for (int i = k + 1; i <= n; ++i) {
        Qw[i] = Pw[i - 1];
    }

    // Dehomogenise new control points.
    std::vector<math::Vec3> newPts(n + 1);
    std::vector<double> newWts(n + 1);
    for (int i = 0; i <= n; ++i) {
        const double w = Qw[i][3];
        newWts[i] = w;
        const double wInv = 1.0 / w;
        newPts[i] = {Qw[i][0] * wInv, Qw[i][1] * wInv, Qw[i][2] * wInv};
    }

    return NurbsCurve(std::move(newPts), std::move(newWts), std::move(newKnots), p);
}

// ---------------------------------------------------------------------------
// elevateDegree — Bezier degree elevation
// ---------------------------------------------------------------------------

NurbsCurve NurbsCurve::segment(double t0, double t1) const {
    t0 = std::clamp(t0, tMin(), tMax());
    t1 = std::clamp(t1, tMin(), tMax());
    if (t1 < t0) std::swap(t0, t1);
    const int p = m_degree;
    // Each end a knot of multiplicity p (a knot already there taken as it),
    // so the curve passes through a control point there.
    const double eps = 1e-12 * std::max(1.0, tMax() - tMin());
    NurbsCurve c = *this;
    for (double* t : {&t0, &t1}) {
        for (const double k : c.m_knots) {
            if (std::abs(k - *t) <= eps) *t = k;
        }
        const auto multiplicity = [&c, t] {
            return static_cast<int>(std::count(c.m_knots.begin(), c.m_knots.end(), *t));
        };
        for (int m = multiplicity(); m < p; ++m) c = c.insertKnot(*t);
    }
    if (!(t1 > t0)) return *this;  // no span: nothing to cut out
    // C(t) is the control point before a run of p knots equal to t: t0's
    // run ends at `last0`, t1's starts at `first1`.
    const auto& knots = c.m_knots;
    const int last0 =
        static_cast<int>(
            std::find_if(knots.rbegin(), knots.rend(), [t0](double k) { return k == t0; }).base() -
            knots.begin()) -
        1;
    const int first1 = static_cast<int>(std::find(knots.begin(), knots.end(), t1) - knots.begin());
    const int from = last0 - p;
    const int to = first1 - 1;
    if (from < 0 || to >= static_cast<int>(c.m_controlPoints.size()) || to < from) return *this;
    std::vector<math::Vec3> points(c.m_controlPoints.begin() + from,
                                   c.m_controlPoints.begin() + to + 1);
    std::vector<double> weights(c.m_weights.begin() + from, c.m_weights.begin() + to + 1);
    std::vector<double> segmentKnots(static_cast<std::size_t>(p + 1), t0);
    for (const double k : knots) {
        if (k > t0 && k < t1) segmentKnots.push_back(k);
    }
    segmentKnots.insert(segmentKnots.end(), static_cast<std::size_t>(p + 1), t1);
    return NurbsCurve(std::move(points), std::move(weights), std::move(segmentKnots), p);
}

NurbsCurve NurbsCurve::elevateDegree() const {
    const int p = m_degree;

    // Step 1: Decompose into Bezier segments by inserting knots to full
    // multiplicity at each interior knot.
    NurbsCurve working = *this;

    // Collect unique interior knots and their current multiplicities.
    std::vector<std::pair<double, int>> interiorKnots;
    {
        const auto& kts = working.knots();
        int i = p + 1;
        while (i < static_cast<int>(kts.size()) - p - 1) {
            double knotVal = kts[i];
            int mult = 0;
            int j = i;
            while (j < static_cast<int>(kts.size()) - p - 1 && kts[j] == knotVal) {
                ++mult;
                ++j;
            }
            if (mult < p) {
                interiorKnots.push_back({knotVal, p - mult});
            }
            i = j;
        }
    }

    // Insert each interior knot until it has multiplicity p.
    for (const auto& [knotVal, insCount] : interiorKnots) {
        for (int ins = 0; ins < insCount; ++ins) {
            working = working.insertKnot(knotVal);
        }
    }

    // Now `working` is a piecewise Bezier (each span has p+1 control points).
    const auto& wPts = working.controlPoints();
    const auto& wWts = working.weights();
    const auto& wKts = working.knots();
    const int wN = working.controlPointCount();
    const int numSegments = (wN - 1) / p;  // each Bezier segment has p+1 points, sharing endpoints

    // Step 2: Elevate each Bezier segment from degree p to degree p+1.
    // Convert to homogeneous coords.
    std::vector<std::array<double, 4>> Pw(wN);
    for (int i = 0; i < wN; ++i) {
        const double w = wWts[i];
        Pw[i] = {wPts[i].x * w, wPts[i].y * w, wPts[i].z * w, w};
    }

    // Each elevated segment has (p+2) control points.
    // Total new control points = numSegments * (p+1) + 1 (sharing endpoints).
    const int newP = p + 1;
    std::vector<std::array<double, 4>> newPw;
    newPw.reserve(numSegments * (p + 1) + 1);

    for (int seg = 0; seg < numSegments; ++seg) {
        const int base = seg * p;

        // Bezier elevation: Q[0] = P[0], Q[p+1] = P[p]
        // Q[i] = (i/(p+1)) * P[i-1] + (1 - i/(p+1)) * P[i]  for i = 1..p
        std::vector<std::array<double, 4>> Q(newP + 1);
        Q[0] = Pw[base];
        Q[newP] = Pw[base + p];

        for (int i = 1; i <= p; ++i) {
            const double alpha = static_cast<double>(i) / static_cast<double>(newP);
            for (int c = 0; c < 4; ++c) {
                Q[i][c] = alpha * Pw[base + i - 1][c] + (1.0 - alpha) * Pw[base + i][c];
            }
        }

        // Append: skip Q[0] for all segments after the first (shared with previous segment's last
        // point).
        const int start = (seg == 0) ? 0 : 1;
        for (int i = start; i <= newP; ++i) {
            newPw.push_back(Q[i]);
        }
    }

    // Step 3: Build the new knot vector.
    // For piecewise Bezier of degree (p+1), each unique knot value gets multiplicity (p+1+1)
    // at the ends and (p+1) at interior knots.
    // Simpler: collect unique knot values from the decomposed curve and increase multiplicity.
    std::vector<double> newKnots;
    {
        // Collect unique knots with their multiplicities in the decomposed curve.
        std::vector<std::pair<double, int>> knotMults;
        int i = 0;
        while (i < static_cast<int>(wKts.size())) {
            double val = wKts[i];
            int mult = 0;
            int j = i;
            while (j < static_cast<int>(wKts.size()) && wKts[j] == val) {
                ++mult;
                ++j;
            }
            knotMults.push_back({val, mult});
            i = j;
        }

        // Each knot's multiplicity increases by 1 for degree elevation.
        for (const auto& [val, mult] : knotMults) {
            for (int m = 0; m < mult + 1; ++m) {
                newKnots.push_back(val);
            }
        }
    }

    // Dehomogenise.
    const int newN = static_cast<int>(newPw.size());
    std::vector<math::Vec3> newPts(newN);
    std::vector<double> newWts(newN);
    for (int i = 0; i < newN; ++i) {
        const double w = newPw[i][3];
        newWts[i] = w;
        const double wInv = 1.0 / w;
        newPts[i] = {newPw[i][0] * wInv, newPw[i][1] * wInv, newPw[i][2] * wInv};
    }

    return NurbsCurve(std::move(newPts), std::move(newWts), std::move(newKnots), newP);
}

// ---------------------------------------------------------------------------
// closestPoint — Newton iteration on f(t) = (C(t) - P) · C'(t)
// ---------------------------------------------------------------------------

double NurbsCurve::closestPoint(const math::Vec3& point, double tol) const {
    const double tLo = tMin();
    const double tHi = tMax();

    // Initial guesses: sample at 21 uniform points, and keep the two closest.
    // Refining only the closest failed at a closed curve's seam: a point just
    // before it is as close to the sample at the start as to the one at the
    // end, the start was taken, and Newton's step from there went out of the
    // domain and was held at the start.
    constexpr int kNumSamples = 20;
    double bestT[2] = {tLo, tLo};
    double bestDistSq[2] = {std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::infinity()};
    for (int i = 0; i <= kNumSamples; ++i) {
        const double t = tLo + (tHi - tLo) * static_cast<double>(i) / kNumSamples;
        const double distSq = (evaluate(t) - point).lengthSquared();
        if (distSq < bestDistSq[0]) {
            bestDistSq[1] = bestDistSq[0];
            bestT[1] = bestT[0];
            bestDistSq[0] = distSq;
            bestT[0] = t;
        } else if (distSq < bestDistSq[1]) {
            bestDistSq[1] = distSq;
            bestT[1] = t;
        }
    }

    // Newton iteration on f(t) = (C(t) - P) . C'(t).
    const auto refine = [&](double t) {
        for (int iter = 0; iter < 50; ++iter) {
            const math::Vec3 c = evaluate(t);
            const math::Vec3 dC = derivative(t, 1);
            const math::Vec3 d2C = derivative(t, 2);
            const math::Vec3 diff = c - point;

            const double f = diff.dot(dC);
            const double df = dC.dot(dC) + diff.dot(d2C);

            if (std::abs(df) < 1e-15) {
                break;
            }

            const double delta = f / df;
            t = t - delta;
            t = std::clamp(t, tLo, tHi);

            if (std::abs(delta) < tol) {
                break;
            }
        }
        return t;
    };

    double best = refine(bestT[0]);
    double bestDist = (evaluate(best) - point).lengthSquared();
    if (std::isfinite(bestDistSq[1])) {
        const double other = refine(bestT[1]);
        const double otherDist = (evaluate(other) - point).lengthSquared();
        if (otherDist < bestDist) best = other;
    }
    return best;
}

// ---------------------------------------------------------------------------
// arcLength — Simpson's rule
// ---------------------------------------------------------------------------

double NurbsCurve::arcLength(double tStart, double tEnd, int segments) const {
    // Ensure even number of segments for Simpson's rule.
    if (segments % 2 != 0) {
        ++segments;
    }

    const double h = (tEnd - tStart) / segments;
    double sum = derivative(tStart, 1).length() + derivative(tEnd, 1).length();

    for (int i = 1; i < segments; ++i) {
        const double t = tStart + i * h;
        const double speed = derivative(t, 1).length();
        if (i % 2 == 1) {
            sum += 4.0 * speed;
        } else {
            sum += 2.0 * speed;
        }
    }

    return sum * h / 3.0;
}

// ---------------------------------------------------------------------------
// parameterAtLength — Newton iteration
// ---------------------------------------------------------------------------

double NurbsCurve::parameterAtLength(double length, double tStart) const {
    if (std::isnan(tStart)) {
        tStart = tMin();
    }
    tStart = std::clamp(tStart, tMin(), tMax());

    const double totalLen = arcLength(tStart, tMax());
    if (totalLen < 1e-15) {
        return tStart;
    }

    // Initial guess: linear interpolation.
    double t = tStart + (tMax() - tStart) * length / totalLen;
    t = std::clamp(t, tStart, tMax());

    for (int iter = 0; iter < 50; ++iter) {
        const double currentLen = arcLength(tStart, t);
        const double error = currentLen - length;

        if (std::abs(error) < 1e-8) {
            break;
        }

        const double speed = derivative(t, 1).length();
        if (speed < 1e-15) {
            break;
        }

        t = t - error / speed;
        t = std::clamp(t, tStart, tMax());
    }

    return t;
}

// ---------------------------------------------------------------------------
// Conic factories — helpers
// ---------------------------------------------------------------------------

namespace {

/// Rotate a point from the XY plane to an arbitrary plane defined by its normal.
math::Vec3 rotateToPlane(const math::Vec3& pt, const math::Vec3& normal) {
    const math::Vec3 unitZ{0.0, 0.0, 1.0};

    // If the normal is already (0,0,1), no rotation needed.
    const math::Vec3 n = normal.normalized();
    const double d = n.dot(unitZ);

    if (d > 1.0 - 1e-12) {
        return pt;  // Already aligned with +Z.
    }
    if (d < -1.0 + 1e-12) {
        // 180-degree rotation around X axis: flip Y and Z.
        return {pt.x, -pt.y, -pt.z};
    }

    // Rodrigues' rotation formula.
    // Axis = UnitZ x normal (normalized), angle = acos(d).
    const math::Vec3 axis = unitZ.cross(n).normalized();
    const double cosA = d;
    const double sinA = std::sqrt(1.0 - d * d);

    // R(p) = p*cos + (axis x p)*sin + axis*(axis.p)*(1-cos)
    return pt * cosA + axis.cross(pt) * sinA + axis * axis.dot(pt) * (1.0 - cosA);
}

}  // namespace

// ---------------------------------------------------------------------------
// makeCircle — degree-2 rational NURBS with 9 control points
// ---------------------------------------------------------------------------

NurbsCurve NurbsCurve::makeCircle(const math::Vec3& center, double radius,
                                  const math::Vec3& normal) {
    const double w = std::cos(math::kPi / 4.0);  // sqrt(2)/2

    // Unit circle control points in XY plane.
    const math::Vec3 unitPts[9] = {
        {1, 0, 0},   {1, 1, 0},  {0, 1, 0},  {-1, 1, 0}, {-1, 0, 0},
        {-1, -1, 0}, {0, -1, 0}, {1, -1, 0}, {1, 0, 0},
    };

    const double wts[9] = {1, w, 1, w, 1, w, 1, w, 1};

    std::vector<math::Vec3> ctrlPts(9);
    std::vector<double> weights(9);

    for (int i = 0; i < 9; ++i) {
        math::Vec3 p = unitPts[i] * radius;
        p = rotateToPlane(p, normal);
        ctrlPts[i] = p + center;
        weights[i] = wts[i];
    }

    std::vector<double> knots = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};

    return NurbsCurve(std::move(ctrlPts), std::move(weights), std::move(knots), 2);
}

// ---------------------------------------------------------------------------
// makeArc — piecewise degree-2 rational NURBS
// ---------------------------------------------------------------------------

NurbsCurve NurbsCurve::makeArc(const math::Vec3& center, double radius, double startAngle,
                               double endAngle, const math::Vec3& normal) {
    // Normalize sweep angle to (0, 2*pi].
    double sweep = endAngle - startAngle;
    if (!std::isfinite(sweep)) {
        throw std::invalid_argument("NurbsCurve::makeArc needs finite angles");
    }
    // Most of the way first: a turn at a time, an angle of 1e20 took for ever.
    if (std::abs(sweep) > 1e3) sweep = std::fmod(sweep, math::kTwoPi);
    while (sweep <= 0.0) sweep += math::kTwoPi;
    while (sweep > math::kTwoPi + 1e-12) sweep -= math::kTwoPi;

    // Determine number of arc segments (each <= 90 degrees).
    int numSegments;
    if (sweep <= math::kHalfPi + 1e-12) {
        numSegments = 1;
    } else if (sweep <= math::kPi + 1e-12) {
        numSegments = 2;
    } else if (sweep <= 3.0 * math::kHalfPi + 1e-12) {
        numSegments = 3;
    } else {
        numSegments = 4;
    }

    const double segSweep = sweep / numSegments;
    const double wMid = std::cos(segSweep / 2.0);

    // Build control points: 2*numSegments + 1 points.
    const int numPts = 2 * numSegments + 1;
    std::vector<math::Vec3> ctrlPts;
    std::vector<double> weights;
    ctrlPts.reserve(numPts);
    weights.reserve(numPts);

    double angle = startAngle;
    for (int seg = 0; seg < numSegments; ++seg) {
        const double a = angle;
        const double b = angle + segSweep;
        const double midAngle = (a + b) / 2.0;

        math::Vec3 p0{std::cos(a) * radius, std::sin(a) * radius, 0.0};
        math::Vec3 p1{std::cos(midAngle) * radius / wMid, std::sin(midAngle) * radius / wMid, 0.0};

        if (seg == 0) {
            ctrlPts.push_back(p0);
            weights.push_back(1.0);
        }
        ctrlPts.push_back(p1);
        weights.push_back(wMid);

        math::Vec3 p2{std::cos(b) * radius, std::sin(b) * radius, 0.0};
        ctrlPts.push_back(p2);
        weights.push_back(1.0);

        angle = b;
    }

    // Build knot vector for piecewise degree-2.
    // Knot vector: [0,0,0, k1,k1, k2,k2, ..., 1,1,1]
    std::vector<double> knots;
    knots.reserve(numPts + 3);
    knots.push_back(0.0);
    knots.push_back(0.0);
    knots.push_back(0.0);
    for (int seg = 1; seg < numSegments; ++seg) {
        double knotVal = static_cast<double>(seg) / numSegments;
        knots.push_back(knotVal);
        knots.push_back(knotVal);
    }
    knots.push_back(1.0);
    knots.push_back(1.0);
    knots.push_back(1.0);

    // Transform control points: rotate to plane then translate.
    for (auto& pt : ctrlPts) {
        pt = rotateToPlane(pt, normal) + center;
    }

    return NurbsCurve(std::move(ctrlPts), std::move(weights), std::move(knots), 2);
}

// ---------------------------------------------------------------------------
// makeEllipse — same as circle but with anisotropic scaling
// ---------------------------------------------------------------------------

NurbsCurve NurbsCurve::makeEllipse(const math::Vec3& center, double semiMajor, double semiMinor,
                                   double rotation, const math::Vec3& normal) {
    const double w = std::cos(math::kPi / 4.0);  // sqrt(2)/2

    // Unit circle control points in XY plane, scaled by (semiMajor, semiMinor).
    const math::Vec3 unitPts[9] = {
        {1, 0, 0},   {1, 1, 0},  {0, 1, 0},  {-1, 1, 0}, {-1, 0, 0},
        {-1, -1, 0}, {0, -1, 0}, {1, -1, 0}, {1, 0, 0},
    };

    const double wts[9] = {1, w, 1, w, 1, w, 1, w, 1};

    const double cosR = std::cos(rotation);
    const double sinR = std::sin(rotation);

    std::vector<math::Vec3> ctrlPts(9);
    std::vector<double> weights(9);

    for (int i = 0; i < 9; ++i) {
        // Scale by semi-axes.
        double sx = unitPts[i].x * semiMajor;
        double sy = unitPts[i].y * semiMinor;

        // Apply rotation in the XY plane.
        double rx = sx * cosR - sy * sinR;
        double ry = sx * sinR + sy * cosR;

        math::Vec3 p{rx, ry, 0.0};
        p = rotateToPlane(p, normal);
        ctrlPts[i] = p + center;
        weights[i] = wts[i];
    }

    std::vector<double> knots = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};

    return NurbsCurve(std::move(ctrlPts), std::move(weights), std::move(knots), 2);
}

}  // namespace hz::geo
