#pragma once

// Internal to the geometry module (not installed): the knot span and de Boor's algorithm that
// every NURBS evaluation here shares (NurbsCurve, NurbsSurface, the pcurve fit), so each that
// must give another's point to the last bit does so by construction.

#include <array>
#include <cmath>
#include <cstddef>

namespace hz::geo::detail {

/// The knot span [knots[k], knots[k + 1]) holding @p t, for @p count control points of degree
/// @p p: the first from the degree's with t < knots[k + 1] (found by bisection, the knots not
/// decreasing); at or past knots[count], the last span that is not empty; for a NaN, count - 1.
inline int knotSpan(const double* knots, int count, int p, double t) {
    if (t >= knots[count]) {
        int k = count - 1;
        while (k > p && !(knots[k] < knots[k + 1])) --k;
        return k;
    }
    if (std::isnan(t)) return count - 1;
    // The first knot above t among knots[p + 1 .. count] (std::upper_bound, by hand: a debug
    // build pays for every call of the library's): one exists, as t < knots[count] here.
    int lo = p + 1;
    int hi = count + 1;
    while (lo < hi) {
        const int mid = lo + (hi - lo) / 2;
        if (t < knots[mid]) {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    return lo - 1;
}

/// A point in homogeneous coordinates: (x w, y w, z w, w).
using Homogeneous = std::array<double, 4>;

/// De Boor's algorithm on the p + 1 homogeneous points @p d of span @p k (d[j] for control point
/// k - p + j), at @p t: the point is left in d[p].
inline void deBoor(Homogeneous* d, const double* knots, int k, int p, double t) {
    for (int r = 1; r <= p; ++r) {
        for (int j = p; j >= r; --j) {
            const int i = k - p + j;
            // Positive for every i of a span that is not empty, however close its knots: against
            // a length tolerance (1e-7) as it once was, knots 1e-8 apart read as one and the curve
            // evaluated to its first control point between them.
            const double denom = knots[i + p + 1 - r] - knots[i];
            double alpha = 0.0;
            if (denom > 0.0) alpha = (t - knots[i]) / denom;
            const double oneMinusAlpha = 1.0 - alpha;
            double* dj = d[j].data();
            const double* below = d[j - 1].data();
            for (std::size_t c = 0; c < 4; ++c) dj[c] = oneMinusAlpha * below[c] + alpha * dj[c];
        }
    }
}

}  // namespace hz::geo::detail
