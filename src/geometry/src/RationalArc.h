#pragma once

// Internal to the geometry library: the unit circle's arc as a rational quadratic B-spline, the
// one way every typed curve and surface here becomes a NURBS (Curve3::toNurbs,
// Surface::toNurbs, NurbsSurface::makeRevolution), and the check of the range it is asked for.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "horizon/math/Constants.h"
#include "horizon/math/Vec2.h"

namespace hz::geo::detail {

/// @p hi for the range [@p lo, @p hi] of something @p what names, checked: finite, not empty,
/// and, where @p period is not 0, at most one period long. A range a period long to rounding
/// (within 1e-12 of the period) is made a period exactly, so a whole turn closes.
/// @throws std::invalid_argument otherwise.
inline double checkedRangeEnd(double lo, double hi, double period, const char* what) {
    if (!std::isfinite(lo) || !std::isfinite(hi) || !(hi > lo)) {
        throw std::invalid_argument(std::string(what) + ": the range must be finite and not empty");
    }
    if (period > 0.0 && hi - lo > period) {
        if (hi - lo > period + 1e-12 * std::max(1.0, period)) {
            throw std::invalid_argument(std::string(what) +
                                        ": the range must not run more than one period round");
        }
        return lo + period;
    }
    return hi;
}

/// A rational quadratic B-spline of the unit circle's arc: its control points (each the unit
/// vector at its angle, the middle of each segment 1 / cos(half its angle) out), their weights
/// and its clamped knots.
struct UnitArc {
    std::vector<math::Vec2> points;
    std::vector<double> weights;
    std::vector<double> knots;
};

/// The unit circle's arc from angle @p a0 to @p a1 (a0 < a1, at most a turn), its parameter
/// running from @p t0 to @p t1: one rational quadratic segment per quarter turn or less, all of
/// one angle, meeting at double knots spaced evenly in [t0, t1], so at each knot the parameter is
/// the angle's affine image and between them it runs at the segment's own pace. Its ends are
/// (cos a0, sin a0) and (cos a1, sin a1), computed as a typed curve computes them.
inline UnitArc unitArc(double a0, double a1, double t0, double t1) {
    const double sweep = a1 - a0;
    const int n = std::clamp(static_cast<int>(std::ceil(sweep / math::kHalfPi - 1e-9)), 1, 4);
    const double half = sweep / (2.0 * n);
    const double w = std::cos(half);
    UnitArc arc;
    arc.points.reserve(static_cast<std::size_t>(2 * n + 1));
    arc.weights.reserve(static_cast<std::size_t>(2 * n + 1));
    arc.knots.assign(3, t0);
    const auto at = [&](int k) { return k == n ? a1 : a0 + sweep * k / n; };
    for (int k = 0; k < n; ++k) {
        const double from = at(k);
        if (k == 0) {
            arc.points.emplace_back(std::cos(from), std::sin(from));
            arc.weights.push_back(1.0);
        } else {
            const double knot = t0 + (t1 - t0) * k / n;
            arc.knots.push_back(knot);
            arc.knots.push_back(knot);
        }
        const double mid = from + half;
        arc.points.emplace_back(std::cos(mid) / w, std::sin(mid) / w);
        arc.weights.push_back(w);
        const double to = at(k + 1);
        arc.points.emplace_back(std::cos(to), std::sin(to));
        arc.weights.push_back(1.0);
    }
    arc.knots.insert(arc.knots.end(), 3, t1);
    return arc;
}

}  // namespace hz::geo::detail
