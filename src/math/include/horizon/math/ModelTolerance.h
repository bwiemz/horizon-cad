#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace hz::math {

/// The modelling resolution of a part: how close two things must be to count as touching.
struct ModelTolerance {
    /// The modelling resolution, in model units.
    double linear = 1e-7;
    /// Angular resolution, in radians.
    double angular = 1e-10;

    /// The largest extent a tolerance is scaled to: the upper end of the robust predicates' range.
    static constexpr double kLargestExtent = 1e100;

    /// A tolerance scaled to a part of the given size. An extent that is not finite and
    /// positive is taken as 1; one above 1e100 (the predicates' upper range) as 1e100. The result
    /// is never finer than 64 units in the last place of the extent, so it stays meaningful for
    /// very large parts.
    static ModelTolerance forExtent(double extent) {
        if (!std::isfinite(extent) || extent <= 0.0) extent = 1.0;
        extent = std::min(extent, kLargestExtent);
        const double ulp = std::nextafter(extent, std::numeric_limits<double>::infinity()) - extent;
        ModelTolerance t;
        t.linear = std::max(1e-7 * extent, 64.0 * ulp);
        return t;
    }

    /// The tolerance that governs an entity: the larger of the model's and the entity's own.
    double effective(double entityTolerance) const { return std::max(linear, entityTolerance); }

    /// True when a distance is within an entity's effective tolerance (inclusive).
    bool within(double distance, double entityTolerance = 0.0) const {
        return distance <= effective(entityTolerance);
    }

    /// True when two entities with the given tolerances, a distance apart, touch: the
    /// distance does not exceed the sum of their effective tolerances (inclusive).
    bool coincident(double distance, double tolA = 0.0, double tolB = 0.0) const {
        return distance <= effective(tolA) + effective(tolB);
    }
};

}  // namespace hz::math
