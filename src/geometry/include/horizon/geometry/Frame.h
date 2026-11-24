#pragma once

#include <limits>

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"

namespace hz::geo {

/// An orthonormal coordinate frame: an origin and three unit axes. Right-handed
/// when built by fromAxis(); a reflecting transform leaves it left-handed.
struct Frame {
    math::Vec3 origin;
    math::Vec3 x{1.0, 0.0, 0.0};
    math::Vec3 y{0.0, 1.0, 0.0};
    math::Vec3 z{0.0, 0.0, 1.0};

    /// A frame with z along @p axis, x along @p xHint made perpendicular to it
    /// (any perpendicular when the hint is parallel to the axis), and y = z x x.
    static Frame fromAxis(const math::Vec3& origin, const math::Vec3& axis,
                          const math::Vec3& xHint = {1.0, 0.0, 0.0});

    /// This frame moved by @p m: the origin as a point, each axis as a direction
    /// and normalised. After a reflection the result is left-handed.
    Frame transformed(const math::Mat4& m) const;

    /// The uniform scale of @p m: the length of its transformed x axis.
    static double scaleOf(const math::Mat4& m);
};

/// A closed range of a parameter; either end may be infinite.
struct Interval {
    double min = 0.0;
    double max = 0.0;
};

}  // namespace hz::geo
