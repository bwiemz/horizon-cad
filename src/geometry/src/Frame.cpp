#include "horizon/geometry/Frame.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace hz::geo {

Frame Frame::fromAxis(const math::Vec3& origin, const math::Vec3& axis, const math::Vec3& xHint) {
    Frame f;
    f.origin = origin;
    assert(axis.length() > 0.0 && "Frame::fromAxis needs a non-zero axis");
    // Not normalized(): that returns the zero vector for lengths below 1e-7.
    f.z = axis / axis.length();
    math::Vec3 x = xHint - f.z * xHint.dot(f.z);
    if (x.length() <= 1e-9 * xHint.length() || xHint.length() == 0.0) {
        // The hint is (nearly) parallel to the axis: take the world axis least aligned with it.
        const double ax = std::abs(f.z.x);
        const double ay = std::abs(f.z.y);
        const double az = std::abs(f.z.z);
        const math::Vec3 w = (ax <= ay && ax <= az) ? math::Vec3{1.0, 0.0, 0.0}
                             : (ay <= az)           ? math::Vec3{0.0, 1.0, 0.0}
                                                    : math::Vec3{0.0, 0.0, 1.0};
        x = w - f.z * w.dot(f.z);
    }
    x = x / x.length();
    // Re-orthogonalise once: a hint a hair off parallel leaves rounding error along z.
    x = x - f.z * x.dot(f.z);
    x = x / x.length();
    f.x = x;
    f.y = f.z.cross(f.x);
    return f;
}

Frame Frame::transformed(const math::Mat4& m) const {
    Frame f;
    f.origin = m.transformPoint(origin);
    // Divided by their lengths, not normalized(): under a scale below 1e-7 every axis is
    // shorter than normalized()'s cut-off.
    const auto unit = [&m](const math::Vec3& axis) {
        const math::Vec3 d = m.transformDirection(axis);
        return d / d.length();
    };
    f.x = unit(x);
    f.y = unit(y);
    f.z = unit(z);
    return f;
}

double Frame::scaleOf(const math::Mat4& m) {
    return m.transformDirection({1.0, 0.0, 0.0}).length();
}

}  // namespace hz::geo
