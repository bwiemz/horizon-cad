#include "horizon/geometry/Frame.h"

#include <algorithm>
#include <cmath>

namespace hz::geo {

Frame Frame::fromAxis(const math::Vec3& origin, const math::Vec3& axis, const math::Vec3& xHint) {
    Frame f;
    f.origin = origin;
    f.z = axis.normalized();
    math::Vec3 x = xHint - f.z * xHint.dot(f.z);
    if (x.length() < 1e-9 * std::max(1.0, xHint.length())) {
        // The hint is (nearly) parallel to the axis: take the world axis least aligned with it.
        const double ax = std::abs(f.z.x);
        const double ay = std::abs(f.z.y);
        const double az = std::abs(f.z.z);
        const math::Vec3 w = (ax <= ay && ax <= az) ? math::Vec3{1.0, 0.0, 0.0}
                             : (ay <= az)           ? math::Vec3{0.0, 1.0, 0.0}
                                                    : math::Vec3{0.0, 0.0, 1.0};
        x = w - f.z * w.dot(f.z);
    }
    f.x = x.normalized();
    f.y = f.z.cross(f.x);
    return f;
}

Frame Frame::transformed(const math::Mat4& m) const {
    Frame f;
    f.origin = m.transformPoint(origin);
    f.x = m.transformDirection(x).normalized();
    f.y = m.transformDirection(y).normalized();
    f.z = m.transformDirection(z).normalized();
    return f;
}

double Frame::scaleOf(const math::Mat4& m) {
    return m.transformDirection({1.0, 0.0, 0.0}).length();
}

}  // namespace hz::geo
