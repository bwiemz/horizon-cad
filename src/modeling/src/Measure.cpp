#include "horizon/modeling/Measure.h"

#include <algorithm>
#include <cmath>

namespace hz::model::measure {

using hz::math::Vec3;

double distance(const Vec3& a, const Vec3& b) {
    return (a - b).length();
}

double angleBetween(const Vec3& u, const Vec3& v) {
    const double lu = u.length();
    const double lv = v.length();
    if (lu < 1e-12 || lv < 1e-12) return 0.0;
    double c = u.dot(v) / (lu * lv);
    c = std::clamp(c, -1.0, 1.0);
    return std::acos(c);
}

double pointToSegment(const Vec3& p, const Vec3& a, const Vec3& b) {
    const Vec3 ab = b - a;
    const double len2 = ab.dot(ab);
    if (len2 < 1e-24) return (p - a).length();  // degenerate segment
    double t = (p - a).dot(ab) / len2;
    t = std::clamp(t, 0.0, 1.0);
    return (p - (a + ab * t)).length();
}

double segmentToSegment(const Vec3& a0, const Vec3& a1, const Vec3& b0, const Vec3& b1) {
    // Ericson, Real-Time Collision Detection — closest points of two segments.
    const Vec3 d1 = a1 - a0;  // direction of segment A
    const Vec3 d2 = b1 - b0;  // direction of segment B
    const Vec3 r = a0 - b0;
    const double aa = d1.dot(d1);
    const double e = d2.dot(d2);
    const double f = d2.dot(r);

    constexpr double kEps = 1e-24;
    double s = 0.0, t = 0.0;

    if (aa < kEps && e < kEps) {
        return (a0 - b0).length();  // both degenerate
    }
    if (aa < kEps) {
        t = std::clamp(f / e, 0.0, 1.0);  // A degenerate
    } else {
        const double c = d1.dot(r);
        if (e < kEps) {
            s = std::clamp(-c / aa, 0.0, 1.0);  // B degenerate
        } else {
            const double b = d1.dot(d2);
            const double denom = aa * e - b * b;
            s = (denom > kEps) ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / aa, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / aa, 0.0, 1.0);
            }
        }
    }

    const Vec3 cpA = a0 + d1 * s;
    const Vec3 cpB = b0 + d2 * t;
    return (cpA - cpB).length();
}

}  // namespace hz::model::measure
