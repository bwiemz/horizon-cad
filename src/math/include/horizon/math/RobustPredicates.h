#pragma once

#include <vector>

#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"

// Range of validity: the results are exact for finite inputs as long as no intermediate value
// overflows or underflows. Two things bound it:
//  - Above: the largest coordinate differences. A predicate of degree n multiplies n of them,
//    so it holds up to about 1e150 (orient2d, classifyPoint), 1e100 (orient3d) and 1e75
//    (incircle); above that it returns 0.
//  - Below: the smallest NONZERO coordinate magnitude m, not the differences. The error term of
//    a difference can be as small as the last bit of one coordinate, ulp(m) ~ m * 2^-52, and
//    the exact arithmetic multiplies n of them, so it needs ulp(m)^n >= 2^-1074 (the smallest
//    double). That gives m of about 1e-146 (degree 2), 7.6e-93 (degree 3) and 9.5e-66
//    (degree 4). Below it the error terms underflow and the sign may be wrong, then 0.
//    Measured: orient2d on (p,q), (2p,2q), (-p,-q) is wrong in about a quarter of cases from
//    1e-155 to 1e-148 and never at 1e-145; orient3d on u, v, 2u, -v from 1e-105 to 1e-93 and
//    never at 1e-90; incircle on full-mantissa rectangles at 1e-66 and never from 1e-65 up.
// So keep every nonzero coordinate above about 1e-140 (orient2d, classifyPoint), 1e-90
// (orient3d) and 1e-60 (incircle); the margin over the measured onsets is deliberate. Differences
// in range do not help: the exactly cocircular rectangle (1.528379950790292, 3.5558643899770326),
// (9.0470469239970971e-90, 3.5558643899770326), (9.0470469239970971e-90, 7.0334986959029244e-90),
// (1.528379950790292, 7.0334986959029244e-90) has differences around 1, yet one tiny coordinate
// per axis makes incircle return -1 instead of 0. A caller that may hold smaller nonzero
// coordinates must flush them to 0 first. Nothing crashes.

namespace hz::math {

/// +1 when c is left of the line a->b (counter-clockwise), -1 right, 0 on it -- exactly.
int orient2d(const Vec2& a, const Vec2& b, const Vec2& c);

/// +1 when d is below the plane through a, b, c (they counter-clockwise seen from above),
/// -1 above, 0 on it -- exactly.
int orient3d(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d);

/// +1 when d is inside the circle through a, b, c (counter-clockwise), -1 outside, 0 on it --
/// exactly.
int incircle(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d);

enum class Containment { Outside, Inside, OnBoundary };

/// Where p is relative to the closed polygon (no closing duplicate), by winding number with
/// orient2d. A point on any edge or vertex is OnBoundary; the polygon may wind either way.
Containment classifyPoint(const std::vector<Vec2>& polygon, const Vec2& p);

}  // namespace hz::math
