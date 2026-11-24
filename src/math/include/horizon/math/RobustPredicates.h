#pragma once

#include <vector>

#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"

// Range of validity: the results are exact for finite inputs as long as no intermediate product
// underflows or overflows. A predicate of degree n multiplies n coordinate differences, so its
// range depends on n. The scale is that of the coordinate differences (zero always counts):
//   orient2d and classifyPoint (degree 2): about 1e-150 to 1e150;
//   orient3d (degree 3):                    about 1e-100 to 1e100;
//   incircle (degree 4):                    about 1e-60 to 1e75.
// (Measured on near-degenerate configurations scaled by powers of two: the sign is kept from
// 4.8e-153 to 6.7e153, 1.8e-102 to 2.2e102 and 6.9e-77 to 5.8e76, for coordinates with few
// mantissa bits.) With full 53-bit mantissas the exact terms reach down further: a degree-n
// product has error terms about 2^(-53 (n - 1)) below its leading term, so incircle's lowest
// ones (about scale^4 * 1e-48) underflow near 1e-65 and the sign is lost from about 1e-60
// (a cocircular rectangle at 6e-62 already answers -1, not 0). Above its range a
// predicate returns 0; just below it, the error terms of the exact arithmetic underflow and it
// may return the wrong sign, then 0 further down. Nothing crashes.

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
