#pragma once

#include <vector>

#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"

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
