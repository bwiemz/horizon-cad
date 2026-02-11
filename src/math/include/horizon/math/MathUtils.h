#pragma once

#include <algorithm>
#include <cmath>

#include "horizon/math/Constants.h"

namespace hz::math {

inline double degToRad(double degrees) { return degrees * kDegToRad; }
inline double radToDeg(double radians) { return radians * kRadToDeg; }

inline double clamp(double val, double lo, double hi) { return std::max(lo, std::min(hi, val)); }

inline double lerp(double a, double b, double t) { return a + t * (b - a); }

inline double normalizeAngle(double radians) {
    radians = std::fmod(radians, kTwoPi);
    if (radians < 0.0) radians += kTwoPi;
    if (radians >= kTwoPi) radians = 0.0;  // Guard floating-point edge case.
    return radians;
}

}  // namespace hz::math
