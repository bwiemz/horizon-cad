#pragma once

#include <cmath>
#include <limits>

namespace hz::math {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;
inline constexpr double kHalfPi = kPi / 2.0;
inline constexpr double kDegToRad = kPi / 180.0;
inline constexpr double kRadToDeg = 180.0 / kPi;
inline constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
inline constexpr double kInfinity = std::numeric_limits<double>::infinity();

}  // namespace hz::math
