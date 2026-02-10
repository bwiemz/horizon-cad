#pragma once

#include <cmath>

namespace hz::math {

struct Tolerance {
    static constexpr double kLinear = 1e-7;
    static constexpr double kAngular = 1e-10;
    static constexpr double kParametric = 1e-9;
    static constexpr double kConfusion = 1e-7;

    static bool isZero(double val, double tol = kLinear) { return std::abs(val) <= tol; }

    static bool areEqual(double a, double b, double tol = kLinear) {
        return std::abs(a - b) <= tol;
    }
};

}  // namespace hz::math
