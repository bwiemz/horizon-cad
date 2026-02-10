#pragma once

#include "horizon/math/Vec2.h"

namespace hz::math {

struct Mat3 {
    double m[3][3];

    Mat3() : m{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}} {}

    static Mat3 identity();
    static Mat3 translation(const Vec2& t);
    static Mat3 rotation(double angleRad);
    static Mat3 scale(const Vec2& s);
    static Mat3 scale(double s);

    Mat3 operator*(const Mat3& rhs) const;
    Vec2 transformPoint(const Vec2& p) const;
    Vec2 transformDirection(const Vec2& d) const;
    Mat3 inverse() const;
    Mat3 transposed() const;

    double& at(int row, int col) { return m[row][col]; }
    double at(int row, int col) const { return m[row][col]; }
};

}  // namespace hz::math
