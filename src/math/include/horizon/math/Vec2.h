#pragma once

#include <cmath>

#include "horizon/math/Tolerance.h"

namespace hz::math {

struct Vec2 {
    double x, y;

    Vec2() : x(0.0), y(0.0) {}
    Vec2(double x, double y) : x(x), y(y) {}

    Vec2 operator+(const Vec2& rhs) const { return {x + rhs.x, y + rhs.y}; }
    Vec2 operator-(const Vec2& rhs) const { return {x - rhs.x, y - rhs.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2 operator/(double s) const { return {x / s, y / s}; }
    Vec2 operator-() const { return {-x, -y}; }

    Vec2& operator+=(const Vec2& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }
    Vec2& operator-=(const Vec2& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }
    Vec2& operator*=(double s) {
        x *= s;
        y *= s;
        return *this;
    }

    double dot(const Vec2& rhs) const { return x * rhs.x + y * rhs.y; }
    double cross(const Vec2& rhs) const { return x * rhs.y - y * rhs.x; }
    double length() const { return std::sqrt(x * x + y * y); }
    double lengthSquared() const { return x * x + y * y; }

    Vec2 normalized() const {
        double len = length();
        if (len < Tolerance::kLinear) return {0.0, 0.0};
        return {x / len, y / len};
    }

    Vec2 perpendicular() const { return {-y, x}; }

    double distanceTo(const Vec2& other) const { return (*this - other).length(); }

    bool isApproxEqual(const Vec2& other, double tol = Tolerance::kLinear) const {
        return distanceTo(other) <= tol;
    }

    static const Vec2 Zero;
    static const Vec2 UnitX;
    static const Vec2 UnitY;
};

inline Vec2 operator*(double s, const Vec2& v) { return v * s; }

}  // namespace hz::math
