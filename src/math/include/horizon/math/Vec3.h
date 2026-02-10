#pragma once

#include <cmath>

#include "horizon/math/Tolerance.h"

namespace hz::math {

struct Vec3 {
    double x, y, z;

    Vec3() : x(0.0), y(0.0), z(0.0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }

    Vec3& operator+=(const Vec3& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }
    Vec3& operator-=(const Vec3& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }
    Vec3& operator*=(double s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    double dot(const Vec3& rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z; }

    Vec3 cross(const Vec3& rhs) const {
        return {y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x};
    }

    double length() const { return std::sqrt(x * x + y * y + z * z); }
    double lengthSquared() const { return x * x + y * y + z * z; }

    Vec3 normalized() const {
        double len = length();
        if (len < Tolerance::kLinear) return {0.0, 0.0, 0.0};
        return {x / len, y / len, z / len};
    }

    double distanceTo(const Vec3& other) const { return (*this - other).length(); }

    bool isApproxEqual(const Vec3& other, double tol = Tolerance::kLinear) const {
        return distanceTo(other) <= tol;
    }

    static const Vec3 Zero;
    static const Vec3 UnitX;
    static const Vec3 UnitY;
    static const Vec3 UnitZ;
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

}  // namespace hz::math
