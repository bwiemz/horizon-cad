#pragma once

#include <cmath>

#include "horizon/math/Vec3.h"

namespace hz::math {

struct Vec4 {
    double x, y, z, w;

    Vec4() : x(0.0), y(0.0), z(0.0), w(0.0) {}
    Vec4(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
    Vec4(const Vec3& v, double w) : x(v.x), y(v.y), z(v.z), w(w) {}

    Vec4 operator+(const Vec4& rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w};
    }
    Vec4 operator*(double s) const { return {x * s, y * s, z * s, w * s}; }

    double dot(const Vec4& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w;
    }

    Vec3 xyz() const { return {x, y, z}; }

    Vec3 perspectiveDivide() const {
        if (std::abs(w) < 1e-15) return {x, y, z};
        return {x / w, y / w, z / w};
    }
};

}  // namespace hz::math
