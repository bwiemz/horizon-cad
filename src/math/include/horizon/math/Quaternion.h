#pragma once

#include <cmath>

#include "horizon/math/Vec3.h"

namespace hz::math {

struct Mat4;  // forward declaration

struct Quaternion {
    double w, x, y, z;

    Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    static Quaternion fromAxisAngle(const Vec3& axis, double angleRad);
    static Quaternion fromEuler(double pitch, double yaw, double roll);
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, double t);

    Quaternion operator*(const Quaternion& rhs) const;
    Vec3 rotate(const Vec3& v) const;

    double length() const;
    Quaternion normalized() const;
    Quaternion conjugate() const;
    Quaternion inverse() const;

    Mat4 toMatrix() const;

    bool isApproxEqual(const Quaternion& other, double tol = Tolerance::kAngular) const;

    static const Quaternion Identity;
};

}  // namespace hz::math
