#pragma once

#include "horizon/math/Vec3.h"
#include "horizon/math/Vec4.h"

namespace hz::math {

struct Quaternion;  // forward declaration

struct Mat4 {
    double m[4][4];

    Mat4();

    static Mat4 identity();
    static Mat4 translation(const Vec3& t);
    static Mat4 rotation(const Quaternion& q);
    static Mat4 rotationX(double angleRad);
    static Mat4 rotationY(double angleRad);
    static Mat4 rotationZ(double angleRad);
    static Mat4 scale(const Vec3& s);
    static Mat4 scale(double s);
    static Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec3& up);
    static Mat4 perspective(double fovY, double aspect, double nearPlane, double farPlane);
    static Mat4 ortho(double left, double right, double bottom, double top, double nearPlane,
                      double farPlane);

    Mat4 operator*(const Mat4& rhs) const;
    Vec4 operator*(const Vec4& v) const;
    Vec3 transformPoint(const Vec3& p) const;
    Vec3 transformDirection(const Vec3& d) const;
    Mat4 inverse() const;
    Mat4 transposed() const;

    double& at(int row, int col) { return m[row][col]; }
    double at(int row, int col) const { return m[row][col]; }

    const double* data() const { return &m[0][0]; }
};

}  // namespace hz::math
