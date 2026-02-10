#include "horizon/math/Mat4.h"

#include <cmath>
#include <cstring>

#include "horizon/math/Quaternion.h"

namespace hz::math {

Mat4::Mat4() { std::memset(m, 0, sizeof(m)); m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.0; }

Mat4 Mat4::identity() { return Mat4(); }

Mat4 Mat4::translation(const Vec3& t) {
    Mat4 r;
    r.m[0][3] = t.x;
    r.m[1][3] = t.y;
    r.m[2][3] = t.z;
    return r;
}

Mat4 Mat4::rotation(const Quaternion& q) { return q.toMatrix(); }

Mat4 Mat4::rotationX(double a) {
    double c = std::cos(a), s = std::sin(a);
    Mat4 r;
    r.m[1][1] = c;
    r.m[1][2] = -s;
    r.m[2][1] = s;
    r.m[2][2] = c;
    return r;
}

Mat4 Mat4::rotationY(double a) {
    double c = std::cos(a), s = std::sin(a);
    Mat4 r;
    r.m[0][0] = c;
    r.m[0][2] = s;
    r.m[2][0] = -s;
    r.m[2][2] = c;
    return r;
}

Mat4 Mat4::rotationZ(double a) {
    double c = std::cos(a), s = std::sin(a);
    Mat4 r;
    r.m[0][0] = c;
    r.m[0][1] = -s;
    r.m[1][0] = s;
    r.m[1][1] = c;
    return r;
}

Mat4 Mat4::scale(const Vec3& s) {
    Mat4 r;
    r.m[0][0] = s.x;
    r.m[1][1] = s.y;
    r.m[2][2] = s.z;
    return r;
}

Mat4 Mat4::scale(double s) { return scale({s, s, s}); }

Mat4 Mat4::lookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
    Vec3 f = (target - eye).normalized();
    Vec3 r = f.cross(up).normalized();
    Vec3 u = r.cross(f);

    Mat4 result;
    result.m[0][0] = r.x;
    result.m[0][1] = r.y;
    result.m[0][2] = r.z;
    result.m[0][3] = -r.dot(eye);
    result.m[1][0] = u.x;
    result.m[1][1] = u.y;
    result.m[1][2] = u.z;
    result.m[1][3] = -u.dot(eye);
    result.m[2][0] = -f.x;
    result.m[2][1] = -f.y;
    result.m[2][2] = -f.z;
    result.m[2][3] = f.dot(eye);
    result.m[3][3] = 1.0;
    return result;
}

Mat4 Mat4::perspective(double fovY, double aspect, double nearPlane, double farPlane) {
    double tanHalf = std::tan(fovY / 2.0);
    Mat4 r;
    std::memset(r.m, 0, sizeof(r.m));
    r.m[0][0] = 1.0 / (aspect * tanHalf);
    r.m[1][1] = 1.0 / tanHalf;
    r.m[2][2] = -(farPlane + nearPlane) / (farPlane - nearPlane);
    r.m[2][3] = -(2.0 * farPlane * nearPlane) / (farPlane - nearPlane);
    r.m[3][2] = -1.0;
    return r;
}

Mat4 Mat4::ortho(double left, double right, double bottom, double top, double nearPlane,
                 double farPlane) {
    Mat4 r;
    std::memset(r.m, 0, sizeof(r.m));
    r.m[0][0] = 2.0 / (right - left);
    r.m[1][1] = 2.0 / (top - bottom);
    r.m[2][2] = -2.0 / (farPlane - nearPlane);
    r.m[0][3] = -(right + left) / (right - left);
    r.m[1][3] = -(top + bottom) / (top - bottom);
    r.m[2][3] = -(farPlane + nearPlane) / (farPlane - nearPlane);
    r.m[3][3] = 1.0;
    return r;
}

Mat4 Mat4::operator*(const Mat4& rhs) const {
    Mat4 r;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            r.m[i][j] = 0.0;
            for (int k = 0; k < 4; ++k) {
                r.m[i][j] += m[i][k] * rhs.m[k][j];
            }
        }
    }
    return r;
}

Vec4 Mat4::operator*(const Vec4& v) const {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w,
            m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w};
}

Vec3 Mat4::transformPoint(const Vec3& p) const {
    Vec4 result = *this * Vec4(p, 1.0);
    return result.perspectiveDivide();
}

Vec3 Mat4::transformDirection(const Vec3& d) const {
    return {m[0][0] * d.x + m[0][1] * d.y + m[0][2] * d.z,
            m[1][0] * d.x + m[1][1] * d.y + m[1][2] * d.z,
            m[2][0] * d.x + m[2][1] * d.y + m[2][2] * d.z};
}

Mat4 Mat4::transposed() const {
    Mat4 r;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) r.m[i][j] = m[j][i];
    return r;
}

Mat4 Mat4::inverse() const {
    // Compute 4x4 inverse using cofactor expansion
    double inv[16];
    const double* s = &m[0][0];

    inv[0] = s[5] * s[10] * s[15] - s[5] * s[11] * s[14] - s[9] * s[6] * s[15] +
             s[9] * s[7] * s[14] + s[13] * s[6] * s[11] - s[13] * s[7] * s[10];
    inv[1] = -s[1] * s[10] * s[15] + s[1] * s[11] * s[14] + s[9] * s[2] * s[15] -
             s[9] * s[3] * s[14] - s[13] * s[2] * s[11] + s[13] * s[3] * s[10];
    inv[2] = s[1] * s[6] * s[15] - s[1] * s[7] * s[14] - s[5] * s[2] * s[15] +
             s[5] * s[3] * s[14] + s[13] * s[2] * s[7] - s[13] * s[3] * s[6];
    inv[3] = -s[1] * s[6] * s[11] + s[1] * s[7] * s[10] + s[5] * s[2] * s[11] -
             s[5] * s[3] * s[10] - s[9] * s[2] * s[7] + s[9] * s[3] * s[6];
    inv[4] = -s[4] * s[10] * s[15] + s[4] * s[11] * s[14] + s[8] * s[6] * s[15] -
             s[8] * s[7] * s[14] - s[12] * s[6] * s[11] + s[12] * s[7] * s[10];
    inv[5] = s[0] * s[10] * s[15] - s[0] * s[11] * s[14] - s[8] * s[2] * s[15] +
             s[8] * s[3] * s[14] + s[12] * s[2] * s[11] - s[12] * s[3] * s[10];
    inv[6] = -s[0] * s[6] * s[15] + s[0] * s[7] * s[14] + s[4] * s[2] * s[15] -
             s[4] * s[3] * s[14] - s[12] * s[2] * s[7] + s[12] * s[3] * s[6];
    inv[7] = s[0] * s[6] * s[11] - s[0] * s[7] * s[10] - s[4] * s[2] * s[11] +
             s[4] * s[3] * s[10] + s[8] * s[2] * s[7] - s[8] * s[3] * s[6];
    inv[8] = s[4] * s[9] * s[15] - s[4] * s[11] * s[13] - s[8] * s[5] * s[15] +
             s[8] * s[7] * s[13] + s[12] * s[5] * s[11] - s[12] * s[7] * s[9];
    inv[9] = -s[0] * s[9] * s[15] + s[0] * s[11] * s[13] + s[8] * s[1] * s[15] -
             s[8] * s[3] * s[13] - s[12] * s[1] * s[11] + s[12] * s[3] * s[9];
    inv[10] = s[0] * s[5] * s[15] - s[0] * s[7] * s[13] - s[4] * s[1] * s[15] +
              s[4] * s[3] * s[13] + s[12] * s[1] * s[7] - s[12] * s[3] * s[5];
    inv[11] = -s[0] * s[5] * s[11] + s[0] * s[7] * s[9] + s[4] * s[1] * s[11] -
              s[4] * s[3] * s[9] - s[8] * s[1] * s[7] + s[8] * s[3] * s[5];
    inv[12] = -s[4] * s[9] * s[14] + s[4] * s[10] * s[13] + s[8] * s[5] * s[14] -
              s[8] * s[6] * s[13] - s[12] * s[5] * s[10] + s[12] * s[6] * s[9];
    inv[13] = s[0] * s[9] * s[14] - s[0] * s[10] * s[13] - s[8] * s[1] * s[14] +
              s[8] * s[2] * s[13] + s[12] * s[1] * s[10] - s[12] * s[2] * s[9];
    inv[14] = -s[0] * s[5] * s[14] + s[0] * s[6] * s[13] + s[4] * s[1] * s[14] -
              s[4] * s[2] * s[13] - s[12] * s[1] * s[6] + s[12] * s[2] * s[5];
    inv[15] = s[0] * s[5] * s[10] - s[0] * s[6] * s[9] - s[4] * s[1] * s[10] +
              s[4] * s[2] * s[9] + s[8] * s[1] * s[6] - s[8] * s[2] * s[5];

    double det = s[0] * inv[0] + s[1] * inv[4] + s[2] * inv[8] + s[3] * inv[12];
    if (std::abs(det) < 1e-15) return identity();

    double invDet = 1.0 / det;
    Mat4 result;
    for (int i = 0; i < 16; ++i) {
        (&result.m[0][0])[i] = inv[i] * invDet;
    }
    return result;
}

}  // namespace hz::math
