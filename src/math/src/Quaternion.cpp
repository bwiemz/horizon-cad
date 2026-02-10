#include "horizon/math/Quaternion.h"

#include <cmath>

#include "horizon/math/Mat4.h"

namespace hz::math {

const Quaternion Quaternion::Identity = {1.0, 0.0, 0.0, 0.0};

Quaternion Quaternion::fromAxisAngle(const Vec3& axis, double angleRad) {
    Vec3 a = axis.normalized();
    double halfAngle = angleRad / 2.0;
    double s = std::sin(halfAngle);
    return {std::cos(halfAngle), a.x * s, a.y * s, a.z * s};
}

Quaternion Quaternion::fromEuler(double pitch, double yaw, double roll) {
    double cp = std::cos(pitch / 2.0);
    double sp = std::sin(pitch / 2.0);
    double cy = std::cos(yaw / 2.0);
    double sy = std::sin(yaw / 2.0);
    double cr = std::cos(roll / 2.0);
    double sr = std::sin(roll / 2.0);

    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

Quaternion Quaternion::slerp(const Quaternion& a, const Quaternion& b, double t) {
    double cosTheta = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

    Quaternion bAdj = b;
    if (cosTheta < 0.0) {
        bAdj = {-b.w, -b.x, -b.y, -b.z};
        cosTheta = -cosTheta;
    }

    if (cosTheta > 0.9995) {
        // Linear interpolation for very close quaternions
        Quaternion r = {a.w + t * (bAdj.w - a.w), a.x + t * (bAdj.x - a.x),
                        a.y + t * (bAdj.y - a.y), a.z + t * (bAdj.z - a.z)};
        return r.normalized();
    }

    double theta = std::acos(cosTheta);
    double sinTheta = std::sin(theta);
    double wa = std::sin((1.0 - t) * theta) / sinTheta;
    double wb = std::sin(t * theta) / sinTheta;

    return {wa * a.w + wb * bAdj.w, wa * a.x + wb * bAdj.x, wa * a.y + wb * bAdj.y,
            wa * a.z + wb * bAdj.z};
}

Quaternion Quaternion::operator*(const Quaternion& rhs) const {
    return {w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w};
}

Vec3 Quaternion::rotate(const Vec3& v) const {
    // q * v * q^-1 (optimized)
    Vec3 qvec = {x, y, z};
    Vec3 uv = qvec.cross(v);
    Vec3 uuv = qvec.cross(uv);
    return v + 2.0 * (w * uv + uuv);
}

double Quaternion::length() const { return std::sqrt(w * w + x * x + y * y + z * z); }

Quaternion Quaternion::normalized() const {
    double len = length();
    if (len < 1e-15) return Identity;
    return {w / len, x / len, y / len, z / len};
}

Quaternion Quaternion::conjugate() const { return {w, -x, -y, -z}; }

Quaternion Quaternion::inverse() const {
    double lenSq = w * w + x * x + y * y + z * z;
    if (lenSq < 1e-15) return Identity;
    double invLen = 1.0 / lenSq;
    return {w * invLen, -x * invLen, -y * invLen, -z * invLen};
}

Mat4 Quaternion::toMatrix() const {
    double xx = x * x, yy = y * y, zz = z * z;
    double xy = x * y, xz = x * z, yz = y * z;
    double wx = w * x, wy = w * y, wz = w * z;

    Mat4 r;
    r.m[0][0] = 1.0 - 2.0 * (yy + zz);
    r.m[0][1] = 2.0 * (xy - wz);
    r.m[0][2] = 2.0 * (xz + wy);
    r.m[0][3] = 0.0;
    r.m[1][0] = 2.0 * (xy + wz);
    r.m[1][1] = 1.0 - 2.0 * (xx + zz);
    r.m[1][2] = 2.0 * (yz - wx);
    r.m[1][3] = 0.0;
    r.m[2][0] = 2.0 * (xz - wy);
    r.m[2][1] = 2.0 * (yz + wx);
    r.m[2][2] = 1.0 - 2.0 * (xx + yy);
    r.m[2][3] = 0.0;
    r.m[3][0] = 0.0;
    r.m[3][1] = 0.0;
    r.m[3][2] = 0.0;
    r.m[3][3] = 1.0;
    return r;
}

bool Quaternion::isApproxEqual(const Quaternion& other, double tol) const {
    // Two quaternions q and -q represent the same rotation
    double dotPos = w * other.w + x * other.x + y * other.y + z * other.z;
    return std::abs(std::abs(dotPos) - 1.0) <= tol;
}

}  // namespace hz::math
