#include "horizon/math/Mat3.h"

#include <algorithm>
#include <cmath>

namespace hz::math {

Mat3 Mat3::identity() {
    Mat3 r;
    return r;
}

Mat3 Mat3::translation(const Vec2& t) {
    Mat3 r;
    r.m[0][2] = t.x;
    r.m[1][2] = t.y;
    return r;
}

Mat3 Mat3::rotation(double angleRad) {
    double c = std::cos(angleRad);
    double s = std::sin(angleRad);
    Mat3 r;
    r.m[0][0] = c;
    r.m[0][1] = -s;
    r.m[1][0] = s;
    r.m[1][1] = c;
    return r;
}

Mat3 Mat3::scale(const Vec2& s) {
    Mat3 r;
    r.m[0][0] = s.x;
    r.m[1][1] = s.y;
    return r;
}

Mat3 Mat3::scale(double s) {
    return scale({s, s});
}

Mat3 Mat3::operator*(const Mat3& rhs) const {
    Mat3 r;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i][j] = 0.0;
            for (int k = 0; k < 3; ++k) {
                r.m[i][j] += m[i][k] * rhs.m[k][j];
            }
        }
    }
    return r;
}

Vec2 Mat3::transformPoint(const Vec2& p) const {
    double rx = m[0][0] * p.x + m[0][1] * p.y + m[0][2];
    double ry = m[1][0] * p.x + m[1][1] * p.y + m[1][2];
    return {rx, ry};
}

Vec2 Mat3::transformDirection(const Vec2& d) const {
    double rx = m[0][0] * d.x + m[0][1] * d.y;
    double ry = m[1][0] * d.x + m[1][1] * d.y;
    return {rx, ry};
}

Mat3 Mat3::transposed() const {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
    return r;
}

Mat3 Mat3::inverse() const {
    double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                 m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                 m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    // Singular relative to its size (see Mat4::inverse): |det| is at most the
    // product of the rows' lengths, and of the columns'.
    double rows = 1.0;
    double columns = 1.0;
    for (int i = 0; i < 3; ++i) {
        rows *= std::sqrt(m[i][0] * m[i][0] + m[i][1] * m[i][1] + m[i][2] * m[i][2]);
        columns *= std::sqrt(m[0][i] * m[0][i] + m[1][i] * m[1][i] + m[2][i] * m[2][i]);
    }
    if (!std::isfinite(det) || !(std::abs(det) > 1e-15 * std::min(rows, columns))) {
        return identity();
    }

    double invDet = 1.0 / det;
    Mat3 r;
    r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
    r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
    r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;
    r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
    r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet;
    r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
    r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet;
    r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;
    return r;
}

}  // namespace hz::math
