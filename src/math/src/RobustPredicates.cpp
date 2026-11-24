#include "horizon/math/RobustPredicates.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

// Exact geometric predicates after Shewchuk, "Adaptive Precision Floating-Point Arithmetic and
// Fast Robust Geometric Predicates". A filtered determinant decides almost every call; when its
// error bound cannot, the determinant is evaluated exactly in floating-point expansions.
//
// This file must be compiled without floating-point contraction (see the CMakeLists): the
// error-free transformations below assume every + and * rounds on its own.

namespace hz::math {

namespace {

constexpr double kEps = 1.1102230246251565e-16;  // 2^-53
constexpr double kCcwErrBoundA = (3.0 + 16.0 * kEps) * kEps;
constexpr double kO3dErrBoundA = (7.0 + 56.0 * kEps) * kEps;
constexpr double kIccErrBoundA = (10.0 + 96.0 * kEps) * kEps;

int signOf(double v) {
    return v > 0.0 ? 1 : (v < 0.0 ? -1 : 0);
}

// ---------------------------------------------------------------------------------------------
// Expansions: sums of doubles, stored with increasing magnitude and non-overlapping, so that
// the sign of the whole is the sign of the last (largest) component.
// ---------------------------------------------------------------------------------------------

using Expansion = std::vector<double>;

struct SumPair {
    double sum, err;
};

SumPair twoSum(double a, double b) {
    const double s = a + b;
    const double bv = s - a;
    const double av = s - bv;
    return {s, (a - av) + (b - bv)};
}

// Requires |a| >= |b| or a == 0.
SumPair fastTwoSum(double a, double b) {
    const double s = a + b;
    return {s, b - (s - a)};
}

SumPair twoProduct(double a, double b) {
    const double p = a * b;
    return {p, std::fma(a, b, -p)};
}

Expansion fromDifference(double a, double b) {
    const SumPair d = twoSum(a, -b);
    Expansion e;
    if (d.err != 0.0) e.push_back(d.err);
    if (d.sum != 0.0 || e.empty()) e.push_back(d.sum);
    return e;
}

Expansion growExpansion(const Expansion& e, double b) {
    Expansion h;
    h.reserve(e.size() + 1);
    double q = b;
    for (const double c : e) {
        const SumPair s = twoSum(q, c);
        q = s.sum;
        if (s.err != 0.0) h.push_back(s.err);
    }
    if (q != 0.0 || h.empty()) h.push_back(q);
    return h;
}

Expansion add(const Expansion& e, const Expansion& f) {
    Expansion h = e;
    for (const double c : f) h = growExpansion(h, c);
    return h;
}

Expansion negate(Expansion e) {
    for (double& c : e) c = -c;
    return e;
}

Expansion scale(const Expansion& e, double b) {
    Expansion h;
    h.reserve(2 * e.size());
    const SumPair first = twoProduct(e[0], b);
    double q = first.sum;
    if (first.err != 0.0) h.push_back(first.err);
    for (std::size_t i = 1; i < e.size(); ++i) {
        const SumPair p = twoProduct(e[i], b);
        const SumPair s = twoSum(q, p.err);
        if (s.err != 0.0) h.push_back(s.err);
        const SumPair t = fastTwoSum(p.sum, s.sum);
        q = t.sum;
        if (t.err != 0.0) h.push_back(t.err);
    }
    if (q != 0.0 || h.empty()) h.push_back(q);
    return h;
}

Expansion multiply(const Expansion& e, const Expansion& f) {
    Expansion h{0.0};
    for (const double c : f) h = add(h, scale(e, c));
    return h;
}

Expansion subtract(const Expansion& e, const Expansion& f) {
    return add(e, negate(f));
}

int signOf(const Expansion& e) {
    return signOf(e.back());
}

// 2x2 cross term  px*qy - qx*py  of expansions.
Expansion cross(const Expansion& px, const Expansion& py, const Expansion& qx,
                const Expansion& qy) {
    return subtract(multiply(px, qy), multiply(qx, py));
}

int orient2dExact(const Vec2& a, const Vec2& b, const Vec2& c) {
    const Expansion acx = fromDifference(a.x, c.x), acy = fromDifference(a.y, c.y);
    const Expansion bcx = fromDifference(b.x, c.x), bcy = fromDifference(b.y, c.y);
    return signOf(cross(acx, acy, bcx, bcy));
}

int orient3dExact(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
    const Expansion adx = fromDifference(a.x, d.x), ady = fromDifference(a.y, d.y),
                    adz = fromDifference(a.z, d.z);
    const Expansion bdx = fromDifference(b.x, d.x), bdy = fromDifference(b.y, d.y),
                    bdz = fromDifference(b.z, d.z);
    const Expansion cdx = fromDifference(c.x, d.x), cdy = fromDifference(c.y, d.y),
                    cdz = fromDifference(c.z, d.z);
    const Expansion det =
        add(add(multiply(adz, cross(bdx, bdy, cdx, cdy)), multiply(bdz, cross(cdx, cdy, adx, ady))),
            multiply(cdz, cross(adx, ady, bdx, bdy)));
    return signOf(det);
}

int incircleExact(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    const Expansion adx = fromDifference(a.x, d.x), ady = fromDifference(a.y, d.y);
    const Expansion bdx = fromDifference(b.x, d.x), bdy = fromDifference(b.y, d.y);
    const Expansion cdx = fromDifference(c.x, d.x), cdy = fromDifference(c.y, d.y);
    const Expansion alift = add(multiply(adx, adx), multiply(ady, ady));
    const Expansion blift = add(multiply(bdx, bdx), multiply(bdy, bdy));
    const Expansion clift = add(multiply(cdx, cdx), multiply(cdy, cdy));
    const Expansion det = add(
        add(multiply(alift, cross(bdx, bdy, cdx, cdy)), multiply(blift, cross(cdx, cdy, adx, ady))),
        multiply(clift, cross(adx, ady, bdx, bdy)));
    return signOf(det);
}

}  // namespace

int orient2d(const Vec2& a, const Vec2& b, const Vec2& c) {
    const double detleft = (a.x - c.x) * (b.y - c.y);
    const double detright = (a.y - c.y) * (b.x - c.x);
    const double det = detleft - detright;

    double detsum;
    if (detleft > 0.0) {
        if (detright <= 0.0) return signOf(det);
        detsum = detleft + detright;
    } else if (detleft < 0.0) {
        if (detright >= 0.0) return signOf(det);
        detsum = -detleft - detright;
    } else {
        return signOf(det);
    }
    const double errbound = kCcwErrBoundA * detsum;
    if (det >= errbound || -det >= errbound) return signOf(det);
    return orient2dExact(a, b, c);
}

int orient3d(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
    const double adx = a.x - d.x, bdx = b.x - d.x, cdx = c.x - d.x;
    const double ady = a.y - d.y, bdy = b.y - d.y, cdy = c.y - d.y;
    const double adz = a.z - d.z, bdz = b.z - d.z, cdz = c.z - d.z;

    const double bdxcdy = bdx * cdy, cdxbdy = cdx * bdy;
    const double cdxady = cdx * ady, adxcdy = adx * cdy;
    const double adxbdy = adx * bdy, bdxady = bdx * ady;

    const double det = adz * (bdxcdy - cdxbdy) + bdz * (cdxady - adxcdy) + cdz * (adxbdy - bdxady);
    const double permanent = (std::fabs(bdxcdy) + std::fabs(cdxbdy)) * std::fabs(adz) +
                             (std::fabs(cdxady) + std::fabs(adxcdy)) * std::fabs(bdz) +
                             (std::fabs(adxbdy) + std::fabs(bdxady)) * std::fabs(cdz);
    const double errbound = kO3dErrBoundA * permanent;
    if (det > errbound || -det > errbound) return signOf(det);
    return orient3dExact(a, b, c, d);
}

int incircle(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    const double adx = a.x - d.x, bdx = b.x - d.x, cdx = c.x - d.x;
    const double ady = a.y - d.y, bdy = b.y - d.y, cdy = c.y - d.y;

    const double bdxcdy = bdx * cdy, cdxbdy = cdx * bdy;
    const double cdxady = cdx * ady, adxcdy = adx * cdy;
    const double adxbdy = adx * bdy, bdxady = bdx * ady;
    const double alift = adx * adx + ady * ady;
    const double blift = bdx * bdx + bdy * bdy;
    const double clift = cdx * cdx + cdy * cdy;

    const double det =
        alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);
    const double permanent = (std::fabs(bdxcdy) + std::fabs(cdxbdy)) * alift +
                             (std::fabs(cdxady) + std::fabs(adxcdy)) * blift +
                             (std::fabs(adxbdy) + std::fabs(bdxady)) * clift;
    const double errbound = kIccErrBoundA * permanent;
    if (det > errbound || -det > errbound) return signOf(det);
    return incircleExact(a, b, c, d);
}

Containment classifyPoint(const std::vector<Vec2>& polygon, const Vec2& p) {
    const std::size_t n = polygon.size();
    if (n == 0) return Containment::Outside;

    int winding = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2& a = polygon[i];
        const Vec2& b = polygon[(i + 1) % n];
        const int side = orient2d(a, b, p);
        if (side == 0 && p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x) &&
            p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y)) {
            return Containment::OnBoundary;
        }
        // Crossing rule on the half-open interval [a.y, b.y): counts each vertex once.
        if (a.y <= p.y) {
            if (b.y > p.y && side > 0) ++winding;
        } else if (b.y <= p.y && side < 0) {
            --winding;
        }
    }
    return winding != 0 ? Containment::Inside : Containment::Outside;
}

}  // namespace hz::math
