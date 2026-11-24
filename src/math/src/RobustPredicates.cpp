#include "horizon/math/RobustPredicates.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>

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
//
// An expansion lives on the stack, in an array whose capacity N is part of its type and is the
// worst-case length of the expression that produces it, derived from how the operations below
// build their results (zero components are dropped, which only shortens them):
//   difference of two doubles         <= 2 components (the rounded sum and its error)
//   grow(e, b)                        <= |e| + 1
//   add(e, f) / subtract(e, f)        <= |e| + |f|   (one grow per component of f)
//   scale(e, b)                       <= 2 |e|       (two-product error and sum error per term)
//   multiply(e, f)                    <= 2 |e| |f|   (|f| scaled copies of e, added)
// Hence orient2d's determinant has at most 16 components (2*2*2 per product, two products),
// orient3d's at most 192 (2 * 16 * 2 = 64 per term, three terms) and incircle's at most 1536
// (a lift is 16, the cross term 16, 2*16*16 = 512 per term, three terms). These are the lengths
// of this straightforward evaluation; Shewchuk's published 1152 for incircle belongs to his
// term-by-term arrangement, which this file does not use. The capacity is checked on every
// store: a result that would not fit is a bug in the bound, asserted in debug builds and
// aborting in release builds, never a write past the end.
// ---------------------------------------------------------------------------------------------

template <std::size_t N>
struct Expansion {
    double v[N];  // deliberately uninitialised: only the first n are ever read
    std::size_t n = 0;

    void push(double d) {
        assert(n < N && "expansion capacity exceeded: the bound is wrong");
        if (n >= N) std::abort();
        v[n++] = d;
    }
    double back() const { return v[n - 1]; }
};

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

Expansion<2> fromDifference(double a, double b) {
    const SumPair d = twoSum(a, -b);
    Expansion<2> e;
    if (d.err != 0.0) e.push(d.err);
    if (d.sum != 0.0 || e.n == 0) e.push(d.sum);
    return e;
}

// h += b, in place. Each output component is written at or behind the input component being
// read, and one more may be appended, so the result needs |h| + 1 slots (checked by push).
template <std::size_t N>
void growInPlace(Expansion<N>& h, double b) {
    const std::size_t count = h.n;
    h.n = 0;
    double q = b;
    for (std::size_t i = 0; i < count; ++i) {
        // h.v[i] is read before the write at index h.n <= i, so the store cannot clobber it.
        const SumPair s = twoSum(q, h.v[i]);
        q = s.sum;
        if (s.err != 0.0) h.push(s.err);
    }
    if (q != 0.0 || h.n == 0) {
        // The final slot may be one past the old length; push checks the capacity.
        h.push(q);
    }
}

// h += f, in place; the caller sizes h for |h| + |f| components.
template <std::size_t N, std::size_t M>
void addInto(Expansion<N>& h, const Expansion<M>& f) {
    for (std::size_t i = 0; i < f.n; ++i) growInPlace(h, f.v[i]);
}

template <std::size_t N, std::size_t M>
Expansion<N + M> add(const Expansion<N>& e, const Expansion<M>& f) {
    Expansion<N + M> h;
    for (std::size_t i = 0; i < e.n; ++i) h.push(e.v[i]);
    addInto(h, f);
    return h;
}

template <std::size_t N, std::size_t M>
Expansion<N + M> subtract(const Expansion<N>& e, const Expansion<M>& f) {
    Expansion<N + M> h;
    for (std::size_t i = 0; i < e.n; ++i) h.push(e.v[i]);
    for (std::size_t i = 0; i < f.n; ++i) growInPlace(h, -f.v[i]);
    return h;
}

template <std::size_t N>
Expansion<2 * N> scale(const Expansion<N>& e, double b) {
    Expansion<2 * N> h;
    const SumPair first = twoProduct(e.v[0], b);
    double q = first.sum;
    if (first.err != 0.0) h.push(first.err);
    for (std::size_t i = 1; i < e.n; ++i) {
        const SumPair p = twoProduct(e.v[i], b);
        const SumPair s = twoSum(q, p.err);
        if (s.err != 0.0) h.push(s.err);
        const SumPair t = fastTwoSum(p.sum, s.sum);
        q = t.sum;
        if (t.err != 0.0) h.push(t.err);
    }
    if (q != 0.0 || h.n == 0) h.push(q);
    return h;
}

template <std::size_t N, std::size_t M>
Expansion<2 * N * M> multiply(const Expansion<N>& e, const Expansion<M>& f) {
    Expansion<2 * N * M> h;
    {
        const Expansion<2 * N> first = scale(e, f.v[0]);
        for (std::size_t i = 0; i < first.n; ++i) h.push(first.v[i]);
    }
    for (std::size_t i = 1; i < f.n; ++i) addInto(h, scale(e, f.v[i]));
    return h;
}

template <std::size_t N>
int signOf(const Expansion<N>& e) {
    return signOf(e.back());
}

// 2x2 cross term  px*qy - qx*py  of expansions.
template <std::size_t A, std::size_t B, std::size_t C, std::size_t D>
Expansion<2 * A * D + 2 * C * B> cross(const Expansion<A>& px, const Expansion<B>& py,
                                       const Expansion<C>& qx, const Expansion<D>& qy) {
    return subtract(multiply(px, qy), multiply(qx, py));
}

int orient2dExact(const Vec2& a, const Vec2& b, const Vec2& c) {
    const auto acx = fromDifference(a.x, c.x), acy = fromDifference(a.y, c.y);
    const auto bcx = fromDifference(b.x, c.x), bcy = fromDifference(b.y, c.y);
    return signOf(cross(acx, acy, bcx, bcy));  // <= 16 components
}

int orient3dExact(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
    const auto adx = fromDifference(a.x, d.x), ady = fromDifference(a.y, d.y),
               adz = fromDifference(a.z, d.z);
    const auto bdx = fromDifference(b.x, d.x), bdy = fromDifference(b.y, d.y),
               bdz = fromDifference(b.z, d.z);
    const auto cdx = fromDifference(c.x, d.x), cdy = fromDifference(c.y, d.y),
               cdz = fromDifference(c.z, d.z);
    const auto det =
        add(add(multiply(adz, cross(bdx, bdy, cdx, cdy)), multiply(bdz, cross(cdx, cdy, adx, ady))),
            multiply(cdz, cross(adx, ady, bdx, bdy)));  // <= 192 components
    return signOf(det);
}

int incircleExact(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    const auto adx = fromDifference(a.x, d.x), ady = fromDifference(a.y, d.y);
    const auto bdx = fromDifference(b.x, d.x), bdy = fromDifference(b.y, d.y);
    const auto cdx = fromDifference(c.x, d.x), cdy = fromDifference(c.y, d.y);
    const auto alift = add(multiply(adx, adx), multiply(ady, ady));
    const auto blift = add(multiply(bdx, bdx), multiply(bdy, bdy));
    const auto clift = add(multiply(cdx, cdx), multiply(cdy, cdy));
    const auto det = add(
        add(multiply(alift, cross(bdx, bdy, cdx, cdy)), multiply(blift, cross(cdx, cdy, adx, ady))),
        multiply(clift, cross(adx, ady, bdx, bdy)));  // <= 1536 components
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
