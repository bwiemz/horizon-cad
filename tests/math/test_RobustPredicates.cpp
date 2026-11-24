#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "../PortableRandom.h"
#include "horizon/math/RobustPredicates.h"

using namespace hz::math;

// The exact integer references need 128-bit integers (GCC/Clang); MSVC skips them.
#ifdef __SIZEOF_INT128__
#define HZ_HAVE_INT128 1
#endif

namespace {

#ifdef HZ_HAVE_INT128
__extension__ using I = __int128;

int sgn(I v) {
    return v > 0 ? 1 : (v < 0 ? -1 : 0);
}

// Exact references on integer-valued inputs (every such double is exact, the arithmetic is
// carried out in 128-bit integers).
int refOrient2d(const int64_t* a, const int64_t* b, const int64_t* c) {
    const I l = static_cast<I>(a[0] - c[0]) * (b[1] - c[1]);
    const I r = static_cast<I>(a[1] - c[1]) * (b[0] - c[0]);
    return sgn(l - r);
}

int refOrient3d(const int64_t* a, const int64_t* b, const int64_t* c, const int64_t* d) {
    const I adx = a[0] - d[0], ady = a[1] - d[1], adz = a[2] - d[2];
    const I bdx = b[0] - d[0], bdy = b[1] - d[1], bdz = b[2] - d[2];
    const I cdx = c[0] - d[0], cdy = c[1] - d[1], cdz = c[2] - d[2];
    return sgn(adz * (bdx * cdy - cdx * bdy) + bdz * (cdx * ady - adx * cdy) +
               cdz * (adx * bdy - bdx * ady));
}

int refIncircle(const int64_t* a, const int64_t* b, const int64_t* c, const int64_t* d) {
    const I adx = a[0] - d[0], ady = a[1] - d[1];
    const I bdx = b[0] - d[0], bdy = b[1] - d[1];
    const I cdx = c[0] - d[0], cdy = c[1] - d[1];
    const I al = adx * adx + ady * ady, bl = bdx * bdx + bdy * bdy, cl = cdx * cdx + cdy * cdy;
    return sgn(al * (bdx * cdy - cdx * bdy) + bl * (cdx * ady - adx * cdy) +
               cl * (adx * bdy - bdx * ady));
}

Vec2 v2(const int64_t* p) {
    return {static_cast<double>(p[0]), static_cast<double>(p[1])};
}
Vec3 v3(const int64_t* p) {
    return {static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2])};
}

#endif  // HZ_HAVE_INT128

}  // namespace

TEST(RobustPredicates, OrientationIsExactWhereDoublesLie) {
    // Shewchuk's classic: points near a line, offset by single ulps. A naive
    // determinant gives inconsistent signs; the exact one never does.
    const Vec2 a(0.5, 0.5), b(12.0, 12.0), c(24.0, 24.0);
    EXPECT_EQ(orient2d(a, b, c), 0);
    for (int i = 0; i < 256; ++i) {
        const Vec2 p(0.5 + i * std::ldexp(1.0, -53), 0.5);
        const int s = orient2d(p, b, c);
        EXPECT_EQ(s, i == 0 ? 0 : -1) << i;  // p moves right of the line y = x
    }
}

TEST(RobustPredicates, Orient3dAndIncircleSigns) {
    EXPECT_EQ(orient3d({0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, -1}), 1);
    EXPECT_EQ(orient3d({0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}), -1);
    EXPECT_EQ(orient3d({0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0.3, 0.7, 0}), 0);
    EXPECT_EQ(incircle({0, 0}, {1, 0}, {0, 1}, {0.5, 0.5}), 1);
    EXPECT_EQ(incircle({0, 0}, {1, 0}, {0, 1}, {1, 1}), 0);
    EXPECT_EQ(incircle({0, 0}, {1, 0}, {0, 1}, {2, 2}), -1);
}

TEST(RobustPredicates, PointsAgainstAPolygon) {
    const std::vector<Vec2> l{{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}};  // an L
    EXPECT_EQ(classifyPoint(l, {0.5, 0.5}), Containment::Inside);
    EXPECT_EQ(classifyPoint(l, {1.5, 1.5}), Containment::Outside);
    EXPECT_EQ(classifyPoint(l, {1.0, 1.5}), Containment::OnBoundary);
    EXPECT_EQ(classifyPoint(l, {2.0, 0.0}), Containment::OnBoundary);
}

TEST(RobustPredicates, ClassifyPointEdgeCases) {
    const std::vector<Vec2> l{{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}};
    // Collinear with an edge but beyond its extent is not on the boundary.
    EXPECT_EQ(classifyPoint(l, {3.0, 0.0}), Containment::Outside);
    EXPECT_EQ(classifyPoint(l, {-1.0, 1.0}), Containment::Outside);
    // A ray through a reflex vertex must not double count.
    EXPECT_EQ(classifyPoint(l, {0.5, 1.0}), Containment::Inside);
    EXPECT_EQ(classifyPoint(l, {1.5, 1.0}), Containment::OnBoundary);
    EXPECT_EQ(classifyPoint(l, {1.5, 1.0 + 1e-15}), Containment::Outside);
    EXPECT_EQ(classifyPoint(l, {1.0, 1.0}), Containment::OnBoundary);
    // Clockwise winding classifies the same.
    const std::vector<Vec2> cw(l.rbegin(), l.rend());
    EXPECT_EQ(classifyPoint(cw, {0.5, 0.5}), Containment::Inside);
    EXPECT_EQ(classifyPoint(cw, {1.5, 1.5}), Containment::Outside);
    // Degenerate input.
    EXPECT_EQ(classifyPoint({}, {0, 0}), Containment::Outside);
    EXPECT_EQ(classifyPoint({{1, 1}}, {1, 1}), Containment::OnBoundary);
    // A point one ulp off a long slanted edge.
    const std::vector<Vec2> tri{{0, 0}, {1, 1}, {0, 1}};
    EXPECT_EQ(classifyPoint(tri, {0.5, 0.5}), Containment::OnBoundary);
    EXPECT_EQ(classifyPoint(tri, {0.5, std::nextafter(0.5, 1.0)}), Containment::Inside);
    EXPECT_EQ(classifyPoint(tri, {0.5, std::nextafter(0.5, 0.0)}), Containment::Outside);
}

#ifdef HZ_HAVE_INT128
TEST(RobustPredicates, Orient2dMatchesExactReferenceOnNearCollinearIntegers) {
    std::mt19937 rng(12345);
    const hz::test::UniformInt small(-(1 << 20), 1 << 20), mult(-(1 << 19), 1 << 19),
        off(-(int64_t{1} << 30), int64_t{1} << 30), tweak(-1, 1);
    int zeros = 0;
    for (int i = 0; i < 20000; ++i) {
        const int64_t a[2] = {off(rng), off(rng)};
        const int64_t d[2] = {small(rng), small(rng)};
        const int64_t m1 = mult(rng), m2 = mult(rng);
        const int64_t b[2] = {a[0] + d[0] * m1, a[1] + d[1] * m1};
        int64_t c[2] = {a[0] + d[0] * m2, a[1] + d[1] * m2};
        c[i & 1] += tweak(rng);
        const int expected = refOrient2d(a, b, c);
        zeros += expected == 0;
        ASSERT_EQ(orient2d(v2(a), v2(b), v2(c)), expected) << i;
    }
    EXPECT_GT(zeros, 1000);  // the exact-zero path is genuinely exercised
}

TEST(RobustPredicates, Orient3dMatchesExactReferenceOnNearCoplanarIntegers) {
    std::mt19937 rng(777);
    const hz::test::UniformInt coord(-(1 << 21), 1 << 21), uv(-8, 8), tweak(-1, 1);
    int zeros = 0;
    for (int i = 0; i < 20000; ++i) {
        int64_t a[3], b[3], c[3], d[3];
        for (int k = 0; k < 3; ++k) {
            a[k] = coord(rng);
            b[k] = coord(rng);
            c[k] = coord(rng);
        }
        const int64_t u = uv(rng), v = uv(rng);
        for (int k = 0; k < 3; ++k) d[k] = a[k] + u * (b[k] - a[k]) + v * (c[k] - a[k]);
        d[i % 3] += tweak(rng);
        const int expected = refOrient3d(a, b, c, d);
        zeros += expected == 0;
        ASSERT_EQ(orient3d(v3(a), v3(b), v3(c), v3(d)), expected) << i;
    }
    EXPECT_GT(zeros, 1000);
}

namespace {
// (2+i)^j (2-i)^(n-j): a Gaussian integer of norm 5^n, so a lattice point on the circle
// of radius 5^(n/2) about the origin.
void circlePoint(int j, int n, int64_t out[2]) {
    int64_t re = 1, im = 0;
    for (int k = 0; k < n; ++k) {
        const int64_t s = k < j ? 1 : -1;  // multiply by 2 + s*i
        const int64_t nr = re * 2 - im * s, ni = re * s + im * 2;
        re = nr;
        im = ni;
    }
    out[0] = re;
    out[1] = im;
}
}  // namespace

TEST(RobustPredicates, IncircleMatchesExactReferenceOnCocircularLatticePoints) {
    std::mt19937 rng(99);
    const hz::test::UniformInt pick(0, 20), tweak(-1, 1);
    const hz::test::UniformInt off(-(int64_t{1} << 30), int64_t{1} << 30);
    int zeros = 0, tried = 0;
    for (int i = 0; i < 20000; ++i) {
        const int64_t js[4] = {pick(rng), pick(rng), pick(rng), pick(rng)};
        if (js[0] == js[1] || js[1] == js[2] || js[0] == js[2] || js[3] == js[0] ||
            js[3] == js[1] || js[3] == js[2])
            continue;
        const int64_t ox = off(rng), oy = off(rng);
        int64_t p[4][2];
        for (int k = 0; k < 4; ++k) {
            circlePoint(static_cast<int>(js[k]), 20, p[k]);
            p[k][0] += ox;
            p[k][1] += oy;
        }
        p[3][i & 1] += tweak(rng);
        const int expected = refIncircle(p[0], p[1], p[2], p[3]);
        zeros += expected == 0;
        ++tried;
        ASSERT_EQ(incircle(v2(p[0]), v2(p[1]), v2(p[2]), v2(p[3])), expected) << i;
    }
    EXPECT_GT(tried, 1000);
    EXPECT_GT(zeros, 100);
}

#else
TEST(RobustPredicates, ExactIntegerReferencesNeedInt128) {
    GTEST_SKIP() << "no 128-bit integers on this compiler; the randomized reference tests are off";
}
#endif  // HZ_HAVE_INT128

TEST(RobustPredicates, UlpPerturbationsOfDegenerateFloatConfigurations) {
    // Non-integer doubles: cocircular unit square corners nudged by one ulp must flip sign
    // consistently, and the opposite nudge must flip it back.
    const Vec2 a(0, 0), b(1, 0), c(0, 1);
    const Vec2 d(1, 1);
    EXPECT_EQ(incircle(a, b, c, d), 0);
    EXPECT_EQ(incircle(a, b, c, Vec2(std::nextafter(1.0, 2.0), 1.0)), -1);
    EXPECT_EQ(incircle(a, b, c, Vec2(std::nextafter(1.0, 0.0), 1.0)), 1);
    // Offsets of 2^-1000 stay well inside the supported range (see the header): every product
    // in the filter and in the exact path is representable, so the sign is exact.
    EXPECT_EQ(orient3d({0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0.3, 0.7, -std::ldexp(1.0, -100)}), 1);
    EXPECT_EQ(orient3d({0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0.3, 0.7, std::ldexp(1.0, -100)}), -1);
}

TEST(RobustPredicates, Orient3dUlpWalkAroundASlantedPlane) {
    // All points satisfy z = x / 2 exactly (halving is exact), so d is coplanar with a, b, c;
    // moving d.z by i ulps puts it above (-1) or below (+1) the plane, by construction. The
    // moves are far under the filter's error bound, so the exact path decides every i != 0.
    const Vec3 a(0.1, 0.7, 0.05), b(0.9, 0.3, 0.45), c(0.2, 0.95, 0.1);  // counter-clockwise
    const double dx = 0.37, dy = 0.61, dz = 0.5 * dx;
    EXPECT_EQ(orient3d(a, b, c, {dx, dy, dz}), 0);
    double up = dz, down = dz;
    for (int i = 1; i <= 64; ++i) {
        up = std::nextafter(up, 1.0);
        down = std::nextafter(down, -1.0);
        EXPECT_EQ(orient3d(a, b, c, {dx, dy, up}), -1) << i;
        EXPECT_EQ(orient3d(a, b, c, {dx, dy, down}), 1) << i;
    }
}

TEST(RobustPredicates, Orient2dUlpWalkAlongASlantedLine) {
    // a, b, c lie exactly on y = x / 2; p walks off the line by i ulps in y.
    const Vec2 a(0.1, 0.05), b(0.3, 0.15), c(0.9, 0.45);
    EXPECT_EQ(orient2d(a, b, c), 0);
    double up = c.y, down = c.y;
    for (int i = 1; i <= 64; ++i) {
        up = std::nextafter(up, 1.0);
        down = std::nextafter(down, -1.0);
        EXPECT_EQ(orient2d(a, b, Vec2(c.x, up)), 1) << i;
        EXPECT_EQ(orient2d(a, b, Vec2(c.x, down)), -1) << i;
    }
}

TEST(PortableRandom, UniformIntDrawsTheSameInRangeOnEveryPlatform) {
    std::mt19937 rng(99);
    // std::mt19937's numbers are fixed by the standard and the mapping is the helper's own, so
    // these are the draws on every platform.
    const hz::test::UniformInt pick(0, 20);
    for (const int64_t want : {7, 16, 9, 3, 16, 16}) EXPECT_EQ(pick(rng), want);
    // Both ends are drawn, nothing outside them.
    const hz::test::UniformInt tweak(-1, 1);
    int seen[3] = {0, 0, 0};
    for (int i = 0; i < 3000; ++i) {
        const int64_t x = tweak(rng);
        ASSERT_GE(x, -1);
        ASSERT_LE(x, 1);
        ++seen[x + 1];
    }
    for (const int n : seen) EXPECT_GT(n, 800);
    // The whole 64-bit range, where hi - lo + 1 wraps to 0.
    const hz::test::UniformInt any(INT64_MIN, INT64_MAX);
    bool negative = false;
    bool positive = false;
    for (int i = 0; i < 64; ++i) {
        const int64_t x = any(rng);
        negative = negative || x < 0;
        positive = positive || x > 0;
    }
    EXPECT_TRUE(negative && positive);
}
