#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "horizon/geometry/Pcurve.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Circle2D.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/curves/Nurbs2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"

using namespace hz::geo;
namespace math = hz::math;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
/// A length that stands for the model's size in the relative bounds.
constexpr double kSize = 10.0;

Frame tilted() {
    return Frame::fromAxis({1.0, -2.0, 0.5}, {0.3, -0.5, 1.0}, {1.0, 0.2, 0.0});
}

/// The pcurve lands on the curve: its ends map onto C(t0) and C(t1), every point of it
/// stays on the curve, and its own deviation is as small as a closed form's.
void expectFollows(const Surface& s, const Curve3& c, double t0, double t1, const Pcurve& pc,
                   double bound) {
    ASSERT_TRUE(pc.problem.empty()) << pc.problem;
    ASSERT_NE(pc.curve, nullptr);
    EXPECT_LE(pc.deviation, bound);
    const math::Vec2 a = pc.curve->evaluate(pc.start);
    const math::Vec2 b = pc.curve->evaluate(pc.end);
    EXPECT_LE((s.evaluate(a.x, a.y) - c.evaluate(t0)).length(), bound);
    EXPECT_LE((s.evaluate(b.x, b.y) - c.evaluate(t1)).length(), bound);
    for (int k = 0; k <= 40; ++k) {
        const double f = k / 40.0;
        const math::Vec2 uv = pc.curve->evaluate(pc.start + f * (pc.end - pc.start));
        const math::Vec3 p = s.evaluate(uv.x, uv.y);
        const math::Vec3 q = c.evaluate(c.closestParameter(p));
        EXPECT_LE((p - q).length(), bound);
    }
}

template <class T>
const T* as(const Pcurve& pc) {
    return dynamic_cast<const T*>(pc.curve.get());
}

}  // namespace

TEST(Pcurve, EachClosedFormIsExact) {
    const double bound = 1e-12 * kSize;
    const double tol = 1e-6;
    const Frame f = tilted();

    {  // A line on a plane, of scale 1.5.
        const Plane s(f, 1.5);
        const math::Vec3 a = s.evaluate(0.5, -1.0);
        const math::Vec3 b = s.evaluate(3.0, 2.0);
        const LineCurve c(a, b - a, 1.0);
        const double len = (b - a).length();
        const Pcurve pc = pcurveOn(s, c, 0.0, len, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 0.0, len, pc, bound);
    }
    for (const double dir : {1.0, -1.0}) {  // A circle on a plane, either way round.
        const Plane s(f, 1.5);
        const CircleCurve c(Frame::fromAxis(s.evaluate(1.0, 2.0), f.z * dir, {0.3, 1.0, 0.1}), 3.0);
        const Pcurve pc = pcurveOn(s, c, 0.4, 5.0, tol);
        EXPECT_NE(as<Circle2D>(pc), nullptr);
        expectFollows(s, c, 0.4, 5.0, pc, bound);
    }
    {  // An ellipse on a plane.
        const Plane s(f, 1.5);
        const EllipseCurve c(Frame::fromAxis(s.evaluate(1.0, 2.0), -f.z, {0.3, 1.0, 0.1}), 4.0,
                             2.0);
        const Pcurve pc = pcurveOn(s, c, 0.4, 5.5, tol);
        EXPECT_NE(as<Nurbs2D>(pc), nullptr);
        expectFollows(s, c, 0.4, 5.5, pc, bound);
    }
    {  // A generator of a cylinder, with an axial scale.
        const Cylinder s(f, 2.0, 1.5);
        const math::Vec3 a = s.evaluate(0.7, -1.0);
        const math::Vec3 b = s.evaluate(0.7, 2.0);
        const LineCurve c(a, b - a, 1.0);
        const double len = (b - a).length();
        const Pcurve pc = pcurveOn(s, c, 0.0, len, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 0.0, len, pc, bound);
    }
    for (const double dir : {1.0, -1.0}) {  // A parallel of a cylinder.
        const Cylinder s(f, 2.0, 1.5);
        const CircleCurve c(Frame::fromAxis(f.origin + f.z * 1.5, f.z * dir, {0.2, 1.0, 0.0}), 2.0);
        const Pcurve pc = pcurveOn(s, c, 0.3, 4.0, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 0.3, 4.0, pc, bound);
    }
    {  // A generator of a cone.
        const Cone s(f, 2.0, 0.4, 1.0);
        const math::Vec3 a = s.evaluate(2.0, 0.0);
        const math::Vec3 b = s.evaluate(2.0, 3.0);
        const LineCurve c(a, b - a, 1.0);
        const double len = (b - a).length();
        const Pcurve pc = pcurveOn(s, c, 0.0, len, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 0.0, len, pc, bound);
    }
    {  // A parallel of a cone, at v = 1.5.
        const Cone s(f, 2.0, 0.4, 1.0);
        const CircleCurve c(Frame::fromAxis(f.origin + f.z * 1.5, f.z, {1.0, 0.0, 0.3}),
                            2.0 + 1.5 * std::tan(0.4));
        const Pcurve pc = pcurveOn(s, c, 0.0, 6.0, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 0.0, 6.0, pc, bound);
    }
    {  // A parallel of a sphere.
        const Sphere s(f, 3.0);
        const CircleCurve c(
            Frame::fromAxis(f.origin + f.z * (3.0 * std::sin(0.5)), -f.z, {0.0, 1.0, 0.3}),
            3.0 * std::cos(0.5));
        const Pcurve pc = pcurveOn(s, c, 1.0, 4.0, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 1.0, 4.0, pc, bound);
    }
    for (const double t0 : {0.4, kPi / 2.0}) {  // A meridian of a sphere, over the pole.
        const Sphere s(f, 3.0);
        const double u0 = 0.7;
        const math::Vec3 e1 = f.x * std::cos(u0) + f.y * std::sin(u0);
        const CircleCurve c(Frame::fromAxis(f.origin, f.z.cross(e1), e1), 3.0);
        const Pcurve pc = pcurveOn(s, c, t0, t0 + 2.0, tol);
        const Line2D* line = as<Line2D>(pc);
        ASSERT_NE(line, nullptr);
        EXPECT_NEAR(line->start().x, line->end().x, 1e-13);
        expectFollows(s, c, t0, t0 + 2.0, pc, bound);
    }
    {  // A parallel of a torus.
        const Torus s(f, 4.0, 1.5);
        const CircleCurve c(Frame::fromAxis(f.origin + f.z * (1.5 * std::sin(1.0)), f.z, {1, 0, 0}),
                            4.0 + 1.5 * std::cos(1.0));
        const Pcurve pc = pcurveOn(s, c, 0.5, 3.0, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 0.5, 3.0, pc, bound);
    }
    {  // A meridian of a torus.
        const Torus s(f, 4.0, 1.5);
        const double u0 = 2.0;
        const math::Vec3 e1 = f.x * std::cos(u0) + f.y * std::sin(u0);
        const CircleCurve c(Frame::fromAxis(f.origin + e1 * 4.0, f.z.cross(e1), e1), 1.5);
        const Pcurve pc = pcurveOn(s, c, 0.2, 5.0, tol);
        const Line2D* line = as<Line2D>(pc);
        ASSERT_NE(line, nullptr);
        EXPECT_NEAR(line->start().x, line->end().x, 1e-13);
        expectFollows(s, c, 0.2, 5.0, pc, bound);
    }
}

TEST(Pcurve, ARimAcrossTheSeamIsContinuous) {
    const Cylinder s(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 2.0);
    const CircleCurve c(Frame::fromAxis({0, 0, 3}, {0, 0, 1}), 2.0);
    const Pcurve pc = pcurveOn(s, c, 3.0 * kPi / 2.0, 5.0 * kPi / 2.0, 1e-6);
    expectFollows(s, c, 3.0 * kPi / 2.0, 5.0 * kPi / 2.0, pc, 1e-11);
    double prev = pc.curve->evaluate(pc.start).x;
    EXPECT_NEAR(prev, 3.0 * kPi / 2.0, 1e-12);
    for (int k = 1; k <= 100; ++k) {
        const double u = pc.curve->evaluate(pc.start + (pc.end - pc.start) * k / 100.0).x;
        EXPECT_NEAR(u - prev, kPi / 100.0, 1e-12);
        prev = u;
    }
    EXPECT_NEAR(prev, 5.0 * kPi / 2.0, 1e-12);
}

TEST(Pcurve, ASeamGeneratorSitsAtTheWindowOfItsStartPoint) {
    // The window is the start point's inverted u, wrapped into [0, 2 pi): a generator on the
    // seam is at u = 0, or at 2 pi where the start point inverts there.
    const Cylinder s(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 2.0);
    const LineCurve c({2.0, 0.0, 0.0}, {0, 0, 1}, 1.0);
    const Pcurve pc = pcurveOn(s, c, 0.0, 5.0, 1e-6);
    const Line2D* line = as<Line2D>(pc);
    ASSERT_NE(line, nullptr);
    const double u = line->start().x;
    EXPECT_EQ(u, s.invert(c.evaluate(0.0)).x);
    EXPECT_TRUE(std::abs(u) < 1e-12 || std::abs(u - kTwoPi) < 1e-9) << u;
    EXPECT_EQ(line->end().x, u);
    expectFollows(s, c, 0.0, 5.0, pc, 1e-11);
}

namespace {

/// A bi-quadratic patch, gently curved.
NurbsSurface makePatch() {
    const double z[3][3] = {{0.0, 0.3, 0.1}, {0.2, 0.8, 0.2}, {0.0, 0.4, 0.3}};
    std::vector<std::vector<math::Vec3>> pts(3);
    std::vector<std::vector<double>> w(3, std::vector<double>(3, 1.0));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pts[static_cast<size_t>(i)].emplace_back(2.0 * i, 2.0 * j, z[i][j]);
        }
    }
    const std::vector<double> k = {0, 0, 0, 1, 1, 1};
    return NurbsSurface(pts, w, k, k, 2, 2);
}

math::Vec2 curvedUV(double l) {
    return {0.15 + 0.6 * l + 0.15 * l * l, 0.2 + 0.3 * l + 0.3 * l * l};
}

/// The degree-8 Bezier that the patch's image of curvedUV is, exactly, found by interpolating
/// it at nine parameters.
std::vector<math::Vec3> imageControlPoints(const NurbsSurface& s) {
    constexpr int kN = 9;
    auto bern = [](int i, double l) {
        double c = 1.0;
        for (int k = 0; k < i; ++k) c = c * (kN - 1 - k) / (k + 1);
        return c * std::pow(l, i) * std::pow(1.0 - l, kN - 1 - i);
    };
    std::array<std::array<double, kN + 3>, kN> m{};
    for (int r = 0; r < kN; ++r) {
        const double l = r / double(kN - 1);
        for (int i = 0; i < kN; ++i) m[r][i] = bern(i, l);
        const math::Vec2 uv = curvedUV(l);
        const math::Vec3 p = s.evaluate(uv.x, uv.y);
        m[r][kN] = p.x;
        m[r][kN + 1] = p.y;
        m[r][kN + 2] = p.z;
    }
    for (int c = 0; c < kN; ++c) {
        int piv = c;
        for (int r = c + 1; r < kN; ++r) {
            if (std::abs(m[r][c]) > std::abs(m[piv][c])) piv = r;
        }
        std::swap(m[c], m[piv]);
        for (int r = 0; r < kN; ++r) {
            if (r == c) continue;
            const double f = m[r][c] / m[c][c];
            for (int k = c; k < kN + 3; ++k) m[r][k] -= f * m[c][k];
        }
    }
    std::vector<math::Vec3> out;
    for (int r = 0; r < kN; ++r) {
        out.emplace_back(m[r][kN] / m[r][r], m[r][kN + 1] / m[r][r], m[r][kN + 2] / m[r][r]);
    }
    return out;
}

NurbsCurve bezier(const std::vector<math::Vec3>& pts) {
    std::vector<double> knots(9, 0.0);
    knots.insert(knots.end(), 9, 1.0);
    return NurbsCurve(pts, std::vector<double>(pts.size(), 1.0), knots, 8);
}

}  // namespace

TEST(Pcurve, ASplineOnASplineIsWithinTolerance) {
    const NurbsSurface s = makePatch();
    const NurbsCurve c = bezier(imageControlPoints(s));
    const double tol = 1e-6;
    for (const double t0 : {0.0, 1.0}) {
        const double t1 = 1.0 - t0;
        const Pcurve pc = pcurveOn(s, c, t0, t1, tol);
        EXPECT_NE(as<Nurbs2D>(pc), nullptr);
        EXPECT_LE(pc.deviation, tol);
        expectFollows(s, c, t0, t1, pc, 2.0 * tol);
        const math::Vec2 a = pc.curve->evaluate(pc.start);
        const math::Vec2 want = curvedUV(t0);
        EXPECT_NEAR(a.x, want.x, 1e-5);
        EXPECT_NEAR(a.y, want.y, 1e-5);
    }
}

TEST(Pcurve, ACurveOffTheSurfaceSaysSo) {
    const NurbsSurface s = makePatch();
    const double tol = 1e-6;
    std::vector<math::Vec3> pts = imageControlPoints(s);
    for (math::Vec3& p : pts) p.z += 10.0 * tol;
    const NurbsCurve c = bezier(pts);
    const Pcurve pc = pcurveOn(s, c, 0.0, 1.0, tol);
    EXPECT_FALSE(pc.problem.empty());
    EXPECT_NE(pc.problem.find("deviation"), std::string::npos) << pc.problem;
    EXPECT_GT(pc.deviation, tol);
}

TEST(Pcurve, AReversedRangeRunsTheOtherWay) {
    const Cylinder s(tilted(), 2.0);
    const CircleCurve c(Frame::fromAxis(tilted().origin + tilted().z * 1.0, tilted().z), 2.0);
    const Pcurve fwd = pcurveOn(s, c, 1.0, 2.5, 1e-6);
    const Pcurve rev = pcurveOn(s, c, 2.5, 1.0, 1e-6);
    expectFollows(s, c, 1.0, 2.5, fwd, 1e-11);
    expectFollows(s, c, 2.5, 1.0, rev, 1e-11);
    EXPECT_GT(rev.curve->evaluate(rev.start).x, rev.curve->evaluate(rev.end).x);
    EXPECT_LT(fwd.curve->evaluate(fwd.start).x, fwd.curve->evaluate(fwd.end).x);

    // The same on the general path.
    const NurbsSurface p = makePatch();
    const NurbsCurve g = bezier(imageControlPoints(p));
    const Pcurve back = pcurveOn(p, g, 1.0, 0.0, 1e-6);
    expectFollows(p, g, 1.0, 0.0, back, 2e-6);
}

TEST(Pcurve, AGeneralRimOverTheSeamIsUnwrappedAndContinuous) {
    // A NURBS circle is no closed form: sampled, inverted from each previous point, fitted.
    const Cylinder cyl(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 2.0);
    const NurbsCurve rim = NurbsCurve::makeCircle({0, 0, 3}, 2.0);
    const Interval r = rim.range();
    // From 0.3 of the circle's range to its end.
    const double t0 = r.min + 0.3 * (r.max - r.min);
    const Pcurve pc = pcurveOn(cyl, rim, t0, r.max, 1e-6);
    expectFollows(cyl, rim, t0, r.max, pc, 2e-6);
    double prev = pc.curve->evaluate(pc.start).x;
    for (int k = 1; k <= 200; ++k) {
        const double u = pc.curve->evaluate(pc.start + (pc.end - pc.start) * k / 200.0).x;
        EXPECT_LT(std::abs(u - prev), 0.2);
        prev = u;
    }
    // From half way round to the end: u runs from pi to 2 pi.
    const Pcurve full = pcurveOn(cyl, rim, r.min + 0.5 * (r.max - r.min), r.max, 1e-6);
    EXPECT_TRUE(full.problem.empty()) << full.problem;
    const double u1 = full.curve->evaluate(full.end).x;
    const double u0 = full.curve->evaluate(full.start).x;
    EXPECT_NEAR(u0, kPi, 1e-5);
    EXPECT_NEAR(u1, kTwoPi, 1e-5);
}

TEST(Pcurve, AGeneralCurveOnASphereAndATorusFollows) {
    const Frame f = tilted();
    const Sphere sph(f, 3.0);
    // A tilted small circle on the sphere: not a parallel, not a meridian.
    const math::Vec3 axis = (f.z + f.x * 0.5).normalized();
    const double h = 1.2;
    const math::Vec3 center = f.origin + axis * h;
    const NurbsCurve small = NurbsCurve::makeCircle(center, std::sqrt(9.0 - h * h), axis);
    const Interval r = small.range();
    const Pcurve a = pcurveOn(sph, small, r.min, r.max, 1e-6);
    expectFollows(sph, small, r.min, r.max, a, 2e-6);

    const Torus tor(f, 4.0, 1.5);
    // The torus's own outer equator as a NURBS circle.
    const NurbsCurve eq = NurbsCurve::makeCircle(f.origin, 5.5, f.z);
    const Interval q = eq.range();
    const Pcurve b = pcurveOn(tor, eq, q.min, q.max, 1e-6);
    expectFollows(tor, eq, q.min, q.max, b, 2e-6);
}
