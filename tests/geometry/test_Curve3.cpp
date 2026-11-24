#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "../PortableRandom.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/curves/Nurbs2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"

using namespace hz::geo;
namespace math = hz::math;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

void expectNear(const math::Vec3& a, const math::Vec3& b, double tol = 1e-12) {
    EXPECT_NEAR(a.x, b.x, tol);
    EXPECT_NEAR(a.y, b.y, tol);
    EXPECT_NEAR(a.z, b.z, tol);
}

Frame tiltedFrame() {
    return Frame::fromAxis({1.0, -2.0, 0.5}, {0.3, -0.5, 1.0}, {1.0, 0.2, 0.0});
}

std::vector<double> samples(const Curve3& c) {
    const Interval r = c.range();
    if (std::isinf(r.min) || std::isinf(r.max)) {
        return {-2.0, -0.4, 0.0, 0.7, 3.0};
    }
    // Clear of a NURBS circle's knots, where d2 jumps.
    std::vector<double> out;
    for (double f : {0.05, 0.23, 0.4, 0.71, 0.93}) {
        out.push_back(r.min + f * (r.max - r.min));
    }
    return out;
}

void checkDerivatives(const Curve3& c) {
    const double h = 1e-6;
    for (double t : samples(c)) {
        const CurveDerivatives d = c.derivatives(t);
        expectNear(d.point, c.evaluate(t));
        const math::Vec3 d1 = (c.evaluate(t + h) - c.evaluate(t - h)) / (2 * h);
        expectNear(d.d1, d1, 1e-6 * std::max(1.0, d.d1.length()));
        const math::Vec3 d2 = (c.derivatives(t + h).d1 - c.derivatives(t - h).d1) / (2 * h);
        expectNear(d.d2, d2, 1e-6 * std::max(1.0, d.d2.length()));
    }
}

void checkClosest(const Curve3& c) {
    for (double t : samples(c)) {
        const double got = c.closestParameter(c.evaluate(t));
        double diff = std::abs(got - t);
        if (c.periodic()) {
            const double period = c.range().max - c.range().min;
            diff = std::min(diff, period - diff);
            EXPECT_GE(got, c.range().min);
            EXPECT_LT(got, c.range().max);
        }
        EXPECT_NEAR(diff, 0.0, 1e-12 * std::max(1.0, std::abs(t))) << t;
    }
}

void checkTransformed(const Curve3& c, const math::Mat4& m) {
    const auto moved = c.transformed(m);
    EXPECT_EQ(moved->kind(), c.kind());
    for (double t : samples(c)) {
        expectNear(moved->evaluate(t), m.transformPoint(c.evaluate(t)), 1e-11);
    }
    checkClosest(*moved);
    checkDerivatives(*moved);
}

std::vector<math::Mat4> similarities() {
    const math::Mat4 rot = math::Mat4::rotationZ(0.7) * math::Mat4::rotationX(-0.4);
    const math::Mat4 move = math::Mat4::translation({3.0, -1.0, 2.0});
    return {move * rot, move * math::Mat4::scale({2.5, 2.5, 2.5}) * rot,
            math::Mat4::scale({1.0, 1.0, -1.0}) * rot * move,
            move * math::Mat4::scale({-0.5, 0.5, 0.5})};
}

std::vector<std::unique_ptr<Curve3>> curves() {
    std::vector<std::unique_ptr<Curve3>> v;
    v.push_back(std::make_unique<LineCurve>(math::Vec3{1.0, 2.0, 3.0}, math::Vec3{1.0, 2.0, -2.0}));
    v.push_back(std::make_unique<CircleCurve>(tiltedFrame(), 2.0));
    v.push_back(std::make_unique<EllipseCurve>(tiltedFrame(), 3.0, 1.5));
    v.push_back(std::make_unique<NurbsCurve>(
        NurbsCurve::makeCircle({1.0, 0.0, 0.0}, 2.0, {0.0, 1.0, 1.0})));
    return v;
}

}  // namespace

TEST(Curve3, Kinds) {
    const auto cs = curves();
    EXPECT_EQ(cs[0]->kind(), CurveKind::Line);
    EXPECT_EQ(cs[1]->kind(), CurveKind::Circle);
    EXPECT_EQ(cs[2]->kind(), CurveKind::Ellipse);
    EXPECT_EQ(cs[3]->kind(), CurveKind::Nurbs);
    EXPECT_FALSE(cs[0]->periodic());
    EXPECT_TRUE(cs[1]->periodic());
    EXPECT_TRUE(cs[2]->periodic());
    EXPECT_FALSE(cs[3]->periodic());
}

TEST(Curve3, LineIsParametrisedByDistance) {
    const LineCurve l({1.0, 0.0, 0.0}, {0.0, 3.0, 4.0});
    expectNear(l.evaluate(5.0), {1.0, 3.0, 4.0});
    EXPECT_TRUE(std::isinf(l.range().min));
    EXPECT_TRUE(std::isinf(l.range().max));
}

TEST(Curve3, Derivatives) {
    for (const auto& c : curves()) {
        checkDerivatives(*c);
    }
}

TEST(Curve3, ClosestParameterRoundTrip) {
    for (const auto& c : curves()) {
        checkClosest(*c);
    }
}

TEST(Curve3, TransformedKeepsParameters) {
    for (const auto& c : curves()) {
        for (const auto& m : similarities()) {
            checkTransformed(*c, m);
        }
    }
}

TEST(Curve3, CircleCentreGivesParameterZero) {
    const CircleCurve c(tiltedFrame(), 2.0);
    EXPECT_EQ(c.closestParameter(tiltedFrame().origin), 0.0);
    const EllipseCurve e(tiltedFrame(), 3.0, 1.5);
    EXPECT_EQ(e.closestParameter(tiltedFrame().origin), 0.0);
}

TEST(Curve3, CircleSeamWrapsToZero) {
    const CircleCurve c(Frame{}, 1.0);
    const double t = c.closestParameter({1.0, -1e-18, 0.0});
    EXPECT_GE(t, 0.0);
    EXPECT_LT(t, kTwoPi);
    EXPECT_EQ(c.closestParameter({1.0, 0.0, 0.0}), 0.0);
}

TEST(Curve3, EllipseClosestNearAxisEnds) {
    const EllipseCurve e(Frame{}, 5.0, 0.1);
    for (double t : {0.0, 1e-4, kPi / 2, kPi / 2 + 1e-3, kPi, 3 * kPi / 2, kTwoPi - 1e-4}) {
        const double got = e.closestParameter(e.evaluate(t));
        double diff = std::abs(got - t);
        diff = std::min(diff, kTwoPi - diff);
        EXPECT_NEAR(diff, 0.0, 1e-10) << t;
    }
}

TEST(Curve3, EllipseClosestOffCurve) {
    const EllipseCurve e(Frame{}, 3.0, 1.0);
    const math::Vec3 p{2.0, 2.0, 0.5};
    const double t = e.closestParameter(p);
    const double h = 1e-3;
    const double d0 = (e.evaluate(t) - p).length();
    EXPECT_LE(d0, (e.evaluate(t + h) - p).length() + 1e-12);
    EXPECT_LE(d0, (e.evaluate(t - h) - p).length() + 1e-12);
}

TEST(Curve3, LineScaleKeepsParameters) {
    const LineCurve l({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    const auto s = l.transformed(math::Mat4::scale({3.0, 3.0, 3.0}));
    expectNear(s->evaluate(2.0), {6.0, 0.0, 0.0});
    EXPECT_NEAR(s->closestParameter({6.0, 1.0, 0.0}), 2.0, 1e-12);
}

TEST(Curve3, ALineWithATinyDirectionStillRunsAlongIt) {
    // A direction is a direction at any length: normalized() returned zero below 1e-7, which
    // made a line of a part 1e-8 across a point.
    const LineCurve l({0.0, 0.0, 0.0}, {1e-8, 0.0, 0.0});
    expectNear(l.direction(), {1.0, 0.0, 0.0});
    expectNear(l.evaluate(1.0), {1.0, 0.0, 0.0});
    // Scaled down by 1e-8, it keeps its parameters: t = 1 is 1e-8 along.
    const auto tiny = LineCurve({0.0, 0.0, 0.0}, {0.0, 3.0, 4.0})
                          .transformed(math::Mat4::scale({1e-8, 1e-8, 1e-8}));
    expectNear(tiny->evaluate(1.0), {0.0, 0.6e-8, 0.8e-8}, 1e-22);
    EXPECT_NEAR(tiny->closestParameter({0.0, 0.6e-8, 0.8e-8}), 1.0, 1e-12);
}

TEST(Curve3, NurbsRangeAndTransform) {
    const NurbsCurve c = NurbsCurve::makeCircle({0.0, 0.0, 0.0}, 1.0);
    EXPECT_EQ(c.range().min, c.tMin());
    EXPECT_EQ(c.range().max, c.tMax());
}

TEST(Nurbs2D, QuarterCircle) {
    const double w = std::sqrt(0.5);
    const Nurbs2D q({{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}, {1.0, w, 1.0},
                    {0.0, 0.0, 0.0, 1.0, 1.0, 1.0}, 2);
    EXPECT_EQ(q.tMin(), 0.0);
    EXPECT_EQ(q.tMax(), 1.0);
    EXPECT_FALSE(q.isClosed());
    const double h = 1e-6;
    for (double t : {0.0, 0.2, 0.5, 0.8, 1.0}) {
        const math::Vec2 p = q.evaluate(t);
        EXPECT_NEAR(p.length(), 1.0, 1e-12);
        const double a = std::max(0.0, t - h);
        const double b = std::min(1.0, t + h);
        const math::Vec2 d = (q.evaluate(b) - q.evaluate(a)) / (b - a);
        const math::Vec2 got = q.derivative(t, 1);
        EXPECT_NEAR(got.x, d.x, 1e-5);  // one-sided differences at the ends
        EXPECT_NEAR(got.y, d.y, 1e-5);
    }
}

TEST(Nurbs2D, ClosedWhenEndsCoincide) {
    const Nurbs2D tri({{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 0.0}}, {1.0, 1.0, 1.0, 1.0},
                      {0.0, 0.0, 1.0, 2.0, 3.0, 3.0}, 1);
    EXPECT_TRUE(tri.isClosed());
}

TEST(Curve3, EllipseClosestMatchesBruteForce) {
    std::mt19937 rng(20261124);
    const hz::test::Uniform radius(1.0, 5.0), ratio(0.1, 1.0), unit(-1.0, 1.0);
    for (int i = 0; i < 400; ++i) {
        const double a = radius(rng);
        const double b = a * ratio(rng);
        const EllipseCurve e(tiltedFrame(), a, b);
        // Inside, outside, near the axes: the offsets run to 1.5 a; every fourth is on an axis.
        double ox = 1.5 * a * unit(rng);
        double oy = 1.5 * a * unit(rng);
        if (i % 4 == 0) {
            oy *= 1e-6;
        } else if (i % 4 == 1) {
            ox *= 1e-6;
        }
        const Frame f = tiltedFrame();
        const math::Vec3 p = f.origin + f.x * ox + f.y * oy + f.z * (0.3 * unit(rng));
        const double t = e.closestParameter(p);
        EXPECT_GE(t, 0.0);
        EXPECT_LT(t, kTwoPi);
        double best = 1e300;
        const int n = 20000;
        for (int k = 0; k < n; ++k) {
            best = std::min(best, (e.evaluate(kTwoPi * k / n) - p).length());
        }
        const double got = (e.evaluate(t) - p).length();
        EXPECT_LE(got, best * (1.0 + 1e-9) + 1e-12) << i << " a=" << a << " b=" << b;
    }
}

namespace {
/// The returned parameter's in-plane distance to @p p is the brute-force minimum.
void expectEllipseFoot(const EllipseCurve& e, const math::Vec3& p, const char* what) {
    const double t = e.closestParameter(p);
    EXPECT_GE(t, 0.0) << what;
    EXPECT_LT(t, kTwoPi) << what;
    double best = 1e300;
    const int n = 200000;
    for (int k = 0; k < n; ++k) {
        best = std::min(best, (e.evaluate(kTwoPi * k / n) - p).length());
    }
    EXPECT_LE((e.evaluate(t) - p).length(), best * (1.0 + 1e-9) + 1e-12) << what;
}
}  // namespace

TEST(Curve3, EllipseClosestJustInsideTheEvoluteCusp) {
    // On the major axis just inside the cusp x = (a^2 - b^2) / a, the vertex is a stationary
    // maximum and the two feet lie close beside it.
    for (double ratio : {0.5, 0.1, 0.01}) {
        const double a = 2.0;
        const double b = a * ratio;
        const EllipseCurve e(Frame{}, a, b);
        for (int k : {1, 2, 5, 10, 30, 100}) {
            const double ox = (a * a - b * b) / a * (1.0 - k * 1e-3);
            expectEllipseFoot(e, {ox, 0.0, 0.0}, "major axis");
            expectEllipseFoot(e, {-ox, 0.0, 0.0}, "major axis, negative side");
        }
    }
    expectEllipseFoot(EllipseCurve(Frame{}, 2.0, 1.0), {1.4988, 0.0, 0.0}, "a=2 b=1");
    expectEllipseFoot(EllipseCurve(Frame{}, 2.0, 0.2), {1.9796, 0.0, 0.0}, "a=2 b=0.2");
}

TEST(Curve3, EllipseClosestOnTheMinorAxis) {
    for (double ratio : {0.9, 0.5, 0.1}) {
        const double a = 2.0;
        const double b = a * ratio;
        const EllipseCurve e(Frame{}, a, b);
        for (double oy : {-1.5, -0.7, -0.1, 0.1, 0.7, 1.5, 1.9}) {
            expectEllipseFoot(e, {0.0, oy * b, 0.0}, "minor axis");
        }
    }
}
