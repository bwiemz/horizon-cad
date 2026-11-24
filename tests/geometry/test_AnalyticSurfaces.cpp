#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "horizon/geometry/surfaces/AnalyticSurfaces.h"

using namespace hz::geo;
namespace math = hz::math;

namespace {

constexpr double kPi = 3.14159265358979323846;

void expectNear(const math::Vec3& a, const math::Vec3& b, double tol = 1e-12) {
    EXPECT_NEAR(a.x, b.x, tol);
    EXPECT_NEAR(a.y, b.y, tol);
    EXPECT_NEAR(a.z, b.z, tol);
}

// Parameter samples away from the cylinder's seam.
const std::vector<double> kUs = {0.3, 1.1, 2.5, 3.9, 5.5};
const std::vector<double> kVsDefault = {-2.0, -0.4, 0.0, 0.7, 3.0};

/// A tilted, offset frame so the shared checks are not axis-aligned.
Frame tiltedFrame() {
    return Frame::fromAxis({1.0, -2.0, 0.5}, {0.3, -0.5, 1.0}, {1.0, 0.2, 0.0});
}

void checkDerivatives(const Surface& s, const std::vector<double>& kVs = kVsDefault) {
    const double h = 1e-6;
    for (double u : kUs) {
        for (double v : kVs) {
            const SurfaceDerivatives d = s.derivatives(u, v);
            expectNear(d.point, s.evaluate(u, v));
            const math::Vec3 du = (s.evaluate(u + h, v) - s.evaluate(u - h, v)) / (2 * h);
            const math::Vec3 dv = (s.evaluate(u, v + h) - s.evaluate(u, v - h)) / (2 * h);
            expectNear(d.du, du, 1e-6 * std::max(1.0, d.du.length()));
            expectNear(d.dv, dv, 1e-6 * std::max(1.0, d.dv.length()));
            const math::Vec3 duu =
                (s.derivatives(u + h, v).du - s.derivatives(u - h, v).du) / (2 * h);
            const math::Vec3 duv =
                (s.derivatives(u, v + h).du - s.derivatives(u, v - h).du) / (2 * h);
            const math::Vec3 dvv =
                (s.derivatives(u, v + h).dv - s.derivatives(u, v - h).dv) / (2 * h);
            expectNear(d.duu, duu, 1e-6 * std::max(1.0, d.duu.length()));
            expectNear(d.duv, duv, 1e-6 * std::max(1.0, d.duv.length()));
            expectNear(d.dvv, dvv, 1e-6 * std::max(1.0, d.dvv.length()));
        }
    }
}

void checkInvertRoundTrip(const Surface& s, const std::vector<double>& kVs = kVsDefault) {
    for (double u : kUs) {
        for (double v : kVs) {
            const math::Vec2 uv = s.invert(s.evaluate(u, v));
            EXPECT_NEAR(uv.x, u, 1e-12 * std::max(1.0, std::abs(u))) << u << "," << v;
            EXPECT_NEAR(uv.y, v, 1e-12 * std::max(1.0, std::abs(v))) << u << "," << v;
        }
    }
}

void checkTransformCommutes(const Surface& s, const math::Mat4& m,
                            const std::vector<double>& kVs = kVsDefault) {
    const auto t = s.transformed(m);
    ASSERT_NE(t, nullptr);
    EXPECT_EQ(t->kind(), s.kind());
    for (double u : kUs) {
        for (double v : kVs) {
            expectNear(t->evaluate(u, v), m.transformPoint(s.evaluate(u, v)), 1e-11);
        }
    }
}

void checkAll(const Surface& s, const std::vector<double>& kVs = kVsDefault) {
    checkDerivatives(s, kVs);
    checkInvertRoundTrip(s, kVs);
    const math::Mat4 rigid = math::Mat4::translation({3.0, -1.0, 2.0}) *
                             math::Mat4::rotationZ(0.7) * math::Mat4::rotationX(-0.4);
    for (const math::Mat4& m :
         {rigid, math::Mat4::scale(2.5), math::Mat4::scale({-1.0, 1.0, 1.0})}) {
        checkTransformCommutes(s, m, kVs);
        const auto t = s.transformed(m);
        checkDerivatives(*t, kVs);
        checkInvertRoundTrip(*t, kVs);
    }
}

}  // namespace

TEST(AnalyticSurfaces, ThePlaneSatisfiesTheSharedChecks) {
    checkAll(Plane(tiltedFrame()));
}

TEST(AnalyticSurfaces, TheCylinderSatisfiesTheSharedChecks) {
    checkAll(Cylinder(tiltedFrame(), 1.7));
}

TEST(AnalyticSurfaces, APlaneMatchesItsFormula) {
    const Plane p(Frame::fromAxis({1, 2, 3}, {0, 0, 1}));
    expectNear(p.evaluate(2.0, 5.0), {3, 7, 3});
    expectNear(p.normal(0.0, 0.0), {0, 0, 1});
    EXPECT_FALSE(p.periodicU());
    EXPECT_TRUE(std::isinf(p.uRange().min) && std::isinf(p.vRange().max));
}

TEST(AnalyticSurfaces, AFrameFromAParallelHintStillHasAPerpendicularX) {
    const Frame f = Frame::fromAxis({0, 0, 0}, {0, 0, 2}, {0, 0, 1});
    EXPECT_NEAR(f.x.length(), 1.0, 1e-12);
    EXPECT_NEAR(f.x.dot(f.z), 0.0, 1e-12);
    expectNear(f.y, f.z.cross(f.x));
}

TEST(AnalyticSurfaces, ACylinderMatchesItsFormula) {
    const Cylinder c(Frame::fromAxis({1, 2, 3}, {0, 0, 1}), 4.0);
    expectNear(c.evaluate(0.0, 0.0), {5, 2, 3});
    expectNear(c.evaluate(kPi / 2, 7.0), {1, 6, 10});
    expectNear(c.normal(kPi, 0.0), {-1, 0, 0});  // outward
    EXPECT_TRUE(c.periodicU());
    EXPECT_FALSE(c.periodicV());
    EXPECT_DOUBLE_EQ(c.uRange().max, 2 * kPi);
}

TEST(AnalyticSurfaces, InversionAtTheSeamStaysInRange) {
    const Cylinder c(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 1.0);
    for (double u : {-1e-12, 2 * kPi + 1e-12, 2 * kPi - 1e-12}) {
        const math::Vec2 uv = c.invert(c.evaluate(u, 0.5));
        EXPECT_GE(uv.x, 0.0);
        EXPECT_LT(uv.x, 2 * kPi) << u;
        if (u > kPi) {
            if (u > 2 * kPi) {
                EXPECT_NEAR(uv.x, 0.0, 1e-9) << u;
            } else {
                EXPECT_NEAR(uv.x, 2 * kPi, 1e-9) << u;
            }
        } else {
            EXPECT_NEAR(uv.x, 2 * kPi, 1e-9) << u;
        }
    }
}

TEST(AnalyticSurfaces, AFrameFromANearlyParallelHintIsOrthonormal) {
    const Frame f = Frame::fromAxis({0, 0, 0}, {0, 0, 1}, {1e-12, 0, 1});
    EXPECT_NEAR(f.x.length(), 1.0, 1e-12);
    EXPECT_NEAR(f.x.dot(f.z), 0.0, 1e-12);
    EXPECT_NEAR(f.y.dot(f.x), 0.0, 1e-12);
    const Frame g = Frame::fromAxis({0, 0, 0}, {0, 0, 1}, {1e-14, 0, 1e-14});
    EXPECT_NEAR(g.x.length(), 1.0, 1e-12);
}

TEST(AnalyticSurfaces, APointOnTheAxisInvertsToFiniteParameters) {
    const Cylinder c(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 1.0);
    const math::Vec2 uv = c.invert({0, 0, 3});
    EXPECT_DOUBLE_EQ(uv.x, 0.0);
    EXPECT_DOUBLE_EQ(uv.y, 3.0);
}

TEST(AnalyticSurfaces, AMirroredCylinderKeepsItsParametersAndTurnsItsNormal) {
    const Cylinder c(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 2.0);
    const math::Mat4 mirror = math::Mat4::scale({-1, 1, 1});
    const auto m = c.transformed(mirror);
    expectNear(m->evaluate(0.3, 1.0), mirror.transformPoint(c.evaluate(0.3, 1.0)));
    // du x dv now points into the cylinder: a face on it is `reversed`.
    EXPECT_LT(m->normal(0.0, 0.0).dot(m->evaluate(0.0, 0.0)), 0.0);
}

TEST(AnalyticSurfaces, AScaledCylinderScalesItsRadius) {
    const Cylinder c(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 2.0);
    const auto s = c.transformed(math::Mat4::scale(2.5));
    EXPECT_DOUBLE_EQ(static_cast<const Cylinder&>(*s).radius(), 5.0);
}

// ---- cones, spheres, tori ----

TEST(AnalyticSurfaces, TheConeSatisfiesTheSharedChecks) {
    // r = 3.2, tan 0.5 = 0.55: rho stays positive for v >= -2, as the checks sample.
    checkAll(Cone(tiltedFrame(), 3.2, 0.5));
    checkAll(Cone(tiltedFrame(), 3.2, 0.5, 1.7));
}

TEST(AnalyticSurfaces, TheSphereSatisfiesTheSharedChecks) {
    checkAll(Sphere(tiltedFrame(), 2.3), {-1.2, -0.4, 0.0, 0.7, 1.4});
}

TEST(AnalyticSurfaces, TheTorusSatisfiesTheSharedChecks) {
    checkAll(Torus(tiltedFrame(), 5.0, 1.3), {0.3, 1.1, 2.5, 3.9, 5.5});
}

const Cone kCone(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 2.0, kPi / 4);

TEST(AnalyticSurfaces, ConeSphereAndTorusMatchTheirFormulas) {
    expectNear(kCone.evaluate(0.0, 1.0), {3, 0, 1});    // r grows with v
    expectNear(kCone.evaluate(0.0, -2.0), {0, 0, -2});  // the apex
    const Sphere s(Frame::fromAxis({1, 1, 1}, {0, 0, 1}), 3.0);
    expectNear(s.evaluate(0.0, kPi / 2), {1, 1, 4});  // the pole
    EXPECT_DOUBLE_EQ(s.vRange().min, -kPi / 2);
    const Torus t(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 1.0);
    expectNear(t.evaluate(0.0, 0.0), {6, 0, 0});
    expectNear(t.evaluate(0.0, kPi), {4, 0, 0});
    EXPECT_TRUE(t.periodicU());
    EXPECT_TRUE(t.periodicV());
}

TEST(AnalyticSurfaces, CentresAndAxesInvertToFiniteParameters) {
    const Sphere s(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 1.0);
    const math::Vec2 c = s.invert({0, 0, 0});
    EXPECT_TRUE(std::isfinite(c.x) && std::isfinite(c.y));
    const math::Vec2 pole = s.invert({0, 0, 5});
    EXPECT_DOUBLE_EQ(pole.x, 0.0);
    EXPECT_DOUBLE_EQ(pole.y, kPi / 2);
    const Torus t(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 1.0);
    const math::Vec2 axis = t.invert({0, 0, 2});
    EXPECT_TRUE(std::isfinite(axis.x) && std::isfinite(axis.y));
    EXPECT_NEAR(Sphere(Frame{}, 1.0).normal(0.0, kPi / 2).length(), 1.0, 1e-12)
        << "a sphere's pole has a normal (the limit along its meridians)";
    EXPECT_DOUBLE_EQ(kCone.normal(0.0, -2.0).length(), 0.0) << "a cone's apex has none";
}

TEST(AnalyticSurfaces, NormalsPointOutwardAndFlipUnderAReflection) {
    const Frame f = Frame::fromAxis({1, 2, 3}, {0, 0, 1});
    const Sphere s(f, 2.0);
    expectNear(s.normal(0.0, 0.0), {1, 0, 0});
    const Torus t(f, 5.0, 1.0);
    expectNear(t.normal(0.0, 0.0), {1, 0, 0});
    expectNear(t.normal(0.0, kPi), {-1, 0, 0});
    const auto m = s.transformed(math::Mat4::scale({-1, 1, 1}));
    EXPECT_LT(m->normal(0.4, 0.3).dot(m->evaluate(0.4, 0.3) - math::Vec3{-1, 2, 3}), 0.0);
    EXPECT_NEAR(m->normal(0.0, kPi / 2).length(), 1.0, 1e-12);
}

TEST(AnalyticSurfaces, ScalingScalesRadiiAndKeepsAngles) {
    const auto c = Cone(Frame{}, 2.0, 0.5, 1.0).transformed(math::Mat4::scale(2.5));
    const auto& cone = static_cast<const Cone&>(*c);
    EXPECT_DOUBLE_EQ(cone.radius(), 5.0);
    EXPECT_DOUBLE_EQ(cone.axialScale(), 2.5);
    EXPECT_DOUBLE_EQ(cone.halfAngle(), 0.5);
    const auto sp = Sphere(Frame{}, 2.0).transformed(math::Mat4::scale(2.5));
    EXPECT_DOUBLE_EQ(static_cast<const Sphere&>(*sp).radius(), 5.0);
    const auto to = Torus(Frame{}, 4.0, 1.0).transformed(math::Mat4::scale(2.5));
    EXPECT_DOUBLE_EQ(static_cast<const Torus&>(*to).majorRadius(), 10.0);
    EXPECT_DOUBLE_EQ(static_cast<const Torus&>(*to).minorRadius(), 2.5);
}

TEST(AnalyticSurfaces, InvertingOffTheSurfaceGivesTheNearestPointsParameters) {
    const Sphere s(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 2.0);
    const math::Vec2 uv = s.invert({0, 6, 0});
    EXPECT_NEAR(uv.x, kPi / 2, 1e-12);
    EXPECT_NEAR(uv.y, 0.0, 1e-12);
    const Torus t(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 1.0);
    const math::Vec2 tv = t.invert({8, 0, 0});
    EXPECT_NEAR(tv.x, 0.0, 1e-12);
    EXPECT_NEAR(tv.y, 0.0, 1e-12);
    for (const math::Vec2 q : {s.invert({0, 0, -1e-300}), t.invert({5.0, -1e-17, 1e-17})}) {
        EXPECT_GE(q.x, 0.0);
        EXPECT_LT(q.x, 2 * kPi);
    }
}

TEST(AnalyticSurfaces, AFrameFromAShortAxisIsStillUnit) {
    const Frame f = Frame::fromAxis({0, 0, 0}, {0, 0, 1e-9});
    EXPECT_NEAR(f.z.length(), 1.0, 1e-12);
    expectNear(f.z, {0, 0, 1});
    EXPECT_NEAR(f.x.length(), 1.0, 1e-12);
}

TEST(AnalyticSurfaces, ConeInvertsPastItsApexOntoTheOtherNappe) {
    const Cone c(tiltedFrame(), 3.2, std::atan(0.55));  // apex at v = -5.82
    for (double u : kUs) {
        for (double v : {-5.5, -8.0, -30.0}) {
            const math::Vec2 uv = c.invert(c.evaluate(u, v));
            expectNear(c.evaluate(uv.x, uv.y), c.evaluate(u, v), 1e-9);
            EXPECT_GE(uv.x, 0.0);
            EXPECT_LT(uv.x, 2 * kPi);
        }
    }
}

TEST(AnalyticSurfaces, ConeInvertsOffSurfacePointsToTheNearestGenerator) {
    const Cone c(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 0.0, std::atan(0.1));  // apex at 0
    // Far below the apex the other nappe's generator (u = pi) is nearer.
    const math::Vec2 uv = c.invert({1, 0, -100});
    EXPECT_NEAR(uv.x, kPi, 1e-12);
    const math::Vec3 hit = c.evaluate(uv.x, uv.y);
    EXPECT_NEAR((math::Vec3{1, 0, -100} - hit).length(), 8.96, 0.01);
    // The nearest point to a point on the axis below the apex is on a generator, at u = 0.
    const math::Vec2 axis = c.invert({0, 0, -3});
    EXPECT_DOUBLE_EQ(axis.x, 0.0);
    EXPECT_NEAR(axis.y, -3.0 / 1.01, 1e-12);
    // The apex itself.
    const math::Vec2 apex = c.invert({0, 0, 0});
    expectNear(c.evaluate(apex.x, apex.y), {0, 0, 0});
}
