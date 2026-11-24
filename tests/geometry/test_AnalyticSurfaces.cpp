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
const std::vector<double> kVs = {-2.0, -0.4, 0.0, 0.7, 3.0};

/// A tilted, offset frame so the shared checks are not axis-aligned.
Frame tiltedFrame() {
    return Frame::fromAxis({1.0, -2.0, 0.5}, {0.3, -0.5, 1.0}, {1.0, 0.2, 0.0});
}

void checkDerivatives(const Surface& s) {
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

void checkInvertRoundTrip(const Surface& s) {
    for (double u : kUs) {
        for (double v : kVs) {
            const math::Vec2 uv = s.invert(s.evaluate(u, v));
            EXPECT_NEAR(uv.x, u, 1e-12 * std::max(1.0, std::abs(u))) << u << "," << v;
            EXPECT_NEAR(uv.y, v, 1e-12 * std::max(1.0, std::abs(v))) << u << "," << v;
        }
    }
}

void checkTransformCommutes(const Surface& s, const math::Mat4& m) {
    const auto t = s.transformed(m);
    ASSERT_NE(t, nullptr);
    EXPECT_EQ(t->kind(), s.kind());
    for (double u : kUs) {
        for (double v : kVs) {
            expectNear(t->evaluate(u, v), m.transformPoint(s.evaluate(u, v)), 1e-11);
        }
    }
}

void checkAll(const Surface& s) {
    checkDerivatives(s);
    checkInvertRoundTrip(s);
    const math::Mat4 rigid = math::Mat4::translation({3.0, -1.0, 2.0}) *
                             math::Mat4::rotationZ(0.7) * math::Mat4::rotationX(-0.4);
    checkTransformCommutes(s, rigid);
    checkTransformCommutes(s, math::Mat4::scale(2.5));
    checkTransformCommutes(s, math::Mat4::scale({-1.0, 1.0, 1.0}));
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
    }
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
