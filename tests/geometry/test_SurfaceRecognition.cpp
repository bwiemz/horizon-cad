#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/geometry/surfaces/SurfaceRecognition.h"

using namespace hz::geo;
namespace math = hz::math;

namespace {

constexpr double kTol = 1e-9;

void expectNear(const math::Vec3& a, const math::Vec3& b, double tol = kTol) {
    EXPECT_NEAR(a.x, b.x, tol);
    EXPECT_NEAR(a.y, b.y, tol);
    EXPECT_NEAR(a.z, b.z, tol);
}

/// A 20 x 20 grid of the NURBS lies on the recognised surface within 1e-9.
void expectOnSurface(const NurbsSurface& n, const Surface& s) {
    for (int i = 0; i <= 19; ++i) {
        for (int j = 0; j <= 19; ++j) {
            const double u = n.uMin() + (n.uMax() - n.uMin()) * i / 19.0;
            const double v = n.vMin() + (n.vMax() - n.vMin()) * j / 19.0;
            const math::Vec3 p = n.evaluate(u, v);
            const math::Vec2 uv = s.invert(p);
            const math::Vec3 q = s.evaluate(uv.x, uv.y);
            EXPECT_LT((p - q).length(), kTol) << "at (" << u << ", " << v << ")";
        }
    }
}

/// The unit direction from the frame's origin towards @p p, projected perpendicular to z.
math::Vec3 towardPerp(const Frame& f, const math::Vec3& p) {
    const math::Vec3 d = p - f.origin;
    const math::Vec3 q = d - f.z * d.dot(f.z);
    return q / q.length();
}

/// The seam ruling: x points towards S(uMin, vMid), perpendicular to the axis.
void expectSeam(const NurbsSurface& n, const Frame& f) {
    const math::Vec3 p = n.evaluate(n.uMin(), 0.5 * (n.vMin() + n.vMax()));
    expectNear(f.x, towardPerp(f, p));
    EXPECT_NEAR(f.x.dot(f.z), 0.0, kTol);
}

const math::Vec3 kCenter{1.5, -2.0, 0.7};
const math::Vec3 kAxis = math::Vec3{0.3, -0.5, 1.0};

}  // namespace

TEST(SurfaceRecognition, RecognisesAPlane) {
    const NurbsSurface n = NurbsSurface::makePlane({1, 2, 3}, {1, 0.2, 0}, {-0.1, 1, 0.3}, 4, 2.5);
    const auto s = recognize(n, kTol);
    ASSERT_NE(s, nullptr);
    ASSERT_EQ(s->kind(), SurfaceKind::Plane);
    const auto& plane = static_cast<const Plane&>(*s);
    expectNear(plane.frame().origin, n.evaluate(n.uMin(), n.vMin()));
    const math::Vec3 normal = n.normal(0.5, 0.5);
    expectNear(plane.frame().z, normal);
    const math::Vec3 du = n.derivatives(n.uMin(), n.vMin()).du;
    expectNear(plane.frame().x, du.normalized());
    expectOnSurface(n, *s);
}

TEST(SurfaceRecognition, RecognisesATiltedPlane) {
    const NurbsSurface n =
        NurbsSurface::makePlane({-4, 1, 2}, {0.2, 0.9, 0.4}, {0.7, -0.3, 0.5}, 3, 3);
    const auto s = recognize(n, kTol);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->kind(), SurfaceKind::Plane);
    expectOnSurface(n, *s);
}

TEST(SurfaceRecognition, RecognisesACylinder) {
    const NurbsSurface n = NurbsSurface::makeCylinder(kCenter, kAxis, 1.7, 4.0);
    const auto s = recognize(n, kTol);
    ASSERT_NE(s, nullptr);
    ASSERT_EQ(s->kind(), SurfaceKind::Cylinder);
    const auto& cyl = static_cast<const Cylinder&>(*s);
    EXPECT_NEAR(cyl.radius(), 1.7, kTol);
    expectNear(cyl.frame().z, kAxis.normalized());
    // The frame's origin is on the axis, at the start of v.
    expectNear(cyl.frame().origin, kCenter);
    expectSeam(n, cyl.frame());
    expectOnSurface(n, *s);
}

TEST(SurfaceRecognition, RecognisesACone) {
    const math::Vec3 apex{-1.0, 0.5, 2.0};
    const NurbsSurface n = NurbsSurface::makeCone(apex, kAxis, 0.4, 3.0);
    const auto s = recognize(n, kTol);
    ASSERT_NE(s, nullptr);
    ASSERT_EQ(s->kind(), SurfaceKind::Cone);
    const auto& cone = static_cast<const Cone&>(*s);
    EXPECT_NEAR(cone.halfAngle(), 0.4, kTol);
    expectNear(cone.frame().z, kAxis.normalized());
    const math::Vec3 foundApex =
        cone.frame().origin - cone.frame().z * (cone.radius() / std::tan(cone.halfAngle()));
    expectNear(foundApex, apex);
    expectSeam(n, cone.frame());
    expectOnSurface(n, *s);
}

TEST(SurfaceRecognition, RecognisesASphere) {
    const NurbsSurface n = NurbsSurface::makeSphere(kCenter, 2.3);
    const auto s = recognize(n, kTol);
    ASSERT_NE(s, nullptr);
    ASSERT_EQ(s->kind(), SurfaceKind::Sphere);
    const auto& sph = static_cast<const Sphere&>(*s);
    EXPECT_NEAR(sph.radius(), 2.3, kTol);
    expectNear(sph.frame().origin, kCenter);
    // z runs from the pole the v = vMin edge collapses to towards the other pole.
    const math::Vec3 south = n.evaluate(n.uMin(), n.vMin());
    const math::Vec3 north = n.evaluate(n.uMin(), n.vMax());
    expectNear(south, n.evaluate(n.uMax(), n.vMin()));
    expectNear(sph.frame().z, (north - south) / (north - south).length());
    expectSeam(n, sph.frame());
    expectOnSurface(n, *s);
}

TEST(SurfaceRecognition, RecognisesASphereOctant) {
    const math::Vec3 e1{1.0 / 3, 2.0 / 3, 2.0 / 3};
    const math::Vec3 e2{2.0 / 3, 1.0 / 3, -2.0 / 3};
    const math::Vec3 e3{-2.0 / 3, 2.0 / 3, -1.0 / 3};
    const NurbsSurface n = NurbsSurface::makeSphereOctant(kCenter, 1.2, e1, e2, e3);
    const auto s = recognize(n, kTol);
    ASSERT_NE(s, nullptr);
    ASSERT_EQ(s->kind(), SurfaceKind::Sphere);
    const auto& sph = static_cast<const Sphere&>(*s);
    EXPECT_NEAR(sph.radius(), 1.2, kTol);
    expectNear(sph.frame().origin, kCenter);
    // The v = vMax edge collapses to e3's pole; z runs from the centre towards it.
    const math::Vec3 pole = n.evaluate(n.uMin(), n.vMax());
    expectNear(pole, n.evaluate(n.uMax(), n.vMax()));
    expectNear(pole, kCenter + e3 * 1.2);
    expectNear(sph.frame().z, e3);
    expectSeam(n, sph.frame());
}

TEST(SurfaceRecognition, RecognisesATorus) {
    const NurbsSurface n = NurbsSurface::makeTorus(kCenter, kAxis, 3.0, 0.8);
    const auto s = recognize(n, kTol);
    ASSERT_NE(s, nullptr);
    ASSERT_EQ(s->kind(), SurfaceKind::Torus);
    const auto& tor = static_cast<const Torus&>(*s);
    EXPECT_NEAR(tor.majorRadius(), 3.0, kTol);
    EXPECT_NEAR(tor.minorRadius(), 0.8, kTol);
    expectNear(tor.frame().origin, kCenter);
    expectSeam(n, tor.frame());
    EXPECT_NEAR(std::abs(tor.frame().z.dot(kAxis.normalized())), 1.0, kTol);
    expectOnSurface(n, *s);
}

TEST(SurfaceRecognition, AxisAlignedPlacements) {
    const auto cyl = recognize(NurbsSurface::makeCylinder({0, 0, 0}, {0, 0, 1}, 1.0, 2.0), kTol);
    ASSERT_NE(cyl, nullptr);
    EXPECT_EQ(cyl->kind(), SurfaceKind::Cylinder);
    const auto sph = recognize(NurbsSurface::makeSphere({0, 0, 0}, 1.0), kTol);
    ASSERT_NE(sph, nullptr);
    EXPECT_EQ(sph->kind(), SurfaceKind::Sphere);
    const auto tor = recognize(NurbsSurface::makeTorus({0, 0, 0}, {0, 0, 1}, 2.0, 0.5), kTol);
    ASSERT_NE(tor, nullptr);
    EXPECT_EQ(tor->kind(), SurfaceKind::Torus);
}

TEST(SurfaceRecognition, ABumpedControlPointIsNotRecognised) {
    const NurbsSurface base = NurbsSurface::makeSphere(kCenter, 2.0);
    auto pts = base.controlPoints();
    pts[3][2].x += 1e-4;
    const NurbsSurface bumped(pts, base.weights(), base.knotsU(), base.knotsV(), base.degreeU(),
                              base.degreeV());
    EXPECT_EQ(recognize(bumped, kTol), nullptr);

    const NurbsSurface cyl = NurbsSurface::makeCylinder(kCenter, kAxis, 1.0, 2.0);
    auto cpts = cyl.controlPoints();
    cpts[2][1] += math::Vec3{1e-4, 0, 0};
    const NurbsSurface bumpedCyl(cpts, cyl.weights(), cyl.knotsU(), cyl.knotsV(), cyl.degreeU(),
                                 cyl.degreeV());
    EXPECT_EQ(recognize(bumpedCyl, kTol), nullptr);

    const NurbsSurface tor = NurbsSurface::makeTorus(kCenter, kAxis, 3.0, 0.8);
    auto tpts = tor.controlPoints();
    tpts[4][3] += math::Vec3{0, 0, 1e-4};
    const NurbsSurface bumpedTor(tpts, tor.weights(), tor.knotsU(), tor.knotsV(), tor.degreeU(),
                                 tor.degreeV());
    EXPECT_EQ(recognize(bumpedTor, kTol), nullptr);
}

TEST(SurfaceRecognition, AFreeFormPatchIsNotRecognised) {
    std::vector<std::vector<math::Vec3>> pts(4, std::vector<math::Vec3>(4));
    std::vector<std::vector<double>> wts(4, std::vector<double>(4, 1.0));
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            pts[i][j] = {double(i), double(j), 0.3 * std::sin(1.3 * i) * std::cos(0.9 * j + i)};
        }
    }
    const std::vector<double> knots{0, 0, 0, 0, 1, 1, 1, 1};
    const NurbsSurface patch(pts, wts, knots, knots, 3, 3);
    EXPECT_EQ(recognize(patch, kTol), nullptr);
}

TEST(SurfaceRecognition, DegenerateInputsGiveNull) {
    // All control points equal: a point, not a surface.
    std::vector<std::vector<math::Vec3>> pts(2, std::vector<math::Vec3>(2, {1, 1, 1}));
    std::vector<std::vector<double>> wts(2, std::vector<double>(2, 1.0));
    const std::vector<double> knots{0, 0, 1, 1};
    const NurbsSurface point(pts, wts, knots, knots, 1, 1);
    EXPECT_EQ(recognize(point, kTol), nullptr);

    // A line: control points collinear.
    std::vector<std::vector<math::Vec3>> line = {{{0, 0, 0}, {0, 0, 0}}, {{1, 0, 0}, {1, 0, 0}}};
    const NurbsSurface sliver(line, wts, knots, knots, 1, 1);
    EXPECT_EQ(recognize(sliver, kTol), nullptr);
}

TEST(SurfaceRecognition, ALooserToleranceStillDistinguishesKinds) {
    const NurbsSurface n = NurbsSurface::makeCone({0, 0, 0}, {0, 0, 1}, 0.3, 2.0);
    const auto s = recognize(n, 1e-6);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->kind(), SurfaceKind::Cone);
}
