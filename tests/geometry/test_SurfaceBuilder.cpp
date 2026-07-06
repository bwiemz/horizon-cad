#include <gtest/gtest.h>

#include "horizon/geometry/surfaces/SurfaceBuilder.h"
#include "horizon/math/Vec3.h"

using hz::geo::NurbsCurve;
using hz::geo::SurfaceBuilder;
using hz::math::Vec3;

namespace {

NurbsCurve line(const Vec3& a, const Vec3& b) {
    return NurbsCurve({a, b}, {1.0, 1.0}, {0.0, 0.0, 1.0, 1.0}, 1);
}

/// A degree-2 curve arching between two points via an apex offset.
NurbsCurve arch(const Vec3& a, const Vec3& b, const Vec3& apexOffset) {
    const Vec3 mid = (a + b) * 0.5 + apexOffset;
    return NurbsCurve({a, mid, b}, {1.0, 1.0, 1.0}, {0, 0, 0, 1, 1, 1}, 2);
}

}  // namespace

TEST(SurfaceBuilderTest, FourLinesMakeExactBilinearPlane) {
    const auto patch =
        SurfaceBuilder::coonsPatch(line({0, 0, 0}, {10, 0, 0}), line({0, 0, 0}, {0, 6, 0}),
                                   line({0, 6, 0}, {10, 6, 0}), line({10, 0, 0}, {10, 6, 0}));
    ASSERT_TRUE(patch.has_value());

    // The patch is the exact plane: interior evaluation matches bilinear.
    const Vec3 p = patch->evaluate(0.5, 0.5);
    EXPECT_NEAR(p.distanceTo(Vec3(5.0, 3.0, 0.0)), 0.0, 1e-9);
    const Vec3 q = patch->evaluate(0.25, 0.75);
    EXPECT_NEAR(q.distanceTo(Vec3(2.5, 4.5, 0.0)), 0.0, 1e-9);
}

TEST(SurfaceBuilderTest, BoundaryCurvesAreReproducedExactly) {
    const NurbsCurve bottom = arch({0, 0, 0}, {10, 0, 0}, {0, 0, 3});
    const NurbsCurve top = arch({0, 6, 1}, {10, 6, 1}, {0, 0, 3});
    const NurbsCurve leftC = arch({0, 0, 0}, {0, 6, 1}, {0, 0, 1});
    const NurbsCurve rightC = arch({10, 0, 0}, {10, 6, 1}, {0, 0, 1});

    const auto patch = SurfaceBuilder::coonsPatch(bottom, leftC, top, rightC);
    ASSERT_TRUE(patch.has_value());

    // Corners exactly.
    EXPECT_NEAR(patch->evaluate(patch->uMin(), patch->vMin()).distanceTo({0, 0, 0}), 0.0, 1e-9);
    EXPECT_NEAR(patch->evaluate(patch->uMax(), patch->vMax()).distanceTo({10, 6, 1}), 0.0, 1e-9);

    // Boundary iso-curves reproduce the inputs across the parameter range.
    for (double t = 0.0; t <= 1.0001; t += 0.1) {
        const double u = patch->uMin() + (patch->uMax() - patch->uMin()) * t;
        const Vec3 onPatch = patch->evaluate(u, patch->vMin());
        const double ct = bottom.tMin() + (bottom.tMax() - bottom.tMin()) * t;
        EXPECT_NEAR(onPatch.distanceTo(bottom.evaluate(ct)), 0.0, 1e-6) << "t=" << t;

        const Vec3 onTop = patch->evaluate(u, patch->vMax());
        EXPECT_NEAR(onTop.distanceTo(top.evaluate(ct)), 0.0, 1e-6) << "t=" << t;
    }
}

TEST(SurfaceBuilderTest, MismatchedCornersAreRejected) {
    const auto patch = SurfaceBuilder::coonsPatch(
        line({0, 0, 0}, {10, 0, 0}), line({0, 0, 5}, {0, 6, 0}),  // left starts off-corner
        line({0, 6, 0}, {10, 6, 0}), line({10, 0, 0}, {10, 6, 0}));
    EXPECT_FALSE(patch.has_value());
}

TEST(SurfaceBuilderTest, MismatchedDegreesAreRejected) {
    const auto patch = SurfaceBuilder::coonsPatch(
        line({0, 0, 0}, {10, 0, 0}), line({0, 0, 0}, {0, 6, 0}),
        arch({0, 6, 0}, {10, 6, 0}, {0, 0, 2}),  // degree 2 vs bottom's degree 1
        line({10, 0, 0}, {10, 6, 0}));
    EXPECT_FALSE(patch.has_value());
}

TEST(SurfaceBuilderTest, RationalBoundariesAreRejectedForNow) {
    const NurbsCurve rational({{0, 0, 0}, {5, 5, 0}, {10, 0, 0}}, {1.0, 0.7071, 1.0},
                              {0, 0, 0, 1, 1, 1}, 2);
    const auto patch = SurfaceBuilder::coonsPatch(rational, line({0, 0, 0}, {0, 6, 0}),
                                                  arch({0, 6, 0}, {10, 6, 0}, {0, 0, 0}),
                                                  line({10, 0, 0}, {10, 6, 0}));
    EXPECT_FALSE(patch.has_value());
}
