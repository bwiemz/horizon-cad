// Entities moved by the edit tools come out where the geometry says.

#include <gtest/gtest.h>

#include <cmath>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"

using hz::math::Vec2;

namespace {

bool near(const Vec2& a, const Vec2& b) {
    return (a - b).length() <= 1e-9;
}

constexpr double kHalfPi = 1.5707963267948966;

}  // namespace

TEST(EntityTransformsTest, AnArcMirroredInAnAxisAwayFromItsCenterLandsWhereItShould) {
    // A quarter from (12, 0) to (10, 2) about (10, 0), mirrored in x = 0: the
    // quarter from (-10, 2) to (-12, 0) about (-10, 0). The ends used to be
    // taken from the moved center with the old angles.
    hz::draft::DraftArc arc(Vec2(10, 0), 2.0, 0.0, kHalfPi);
    arc.mirror(Vec2(0, 0), Vec2(0, 1));
    EXPECT_TRUE(near(arc.center(), Vec2(-10, 0)));
    EXPECT_TRUE(near(arc.startPoint(), Vec2(-10, 2)));
    EXPECT_TRUE(near(arc.endPoint(), Vec2(-12, 0)));
    EXPECT_NEAR(arc.sweepAngle(), kHalfPi, 1e-12) << "still a quarter, not the other three";

    // And in a slanted axis, y = x: (x, y) -> (y, x).
    hz::draft::DraftArc slanted(Vec2(5, 0), 1.0, 0.0, kHalfPi);
    slanted.mirror(Vec2(0, 0), Vec2(1, 1));
    EXPECT_TRUE(near(slanted.center(), Vec2(0, 5)));
    EXPECT_TRUE(near(slanted.startPoint(), Vec2(1, 5)));
    EXPECT_TRUE(near(slanted.endPoint(), Vec2(0, 6)));
}

TEST(EntityTransformsTest, CirclesAndEllipsesMirrorToo) {
    hz::draft::DraftCircle circle(Vec2(3, 4), 1.0);
    circle.mirror(Vec2(0, 0), Vec2(1, 0));
    EXPECT_TRUE(near(circle.center(), Vec2(3, -4)));

    hz::draft::DraftEllipse ellipse(Vec2(3, 0), 2.0, 1.0, 0.3);
    ellipse.mirror(Vec2(0, 0), Vec2(0, 1));
    EXPECT_TRUE(near(ellipse.center(), Vec2(-3, 0)));
    EXPECT_NEAR(std::cos(2 * ellipse.rotation()), std::cos(2 * (3.141592653589793 - 0.3)), 1e-12);
}
