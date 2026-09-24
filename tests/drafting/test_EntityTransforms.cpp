// Entities moved by the edit tools come out where the geometry says.

#include <gtest/gtest.h>

#include <cmath>

#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/math/Constants.h"

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

// A negative scale is a half turn about the centre: an arc's ends and a text's
// reading direction go round with it. They kept their angles, so an arc in a
// block scaled by -1 was drawn on the wrong side of its moved centre.
TEST(EntityTransformsTest, ANegativeScaleTurnsArcsAndTextHalfRound) {
    hz::draft::DraftArc arc(Vec2(2, 0), 1.0, 0.0, kHalfPi);
    arc.scale(Vec2(0, 0), -2.0);
    EXPECT_TRUE(near(arc.center(), Vec2(-4, 0)));
    EXPECT_NEAR(arc.radius(), 2.0, 1e-12);
    EXPECT_TRUE(near(arc.startPoint(), Vec2(-6, 0))) << "(3, 0) doubled through the origin";
    EXPECT_TRUE(near(arc.endPoint(), Vec2(-4, -2))) << "(2, 1) doubled through the origin";

    hz::draft::DraftText text(Vec2(1, 1), "T", 2.0);
    text.scale(Vec2(0, 0), -1.0);
    EXPECT_TRUE(near(text.position(), Vec2(-1, -1)));
    EXPECT_NEAR(text.rotation(), hz::math::kPi, 1e-12) << "upside down, as the half turn puts it";
}

namespace {

/// A block with a circle at (1, 2), off its base: not symmetric about it.
std::shared_ptr<hz::draft::BlockDefinition> flag() {
    auto def = std::make_shared<hz::draft::BlockDefinition>();
    def->name = "Flag";
    def->entities.push_back(std::make_shared<hz::draft::DraftCircle>(Vec2(1, 2), 0.5));
    return def;
}

/// @p p mirrored in the line through @p a and @p b.
Vec2 reflect(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 d = (b - a).normalized();
    const Vec2 v = p - a;
    return a + d * (2.0 * v.dot(d)) - v;
}

}  // namespace

// Mirroring a block reference mirrors what it places. It negated the scale,
// which turns the content half round instead: mirrored in a vertical axis,
// a block placed at the origin came out unchanged.
TEST(EntityTransformsTest, AMirroredBlockReferenceIsMirrored) {
    const auto def = flag();
    const std::vector<Vec2> probes = {Vec2(0, 0), Vec2(4, 0), Vec2(1, 2), Vec2(-3, 5)};
    struct Axis {
        Vec2 a, b;
    };
    for (const Axis& axis : {Axis{Vec2(0, 0), Vec2(0, 1)}, Axis{Vec2(0, 0), Vec2(1, 0)},
                             Axis{Vec2(5, 0), Vec2(6, 1)}, Axis{Vec2(-2, 3), Vec2(1, -1)}}) {
        hz::draft::DraftBlockRef ref(def, Vec2(3, 1), 0.4, 1.5);
        const hz::draft::DraftBlockRef before = ref;
        ref.mirror(axis.a, axis.b);
        EXPECT_TRUE(ref.mirrored());
        for (const Vec2& p : probes) {
            EXPECT_TRUE(
                near(ref.transformPoint(p), reflect(before.transformPoint(p), axis.a, axis.b)))
                << "axis (" << axis.a.x << "," << axis.a.y << ")-(" << axis.b.x << "," << axis.b.y
                << ")";
            EXPECT_TRUE(near(ref.inverseTransformPoint(ref.transformPoint(p)), p));
        }
        ref.mirror(axis.a, axis.b);
        EXPECT_FALSE(ref.mirrored()) << "mirrored twice is as it was";
        for (const Vec2& p : probes) {
            EXPECT_TRUE(near(ref.transformPoint(p), before.transformPoint(p)));
        }
    }

    // It is picked where it is drawn.
    hz::draft::DraftBlockRef ref(def, Vec2(0, 0));
    ref.mirror(Vec2(0, 0), Vec2(0, 1));
    EXPECT_TRUE(ref.hitTest(Vec2(-1.5, 2), 0.01)) << "the circle, now left of the axis";
    EXPECT_FALSE(ref.hitTest(Vec2(1.5, 2), 0.01));
    EXPECT_TRUE(ref.clone() != nullptr &&
                dynamic_cast<hz::draft::DraftBlockRef&>(*ref.clone()).mirrored());
}

// A mirrored text covers its mirror image's place, readable: mirrored in an
// upright axis it still reads left to right, ending where it began; in a
// level one it hangs below its baseline. It came out upside down, on the far
// side of its point.
TEST(EntityTransformsTest, AMirroredTextCoversItsMirrorImage) {
    hz::draft::DraftText upright(Vec2(2, 1), "ABC", 1.0);
    upright.setAlignment(hz::draft::TextAlignment::Left);
    upright.mirror(Vec2(0, 0), Vec2(0, 1));
    EXPECT_TRUE(near(upright.position(), Vec2(-2, 1)));
    EXPECT_NEAR(std::cos(upright.rotation()), 1.0, 1e-12) << "reads left to right";
    EXPECT_EQ(upright.alignment(), hz::draft::TextAlignment::Right);
    const auto box = upright.boundingBox();
    EXPECT_LE(box.max().x, -2.0 + 1e-9) << "all of it left of its point, as the image is";

    hz::draft::DraftText level(Vec2(2, 1), "ABC", 1.0);
    level.setAlignment(hz::draft::TextAlignment::Left);
    level.mirror(Vec2(0, 0), Vec2(1, 0));
    EXPECT_TRUE(near(level.position(), Vec2(2, -1)));
    const auto below = level.boundingBox();
    EXPECT_LE(below.max().y, -1.0 + 0.25 + 1e-9) << "below its baseline, as the image is";
    EXPECT_GE(below.min().x, 2.0 - 1e-9) << "right of its point, as the image is";
}
