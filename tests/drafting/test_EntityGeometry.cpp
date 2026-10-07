// What entities draw, and where a click or a line finds them.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/Intersection.h"

using hz::math::Vec2;
using Segments = std::vector<std::pair<Vec2, Vec2>>;

namespace {

bool near(const Vec2& a, const Vec2& b, double tol = 1e-9) {
    return (a - b).length() <= tol;
}

/// @p lines, each with its ends in order of x, sorted by y then x.
Segments sorted(Segments lines) {
    for (auto& [a, b] : lines) {
        if (b.x < a.x) std::swap(a, b);
    }
    std::sort(lines.begin(), lines.end(), [](const auto& l, const auto& r) {
        if (std::abs(l.first.y - r.first.y) > 1e-9) return l.first.y < r.first.y;
        return l.first.x < r.first.x;
    });
    return lines;
}

}  // namespace

// -- Hatching ------------------------------------------------------------------

// A hatch line through a corner of the outline counted the corner once for
// each side meeting there, and paired the crossings wrongly after it: through
// the corner of this notch, a line ran across the empty notch.
TEST(HatchTest, ALineThroughACornerStaysInside) {
    const hz::draft::DraftHatch hatch(
        {{0, 0}, {10, 0}, {10, 6}, {7, 6}, {7, 2}, {3, 2}, {2.5, 3}, {3, 6}, {0, 6}},
        hz::draft::HatchPattern::Lines, 0.0, 2.0);
    const Segments lines = sorted(hatch.generateHatchLines());
    // Lines at y = 1, 3 and 5; the one at 3 through the corner (2.5, 3).
    const Segments expected = {{{0, 1}, {10, 1}},
                               {{0, 3}, {2.5, 3}},
                               {{7, 3}, {10, 3}},
                               {{0, 5}, {2.5 + 0.5 * 2.0 / 3.0, 5}},
                               {{7, 5}, {10, 5}}};
    ASSERT_EQ(lines.size(), expected.size());
    for (size_t i = 0; i < lines.size(); ++i) {
        EXPECT_TRUE(near(lines[i].first, expected[i].first) &&
                    near(lines[i].second, expected[i].second))
            << i << ": (" << lines[i].first.x << ", " << lines[i].first.y << ") - ("
            << lines[i].second.x << ", " << lines[i].second.y << ")";
    }
}

// Through two corners on either side of a diamond, the line across its middle
// was lost: each corner counted twice made two empty lines of it.
TEST(HatchTest, ALineThroughTwoCornersIsDrawn) {
    const hz::draft::DraftHatch diamond({{0, -1}, {1, 0}, {0, 1}, {-1, 0}},
                                        hz::draft::HatchPattern::Lines, 0.0, 2.0);
    const Segments lines = sorted(diamond.generateHatchLines());
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_TRUE(near(lines[0].first, Vec2(-1, 0)));
    EXPECT_TRUE(near(lines[0].second, Vec2(1, 0)));
}

// -- Picking an ellipse --------------------------------------------------------

// A click is on an ellipse when it is within the pick distance of the curve.
// The distance was taken along the line to the centre, which on a long thin
// ellipse is several times the true one: a click half a unit off was taken
// to be 2.4 off, and missed.
TEST(EllipseTest, AClickIsMeasuredToTheNearestPointOfTheCurve) {
    const hz::draft::DraftEllipse ellipse(Vec2(0, 0), 100.0, 10.0);
    // The true distances, 0.4956 and 2.9729: (95, 0) is nearest the curve
    // off the axis, not at its end 5 away.
    EXPECT_TRUE(ellipse.hitTest(Vec2(80, 6.5), 0.5));
    EXPECT_FALSE(ellipse.hitTest(Vec2(80, 6.5), 0.49));
    EXPECT_TRUE(ellipse.hitTest(Vec2(95, 0), 2.98));
    EXPECT_FALSE(ellipse.hitTest(Vec2(95, 0), 2.96));
    EXPECT_TRUE(ellipse.hitTest(Vec2(0, 30), 20.01));
    EXPECT_FALSE(ellipse.hitTest(Vec2(0, 30), 19.99));
    EXPECT_TRUE(ellipse.hitTest(Vec2(0, 0), 10.01)) << "the centre";
    EXPECT_FALSE(ellipse.hitTest(Vec2(0, 0), 9.99));

    // Turned and moved, the same.
    const double turn = 0.7;
    const hz::draft::DraftEllipse turned(Vec2(5, -3), 100.0, 10.0, turn);
    const Vec2 p(5 + 80 * std::cos(turn) - 6.5 * std::sin(turn),
                 -3 + 80 * std::sin(turn) + 6.5 * std::cos(turn));
    EXPECT_TRUE(turned.hitTest(p, 0.5));
    EXPECT_FALSE(turned.hitTest(p, 0.49));
}

// -- A line touching a circle --------------------------------------------------

// A line that touches a circle meets it at one point. The test for touching
// compared a number that grows with the fourth power of the drawing's size to
// a fixed tiny one, so lines that touched were found to miss, or to cross
// twice at nearly the same point, most of the time.
TEST(IntersectionTest, ALineTouchingACircleMeetsItOnce) {
    int once = 0;
    for (int k = 0; k < 200; ++k) {
        const double angle = 0.37 * k + 0.1;
        const Vec2 centre(123.4 + k, -56.7 + 2.0 * k);
        const double radius = 17.3 + 0.11 * k;
        const Vec2 touch = centre + Vec2(std::cos(angle), std::sin(angle)) * radius;
        const Vec2 along(-std::sin(angle), std::cos(angle));
        const Vec2 from = touch - along * 14.8;
        const Vec2 to = touch + along * 25.2;

        const auto onSegment = hz::draft::intersectLineCircle(from, to, centre, radius);
        const auto onRay = hz::draft::intersectRayCircle(from, along, centre, radius);
        if (onSegment.size() == 1 && near(onSegment[0], touch, 1e-6) && onRay.size() == 1 &&
            near(onRay[0], touch, 1e-6)) {
            ++once;
        }
    }
    EXPECT_EQ(once, 200);
}

// Lines that cross or miss a circle by a little are still told apart.
TEST(IntersectionTest, ALineJustCrossingOrMissingACircleIsToldApart) {
    const Vec2 centre(1000, 1000);
    const double radius = 50.0;
    for (const double gap : {1e-6, -1e-6}) {
        SCOPED_TRACE(gap);
        const Vec2 from(900, 1050 + gap);
        const Vec2 to(1100, 1050 + gap);
        const size_t expected = gap > 0 ? 0u : 2u;
        EXPECT_EQ(hz::draft::intersectLineCircle(from, to, centre, radius).size(), expected);
        EXPECT_EQ(hz::draft::intersectRayCircle(from, Vec2(1, 0), centre, radius).size(), expected);
    }
    const auto across =
        hz::draft::intersectLineCircle(Vec2(900, 1000), Vec2(1100, 1000), centre, radius);
    ASSERT_EQ(across.size(), 2u);
    EXPECT_TRUE(near(across[0], Vec2(950, 1000)));
    EXPECT_TRUE(near(across[1], Vec2(1050, 1000)));
    const auto ahead = hz::draft::intersectRayCircle(Vec2(1000, 1000), Vec2(0, 2), centre, radius);
    ASSERT_EQ(ahead.size(), 1u) << "from inside, only the way the ray runs";
    EXPECT_TRUE(near(ahead[0], Vec2(1000, 1050)));
}
