#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "horizon/math/Constants.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/SheetMetalSolid.h"

using hz::math::Vec2;
using hz::model::MassPropertiesCalculator;
using hz::model::SheetMetalParams;
using hz::model::SheetMetalSolid;
using hz::model::SheetMetalStrip;

namespace {

constexpr double kPi = hz::math::kPi;

SheetMetalParams params() {
    SheetMetalParams p;
    p.thickness = 1.0;
    p.bendRadius = 2.0;
    p.kFactor = 0.44;
    return p;
}

double maxX(const std::vector<Vec2>& pts) {
    double v = -1e300;
    for (const Vec2& p : pts) v = std::max(v, p.x);
    return v;
}
double maxY(const std::vector<Vec2>& pts) {
    double v = -1e300;
    for (const Vec2& p : pts) v = std::max(v, p.y);
    return v;
}
double minY(const std::vector<Vec2>& pts) {
    double v = 1e300;
    for (const Vec2& p : pts) v = std::min(v, p.y);
    return v;
}

double pathLength(const std::vector<Vec2>& pts, size_t begin, size_t end) {
    double len = 0.0;
    for (size_t i = begin + 1; i < end; ++i) len += (pts[i] - pts[i - 1]).length();
    return len;
}

}  // namespace

TEST(SheetMetalSolidTest, FlatStripCrossSectionIsARectangle) {
    SheetMetalStrip strip;
    strip.segments = {25.0};

    const auto polygon = SheetMetalSolid::crossSection(strip, params());
    ASSERT_EQ(polygon.size(), 4u);
    // Counter-clockwise: bottom forward, top back.
    EXPECT_NEAR((polygon[0] - Vec2(0, 0)).length(), 0.0, 1e-12);
    EXPECT_NEAR((polygon[1] - Vec2(25, 0)).length(), 0.0, 1e-12);
    EXPECT_NEAR((polygon[2] - Vec2(25, 1)).length(), 0.0, 1e-12);
    EXPECT_NEAR((polygon[3] - Vec2(0, 1)).length(), 0.0, 1e-12);
}

TEST(SheetMetalSolidTest, LBracketBoundsMatchTheAnalyticFold) {
    SheetMetalStrip strip;
    strip.segments = {30.0, 20.0};
    strip.bendAngles = {kPi / 2.0};
    const SheetMetalParams p = params();  // r = 2, t = 1

    const auto polygon = SheetMetalSolid::crossSection(strip, p, 64);
    ASSERT_GE(polygon.size(), 6u);
    // 90-degree upward fold: x spans L1 + (r + t), y spans L2 + (r + t).
    EXPECT_NEAR(maxX(polygon), 30.0 + 3.0, 1e-9);
    EXPECT_NEAR(maxY(polygon), 20.0 + 3.0, 1e-9);
    EXPECT_NEAR(minY(polygon), 0.0, 1e-12);
}

TEST(SheetMetalSolidTest, LBracketVolumeMatchesAnalytic) {
    SheetMetalStrip strip;
    strip.segments = {30.0, 20.0};
    strip.bendAngles = {kPi / 2.0};
    const SheetMetalParams p = params();
    const double width = 10.0;

    const auto solid = SheetMetalSolid::fold(strip, p, width, "flange_test", 64);
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid());
    ASSERT_EQ(solid->shells().size(), 1u);

    // Cross-section area = flats + bend annulus sector:
    //   t*(L1+L2) + angle * t * (r + t/2)
    const double area = 1.0 * 50.0 + (kPi / 2.0) * 1.0 * (2.0 + 0.5);
    const double expected = area * width;
    const double volume = MassPropertiesCalculator::compute(*solid).volume;
    EXPECT_NEAR(volume, expected, expected * 0.005);  // 64 chords per bend
}

TEST(SheetMetalSolidTest, NegativeBendFoldsDownward) {
    SheetMetalStrip strip;
    strip.segments = {30.0, 20.0};
    strip.bendAngles = {-kPi / 2.0};
    const SheetMetalParams p = params();

    const auto polygon = SheetMetalSolid::crossSection(strip, p, 64);
    ASSERT_FALSE(polygon.empty());
    // Mirror of the upward fold about the mid-thickness plane y = t/2:
    // the upward fold spans [0, L2 + r + t], so the downward one spans
    // [t - (L2 + r + t), t].
    EXPECT_NEAR(minY(polygon), 1.0 - (20.0 + 3.0), 1e-9);
    EXPECT_NEAR(maxY(polygon), 1.0, 1e-12);
}

TEST(SheetMetalSolidTest, NeutralAxisLiesBetweenTheBoundaryLengths) {
    SheetMetalStrip strip;
    strip.segments = {30.0, 20.0};
    strip.bendAngles = {kPi / 2.0};
    const SheetMetalParams p = params();
    const int perBend = 64;

    const auto polygon = SheetMetalSolid::crossSection(strip, p, perBend);
    // Bottom boundary: segments+1 straight points plus perBend arc points.
    const size_t bottomCount = strip.segments.size() + 1 + perBend;
    ASSERT_EQ(polygon.size(), 2 * bottomCount);
    const double bottomLen = pathLength(polygon, 0, bottomCount);
    // Top boundary is stored reversed as the polygon's second half.
    const double topLen = pathLength(polygon, bottomCount, polygon.size());

    // 90-degree fold up: bottom is the convex (outside) surface.
    EXPECT_GT(bottomLen, topLen);
    const double developed = hz::model::developedLength(strip, p);
    EXPECT_GT(bottomLen + 1e-6, developed);
    EXPECT_LT(topLen - 1e-6, developed);

    // And the flat pattern is exactly developedLength x width.
    const auto flat = SheetMetalSolid::flatPattern(strip, p, 10.0);
    ASSERT_EQ(flat.size(), 4u);
    EXPECT_NEAR(flat[1].x, developed, 1e-12);
    EXPECT_NEAR(flat[2].y, 10.0, 1e-12);
}

TEST(SheetMetalSolidTest, UChannelIsAValidManifold) {
    SheetMetalStrip strip;
    strip.segments = {20.0, 30.0, 20.0};
    strip.bendAngles = {kPi / 2.0, kPi / 2.0};
    const SheetMetalParams p = params();
    const double width = 8.0;

    const auto solid = SheetMetalSolid::fold(strip, p, width, "channel", 32);
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid());

    const double area = 1.0 * 70.0 + 2.0 * (kPi / 2.0) * 1.0 * 2.5;
    const double volume = MassPropertiesCalculator::compute(*solid).volume;
    EXPECT_NEAR(volume, area * width, area * width * 0.005);
}

TEST(SheetMetalSolidTest, InvalidInputsAreRejected) {
    SheetMetalStrip strip;
    strip.segments = {30.0, 20.0};
    strip.bendAngles = {kPi / 2.0};
    const SheetMetalParams good = params();

    SheetMetalParams bad = good;
    bad.thickness = 0.0;
    EXPECT_TRUE(SheetMetalSolid::crossSection(strip, bad).empty());

    SheetMetalStrip mismatched = strip;
    mismatched.bendAngles.push_back(0.3);
    EXPECT_TRUE(SheetMetalSolid::crossSection(mismatched, good).empty());

    SheetMetalStrip overfold = strip;
    overfold.bendAngles = {kPi};  // 180-degree hems need their own feature
    EXPECT_TRUE(SheetMetalSolid::crossSection(overfold, good).empty());

    SheetMetalStrip zeroSegment = strip;
    zeroSegment.segments = {30.0, 0.0};
    EXPECT_TRUE(SheetMetalSolid::crossSection(zeroSegment, good).empty());

    EXPECT_TRUE(SheetMetalSolid::crossSection(strip, good, 0).empty());
    EXPECT_EQ(SheetMetalSolid::fold(strip, good, 0.0, "w0"), nullptr);
    EXPECT_TRUE(SheetMetalSolid::flatPattern(strip, good, -1.0).empty());
}

TEST(SheetMetalSolidTest, ZeroAngleJointBehavesAsOneFlat) {
    SheetMetalStrip strip;
    strip.segments = {10.0, 15.0};
    strip.bendAngles = {0.0};
    const SheetMetalParams p = params();

    const auto solid = SheetMetalSolid::fold(strip, p, 5.0, "flat_joint");
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid());
    const double volume = MassPropertiesCalculator::compute(*solid).volume;
    EXPECT_NEAR(volume, 25.0 * 1.0 * 5.0, 1e-6);
}
