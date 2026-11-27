#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/modeling/SheetMetalSolid.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/ExactValidator.h"

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

using Profile = std::vector<std::shared_ptr<hz::draft::DraftEntity>>;

/// Each entity of @p profile that is a line runs along crossSection()'s edge from its point
/// edges[k] to the next, at @p perBend chords a bend; each arc (edges[k] < 0) is not one.
void expectLinesAlongTheCrossSection(const Profile& profile, const std::vector<int>& edges,
                                     const SheetMetalStrip& strip, const SheetMetalParams& p,
                                     int perBend) {
    const auto polygon = SheetMetalSolid::crossSection(strip, p, perBend);
    ASSERT_EQ(profile.size(), edges.size());
    for (std::size_t k = 0; k < profile.size(); ++k) {
        const auto* line = dynamic_cast<const hz::draft::DraftLine*>(profile[k].get());
        if (edges[k] < 0) {
            EXPECT_EQ(line, nullptr) << k;
            continue;
        }
        ASSERT_NE(line, nullptr) << k;
        const auto e = static_cast<std::size_t>(edges[k]);
        EXPECT_LT((line->start() - polygon[e]).length(), 1e-12) << k;
        EXPECT_LT((line->end() - polygon[(e + 1) % polygon.size()]).length(), 1e-12) << k;
    }
}

/// @p profile is one closed loop (ProfileValidator), and extruded exactly by @p width it is a
/// valid exact solid of @p area times @p width, its bends' faces cylinders of @p radii.
void expectExtrudesExactly(const Profile& profile, double area, double width,
                           std::vector<double> radii) {
    const auto loop = hz::model::ProfileValidator::validate(profile);
    EXPECT_TRUE(loop.isClosed) << loop.errorMessage;
    EXPECT_EQ(loop.orderedEdges.size(), profile.size());
    const hz::draft::SketchPlane plane;
    std::string why;
    const auto solid =
        hz::model::Extrude::exact(profile, plane, plane.normal(), width, "flange", &why);
    ASSERT_NE(solid, nullptr) << why;
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    const auto issues = hz::topo::validateExact(*solid, hz::topo::exactTolerance(*solid));
    EXPECT_TRUE(issues.empty()) << issues.size() << " issues, the first: "
                                << (issues.empty() ? "" : issues.front().detail);
    std::vector<double> found;
    for (const auto& face : solid->faces()) {
        if (const auto* c = dynamic_cast<const hz::geo::Cylinder*>(face.exactSurface.get())) {
            found.push_back(c->radius());
        }
    }
    std::sort(found.begin(), found.end());
    std::sort(radii.begin(), radii.end());
    ASSERT_EQ(found.size(), radii.size());
    for (std::size_t k = 0; k < radii.size(); ++k) EXPECT_NEAR(found[k], radii[k], 1e-12);
    const auto props = MassPropertiesCalculator::compute(*solid);
    ASSERT_TRUE(props.valid) << props.problem;
    EXPECT_NEAR(props.volume, area * width, 1e-9 * area * width);
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

    // A bend with zero inside radius would collapse the arc to coincident
    // points (zero-length edges); refuse it for the 3D fold.
    SheetMetalParams sharp = good;
    sharp.bendRadius = 0.0;
    EXPECT_TRUE(SheetMetalSolid::crossSection(strip, sharp).empty());
    EXPECT_EQ(SheetMetalSolid::fold(strip, sharp, 5.0, "sharp"), nullptr);
    // But a zero-radius *flat* strip (no bends) is still fine.
    SheetMetalStrip flat;
    flat.segments = {20.0};
    EXPECT_EQ(SheetMetalSolid::crossSection(flat, sharp).size(), 4u);

    SheetMetalStrip zeroSegment = strip;
    zeroSegment.segments = {30.0, 0.0};
    EXPECT_TRUE(SheetMetalSolid::crossSection(zeroSegment, good).empty());

    EXPECT_TRUE(SheetMetalSolid::crossSection(strip, good, 0).empty());
    EXPECT_EQ(SheetMetalSolid::fold(strip, good, 0.0, "w0"), nullptr);
    EXPECT_TRUE(SheetMetalSolid::flatPattern(strip, good, -1.0).empty());
}

TEST(SheetMetalSolidTest, FlatPatternCountsNegativeBendAllowance) {
    // A downward bend develops the same flat length as an upward one of equal
    // magnitude; the blank must not come out short by dropping its allowance.
    SheetMetalStrip up;
    up.segments = {30.0, 20.0};
    up.bendAngles = {kPi / 2.0};
    SheetMetalStrip down;
    down.segments = {30.0, 20.0};
    down.bendAngles = {-kPi / 2.0};
    const SheetMetalParams p = params();

    const auto upFlat = SheetMetalSolid::flatPattern(up, p, 10.0);
    const auto downFlat = SheetMetalSolid::flatPattern(down, p, 10.0);
    ASSERT_EQ(upFlat.size(), 4u);
    ASSERT_EQ(downFlat.size(), 4u);
    EXPECT_NEAR(downFlat[1].x, upFlat[1].x, 1e-12);
    // And strictly longer than the bare flats (allowance actually included).
    EXPECT_GT(downFlat[1].x, 50.0);
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

// The fold exactly (stage 3): crossSection()'s outline with each bend one arc on each boundary in
// place of its chords, the inside one of the bend radius, the outside one of the radius and the
// thickness, about one centre. In crossSection()'s order, the bottom walked forward, the far
// end, the top walked back and the near end; each flat a line along crossSection()'s edge.
TEST(SheetMetalSolidTest, ProfileIsTheFlatsAndEachBendsTwoArcs) {
    SheetMetalStrip strip;
    strip.segments = {30.0, 20.0};
    strip.bendAngles = {kPi / 2.0};
    const SheetMetalParams p = params();  // r = 2, t = 1

    const auto profile = SheetMetalSolid::profile(strip, p);
    // At 8 chords a bend crossSection's edges are: the bottom's flat 0, its bend's chords 1 to 8,
    // its flat 9, the far end 10, the top's flat 11, its bend's chords 12 to 19, its flat 20 and
    // the near end 21.
    expectLinesAlongTheCrossSection(profile, {0, -1, 9, 10, 11, -1, 20, 21}, strip, p, 8);
    ASSERT_EQ(profile.size(), 8u);
    const auto* outside = dynamic_cast<const hz::draft::DraftArc*>(profile[1].get());
    const auto* inside = dynamic_cast<const hz::draft::DraftArc*>(profile[5].get());
    ASSERT_NE(outside, nullptr);
    ASSERT_NE(inside, nullptr);
    // Bent up, toward the top: the bottom is its outside.
    EXPECT_NEAR(outside->radius(), 3.0, 1e-12);
    EXPECT_NEAR(inside->radius(), 2.0, 1e-12);
    EXPECT_LT((outside->center() - Vec2(30.0, 3.0)).length(), 1e-12);
    EXPECT_LT((inside->center() - Vec2(30.0, 3.0)).length(), 1e-12);
    EXPECT_NEAR(outside->sweepAngle(), kPi / 2.0, 1e-12);
    EXPECT_NEAR(inside->sweepAngle(), kPi / 2.0, 1e-12);
    // The flats and the ring between the arcs, exactly.
    expectExtrudesExactly(profile, 50.0 + (kPi / 4.0) * (9.0 - 4.0), 10.0, {2.0, 3.0});
}

// Bent down, the bottom is the bend's inside; a joint of no angle is no bend, its two flats two
// lines in line; and bends both ways in one strip, a Z.
TEST(SheetMetalSolidTest, ProfileBentDownAndNotBent) {
    const SheetMetalParams p = params();  // r = 2, t = 1
    {
        SCOPED_TRACE("bent down");
        SheetMetalStrip strip;
        strip.segments = {30.0, 20.0};
        strip.bendAngles = {-kPi / 2.0};
        const auto profile = SheetMetalSolid::profile(strip, p);
        expectLinesAlongTheCrossSection(profile, {0, -1, 9, 10, 11, -1, 20, 21}, strip, p, 8);
        ASSERT_EQ(profile.size(), 8u);
        const auto* bottom = dynamic_cast<const hz::draft::DraftArc*>(profile[1].get());
        const auto* top = dynamic_cast<const hz::draft::DraftArc*>(profile[5].get());
        ASSERT_NE(bottom, nullptr);
        ASSERT_NE(top, nullptr);
        EXPECT_NEAR(bottom->radius(), 2.0, 1e-12);
        EXPECT_NEAR(top->radius(), 3.0, 1e-12);
        EXPECT_LT((bottom->center() - Vec2(30.0, -2.0)).length(), 1e-12);
        expectExtrudesExactly(profile, 50.0 + (kPi / 4.0) * (9.0 - 4.0), 10.0, {2.0, 3.0});
    }
    {
        SCOPED_TRACE("not bent");
        SheetMetalStrip strip;
        strip.segments = {10.0, 15.0};
        strip.bendAngles = {0.0};
        const auto profile = SheetMetalSolid::profile(strip, p);
        expectLinesAlongTheCrossSection(profile, {0, 1, 2, 3, 4, 5}, strip, p, 8);
        expectExtrudesExactly(profile, 25.0, 5.0, {});
    }
    {
        SCOPED_TRACE("a Z, at 60 degrees each way");
        SheetMetalStrip strip;
        strip.segments = {20.0, 15.0, 25.0};
        strip.bendAngles = {kPi / 3.0, -kPi / 3.0};
        const auto profile = SheetMetalSolid::profile(strip, p);
        // At 2 chords a bend: flat 0, chords 1-2, flat 3, chords 4-5, flat 6, the far end 7, and
        // back along the top: flat 8, chords 9-10, flat 11, chords 12-13, flat 14, the near end 15.
        expectLinesAlongTheCrossSection(profile, {0, -1, 3, -1, 6, 7, 8, -1, 11, -1, 14, 15}, strip,
                                        p, 2);
        expectExtrudesExactly(profile, 60.0 + 2.0 * (kPi / 6.0) * (9.0 - 4.0), 10.0,
                              {2.0, 2.0, 3.0, 3.0});
    }
}

// What crossSection() refuses, profile() does: empty.
TEST(SheetMetalSolidTest, ProfileOfWhatCannotBeFoldedIsEmpty) {
    SheetMetalStrip strip;
    strip.segments = {30.0, 20.0};
    strip.bendAngles = {kPi / 2.0};
    SheetMetalParams sharp = params();
    sharp.bendRadius = 0.0;
    EXPECT_TRUE(SheetMetalSolid::profile(strip, sharp).empty());
    SheetMetalStrip overfold = strip;
    overfold.bendAngles = {kPi};
    EXPECT_TRUE(SheetMetalSolid::profile(overfold, params()).empty());
    SheetMetalStrip mismatched = strip;
    mismatched.bendAngles.push_back(0.3);
    EXPECT_TRUE(SheetMetalSolid::profile(mismatched, params()).empty());
}
