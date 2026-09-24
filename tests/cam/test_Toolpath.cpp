#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "horizon/cam/GcodeWriter.h"
#include "horizon/cam/Toolpath.h"

using hz::cam::CamGenerator;
using hz::cam::GcodeOptions;
using hz::cam::GcodeWriter;
using hz::cam::Move;
using hz::cam::MoveType;
using hz::cam::Toolpath;
using hz::math::Vec2;

namespace {
int countType(const Toolpath& p, MoveType t) {
    int n = 0;
    for (const Move& m : p.moves) {
        if (m.type == t) ++n;
    }
    return n;
}
int countOccurrences(const std::string& hay, const std::string& needle) {
    int n = 0;
    for (std::size_t pos = hay.find(needle); pos != std::string::npos;
         pos = hay.find(needle, pos + needle.size())) {
        ++n;
    }
    return n;
}
}  // namespace

// A closed square contour: rapid in, plunge, feed the 4 edges back to start,
// rapid out. Cutting length equals the plunge depth plus the perimeter.
TEST(ToolpathTest, ClosedContour) {
    const std::vector<Vec2> square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Toolpath p = CamGenerator::contour(square, /*cutDepth=*/-2.0, /*safeZ=*/5.0,
                                             /*feed=*/100.0, /*closed=*/true);

    // Rapid-in + plunge + 3 edge feeds + closing feed + rapid-out.
    EXPECT_EQ(countType(p, MoveType::Rapid), 2);
    EXPECT_EQ(countType(p, MoveType::Feed), 5);  // plunge + 4 sides (incl. closing)

    // Perimeter = 40; plunge from safeZ(5) to depth(-2) contributes 7.
    EXPECT_NEAR(p.cuttingLength(), 40.0 + 7.0, 1e-9);
    // Rapids: in (down to safeZ has no prior... first move has no length) + retract.
    EXPECT_NEAR(p.rapidLength(), 7.0, 1e-9);  // retract from depth -2 to safeZ 5
}

// An open contour does not add a closing feed back to the start.
TEST(ToolpathTest, OpenContour) {
    const std::vector<Vec2> line = {{0, 0}, {10, 0}, {10, 10}};
    const Toolpath p = CamGenerator::contour(line, -1.0, 2.0, 50.0, /*closed=*/false);
    // plunge + 2 segment feeds; no closing feed.
    EXPECT_EQ(countType(p, MoveType::Feed), 3);
    EXPECT_NEAR(p.cuttingLength(), 20.0 + 3.0, 1e-9);  // 10 + 10 + plunge(3)
}

// A drilling cycle emits three moves per hole (approach, plunge, retract).
TEST(ToolpathTest, DrillCycle) {
    const std::vector<Vec2> holes = {{1, 1}, {5, 5}, {9, 1}};
    const Toolpath p = CamGenerator::drill(holes, -3.0, 2.0, 60.0);
    EXPECT_EQ(p.moves.size(), 9u);               // 3 per hole
    EXPECT_EQ(countType(p, MoveType::Feed), 3);  // one plunge per hole
    // Each plunge is from safeZ(2) to depth(-3) = 5, three holes -> 15.
    EXPECT_NEAR(p.cuttingLength(), 15.0, 1e-9);
}

TEST(ToolpathTest, EmptyProfile) {
    EXPECT_TRUE(CamGenerator::contour({}, -1.0, 2.0, 100.0, true).moves.empty());
    EXPECT_TRUE(CamGenerator::drill({}, -1.0, 2.0, 100.0).moves.empty());
}

// A rectangular pocket cleared with a 5 mm-radius tool at 10 mm stepover: the
// raster stays a full radius inside the walls and the cutting length matches the
// analytical pass + step-over + plunge total.
TEST(ToolpathTest, PocketRectRaster) {
    const Toolpath p = CamGenerator::pocketRect(/*min=*/{0, 0}, /*max=*/{100, 100},
                                                /*toolRadius=*/5.0, /*stepover=*/10.0,
                                                /*cutDepth=*/-2.0, /*safeZ=*/5.0, /*feed=*/100.0);
    ASSERT_FALSE(p.moves.empty());

    // Inset rectangle is [5,95] x [5,95]; passes at y = 5,15,...,85,95 -> 10 lanes.
    // Feeds: plunge + lane0 + (9 lanes * (stepover + lane)) = 1 + 1 + 18 = 20.
    EXPECT_EQ(countType(p, MoveType::Rapid), 2);
    EXPECT_EQ(countType(p, MoveType::Feed), 20);

    // Every cutting move stays within the inset walls and at the cut depth.
    for (const Move& m : p.moves) {
        if (m.type != MoveType::Feed) continue;
        EXPECT_GE(m.target.x, 5.0 - 1e-9);
        EXPECT_LE(m.target.x, 95.0 + 1e-9);
        EXPECT_GE(m.target.y, 5.0 - 1e-9);
        EXPECT_LE(m.target.y, 95.0 + 1e-9);
        EXPECT_NEAR(m.target.z, -2.0, 1e-9);
    }

    // 10 passes * 90 (width) + 9 step-overs * 10 + plunge (safeZ 5 -> depth -2 = 7).
    EXPECT_NEAR(p.cuttingLength(), 10 * 90.0 + 9 * 10.0 + 7.0, 1e-9);
    EXPECT_NEAR(p.rapidLength(), 7.0, 1e-9);  // retract only

    // Boustrophedon: pass 0 runs left->right (ends at x=95), pass 1 right->left.
    EXPECT_NEAR(p.moves[2].target.x, 95.0, 1e-9);  // end of first pass
    EXPECT_NEAR(p.moves[4].target.x, 5.0, 1e-9);   // end of second pass
}

// When the stepover does not divide the inset height evenly, the final pass is
// snapped to the far wall so the floor is fully covered.
TEST(ToolpathTest, PocketRectSnapsLastPassToWall) {
    const Toolpath p =
        CamGenerator::pocketRect({0, 0}, {100, 100}, 5.0, /*stepover=*/30.0, -1.0, 2.0, 50.0);
    ASSERT_FALSE(p.moves.empty());

    // Inset y in [5,95]; passes at 5,35,65 then snapped 95 -> 4 lanes.
    double maxY = 0.0;
    for (const Move& m : p.moves) {
        if (m.type == MoveType::Feed) maxY = std::max(maxY, m.target.y);
    }
    EXPECT_NEAR(maxY, 95.0, 1e-9);  // reaches the top inset wall
    // 4 lanes -> plunge + lane0 + 3*(step + lane) = 8 feeds.
    EXPECT_EQ(countType(p, MoveType::Feed), 8);
}

// Guards: a tool too big for the pocket, or non-positive parameters, yield no path.
TEST(ToolpathTest, PocketRectRejectsBadInput) {
    EXPECT_TRUE(CamGenerator::pocketRect({0, 0}, {100, 100}, 60.0, 10.0, -1, 2, 50).moves.empty());
    EXPECT_TRUE(CamGenerator::pocketRect({0, 0}, {100, 100}, 5.0, 0.0, -1, 2, 50).moves.empty());
    EXPECT_TRUE(CamGenerator::pocketRect({0, 0}, {100, 100}, 5.0, 10.0, -1, 2, 0.0).moves.empty());
    EXPECT_TRUE(CamGenerator::pocketRect({0, 0}, {100, 100}, -5.0, 10.0, -1, 2, 50).moves.empty());
}

// A pocket path feeds through the G-code writer like any other toolpath.
TEST(GcodeWriterTest, EmitsPocket) {
    const Toolpath p = CamGenerator::pocketRect({0, 0}, {40, 40}, 5.0, 10.0, -2.0, 5.0, 120.0);
    GcodeOptions options;
    options.spindleRpm = 10000.0;
    std::string error;
    const auto g = GcodeWriter::toGcode(p, options, &error);
    ASSERT_TRUE(g) << error;
    EXPECT_EQ(countOccurrences(*g, "G0 "), 3);        // climb, cross, retract
    EXPECT_GT(countOccurrences(*g, "G1 "), 2);        // several cutting passes
    EXPECT_EQ(countOccurrences(*g, " F120.000"), 1);  // modal feed once
}

// A contour program, line by line: a known modal state, the tool and spindle
// before any motion, a first rapid that climbs before it crosses, and an end
// that stops the spindle.
TEST(GcodeWriterTest, WritesAProgramThatStartsSafely) {
    const std::vector<Vec2> square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Toolpath p = CamGenerator::contour(square, -2.0, 5.0, 100.0, true);
    GcodeOptions options;
    options.toolNumber = 3;
    options.spindleRpm = 12000.0;
    const auto g = GcodeWriter::toGcode(p, options);
    ASSERT_TRUE(g);
    EXPECT_EQ(*g,
              "G21 G90 G94 G17 G40 G49 G80\n"
              "T3 M6\n"
              "S12000 M3\n"
              "G0 G43 H3 Z5.000\n"
              "G0 X0.000 Y0.000\n"
              "G1 X0.000 Y0.000 Z-2.000 F100.000\n"
              "G1 X10.000 Y0.000 Z-2.000\n"
              "G1 X10.000 Y10.000 Z-2.000\n"
              "G1 X0.000 Y10.000 Z-2.000\n"
              "G1 X0.000 Y0.000 Z-2.000\n"
              "G0 Z5.000\n"
              "M5\n"
              "M30\n");
}

TEST(GcodeWriterTest, CoolantRunsWithTheSpindle) {
    const Toolpath p = CamGenerator::drill({{1, 1}}, -3.0, 2.0, 50.0);
    GcodeOptions options;
    options.spindleRpm = 3000.0;
    options.coolant = true;
    const auto g = GcodeWriter::toGcode(p, options);
    ASSERT_TRUE(g);
    EXPECT_NE(g->find("S3000 M3\nM8\n"), std::string::npos) << *g;
    EXPECT_NE(g->find("M9\nM5\nM30\n"), std::string::npos) << *g;
}

// A rapid that both climbs and crosses climbs first; one that descends and
// crosses crosses first. Neither drags the tool through the part.
TEST(GcodeWriterTest, RapidsClimbBeforeTheyCross) {
    Toolpath p;
    p.moves.push_back({MoveType::Rapid, {0, 0, 10}, 0.0});
    p.moves.push_back({MoveType::Rapid, {0, 0, 1}, 0.0});    // approach
    p.moves.push_back({MoveType::Feed, {0, 0, -1}, 60.0});   // plunge
    p.moves.push_back({MoveType::Rapid, {20, 0, 10}, 0.0});  // up and across
    p.moves.push_back({MoveType::Rapid, {30, 5, 2}, 0.0});   // across and down
    GcodeOptions options;
    options.spindleRpm = 1000.0;
    options.decimals = 0;
    const auto g = GcodeWriter::toGcode(p, options);
    ASSERT_TRUE(g);
    EXPECT_NE(g->find("G1 X0 Y0 Z-1 F60\nG0 Z10\nG0 X20 Y0\n"), std::string::npos) << *g;
    EXPECT_NE(g->find("G0 X30 Y5\nG0 Z2\n"), std::string::npos) << *g;
}

// Programs that could hurt a machine are refused, with the reason.
TEST(GcodeWriterTest, RefusesUnsafePrograms) {
    const std::vector<Vec2> square = {{0, 0}, {10, 0}, {10, 10}};
    const Toolpath good = CamGenerator::contour(square, -2.0, 5.0, 100.0, true);
    GcodeOptions options;
    options.spindleRpm = 12000.0;
    ASSERT_TRUE(GcodeWriter::validate(good, options).empty());

    const auto refused = [](const Toolpath& path, const GcodeOptions& o, const std::string& why) {
        std::string error;
        EXPECT_FALSE(GcodeWriter::toGcode(path, o, &error)) << why;
        EXPECT_NE(error.find(why), std::string::npos) << error;
    };

    GcodeOptions noSpindle = options;
    noSpindle.spindleRpm = 0.0;
    refused(good, noSpindle, "spindle");
    GcodeOptions badTool = options;
    badTool.toolNumber = 0;
    refused(good, badTool, "tool number");
    GcodeOptions badDecimals = options;
    badDecimals.decimals = 9;
    refused(good, badDecimals, "decimals");

    refused(Toolpath{}, options, "empty");

    Toolpath cutsFirst = good;
    cutsFirst.moves.erase(cutsFirst.moves.begin());
    refused(cutsFirst, options, "start with a rapid");

    Toolpath lowRapid = good;
    lowRapid.moves.insert(lowRapid.moves.begin() + 2, {MoveType::Rapid, {5, 5, -2.0}, 0.0});
    refused(lowRapid, options, "through material");

    Toolpath noFeed = good;
    noFeed.moves[1].feed = 0.0;
    refused(noFeed, options, "feed");

    Toolpath notANumber = good;
    notANumber.moves[2].target.x = std::nan("");
    refused(notANumber, options, "finite");
}

// The generators make no path from parameters that cannot make a safe one.
TEST(ToolpathTest, GeneratorsRefuseUnsafeParameters) {
    const std::vector<Vec2> square = {{0, 0}, {10, 0}, {10, 10}};
    const double inf = std::numeric_limits<double>::infinity();
    // The cut must be below the safe plane.
    EXPECT_TRUE(CamGenerator::contour(square, 5.0, 5.0, 100.0, true).moves.empty());
    EXPECT_TRUE(CamGenerator::contour(square, 6.0, 5.0, 100.0, true).moves.empty());
    EXPECT_TRUE(CamGenerator::drill({{1, 1}}, 3.0, 2.0, 50.0).moves.empty());
    EXPECT_TRUE(
        CamGenerator::pocketRect({0, 0}, {40, 40}, 5.0, 10.0, 5.0, 5.0, 120.0).moves.empty());
    // The feed must be positive and every number finite.
    EXPECT_TRUE(CamGenerator::contour(square, -2.0, 5.0, 0.0, true).moves.empty());
    EXPECT_TRUE(CamGenerator::drill({{1, 1}}, -3.0, 2.0, -1.0).moves.empty());
    EXPECT_TRUE(CamGenerator::contour({{0, 0}, {inf, 0}}, -2.0, 5.0, 100.0, true).moves.empty());
    EXPECT_TRUE(CamGenerator::drill({{1, 1}}, -3.0, inf, 50.0).moves.empty());
    EXPECT_TRUE(
        CamGenerator::pocketRect({0, 0}, {40, inf}, 5.0, 10.0, -2.0, 5.0, 120.0).moves.empty());
}
