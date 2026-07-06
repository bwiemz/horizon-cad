#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "horizon/cam/GcodeWriter.h"
#include "horizon/cam/Toolpath.h"

using hz::cam::CamGenerator;
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
    const std::string g = GcodeWriter::toGcode(p);
    EXPECT_NE(g.find("G21\n"), std::string::npos);
    EXPECT_NE(g.find("M2\n"), std::string::npos);
    EXPECT_EQ(countOccurrences(g, "G0 "), 2);        // rapid in + retract
    EXPECT_GT(countOccurrences(g, "G1 "), 2);        // several cutting passes
    EXPECT_EQ(countOccurrences(g, " F120.000"), 1);  // modal feed once
}

// G-code carries the metric/absolute preamble, G0/G1 words, a modal feed, and an
// end-of-program marker.
TEST(GcodeWriterTest, EmitsRs274) {
    const std::vector<Vec2> square = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Toolpath p = CamGenerator::contour(square, -2.0, 5.0, 100.0, true);
    const std::string g = GcodeWriter::toGcode(p);

    EXPECT_NE(g.find("G21\n"), std::string::npos);  // metric
    EXPECT_NE(g.find("G90\n"), std::string::npos);  // absolute
    EXPECT_NE(g.find("M2\n"), std::string::npos);   // end
    EXPECT_EQ(countOccurrences(g, "G0 "), 2);       // two rapids
    EXPECT_EQ(countOccurrences(g, "G1 "), 5);       // five feeds
    // The feed word appears once (modal F, emitted on first Feed only).
    EXPECT_EQ(countOccurrences(g, " F100.000"), 1);
    EXPECT_NE(g.find("X10.000 Y0.000 Z-2.000"), std::string::npos);
}
