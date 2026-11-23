// A hole's callout text: its size and depth as a drawing states them, a
// counterbore's or countersink's on a line of its own, and the count.

#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

#include "horizon/modeling/HoleCallout.h"

using hz::model::HoleCallout;

namespace {

std::string mm(double value) {
    char text[32];
    std::snprintf(text, sizeof text, "%g", value);
    return text;
}

std::vector<std::string> lines(const HoleCallout& c) {
    return hz::model::holeCalloutLines(c, &mm);
}

}  // namespace

TEST(HoleCalloutTest, ASimpleHoleThroughOrToADepth) {
    HoleCallout c;
    c.diameter = 6.6;
    c.depth = HoleCallout::Depth::Through;
    EXPECT_EQ(lines(c), (std::vector<std::string>{"\xE2\x8C\x80"
                                                  "6.6 THRU"}));
    c.depth = HoleCallout::Depth::Blind;
    c.holeDepth = 10;
    EXPECT_EQ(lines(c), (std::vector<std::string>{"\xE2\x8C\x80"
                                                  "6.6 \xC3\x97 10 DEEP"}));
    c.depth = HoleCallout::Depth::UpToFace;
    EXPECT_EQ(lines(c), (std::vector<std::string>{"\xE2\x8C\x80"
                                                  "6.6"}));
}

TEST(HoleCalloutTest, ACountCounterboreAndCountersink) {
    HoleCallout c;
    c.count = 4;
    c.diameter = 6.6;
    c.depth = HoleCallout::Depth::Through;
    c.kind = HoleCallout::Kind::Counterbore;
    c.boreDiameter = 11;
    c.boreDepth = 6.4;
    EXPECT_EQ(lines(c), (std::vector<std::string>{"4\xC3\x97 \xE2\x8C\x80"
                                                  "6.6 THRU",
                                                  "CBORE \xE2\x8C\x80"
                                                  "11 \xC3\x97 6.4 DEEP"}));
    c.kind = HoleCallout::Kind::Countersink;
    c.sinkDiameter = 12;
    c.sinkAngleDegrees = 90;
    EXPECT_EQ(lines(c)[1],
              "CSK \xE2\x8C\x80"
              "12 \xC3\x97 90\xC2\xB0");
}

TEST(HoleCalloutTest, AnEdgeIsNamedWithoutItsPiece) {
    EXPECT_EQ(hz::model::withoutPiece("edge:hole_1/bore|primitive_1/top/piece:1"),
              "edge:hole_1/bore|primitive_1/top");
    EXPECT_EQ(hz::model::withoutPiece("edge:hole_1/bore|primitive_1/top"),
              "edge:hole_1/bore|primitive_1/top");
    EXPECT_EQ(hz::model::withoutPiece("edge:a|b/piece:x"), "edge:a|b/piece:x");
    EXPECT_EQ(hz::model::withoutPiece("edge:a|b/piece:"), "edge:a|b/piece:");
}
