#include <gtest/gtest.h>

#include "horizon/cam/FeedsAndSpeeds.h"

using hz::cam::feedRate;
using hz::cam::spindleRpm;
using hz::cam::Tool;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

// N = 1000 v / (pi D).
TEST(FeedsAndSpeedsTest, SpindleRpmFormula) {
    // 100 m/min surface speed, 10 mm cutter.
    EXPECT_NEAR(spindleRpm(100.0, 10.0), 1000.0 * 100.0 / (kPi * 10.0), 1e-9);
    // Halving the diameter doubles the RPM for the same surface speed.
    EXPECT_NEAR(spindleRpm(100.0, 5.0), 2.0 * spindleRpm(100.0, 10.0), 1e-9);
    // Guard: non-positive diameter -> 0.
    EXPECT_EQ(spindleRpm(100.0, 0.0), 0.0);
}

// f = N * flutes * chip load.
TEST(FeedsAndSpeedsTest, FeedRateFormula) {
    EXPECT_NEAR(feedRate(3000.0, 2, 0.05), 3000.0 * 2 * 0.05, 1e-9);  // 300 mm/min
    EXPECT_EQ(feedRate(0.0, 2, 0.05), 0.0);                           // no rpm
    EXPECT_EQ(feedRate(3000.0, 0, 0.05), 0.0);                        // no flutes
    EXPECT_EQ(feedRate(3000.0, 2, 0.0), 0.0);                         // no chip load
}

// A worked aluminium example: a 6 mm 2-flute cutter at the preset surface speed.
TEST(FeedsAndSpeedsTest, WorkedExample) {
    Tool tool;
    tool.diameter = 6.0;
    tool.flutes = 2;

    const double rpm = spindleRpm(hz::cam::surfaceSpeed::kAluminum, tool.diameter);
    const double feed = feedRate(rpm, tool.flutes, 0.04);

    // ~15915 RPM, feed ~1273 mm/min — sanity-bound rather than pin exact values.
    EXPECT_GT(rpm, 15000.0);
    EXPECT_LT(rpm, 16500.0);
    EXPECT_NEAR(feed, rpm * 2 * 0.04, 1e-9);
    // Aluminium cuts faster than mild steel.
    EXPECT_GT(spindleRpm(hz::cam::surfaceSpeed::kAluminum, 6.0),
              spindleRpm(hz::cam::surfaceSpeed::kMildSteel, 6.0));
}
