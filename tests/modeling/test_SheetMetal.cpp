#include <gtest/gtest.h>

#include <cmath>

#include "horizon/modeling/SheetMetal.h"

using hz::model::bendAllowance;
using hz::model::bendDeduction;
using hz::model::developedLength;
using hz::model::SheetMetalParams;
using hz::model::SheetMetalStrip;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

// Bend allowance is the neutral-axis arc length: BA = angle * (r + K*t).
TEST(SheetMetalTest, BendAllowanceFormula) {
    SheetMetalParams p;
    p.thickness = 2.0;
    p.bendRadius = 3.0;
    p.kFactor = 0.44;

    const double ba90 = bendAllowance(kPi / 2.0, p);
    EXPECT_NEAR(ba90, (kPi / 2.0) * (3.0 + 0.44 * 2.0), 1e-9);

    // A full-thickness axis (K=0) gives the inside-radius arc.
    p.kFactor = 0.0;
    EXPECT_NEAR(bendAllowance(kPi / 2.0, p), (kPi / 2.0) * 3.0, 1e-9);

    // Non-positive angle or invalid params yield zero.
    EXPECT_EQ(bendAllowance(0.0, p), 0.0);
    SheetMetalParams bad;
    bad.thickness = 0.0;
    EXPECT_EQ(bendAllowance(kPi / 2.0, bad), 0.0);
}

// Bend deduction relates to setback: BD = 2*(r+t)*tan(a/2) - BA.
TEST(SheetMetalTest, BendDeductionFormula) {
    SheetMetalParams p;
    p.thickness = 1.5;
    p.bendRadius = 2.0;
    p.kFactor = 0.4;

    const double a = kPi / 2.0;
    const double expected =
        2.0 * (p.bendRadius + p.thickness) * std::tan(a / 2.0) - bendAllowance(a, p);
    EXPECT_NEAR(bendDeduction(a, p), expected, 1e-9);
}

// A single flat strip with no bends develops to its own length.
TEST(SheetMetalTest, FlatStripIsUnchanged) {
    SheetMetalParams p;
    SheetMetalStrip strip;
    strip.segments = {50.0};  // one segment, no bends
    EXPECT_NEAR(developedLength(strip, p), 50.0, 1e-9);
}

// A single 90-degree bend: developed length = leg1 + leg2 + bend allowance.
TEST(SheetMetalTest, SingleBendDevelopedLength) {
    SheetMetalParams p;
    p.thickness = 2.0;
    p.bendRadius = 3.0;
    p.kFactor = 0.44;

    SheetMetalStrip strip;
    strip.segments = {20.0, 30.0};
    strip.bendAngles = {kPi / 2.0};

    const double expected = 20.0 + 30.0 + bendAllowance(kPi / 2.0, p);
    EXPECT_NEAR(developedLength(strip, p), expected, 1e-9);
}

// Several bends sum their allowances on top of the segment lengths.
TEST(SheetMetalTest, MultiBendDevelopedLength) {
    SheetMetalParams p;
    p.thickness = 1.0;
    p.bendRadius = 1.0;
    p.kFactor = 0.5;

    SheetMetalStrip strip;
    strip.segments = {10.0, 10.0, 10.0};
    strip.bendAngles = {kPi / 2.0, kPi / 2.0};

    const double expected = 30.0 + 2.0 * bendAllowance(kPi / 2.0, p);
    EXPECT_NEAR(developedLength(strip, p), expected, 1e-9);
}

// Inconsistent segment/bend counts (or invalid params) return zero.
TEST(SheetMetalTest, InconsistentStripReturnsZero) {
    SheetMetalParams p;
    SheetMetalStrip strip;
    strip.segments = {10.0, 10.0};
    strip.bendAngles = {kPi / 2.0, kPi / 2.0};  // too many bends for two segments
    EXPECT_EQ(developedLength(strip, p), 0.0);

    SheetMetalParams bad;
    bad.kFactor = 2.0;  // invalid
    SheetMetalStrip ok;
    ok.segments = {5.0};
    EXPECT_EQ(developedLength(ok, bad), 0.0);
}
