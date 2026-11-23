#include <gtest/gtest.h>

#include <cmath>
#include <string>

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

// --- A base flange from its sketch -------------------------------------------

// An L sketched as the outside of the sheet: material to the left of travel,
// so the bend turns toward it and the sketched surface is the bend's outside
// (radius r + t). The flat lengths are the lines less the setbacks, and the
// developed length is the classic outside-dimension one: the two legs less
// the bend deduction.
TEST(BaseFlangeStrip, AnLSketchedOnTheOutside) {
    const SheetMetalParams p{2.0, 1.0, 0.44};
    std::string why;
    const auto strip = hz::model::stripFromPolyline({{0, 0}, {50, 0}, {50, 30}}, p, false, &why);
    ASSERT_TRUE(strip.has_value()) << why;
    ASSERT_EQ(strip->segments.size(), 2u);
    ASSERT_EQ(strip->bendAngles.size(), 1u);
    EXPECT_NEAR(strip->bendAngles[0], kPi / 2, 1e-12);
    EXPECT_NEAR(strip->segments[0], 50.0 - 3.0, 1e-9);  // setback (r + t) tan 45° = 3
    EXPECT_NEAR(strip->segments[1], 30.0 - 3.0, 1e-9);
    EXPECT_NEAR(developedLength(*strip, p), 50.0 + 30.0 - bendDeduction(kPi / 2, p), 1e-9);
}

// The same lines with the material on the other side: the sketched surface
// is the bend's inside (radius r), and the bend turns the other way.
TEST(BaseFlangeStrip, FlippedTheSketchIsTheInside) {
    const SheetMetalParams p{2.0, 1.0, 0.44};
    std::string why;
    const auto strip = hz::model::stripFromPolyline({{0, 0}, {50, 0}, {50, 30}}, p, true, &why);
    ASSERT_TRUE(strip.has_value()) << why;
    EXPECT_NEAR(strip->bendAngles[0], -kPi / 2, 1e-12);
    EXPECT_NEAR(strip->segments[0], 49.0, 1e-9);  // setback r tan 45° = 1
    EXPECT_NEAR(strip->segments[1], 29.0, 1e-9);
}

// A U and a Z: each corner its own bend, a line between two bends losing a
// setback at each end, and a straight run with no bend at all.
TEST(BaseFlangeStrip, AUAZAndAStraightRun) {
    const SheetMetalParams p{1.0, 2.0, 0.5};
    std::string why;
    const auto u =
        hz::model::stripFromPolyline({{0, 30}, {0, 0}, {40, 0}, {40, 30}}, p, true, &why);
    ASSERT_TRUE(u.has_value()) << why;
    ASSERT_EQ(u->bendAngles.size(), 2u);
    EXPECT_NEAR(u->bendAngles[0], u->bendAngles[1], 1e-12);  // both the same way
    EXPECT_NEAR(u->segments[1], 40.0 - 2.0 * 2.0, 1e-9)
        << "the base loses a setback of r tan 45° = 2 at each end";
    EXPECT_NEAR(u->segments[0], 30.0 - 2.0, 1e-9);

    const auto z =
        hz::model::stripFromPolyline({{0, 0}, {20, 0}, {20, 20}, {40, 20}}, p, false, &why);
    ASSERT_TRUE(z.has_value()) << why;
    EXPECT_NEAR(z->bendAngles[0], -z->bendAngles[1], 1e-12) << "a Z bends one way then back";

    const auto straight = hz::model::stripFromPolyline({{0, 0}, {10, 0}, {25, 0}}, p, false, &why);
    ASSERT_TRUE(straight.has_value()) << why;
    EXPECT_NEAR(straight->bendAngles[0], 0.0, 1e-12);
    EXPECT_NEAR(developedLength(*straight, p), 25.0, 1e-9);
}

// What a sketch cannot be folded from, said in the user's terms.
TEST(BaseFlangeStrip, WhatCannotBeFolded) {
    const SheetMetalParams p{2.0, 1.0, 0.44};
    std::string why;
    EXPECT_FALSE(hz::model::stripFromPolyline({{0, 0}}, p, false, &why).has_value());
    EXPECT_NE(why.find("at least one line"), std::string::npos) << why;
    EXPECT_FALSE(
        hz::model::stripFromPolyline({{0, 0}, {0, 0}, {5, 0}}, p, false, &why).has_value());
    EXPECT_NE(why.find("no length"), std::string::npos) << why;
    // A line shorter than the bends at its ends: 4 mm between two bends that
    // each take 3 mm of it.
    EXPECT_FALSE(hz::model::stripFromPolyline({{0, 0}, {20, 0}, {20, 4}, {0, 4}}, p, false, &why)
                     .has_value());
    EXPECT_NE(why.find("line 2"), std::string::npos) << why;
    EXPECT_NE(why.find("too short"), std::string::npos) << why;
    // A corner that turns back on itself.
    EXPECT_FALSE(
        hz::model::stripFromPolyline({{0, 0}, {20, 0}, {0, 0.0}}, p, false, &why).has_value());
    EXPECT_FALSE(why.empty());
    EXPECT_FALSE(
        hz::model::stripFromPolyline({{0, 0}, {20, 0}}, {0.0, 1.0, 0.44}, false, &why).has_value());
    EXPECT_NE(why.find("thickness"), std::string::npos) << why;
}

// Bend lines run across the flat pattern at the middle of each bend's
// allowance, whichever way the bend turns.
TEST(BaseFlangeStrip, BendLinesSitAtTheMiddleOfEachAllowance) {
    const SheetMetalParams p{2.0, 1.0, 0.44};
    const SheetMetalStrip strip{{10.0, 20.0, 5.0}, {kPi / 2, -kPi / 2}};
    const double ba = bendAllowance(kPi / 2, p);
    const auto lines = hz::model::bendLinePositions(strip, p);
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_NEAR(lines[0], 10.0 + ba / 2, 1e-12);
    EXPECT_NEAR(lines[1], 10.0 + ba + 20.0 + ba / 2, 1e-12);
    EXPECT_TRUE(hz::model::bendLinePositions(SheetMetalStrip{{10.0}, {}}, p).empty());
}

// A bend turning the other way takes as much sheet: a flipped flange, the
// second bend of a Z.
TEST(BaseFlangeStrip, ABendEitherWayTakesItsAllowance) {
    const SheetMetalParams p{2.0, 1.0, 0.44};
    const SheetMetalStrip up{{10.0, 20.0}, {kPi / 2}};
    const SheetMetalStrip down{{10.0, 20.0}, {-kPi / 2}};
    EXPECT_NEAR(developedLength(down, p), developedLength(up, p), 1e-12);
    EXPECT_NEAR(developedLength(down, p), 30.0 + bendAllowance(kPi / 2, p), 1e-12);
}
