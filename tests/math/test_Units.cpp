// Phase 154: lengths shown in a unit, and typed with or without one.

#include <gtest/gtest.h>

#include <limits>
#include <optional>
#include <string>

#include "horizon/math/Constants.h"
#include "horizon/math/Units.h"

using hz::math::formatLength;
using hz::math::LengthUnit;
using hz::math::lengthUnitFrom;
using hz::math::parseAngle;
using hz::math::parseLength;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

void expectLength(const char* text, LengthUnit unit, double mm) {
    const std::optional<double> read = parseLength(text, unit);
    ASSERT_TRUE(read.has_value()) << text;
    EXPECT_NEAR(read.value_or(kNaN), mm, 1e-12) << text;
}

}  // namespace

TEST(UnitsTest, ALengthIsReadInTheUnitItIsTypedIn) {
    expectLength("25.4", LengthUnit::Millimetre, 25.4);
    expectLength("2", LengthUnit::Inch, 50.8);  // no unit: the document's
    expectLength("2 in", LengthUnit::Millimetre, 50.8);
    expectLength("2in", LengthUnit::Millimetre, 50.8);
    expectLength("2\"", LengthUnit::Millimetre, 50.8);
    expectLength("50.8 mm", LengthUnit::Inch, 50.8);
    expectLength("5.08 cm", LengthUnit::Inch, 50.8);
    expectLength("0.0508 m", LengthUnit::Inch, 50.8);
    expectLength("  -3 MM ", LengthUnit::Inch, -3.0);
    expectLength("+.5 inch", LengthUnit::Millimetre, 12.7);
    expectLength("1.5e3 mm", LengthUnit::Inch, 1500.0);
    expectLength("2 millimetres", LengthUnit::Inch, 2.0);
    expectLength("1 meter", LengthUnit::Inch, 1000.0);
}

TEST(UnitsTest, FeetAndInchesAndFractionsAreRead) {
    expectLength("1' 6\"", LengthUnit::Millimetre, 457.2);
    expectLength("1'6", LengthUnit::Millimetre, 457.2);  // after feet, inches
    expectLength("1 ft 6 in", LengthUnit::Millimetre, 457.2);
    expectLength("-1' 6\"", LengthUnit::Millimetre, -457.2);  // one sign for the whole
    expectLength("3/4 in", LengthUnit::Millimetre, 19.05);
    expectLength("3/4", LengthUnit::Inch, 19.05);
    expectLength("1 1/2 in", LengthUnit::Millimetre, 38.1);
    expectLength("2' 1 1/2\"", LengthUnit::Millimetre, 609.6 + 38.1);
}

TEST(UnitsTest, WhatIsNotALengthIsRefused) {
    for (const char* text : {"", "  ", "in", "2 furlongs", "2 in 3", "2 3", "1/0 in", "--2",
                             "2 - 3", "inf", "nan mm", "2,5", "1e999 mm", "2 in 3 cm x", "1/2/3"}) {
        EXPECT_FALSE(parseLength(text, LengthUnit::Millimetre).has_value()) << text;
    }
}

TEST(UnitsTest, AnAngleIsReadInDegreesUnlessItSaysRadians) {
    EXPECT_NEAR(parseAngle("30").value_or(kNaN), 30.0 * hz::math::kDegToRad, 1e-15);
    EXPECT_NEAR(parseAngle("30 deg").value_or(kNaN), 30.0 * hz::math::kDegToRad, 1e-15);
    EXPECT_NEAR(parseAngle("30\xC2\xB0").value_or(kNaN), 30.0 * hz::math::kDegToRad, 1e-15);
    EXPECT_NEAR(parseAngle("-45 Degrees").value_or(kNaN), -45.0 * hz::math::kDegToRad, 1e-15);
    EXPECT_NEAR(parseAngle("0.5 rad").value_or(kNaN), 0.5, 1e-15);
    for (const char* text : {"", "30 in", "30 deg 15", "rad", "1/0"}) {
        EXPECT_FALSE(parseAngle(text).has_value()) << text;
    }
}

TEST(UnitsTest, ALengthIsShownInItsUnitWithAPoint) {
    EXPECT_EQ(formatLength(25.4, LengthUnit::Inch, 3), "1.000 in");
    EXPECT_EQ(formatLength(25.4, LengthUnit::Millimetre, 2), "25.40 mm");
    EXPECT_EQ(formatLength(1500.0, LengthUnit::Metre, 1, false), "1.5");
    EXPECT_EQ(formatLength(-0.0001, LengthUnit::Millimetre, 3), "0.000 mm") << "no minus zero";
    EXPECT_EQ(formatLength(304.8, LengthUnit::Foot, 0), "1 ft");
}

TEST(UnitsTest, AUnitIsFoundBySymbolNameOrMark) {
    EXPECT_EQ(lengthUnitFrom("mm"), LengthUnit::Millimetre);
    EXPECT_EQ(lengthUnitFrom("IN"), LengthUnit::Inch);
    EXPECT_EQ(lengthUnitFrom("feet"), LengthUnit::Foot);
    EXPECT_EQ(lengthUnitFrom("'"), LengthUnit::Foot);
    EXPECT_EQ(lengthUnitFrom("\""), LengthUnit::Inch);
    EXPECT_FALSE(lengthUnitFrom("yd").has_value());
    for (const LengthUnit unit : hz::math::kLengthUnits) {
        EXPECT_EQ(lengthUnitFrom(hz::math::symbolOf(unit)), unit);
    }
}
