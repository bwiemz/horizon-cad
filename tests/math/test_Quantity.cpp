// Phase 155: expressions with units, evaluated with what they measure, and
// kept with their units written, whatever the document's unit.

#include <gtest/gtest.h>

#include <limits>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "horizon/math/Constants.h"
#include "horizon/math/Expression.h"
#include "horizon/math/Quantity.h"

using hz::math::evaluateQuantity;
using hz::math::Expression;
using hz::math::LengthUnit;
using hz::math::normalized;
using hz::math::Quantity;
using hz::math::QuantityKind;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double plain(const std::string& text) {
    const auto e = Expression::parse(text);
    EXPECT_NE(e, nullptr) << text;
    return e ? e->evaluate({}) : kNaN;
}

const std::map<std::string, Quantity> kVariables{
    {"wall", {10.0, 1, 0}},  // 10 mm
    {"count", {4.0, 0, 0}},  // a number
    {"tilt", {0.5, 0, 1}},   // 0.5 rad
};

std::optional<Quantity> measured(const std::string& text, std::string* why = nullptr) {
    const auto e = Expression::parse(text);
    EXPECT_NE(e, nullptr) << text;
    return e ? evaluateQuantity(*e, kVariables, why) : std::nullopt;
}

std::string kept(const std::string& text, QuantityKind kind, LengthUnit unit,
                 std::string* why = nullptr) {
    const auto e = Expression::parse(text);
    EXPECT_NE(e, nullptr) << text;
    if (!e) return {};
    const auto n = normalized(*e, kVariables, kind, unit, why);
    return n ? n->toString() : std::string();
}

}  // namespace

TEST(QuantityTest, AUnitAfterANumberOrABracketIsParsedAndKept) {
    EXPECT_DOUBLE_EQ(plain("2 in"), 50.8) << "in millimetres, the model's unit";
    EXPECT_DOUBLE_EQ(plain("2in"), 50.8);
    EXPECT_DOUBLE_EQ(plain("(1 + 1) cm"), 20.0);
    EXPECT_DOUBLE_EQ(plain("1.5 m"), 1500.0);
    EXPECT_DOUBLE_EQ(plain("1 ft + 6 in"), 457.2);
    EXPECT_NEAR(plain("30 deg"), hz::math::kPi / 6, 1e-15) << "in radians";
    EXPECT_DOUBLE_EQ(plain("0.5 rad"), 0.5);

    // Printed so as to parse back the same, and through JSON too.
    for (const char* text : {"2 in", "(wall + 1) mm", "-3 deg", "sqrt(2 in * 2 in)"}) {
        const auto e = Expression::parse(text);
        ASSERT_NE(e, nullptr) << text;
        const auto again = Expression::parse(e->toString());
        ASSERT_NE(again, nullptr) << e->toString();
        EXPECT_EQ(again->toString(), e->toString());
        const auto fromJson = Expression::fromJson(e->toJson());
        ASSERT_NE(fromJson, nullptr) << text;
        EXPECT_EQ(fromJson->toString(), e->toString());
    }
    EXPECT_EQ(Expression::fromJson(nlohmann::json::parse(
                  R"({"type":"unit","unit":"furlong","child":{"type":"literal","value":1}})")),
              nullptr)
        << "a unit that is not one";
    EXPECT_EQ(Expression::parse("2 in in"), nullptr);
}

TEST(QuantityTest, AnExpressionMeasuresWhatItsUnitsSay) {
    auto q = measured("2 in + 3 mm");
    ASSERT_TRUE(q);
    EXPECT_DOUBLE_EQ(q->value, 53.8);
    EXPECT_EQ(q->length, 1);
    q = measured("2 in * 3 in");
    ASSERT_TRUE(q);
    EXPECT_EQ(q->length, 2) << "an area";
    q = measured("sqrt(4 in * 1 in)");
    ASSERT_TRUE(q);
    EXPECT_DOUBLE_EQ(q->value, 50.8);
    EXPECT_EQ(q->length, 1);
    q = measured("wall * count / 2");
    ASSERT_TRUE(q);
    EXPECT_DOUBLE_EQ(q->value, 20.0);
    EXPECT_EQ(q->length, 1);
    q = measured("sin(30 deg)");
    ASSERT_TRUE(q);
    EXPECT_NEAR(q->value, 0.5, 1e-15);
    EXPECT_TRUE(q->pure());
    q = measured("atan2(1 in, 25.4 mm)");
    ASSERT_TRUE(q);
    EXPECT_NEAR(q->value, hz::math::kPi / 4, 1e-15);
    EXPECT_EQ(q->angle, 1);
    q = measured("tilt * 2");
    ASSERT_TRUE(q);
    EXPECT_EQ(q->angle, 1);

    std::string why;
    for (const char* text :
         {"2 in + 3", "wall + count", "(2 in) mm", "2 in ^ 0.5", "1 / 0", "nosuch * 2", "sin(2 in)",
          "asin(2 in)", "sqrt(2 in)", "frobnicate(1)", "wall ^ wall"}) {
        why.clear();
        EXPECT_FALSE(measured(text, &why)) << text;
        EXPECT_FALSE(why.empty()) << text;
    }
}

// Kept with units written: what a plain number added to a length means is
// fixed when it is typed, so a later change of the document's unit changes
// nothing.
TEST(QuantityTest, APlainNumberIsKeptInTheUnitItWasTypedIn) {
    const std::string inInches = kept("wall + 1", QuantityKind::Length, LengthUnit::Inch);
    EXPECT_EQ(inInches, "(wall + (1 in))");
    EXPECT_DOUBLE_EQ(evaluateQuantity(*Expression::parse(inInches), kVariables)->value, 35.4);

    EXPECT_EQ(kept("2 * 3", QuantityKind::Length, LengthUnit::Inch), "((2 * 3) in)");
    EXPECT_EQ(kept("2 * 3", QuantityKind::Number, LengthUnit::Inch), "(2 * 3)");
    EXPECT_EQ(kept("30", QuantityKind::Angle, LengthUnit::Inch), "(30 deg)");
    EXPECT_EQ(kept("tilt + 5", QuantityKind::Angle, LengthUnit::Millimetre), "(tilt + (5 deg))");
    EXPECT_EQ(kept("wall * 2", QuantityKind::Length, LengthUnit::Foot), "(wall * 2)")
        << "a factor is a factor";
    EXPECT_EQ(kept("2 * (wall + 1)", QuantityKind::Length, LengthUnit::Millimetre),
              "(2 * (wall + (1 mm)))");

    std::string why;
    EXPECT_EQ(kept("wall * wall", QuantityKind::Length, LengthUnit::Inch, &why), "");
    EXPECT_NE(why.find("an area"), std::string::npos) << why;
    EXPECT_EQ(kept("wall", QuantityKind::Number, LengthUnit::Inch, &why), "");
    EXPECT_EQ(kept("wall * wall + 1", QuantityKind::Length, LengthUnit::Inch, &why), "")
        << "a plain number added to an area has no unit to take";
}
