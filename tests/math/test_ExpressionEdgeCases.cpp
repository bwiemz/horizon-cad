#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "horizon/math/Expression.h"

using namespace hz::math;

TEST(ExpressionEdgeCaseTest, DivisionByZero) {
    auto expr = Expression::parse("1 / 0");
    ASSERT_NE(expr, nullptr);
    double result = expr->evaluate({});
    EXPECT_TRUE(std::isinf(result));
}

TEST(ExpressionEdgeCaseTest, NestedFunctions) {
    auto expr = Expression::parse("sin(cos(0))");
    ASSERT_NE(expr, nullptr);
    // cos(0) = 1, sin(1) ~ 0.8414709848
    EXPECT_NEAR(expr->evaluate({}), std::sin(1.0), 1e-10);
}

TEST(ExpressionEdgeCaseTest, DeepNesting) {
    auto expr = Expression::parse("((((1 + 2) + 3) + 4) + 5)");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 15.0);
}

TEST(ExpressionEdgeCaseTest, WhitespaceHandling) {
    auto expr = Expression::parse("  2  +  3  ");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 5.0);
}

TEST(ExpressionEdgeCaseTest, MultipleOperators) {
    // 2 + 3 - 1 * 4 / 2 = 2 + 3 - 2 = 3
    auto expr = Expression::parse("2 + 3 - 1 * 4 / 2");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 3.0);
}

TEST(ExpressionEdgeCaseTest, PowerRightAssociativity) {
    // 2 ^ 3 ^ 2 = 2 ^ 9 = 512 (right-associative)
    auto expr = Expression::parse("2 ^ 3 ^ 2");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 512.0);
}

TEST(ExpressionEdgeCaseTest, MultiArgFunction) {
    // atan2(0, -1) = pi
    auto expr = Expression::parse("atan2(0, -1)");
    ASSERT_NE(expr, nullptr);
    EXPECT_NEAR(expr->evaluate({}), 3.14159265358979, 1e-10);
}

TEST(ExpressionEdgeCaseTest, TrailingWhitespace) {
    auto expr = Expression::parse("42   ");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 42.0);
}

TEST(ExpressionEdgeCaseTest, ConsecutiveUnaryMinus) {
    // --5 should parse as -(-5) = 5
    auto expr = Expression::parse("--5");
    if (expr) {
        EXPECT_DOUBLE_EQ(expr->evaluate({}), 5.0);
    }
    // It's also acceptable for the parser to reject this
}

TEST(ExpressionEdgeCaseTest, ZeroPower) {
    auto expr = Expression::parse("5 ^ 0");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 1.0);
}

// ---------------------------------------------------------------------------
// Hostile input: expressions arrive from files, so size and depth are bounded
// ---------------------------------------------------------------------------

namespace {

std::string nestedParens(int depth) {
    return std::string(static_cast<size_t>(depth), '(') + "1" +
           std::string(static_cast<size_t>(depth), ')');
}

std::string flatSum(int terms) {
    std::string s = "1";
    for (int i = 1; i < terms; ++i) s += "+1";
    return s;
}

}  // namespace

TEST(ExpressionLimitsTest, NestingUpToTheLimitParses) {
    // Each bracket is one level of nesting.
    auto e = hz::math::Expression::parse(nestedParens(hz::math::Expression::kMaxNestingDepth));
    ASSERT_NE(e, nullptr);
    EXPECT_DOUBLE_EQ(e->evaluate({}), 1.0);
}

TEST(ExpressionLimitsTest, NestingPastTheLimitIsRejected) {
    EXPECT_EQ(hz::math::Expression::parse(nestedParens(hz::math::Expression::kMaxNestingDepth + 1)),
              nullptr);
}

TEST(ExpressionLimitsTest, AStackOverflowOfMinusSignsIsRejected) {
    // 200 000 nested negations used to recurse once each and overflow the stack.
    EXPECT_EQ(hz::math::Expression::parse(std::string(200000, '-') + "1"), nullptr);
}

TEST(ExpressionLimitsTest, ALongPowerTowerIsRejected) {
    std::string tower = "2";
    for (int i = 0; i < 10000; ++i) tower += "^1";
    EXPECT_EQ(hz::math::Expression::parse(tower), nullptr);
}

TEST(ExpressionLimitsTest, ALongFlatChainIsRejected) {
    // Parsed in a loop, but it builds a tree as deep as it is long, which
    // evaluation and destruction would recurse through.
    EXPECT_EQ(hz::math::Expression::parse(flatSum(200000)), nullptr);
}

TEST(ExpressionLimitsTest, AReasonableChainStillParses) {
    auto e = hz::math::Expression::parse(flatSum(400));  // 799 nodes
    ASSERT_NE(e, nullptr);
    EXPECT_DOUBLE_EQ(e->evaluate({}), 400.0);
}

TEST(ExpressionLimitsTest, FromJsonRejectsDeepTreesAndWrongTypes) {
    nlohmann::json deep = {{"type", "literal"}, {"value", 1.0}};
    for (int i = 0; i < 5000; ++i) {
        deep = {{"type", "unary"}, {"op", "-"}, {"child", deep}};
    }
    EXPECT_EQ(hz::math::Expression::fromJson(deep), nullptr);

    // A number where the operator string belongs made json::get throw.
    const nlohmann::json wrongType = {
        {"type", "unary"}, {"op", 7}, {"child", {{"type", "literal"}, {"value", 1.0}}}};
    std::unique_ptr<hz::math::Expression> parsed;
    ASSERT_NO_THROW(parsed = hz::math::Expression::fromJson(wrongType));
    EXPECT_EQ(parsed, nullptr);

    // A normal tree still round-trips.
    auto e = hz::math::Expression::parse("2*(x+3)^2");
    ASSERT_NE(e, nullptr);
    auto back = hz::math::Expression::fromJson(e->toJson());
    ASSERT_NE(back, nullptr);
    EXPECT_DOUBLE_EQ(back->evaluate({{"x", 1.0}}), 32.0);
}
