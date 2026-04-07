#include <gtest/gtest.h>

#include <cmath>

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
