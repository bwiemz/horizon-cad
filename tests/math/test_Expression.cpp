#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <set>
#include <string>

#include <nlohmann/json.hpp>

#include "horizon/math/Expression.h"

using namespace hz::math;

// ===========================================================================
// Parsing & Evaluation
// ===========================================================================

TEST(ExpressionTest, ParseLiteral) {
    auto expr = Expression::parse("42.5");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 42.5);
}

TEST(ExpressionTest, ParseNegativeLiteral) {
    auto expr = Expression::parse("-3.14");
    ASSERT_NE(expr, nullptr);
    EXPECT_NEAR(expr->evaluate({}), -3.14, 1e-10);
}

TEST(ExpressionTest, ParseAddition) {
    auto expr = Expression::parse("2 + 3");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 5.0);
}

TEST(ExpressionTest, ParseSubtraction) {
    auto expr = Expression::parse("10 - 4");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 6.0);
}

TEST(ExpressionTest, ParseMultiplication) {
    auto expr = Expression::parse("3 * 7");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 21.0);
}

TEST(ExpressionTest, ParseDivision) {
    auto expr = Expression::parse("20 / 4");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 5.0);
}

TEST(ExpressionTest, ParsePower) {
    auto expr = Expression::parse("2 ^ 10");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 1024.0);
}

TEST(ExpressionTest, ParseParentheses) {
    auto expr = Expression::parse("(2 + 3) * 4");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 20.0);
}

TEST(ExpressionTest, ParseNestedParens) {
    auto expr = Expression::parse("((1 + 2) * (3 + 4))");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 21.0);
}

TEST(ExpressionTest, OperatorPrecedence) {
    auto expr = Expression::parse("2 + 3 * 4");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 14.0);  // not 20
}

TEST(ExpressionTest, ParseVariable) {
    auto expr = Expression::parse("width");
    ASSERT_NE(expr, nullptr);
    std::map<std::string, double> vars = {{"width", 50.0}};
    EXPECT_DOUBLE_EQ(expr->evaluate(vars), 50.0);
}

TEST(ExpressionTest, ParseVariableExpression) {
    auto expr = Expression::parse("width * 2 + offset");
    ASSERT_NE(expr, nullptr);
    std::map<std::string, double> vars = {{"width", 10.0}, {"offset", 5.0}};
    EXPECT_DOUBLE_EQ(expr->evaluate(vars), 25.0);
}

TEST(ExpressionTest, ParsePi) {
    auto expr = Expression::parse("pi");
    ASSERT_NE(expr, nullptr);
    EXPECT_NEAR(expr->evaluate({}), 3.14159265358979323846, 1e-15);
}

TEST(ExpressionTest, ParseSin) {
    auto expr = Expression::parse("sin(0)");
    ASSERT_NE(expr, nullptr);
    EXPECT_NEAR(expr->evaluate({}), 0.0, 1e-10);
}

TEST(ExpressionTest, ParseCos) {
    auto expr = Expression::parse("cos(0)");
    ASSERT_NE(expr, nullptr);
    EXPECT_NEAR(expr->evaluate({}), 1.0, 1e-10);
}

TEST(ExpressionTest, ParseTan) {
    auto expr = Expression::parse("tan(0)");
    ASSERT_NE(expr, nullptr);
    EXPECT_NEAR(expr->evaluate({}), 0.0, 1e-10);
}

TEST(ExpressionTest, ParseSqrt) {
    auto expr = Expression::parse("sqrt(144)");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 12.0);
}

TEST(ExpressionTest, ParseAbs) {
    auto expr = Expression::parse("abs(-7.5)");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 7.5);
}

TEST(ExpressionTest, ParseAtan2) {
    auto expr = Expression::parse("atan2(1, 1)");
    ASSERT_NE(expr, nullptr);
    EXPECT_NEAR(expr->evaluate({}), std::atan2(1.0, 1.0), 1e-10);
}

TEST(ExpressionTest, ComplexExpression) {
    auto expr = Expression::parse("sqrt(width^2 + height^2)");
    ASSERT_NE(expr, nullptr);
    std::map<std::string, double> vars = {{"width", 3.0}, {"height", 4.0}};
    EXPECT_NEAR(expr->evaluate(vars), 5.0, 1e-10);
}

TEST(ExpressionTest, UnaryNegation) {
    auto expr = Expression::parse("-(2 + 3)");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), -5.0);
}

TEST(ExpressionTest, InvalidExpressionReturnsNull) {
    EXPECT_EQ(Expression::parse(""), nullptr);
    EXPECT_EQ(Expression::parse("2 +"), nullptr);
    EXPECT_EQ(Expression::parse("* 3"), nullptr);
    EXPECT_EQ(Expression::parse("(2 + 3"), nullptr);
}

TEST(ExpressionTest, UndefinedVariableReturnsZero) {
    auto expr = Expression::parse("undefined_var");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 0.0);
}

// ===========================================================================
// Variable Extraction
// ===========================================================================

TEST(ExpressionTest, ExtractVariables) {
    auto expr = Expression::parse("width * 2 + height");
    ASSERT_NE(expr, nullptr);
    auto vars = expr->variables();
    std::set<std::string> expected = {"width", "height"};
    EXPECT_EQ(vars, expected);
}

TEST(ExpressionTest, LiteralHasNoVariables) {
    auto expr = Expression::parse("42");
    ASSERT_NE(expr, nullptr);
    EXPECT_TRUE(expr->variables().empty());
}

// ===========================================================================
// toString Round-Trip
// ===========================================================================

TEST(ExpressionTest, ToStringRoundTrip) {
    auto expr1 = Expression::parse("(2 + 3) * 4");
    ASSERT_NE(expr1, nullptr);

    std::string str = expr1->toString();
    auto expr2 = Expression::parse(str);
    ASSERT_NE(expr2, nullptr);

    // Both should evaluate to the same value
    EXPECT_DOUBLE_EQ(expr1->evaluate({}), expr2->evaluate({}));
    EXPECT_DOUBLE_EQ(expr2->evaluate({}), 20.0);
}

// ===========================================================================
// JSON Serialization
// ===========================================================================

TEST(ExpressionTest, SerializeToJson) {
    auto expr = Expression::parse("width * 2 + 5");
    ASSERT_NE(expr, nullptr);
    nlohmann::json j = expr->toJson();
    EXPECT_TRUE(j.contains("type"));
}

TEST(ExpressionTest, DeserializeFromJson) {
    auto expr1 = Expression::parse("sqrt(width ^ 2 + height ^ 2)");
    ASSERT_NE(expr1, nullptr);
    nlohmann::json j = expr1->toJson();
    auto expr2 = Expression::fromJson(j);
    ASSERT_NE(expr2, nullptr);
    std::map<std::string, double> vars = {{"width", 3.0}, {"height", 4.0}};
    EXPECT_NEAR(expr1->evaluate(vars), expr2->evaluate(vars), 1e-10);
}

TEST(ExpressionTest, JsonRoundTripAllNodeTypes) {
    // Literal
    auto lit = Expression::parse("42.5");
    EXPECT_NEAR(Expression::fromJson(lit->toJson())->evaluate({}), 42.5, 1e-10);
    // Variable
    auto var = Expression::parse("width");
    EXPECT_DOUBLE_EQ(Expression::fromJson(var->toJson())->evaluate({{"width", 7.0}}), 7.0);
    // BinaryOp
    auto binop = Expression::parse("2 + 3");
    EXPECT_DOUBLE_EQ(Expression::fromJson(binop->toJson())->evaluate({}), 5.0);
    // UnaryOp
    auto unary = Expression::parse("-(5)");
    EXPECT_DOUBLE_EQ(Expression::fromJson(unary->toJson())->evaluate({}), -5.0);
    // FunctionCall
    auto func = Expression::parse("sin(pi / 2)");
    EXPECT_NEAR(Expression::fromJson(func->toJson())->evaluate({}), 1.0, 1e-10);
}

TEST(ExpressionTest, JsonFromJsonInvalid) {
    // Missing type
    nlohmann::json j1 = {{"value", 42.0}};
    EXPECT_EQ(Expression::fromJson(j1), nullptr);
    // Unknown type
    nlohmann::json j2 = {{"type", "unknown"}};
    EXPECT_EQ(Expression::fromJson(j2), nullptr);
    // Not an object
    nlohmann::json j3 = 42;
    EXPECT_EQ(Expression::fromJson(j3), nullptr);
}
