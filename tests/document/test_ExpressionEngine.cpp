#include <gtest/gtest.h>

#include "horizon/document/ExpressionEngine.h"

#include <algorithm>

using namespace hz::doc;

TEST(ExpressionEngineTest, DefineAndEvaluateVariable) {
    ExpressionEngine engine;
    engine.setLiteral("width", 50.0);
    EXPECT_DOUBLE_EQ(engine.getValue("width"), 50.0);
}

TEST(ExpressionEngineTest, DefineExpressionVariable) {
    ExpressionEngine engine;
    engine.setLiteral("width", 10.0);
    engine.setExpression("doubled", "width * 2");
    EXPECT_DOUBLE_EQ(engine.getValue("doubled"), 20.0);
}

TEST(ExpressionEngineTest, ChainedExpressions) {
    ExpressionEngine engine;
    engine.setLiteral("base", 5.0);
    engine.setExpression("double_base", "base * 2");
    engine.setExpression("quad_base", "double_base * 2");
    EXPECT_DOUBLE_EQ(engine.getValue("quad_base"), 20.0);
}

TEST(ExpressionEngineTest, ChangeVariablePropagates) {
    ExpressionEngine engine;
    engine.setLiteral("width", 10.0);
    engine.setExpression("area", "width * width");
    EXPECT_DOUBLE_EQ(engine.getValue("area"), 100.0);
    engine.setLiteral("width", 20.0);
    EXPECT_DOUBLE_EQ(engine.getValue("area"), 400.0);
}

TEST(ExpressionEngineTest, CircularDependencyDetected) {
    ExpressionEngine engine;
    engine.setExpression("a", "b + 1");
    engine.setExpression("b", "a + 1");
    EXPECT_TRUE(engine.hasCycle());
    auto cycle = engine.describeCycle();
    EXPECT_FALSE(cycle.empty());
}

TEST(ExpressionEngineTest, SelfReferenceDetected) {
    ExpressionEngine engine;
    engine.setExpression("x", "x + 1");
    EXPECT_TRUE(engine.hasCycle());
}

TEST(ExpressionEngineTest, NoCycleWithIndependentVars) {
    ExpressionEngine engine;
    engine.setLiteral("a", 1.0);
    engine.setLiteral("b", 2.0);
    engine.setExpression("c", "a + b");
    EXPECT_FALSE(engine.hasCycle());
}

TEST(ExpressionEngineTest, EvaluationOrder) {
    ExpressionEngine engine;
    engine.setLiteral("x", 3.0);
    engine.setExpression("y", "x + 1");
    engine.setExpression("z", "y * 2");
    auto order = engine.evaluationOrder();
    auto xPos = std::find(order.begin(), order.end(), "x");
    auto yPos = std::find(order.begin(), order.end(), "y");
    auto zPos = std::find(order.begin(), order.end(), "z");
    EXPECT_LT(xPos, yPos);
    EXPECT_LT(yPos, zPos);
}

TEST(ExpressionEngineTest, RemoveVariable) {
    ExpressionEngine engine;
    engine.setLiteral("width", 10.0);
    engine.setExpression("area", "width * width");
    engine.remove("area");
    EXPECT_DOUBLE_EQ(engine.getValue("area"), 0.0);
}

TEST(ExpressionEngineTest, AllVariablesReturned) {
    ExpressionEngine engine;
    engine.setLiteral("a", 1.0);
    engine.setExpression("b", "a * 2");
    auto all = engine.allValues();
    EXPECT_EQ(all.size(), 2u);
    EXPECT_DOUBLE_EQ(all.at("a"), 1.0);
    EXPECT_DOUBLE_EQ(all.at("b"), 2.0);
}

TEST(ExpressionEngineTest, ClearRemovesEverything) {
    ExpressionEngine engine;
    engine.setLiteral("x", 5.0);
    engine.setExpression("y", "x * 2");
    engine.clear();
    EXPECT_FALSE(engine.has("x"));
    EXPECT_FALSE(engine.has("y"));
    EXPECT_TRUE(engine.allValues().empty());
}
