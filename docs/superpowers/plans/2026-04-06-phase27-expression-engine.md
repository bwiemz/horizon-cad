# Phase 27: Expression Engine & Driven Dimensions Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enable parametric relationships via algebraic expressions in dimensions and constraints — `width * 2 + 5` evaluates to a numeric value driven by design variables.

**Architecture:** A recursive descent parser in `hz::math` produces a serializable AST. An `ExpressionEngine` in `hz::doc` manages the dependency graph, detects cycles via topological sort, and propagates variable changes through the evaluate→re-solve→re-render chain. The AST serializes to JSON for `.hcad` persistence.

**Tech Stack:** C++20, nlohmann::json, Google Test

**Spec Reference:** `docs/superpowers/specs/2026-04-05-horizon-cad-roadmap-design.md` — Section 3.3 (Phase 27)

---

## Spec Compliance Check

| Spec Requirement | Plan Task | Status |
|---|---|---|
| Recursive descent parser producing serializable AST | Task 1 | ✅ |
| Operations: `+`, `-`, `*`, `/`, `^`, `sin`, `cos`, `tan`, `sqrt`, `abs`, `pi`, `atan2`, parens, variables | Task 1 | ✅ |
| AST node types: Literal, Variable, BinaryOp, UnaryOp, FunctionCall | Task 1 | ✅ |
| AST serializes to `.hcad` JSON | Task 3 | ✅ |
| DimensionStyle gains optional expression field | Task 4 | ✅ |
| Design variables accessible in expressions: `width * 2 + offset` | Task 2 | ✅ |
| Circular dependency detection via topological sort | Task 2 | ✅ |
| Error on cycles with clear message | Task 2 | ✅ |
| Propagation: variable change → re-evaluate → re-solve → re-render | Task 5 | ✅ |
| Tests: parse/evaluate, variable substitution, circular deps, serialization round-trip | Tasks 1-3 | ✅ |

---

## File Map

| Action | File | Responsibility |
|--------|------|---------------|
| Create | `src/math/include/horizon/math/Expression.h` | AST node types + recursive descent parser |
| Create | `src/math/src/Expression.cpp` | Parser and evaluator implementation |
| Create | `tests/math/test_Expression.cpp` | Parser, evaluator, and serialization tests |
| Modify | `src/math/CMakeLists.txt` | Add Expression.cpp |
| Modify | `tests/math/CMakeLists.txt` | Add test_Expression.cpp, link nlohmann_json |
| Create | `src/document/include/horizon/document/ExpressionEngine.h` | Dependency graph, cycle detection, propagation |
| Create | `src/document/src/ExpressionEngine.cpp` | Implementation |
| Create | `tests/document/test_ExpressionEngine.cpp` | Engine integration tests |
| Modify | `src/document/CMakeLists.txt` | Add ExpressionEngine.cpp |
| Modify | `tests/document/CMakeLists.txt` | Add test_ExpressionEngine.cpp |
| Modify | `src/document/include/horizon/document/ParameterRegistry.h` | Add expression storage per variable |
| Modify | `src/document/src/ParameterRegistry.cpp` | Implementation |
| Modify | `src/document/include/horizon/document/Document.h` | Add ExpressionEngine member |
| Modify | `src/document/src/Document.cpp` | Wire ExpressionEngine |
| Modify | `src/fileio/src/NativeFormat.cpp` | Serialize expressions in .hcad v14 |

---

## Task 1: AST + Recursive Descent Parser + Evaluator

**Files:**
- Create: `src/math/include/horizon/math/Expression.h`
- Create: `src/math/src/Expression.cpp`
- Create: `tests/math/test_Expression.cpp`
- Modify: `src/math/CMakeLists.txt`
- Modify: `tests/math/CMakeLists.txt`

- [ ] **Step 1: Write failing tests for expression parsing and evaluation**

Create `tests/math/test_Expression.cpp`:
```cpp
#include <gtest/gtest.h>
#include "horizon/math/Expression.h"

using namespace hz::math;

// --- Parsing & Evaluation ---

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
    // 2 + 3 * 4 = 14 (not 20)
    auto expr = Expression::parse("2 + 3 * 4");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 14.0);
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
    EXPECT_NEAR(expr->evaluate({}), 3.14159265358979, 1e-10);
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
    EXPECT_NEAR(expr->evaluate({}), 0.7853981633974483, 1e-10);  // pi/4
}

TEST(ExpressionTest, ComplexExpression) {
    auto expr = Expression::parse("sqrt(width ^ 2 + height ^ 2)");
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
    EXPECT_EQ(Expression::parse("(2 + 3"), nullptr);  // Unbalanced parens
}

TEST(ExpressionTest, UndefinedVariableReturnsZero) {
    auto expr = Expression::parse("undefined_var");
    ASSERT_NE(expr, nullptr);
    EXPECT_DOUBLE_EQ(expr->evaluate({}), 0.0);
}

// --- Variable Extraction ---

TEST(ExpressionTest, ExtractVariables) {
    auto expr = Expression::parse("width * 2 + height");
    ASSERT_NE(expr, nullptr);
    auto vars = expr->variables();
    EXPECT_EQ(vars.size(), 2u);
    EXPECT_TRUE(vars.count("width"));
    EXPECT_TRUE(vars.count("height"));
}

TEST(ExpressionTest, LiteralHasNoVariables) {
    auto expr = Expression::parse("42");
    ASSERT_NE(expr, nullptr);
    EXPECT_TRUE(expr->variables().empty());
}

// --- Serialization ---

TEST(ExpressionTest, ToStringRoundTrip) {
    auto expr1 = Expression::parse("width * 2 + 5");
    ASSERT_NE(expr1, nullptr);
    std::string str = expr1->toString();
    auto expr2 = Expression::parse(str);
    ASSERT_NE(expr2, nullptr);

    std::map<std::string, double> vars = {{"width", 10.0}};
    EXPECT_DOUBLE_EQ(expr1->evaluate(vars), expr2->evaluate(vars));
}
```

- [ ] **Step 2: Update CMakeLists**

Add `src/Expression.cpp` to `src/math/CMakeLists.txt`. The math library needs `nlohmann_json` for AST serialization — add it as a dependency.

Add `test_Expression.cpp` to `tests/math/CMakeLists.txt`.

- [ ] **Step 3: Implement Expression.h — AST types and parser interface**

Create `src/math/include/horizon/math/Expression.h`:
```cpp
#pragma once

#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace hz::math {

/// Base class for expression AST nodes.
class Expression {
public:
    virtual ~Expression() = default;

    /// Evaluate the expression with the given variable bindings.
    /// Undefined variables resolve to 0.0.
    virtual double evaluate(const std::map<std::string, double>& variables) const = 0;

    /// Collect all variable names referenced in this expression.
    virtual std::set<std::string> variables() const = 0;

    /// Serialize to a human-readable string that can be re-parsed.
    virtual std::string toString() const = 0;

    /// Parse an expression string into an AST.
    /// Returns nullptr on parse error.
    static std::unique_ptr<Expression> parse(const std::string& input);
};

/// Literal numeric value.
class LiteralExpr : public Expression {
public:
    explicit LiteralExpr(double value) : m_value(value) {}
    double evaluate(const std::map<std::string, double>&) const override { return m_value; }
    std::set<std::string> variables() const override { return {}; }
    std::string toString() const override;
    double value() const { return m_value; }
private:
    double m_value;
};

/// Variable reference.
class VariableExpr : public Expression {
public:
    explicit VariableExpr(std::string name) : m_name(std::move(name)) {}
    double evaluate(const std::map<std::string, double>& vars) const override {
        auto it = vars.find(m_name);
        return (it != vars.end()) ? it->second : 0.0;
    }
    std::set<std::string> variables() const override { return {m_name}; }
    std::string toString() const override { return m_name; }
    const std::string& name() const { return m_name; }
private:
    std::string m_name;
};

/// Binary operation: left op right.
class BinaryOpExpr : public Expression {
public:
    enum class Op { Add, Sub, Mul, Div, Pow };

    BinaryOpExpr(Op op, std::unique_ptr<Expression> left, std::unique_ptr<Expression> right)
        : m_op(op), m_left(std::move(left)), m_right(std::move(right)) {}
    double evaluate(const std::map<std::string, double>& vars) const override;
    std::set<std::string> variables() const override;
    std::string toString() const override;
    Op op() const { return m_op; }
private:
    Op m_op;
    std::unique_ptr<Expression> m_left, m_right;
};

/// Unary operation: op(child).
class UnaryOpExpr : public Expression {
public:
    enum class Op { Negate };

    UnaryOpExpr(Op op, std::unique_ptr<Expression> child)
        : m_op(op), m_child(std::move(child)) {}
    double evaluate(const std::map<std::string, double>& vars) const override;
    std::set<std::string> variables() const override { return m_child->variables(); }
    std::string toString() const override;
private:
    Op m_op;
    std::unique_ptr<Expression> m_child;
};

/// Function call: name(args...).
class FunctionCallExpr : public Expression {
public:
    FunctionCallExpr(std::string name, std::vector<std::unique_ptr<Expression>> args)
        : m_name(std::move(name)), m_args(std::move(args)) {}
    double evaluate(const std::map<std::string, double>& vars) const override;
    std::set<std::string> variables() const override;
    std::string toString() const override;
private:
    std::string m_name;
    std::vector<std::unique_ptr<Expression>> m_args;
};

}  // namespace hz::math
```

- [ ] **Step 4: Implement Expression.cpp — recursive descent parser**

Create `src/math/src/Expression.cpp`:

The parser should implement standard recursive descent with these precedence levels:
1. `^` (right-associative, highest)
2. `*`, `/`
3. `+`, `-` (lowest)

Grammar:
```
expression = term (('+' | '-') term)*
term       = power (('*' | '/') power)*
power      = unary ('^' power)?       // right-associative
unary      = '-' unary | primary
primary    = NUMBER | IDENTIFIER | IDENTIFIER '(' args ')' | '(' expression ')'
args       = expression (',' expression)*
```

Built-in constants: `pi` → 3.14159265358979...

Built-in functions (1-arg): `sin`, `cos`, `tan`, `sqrt`, `abs`, `asin`, `acos`, `atan`
Built-in functions (2-arg): `atan2`

The `evaluate()` methods:
- `BinaryOpExpr`: switch on op, compute left op right
- `UnaryOpExpr::Negate`: return -child.evaluate()
- `FunctionCallExpr`: lookup function name, evaluate args, call std::sin/cos/etc.

The `toString()` methods:
- `LiteralExpr`: format double with enough precision to round-trip
- `VariableExpr`: return name
- `BinaryOpExpr`: `"(" + left.toString() + " op " + right.toString() + ")"`
- `UnaryOpExpr::Negate`: `"(-" + child.toString() + ")"`
- `FunctionCallExpr`: `name + "(" + args.join(", ") + ")"`

- [ ] **Step 5: Build and run tests**

Run: `cmake --build build/debug --config Debug && ctest --test-dir build/debug -C Debug --output-on-failure`
Expected: All existing tests + ~25 new Expression tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/math/include/horizon/math/Expression.h src/math/src/Expression.cpp \
        tests/math/test_Expression.cpp src/math/CMakeLists.txt tests/math/CMakeLists.txt
git commit -m "feat(math): add expression engine with recursive descent parser

AST node types: Literal, Variable, BinaryOp, UnaryOp, FunctionCall.
Supports +, -, *, /, ^, sin, cos, tan, sqrt, abs, atan2, pi, variables.
Recursive descent parser with correct operator precedence."
```

---

## Task 2: ExpressionEngine — Dependency Graph & Cycle Detection

**Files:**
- Create: `src/document/include/horizon/document/ExpressionEngine.h`
- Create: `src/document/src/ExpressionEngine.cpp`
- Create: `tests/document/test_ExpressionEngine.cpp`
- Modify: `src/document/CMakeLists.txt`
- Modify: `tests/document/CMakeLists.txt`

- [ ] **Step 1: Write failing tests for ExpressionEngine**

Create `tests/document/test_ExpressionEngine.cpp`:
```cpp
#include <gtest/gtest.h>
#include "horizon/document/ExpressionEngine.h"

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
    // Should contain something like "a → b → a"
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
    // Topological order should evaluate x first, then y, then z.
    auto order = engine.evaluationOrder();
    // x before y before z.
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
    EXPECT_DOUBLE_EQ(engine.getValue("area"), 0.0);  // Not found → 0.0
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
```

- [ ] **Step 2: Implement ExpressionEngine.h**

```cpp
#pragma once

#include "horizon/math/Expression.h"
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace hz::doc {

/// Manages named variables with optional expressions.
/// Handles dependency tracking, cycle detection, and evaluation ordering.
class ExpressionEngine {
public:
    ExpressionEngine() = default;

    /// Set a variable to a literal value (no expression).
    void setLiteral(const std::string& name, double value);

    /// Set a variable to an expression string.
    /// The expression is parsed immediately. Variables in the expression
    /// are resolved from other variables in the engine.
    void setExpression(const std::string& name, const std::string& expressionStr);

    /// Get the current evaluated value of a variable (0.0 if not found).
    [[nodiscard]] double getValue(const std::string& name) const;

    /// Get the expression string for a variable (empty if literal or not found).
    [[nodiscard]] std::string getExpression(const std::string& name) const;

    /// Check if a variable is defined.
    [[nodiscard]] bool has(const std::string& name) const;

    /// Check if a variable is expression-driven (vs. literal).
    [[nodiscard]] bool isExpression(const std::string& name) const;

    /// Remove a variable.
    void remove(const std::string& name);

    /// Clear all variables.
    void clear();

    /// Get all variable names → values.
    [[nodiscard]] std::map<std::string, double> allValues() const;

    /// Check for circular dependencies.
    [[nodiscard]] bool hasCycle() const;

    /// Describe the circular dependency (e.g., "a → b → a").
    [[nodiscard]] std::string describeCycle() const;

    /// Get evaluation order (topological sort). Empty if cycle exists.
    [[nodiscard]] std::vector<std::string> evaluationOrder() const;

private:
    struct Variable {
        double value = 0.0;
        std::string expressionStr;                    // Empty for literals
        std::unique_ptr<math::Expression> expression; // nullptr for literals
    };

    std::map<std::string, Variable> m_variables;

    /// Re-evaluate all expression-driven variables in dependency order.
    void reevaluate();

    /// Build a variable → {dependencies} map.
    std::map<std::string, std::set<std::string>> buildDependencyGraph() const;

    /// Topological sort. Returns empty vector if cycle detected.
    std::vector<std::string> topologicalSort(
        const std::map<std::string, std::set<std::string>>& graph) const;
};

}  // namespace hz::doc
```

- [ ] **Step 3: Implement ExpressionEngine.cpp**

Key implementation details:

**setLiteral**: Store value, clear expression fields, call `reevaluate()`.

**setExpression**: Parse expression string, store AST, call `reevaluate()`.

**reevaluate**:
1. Build dependency graph from all expression variables
2. Topological sort
3. If cycle → don't evaluate, leave values stale
4. For each variable in topological order: if expression-driven, evaluate using current values of all other variables

**buildDependencyGraph**: For each expression variable, `expr->variables()` gives the set of dependencies. Only include dependencies that are actually defined variables.

**topologicalSort** (Kahn's algorithm):
1. Compute in-degree for each node
2. Queue all nodes with in-degree 0
3. Process: dequeue, add to result, decrement in-degree of dependents
4. If result.size() < total nodes → cycle detected

**hasCycle/describeCycle**: Build graph, attempt topological sort. If it fails, find the cycle by DFS and format as "a → b → a".

- [ ] **Step 4: Update CMakeLists**

Add `src/ExpressionEngine.cpp` to `src/document/CMakeLists.txt`.
Add `test_ExpressionEngine.cpp` to `tests/document/CMakeLists.txt`.

- [ ] **Step 5: Build and run all tests**

Expected: All tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/document/include/horizon/document/ExpressionEngine.h \
        src/document/src/ExpressionEngine.cpp \
        tests/document/test_ExpressionEngine.cpp \
        src/document/CMakeLists.txt tests/document/CMakeLists.txt
git commit -m "feat(document): add ExpressionEngine with dependency graph and cycle detection

Manages named variables with optional expressions. Topological sort
for evaluation order. Detects circular dependencies with clear error messages."
```

---

## Task 3: AST JSON Serialization + Round-Trip Tests

**Files:**
- Modify: `src/math/include/horizon/math/Expression.h`
- Modify: `src/math/src/Expression.cpp`
- Modify: `tests/math/test_Expression.cpp`

- [ ] **Step 1: Write failing tests for JSON serialization**

Append to `tests/math/test_Expression.cpp`:
```cpp
#include <nlohmann/json.hpp>

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
    auto varRT = Expression::fromJson(var->toJson());
    EXPECT_DOUBLE_EQ(varRT->evaluate({{"width", 7.0}}), 7.0);

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
```

- [ ] **Step 2: Add toJson() and fromJson() to Expression classes**

Add to `Expression` base class:
```cpp
virtual nlohmann::json toJson() const = 0;
static std::unique_ptr<Expression> fromJson(const nlohmann::json& j);
```

JSON format:
```json
// Literal
{"type": "literal", "value": 42.5}

// Variable
{"type": "variable", "name": "width"}

// BinaryOp
{"type": "binary", "op": "+", "left": {...}, "right": {...}}

// UnaryOp
{"type": "unary", "op": "-", "child": {...}}

// FunctionCall
{"type": "function", "name": "sin", "args": [{...}]}
```

- [ ] **Step 3: Build and run tests**

Expected: All tests pass including JSON serialization round-trips.

- [ ] **Step 4: Commit**

```bash
git add src/math/include/horizon/math/Expression.h src/math/src/Expression.cpp \
        tests/math/test_Expression.cpp
git commit -m "feat(math): add JSON serialization for expression AST

toJson()/fromJson() for all node types. Round-trip preserves semantics."
```

---

## Task 4: Wire Expressions into ParameterRegistry and Document

**Files:**
- Modify: `src/document/include/horizon/document/ParameterRegistry.h`
- Modify: `src/document/src/ParameterRegistry.cpp`
- Modify: `src/document/include/horizon/document/Document.h`
- Modify: `src/document/src/Document.cpp`

- [ ] **Step 1: Extend ParameterRegistry with expression support**

Add to `ParameterRegistry.h`:
```cpp
/// Set a variable with an expression string. The expression is parsed
/// and the value is computed from the expression using current variables.
void setExpression(const std::string& name, const std::string& expressionStr);

/// Get the expression string for a variable (empty if literal).
[[nodiscard]] std::string getExpression(const std::string& name) const;

/// Check if a variable is expression-driven.
[[nodiscard]] bool isExpression(const std::string& name) const;
```

Internally, ParameterRegistry should delegate to ExpressionEngine for expression-driven variables. The simplest approach: replace the internal `std::map<std::string, double>` with an `ExpressionEngine` member.

- [ ] **Step 2: Add ExpressionEngine to Document**

Add to `Document.h`:
```cpp
ExpressionEngine& expressionEngine();
const ExpressionEngine& expressionEngine() const;
```

The ExpressionEngine should be the authoritative source for variable values. When the constraint solver needs variable resolution, it queries the ExpressionEngine.

- [ ] **Step 3: Build and run tests**

Verify all existing ParameterRegistry tests still pass (the API is backward compatible).

- [ ] **Step 4: Commit**

```bash
git add src/document/include/horizon/document/ParameterRegistry.h \
        src/document/src/ParameterRegistry.cpp \
        src/document/include/horizon/document/Document.h \
        src/document/src/Document.cpp
git commit -m "feat(document): wire ExpressionEngine into ParameterRegistry and Document

ParameterRegistry now supports expression-driven variables via ExpressionEngine.
Document gains expressionEngine() accessor."
```

---

## Task 5: Propagation Chain — Variable Change → Re-Evaluate → Re-Solve → Re-Render

**Files:**
- Modify: `src/document/src/ConstraintSolveHelper.cpp`
- Modify: `src/ui/src/ViewportWidget.cpp`
- Modify: `src/ui/src/SelectTool.cpp` (update double-click editing)

- [ ] **Step 1: Update ConstraintSolveHelper to use ExpressionEngine**

When solving, the variable resolver should pull values from the ExpressionEngine (which handles expression evaluation internally). Update the constraint tool and any callers that pass a variable resolver to use `ExpressionEngine::getValue`.

- [ ] **Step 2: Update double-click editing to re-evaluate expressions**

When a design variable is changed (e.g., via a future UI panel or double-click editing a variable-driven constraint), the ExpressionEngine re-evaluates all dependent expressions, then the constraint solver re-solves.

The propagation chain:
1. User changes variable `width` from 10 to 20
2. `ExpressionEngine::setLiteral("width", 20.0)` → reevaluates `area = width * width` → area becomes 400
3. `ConstraintSystem::resolveVariables()` → updates Distance constraints referencing `area` → distance becomes 400
4. `ConstraintSolveHelper::solveAndApply()` → entities move to satisfy constraints
5. `ViewportWidget::update()` → entities re-render at new positions

This chain should be triggered automatically by the ExpressionEngine when a variable changes.

- [ ] **Step 3: Build and run all tests**

Expected: All tests pass.

- [ ] **Step 4: Commit**

```bash
git add src/document/src/ConstraintSolveHelper.cpp src/ui/src/ViewportWidget.cpp \
        src/ui/src/SelectTool.cpp
git commit -m "feat(document): propagation chain for variable→expression→constraint→render

Variable change triggers re-evaluation of dependent expressions,
re-solve of dependent constraints, and viewport re-render."
```

---

## Task 6: Serialize Expressions in .hcad v14

**Files:**
- Modify: `src/fileio/src/NativeFormat.cpp`

- [ ] **Step 1: Extend save to serialize expressions**

In the save function, extend the `designVariables` section to include expression strings:

```json
"designVariables": {
    "width": {"value": 50.0},
    "area": {"value": 2500.0, "expression": "width * width"}
}
```

For variables with expressions, serialize both the current value (for preview) and the expression string (for re-evaluation on load).

- [ ] **Step 2: Extend load to deserialize expressions**

On load, for each design variable:
- If it has an `expression` field → call `engine.setExpression(name, expr)`
- If it only has a `value` → call `engine.setLiteral(name, value)`

Handle backward compatibility: v13 files have `designVariables` as a flat `{name: value}` map. v14 files have the nested format. Detect by checking if the value is a number (v13) or an object (v14).

- [ ] **Step 3: Bump format version to v14**

Only if the file contains expression-driven variables. Otherwise keep v13 for simpler files.

Actually, simpler approach: always write v14 format. v14 reader handles v13 files by treating numeric values as literals.

- [ ] **Step 4: Build and test round-trip**

Create a document with literal and expression-driven variables. Save, load, verify all values and expressions are preserved.

- [ ] **Step 5: Run all tests**

Expected: All tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/fileio/src/NativeFormat.cpp
git commit -m "feat(fileio): serialize expressions in .hcad v14

Design variables now serialize with expression strings. Backward
compatible with v13 flat format. Round-trips expression ASTs via
string representation."
```

---

## Task 7: Final Regression Testing + Phase Commit

- [ ] **Step 1: Run the complete test suite**

Run: `cmake --build build/debug --config Debug && ctest --test-dir build/debug -C Debug --output-on-failure`
Report exact test count.

- [ ] **Step 2: Commit**

```bash
git add -A
git commit -m "Phase 27: Expression engine with AST parser, dependency graph, and driven dimensions

- Recursive descent parser producing serializable AST (Literal, Variable, BinaryOp, UnaryOp, FunctionCall)
- Supports +, -, *, /, ^, sin, cos, tan, sqrt, abs, atan2, pi, parentheses, variables
- ExpressionEngine with topological sort for evaluation order
- Circular dependency detection with clear error messages
- Propagation chain: variable change → re-evaluate → re-solve → re-render
- .hcad v14 with expression serialization"
```
