#pragma once

#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace hz::math {

// ---------------------------------------------------------------------------
// Abstract base class for expression AST nodes
// ---------------------------------------------------------------------------
class Expression {
public:
    virtual ~Expression() = default;

    /// Evaluate the expression with the given variable bindings.
    virtual double evaluate(const std::map<std::string, double>& variables) const = 0;

    /// Collect all referenced variable names.
    virtual std::set<std::string> variables() const = 0;

    /// Produce a re-parseable string representation.
    virtual std::string toString() const = 0;

    /// Serialize AST node to JSON.
    virtual nlohmann::json toJson() const = 0;

    /// Deserialize AST node from JSON; returns nullptr on any error.
    static std::unique_ptr<Expression> fromJson(const nlohmann::json& j);

    /// Parse an expression string; returns nullptr on any error.
    static std::unique_ptr<Expression> parse(const std::string& input);
};

// ---------------------------------------------------------------------------
// LiteralExpr -- constant numeric value
// ---------------------------------------------------------------------------
class LiteralExpr : public Expression {
public:
    explicit LiteralExpr(double value) : m_value(value) {}

    double evaluate(const std::map<std::string, double>& variables) const override;
    std::set<std::string> variables() const override;
    std::string toString() const override;
    nlohmann::json toJson() const override;

    double value() const { return m_value; }

private:
    double m_value;
};

// ---------------------------------------------------------------------------
// VariableExpr -- variable reference
// ---------------------------------------------------------------------------
class VariableExpr : public Expression {
public:
    explicit VariableExpr(std::string name) : m_name(std::move(name)) {}

    double evaluate(const std::map<std::string, double>& variables) const override;
    std::set<std::string> variables() const override;
    std::string toString() const override;
    nlohmann::json toJson() const override;

    const std::string& name() const { return m_name; }

private:
    std::string m_name;
};

// ---------------------------------------------------------------------------
// BinaryOpExpr -- binary operator (Add, Sub, Mul, Div, Pow)
// ---------------------------------------------------------------------------
class BinaryOpExpr : public Expression {
public:
    enum class Op { Add, Sub, Mul, Div, Pow };

    BinaryOpExpr(Op op, std::unique_ptr<Expression> left, std::unique_ptr<Expression> right)
        : m_op(op), m_left(std::move(left)), m_right(std::move(right)) {}

    double evaluate(const std::map<std::string, double>& variables) const override;
    std::set<std::string> variables() const override;
    std::string toString() const override;
    nlohmann::json toJson() const override;

    Op op() const { return m_op; }
    const Expression& left() const { return *m_left; }
    const Expression& right() const { return *m_right; }

private:
    Op m_op;
    std::unique_ptr<Expression> m_left;
    std::unique_ptr<Expression> m_right;
};

// ---------------------------------------------------------------------------
// UnaryOpExpr -- unary operator (Negate)
// ---------------------------------------------------------------------------
class UnaryOpExpr : public Expression {
public:
    enum class Op { Negate };

    UnaryOpExpr(Op op, std::unique_ptr<Expression> child)
        : m_op(op), m_child(std::move(child)) {}

    double evaluate(const std::map<std::string, double>& variables) const override;
    std::set<std::string> variables() const override;
    std::string toString() const override;
    nlohmann::json toJson() const override;

    Op op() const { return m_op; }
    const Expression& child() const { return *m_child; }

private:
    Op m_op;
    std::unique_ptr<Expression> m_child;
};

// ---------------------------------------------------------------------------
// FunctionCallExpr -- built-in function call
// ---------------------------------------------------------------------------
class FunctionCallExpr : public Expression {
public:
    FunctionCallExpr(std::string name, std::vector<std::unique_ptr<Expression>> args)
        : m_name(std::move(name)), m_args(std::move(args)) {}

    double evaluate(const std::map<std::string, double>& variables) const override;
    std::set<std::string> variables() const override;
    std::string toString() const override;
    nlohmann::json toJson() const override;

    const std::string& name() const { return m_name; }
    const std::vector<std::unique_ptr<Expression>>& args() const { return m_args; }

private:
    std::string m_name;
    std::vector<std::unique_ptr<Expression>> m_args;
};

}  // namespace hz::math
