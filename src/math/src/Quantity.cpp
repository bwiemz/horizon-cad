#include "horizon/math/Quantity.h"

#include <cmath>
#include <nlohmann/json.hpp>
#include <utility>
#include <vector>

namespace hz::math {

namespace {

std::nullopt_t fail(std::string* why, std::string text) {
    if (why != nullptr) *why = std::move(text);
    return std::nullopt;
}

/// A power a length or an angle may be raised to: whole, and small; and
/// the most lengths (or angles) a value may measure: chained variables
/// (a1 = wall ^ 12, a2 = a1 ^ 12, ...) multiply them, and the count must
/// not overflow.
constexpr double kMaxPower = 12.0;
constexpr int kMaxMeasure = 12;

/// @p q, if it measures no more than kMaxMeasure lengths and angles.
std::optional<Quantity> bounded(const Quantity& q, std::string* why) {
    if (std::abs(q.length) > kMaxMeasure || std::abs(q.angle) > kMaxMeasure) {
        return fail(why, "it measures " + measureName(q) + ", more than a value can");
    }
    return q;
}

std::optional<Quantity> evaluate(const Expression& e, const std::map<std::string, Quantity>& vars,
                                 std::string* why) {
    if (const auto* literal = dynamic_cast<const LiteralExpr*>(&e)) {
        return Quantity{literal->value(), 0, 0};
    }
    if (const auto* variable = dynamic_cast<const VariableExpr*>(&e)) {
        const auto found = vars.find(variable->name());
        if (found == vars.end()) return fail(why, "no variable is named " + variable->name());
        return found->second;
    }
    if (const auto* unit = dynamic_cast<const UnitExpr*>(&e)) {
        const auto child = evaluate(unit->child(), vars, why);
        if (!child) return std::nullopt;
        if (!child->pure()) {
            return fail(why, "a unit goes after a plain number, not after " + measureName(*child));
        }
        const bool angle = UnitExpr::isAngle(unit->unit());
        return Quantity{child->value * UnitExpr::factor(unit->unit()).value_or(1.0), angle ? 0 : 1,
                        angle ? 1 : 0};
    }
    if (const auto* unary = dynamic_cast<const UnaryOpExpr*>(&e)) {
        auto child = evaluate(unary->child(), vars, why);
        if (child) child->value = -child->value;
        return child;
    }
    if (const auto* binary = dynamic_cast<const BinaryOpExpr*>(&e)) {
        const auto l = evaluate(binary->left(), vars, why);
        if (!l) return std::nullopt;
        const auto r = evaluate(binary->right(), vars, why);
        if (!r) return std::nullopt;
        switch (binary->op()) {
            case BinaryOpExpr::Op::Add:
            case BinaryOpExpr::Op::Sub:
                if (!l->sameMeasure(*r)) {
                    return fail(why, "cannot add " + measureName(*r) + " to " + measureName(*l));
                }
                return Quantity{binary->op() == BinaryOpExpr::Op::Add ? l->value + r->value
                                                                      : l->value - r->value,
                                l->length, l->angle};
            case BinaryOpExpr::Op::Mul:
                return bounded(
                    Quantity{l->value * r->value, l->length + r->length, l->angle + r->angle}, why);
            case BinaryOpExpr::Op::Div:
                if (r->value == 0.0) return fail(why, "it divides by zero");
                return bounded(
                    Quantity{l->value / r->value, l->length - r->length, l->angle - r->angle}, why);
            case BinaryOpExpr::Op::Pow: {
                if (!r->pure()) return fail(why, "a power is a plain number");
                if (!std::isfinite(r->value)) return fail(why, "the power is not a number");
                if (l->pure()) return Quantity{std::pow(l->value, r->value), 0, 0};
                const double n = std::round(r->value);
                if (std::abs(r->value - n) > 1e-9 || std::abs(n) > kMaxPower) {
                    return fail(why, measureName(*l) + " is raised only to a whole power");
                }
                // Whole and small, and the measure it makes bounded before
                // it is an int.
                const double length = static_cast<double>(l->length) * n;
                const double angle = static_cast<double>(l->angle) * n;
                if (std::abs(length) > kMaxMeasure || std::abs(angle) > kMaxMeasure) {
                    return fail(why, "it measures more than a value can");
                }
                return Quantity{std::pow(l->value, n), static_cast<int>(length),
                                static_cast<int>(angle)};
            }
        }
        return fail(why, "not an operator");
    }
    if (const auto* call = dynamic_cast<const FunctionCallExpr*>(&e)) {
        std::vector<Quantity> args;
        args.reserve(call->args().size());
        for (const auto& arg : call->args()) {
            const auto q = evaluate(*arg, vars, why);
            if (!q) return std::nullopt;
            args.push_back(*q);
        }
        const std::string& name = call->name();
        const auto isAngle = [](const Quantity& q) { return q.length == 0 && q.angle == 1; };
        if ((name == "sin" || name == "cos" || name == "tan") && args.size() == 1) {
            // Of an angle, or of a plain number in radians.
            if (!args[0].pure() && !isAngle(args[0])) {
                return fail(why, name + " is of an angle, not of " + measureName(args[0]));
            }
            const double x = args[0].value;
            return Quantity{name == "sin"   ? std::sin(x)
                            : name == "cos" ? std::cos(x)
                                            : std::tan(x),
                            0, 0};
        }
        if ((name == "asin" || name == "acos" || name == "atan") && args.size() == 1) {
            if (!args[0].pure()) return fail(why, name + " is of a plain number");
            const double x = args[0].value;
            return Quantity{name == "asin"   ? std::asin(x)
                            : name == "acos" ? std::acos(x)
                                             : std::atan(x),
                            0, 1};
        }
        if (name == "atan2" && args.size() == 2) {
            if (!args[0].sameMeasure(args[1])) return fail(why, "atan2 is of two alike values");
            return Quantity{std::atan2(args[0].value, args[1].value), 0, 1};
        }
        if (name == "sqrt" && args.size() == 1) {
            if (args[0].length % 2 != 0 || args[0].angle % 2 != 0) {
                return fail(why,
                            "the square root of " + measureName(args[0]) + " measures nothing");
            }
            if (args[0].value < 0.0) return fail(why, "the square root of a negative value");
            return Quantity{std::sqrt(args[0].value), args[0].length / 2, args[0].angle / 2};
        }
        if (name == "abs" && args.size() == 1) {
            return Quantity{std::abs(args[0].value), args[0].length, args[0].angle};
        }
        return fail(why, "no function " + name + " takes " + std::to_string(args.size()) +
                             (args.size() == 1 ? " value" : " values"));
    }
    return fail(why, "not an expression");
}

/// @p expression's tree, copied.
std::unique_ptr<Expression> copyOf(const Expression& expression) {
    return Expression::fromJson(expression.toJson());
}

/// The tree rebuilt with each plain number added to a length or an angle
/// given its unit.
class Normalizer {
public:
    Normalizer(const std::map<std::string, Quantity>& vars, LengthUnit unit, std::string* why)
        : m_vars(vars), m_unit(symbolOf(unit)), m_why(why) {}

    std::unique_ptr<Expression> rebuild(const Expression& e) {
        if (const auto* binary = dynamic_cast<const BinaryOpExpr*>(&e)) {
            auto l = rebuild(binary->left());
            auto r = rebuild(binary->right());
            if (!l || !r) return nullptr;
            if (binary->op() == BinaryOpExpr::Op::Add || binary->op() == BinaryOpExpr::Op::Sub) {
                const auto ql = evaluate(*l, m_vars, m_why);
                const auto qr = evaluate(*r, m_vars, m_why);
                if (!ql || !qr) return nullptr;
                if (ql->pure() && !qr->pure()) l = inUnitOf(std::move(l), *qr);
                if (qr->pure() && !ql->pure()) r = inUnitOf(std::move(r), *ql);
                if (!l || !r) return nullptr;
            }
            return std::make_unique<BinaryOpExpr>(binary->op(), std::move(l), std::move(r));
        }
        if (const auto* unary = dynamic_cast<const UnaryOpExpr*>(&e)) {
            auto child = rebuild(unary->child());
            if (!child) return nullptr;
            return std::make_unique<UnaryOpExpr>(unary->op(), std::move(child));
        }
        if (const auto* call = dynamic_cast<const FunctionCallExpr*>(&e)) {
            std::vector<std::unique_ptr<Expression>> args;
            args.reserve(call->args().size());
            for (const auto& arg : call->args()) {
                auto rebuilt = rebuild(*arg);
                if (!rebuilt) return nullptr;
                args.push_back(std::move(rebuilt));
            }
            return std::make_unique<FunctionCallExpr>(call->name(), std::move(args));
        }
        // A literal, a variable, a value already in a unit: as it is.
        return copyOf(e);
    }

    /// Plain @p value given the unit @p other is measured in: the
    /// document's for a length, degrees for an angle.
    std::unique_ptr<Expression> inUnitOf(std::unique_ptr<Expression> value, const Quantity& other) {
        if (other.length == 1 && other.angle == 0) {
            return std::make_unique<UnitExpr>(std::move(value), std::string(m_unit));
        }
        if (other.length == 0 && other.angle == 1) {
            return std::make_unique<UnitExpr>(std::move(value), "deg");
        }
        fail(m_why,
             "cannot add " + value->toString() + " to " + measureName(other) + ": give it a unit");
        return nullptr;
    }

private:
    const std::map<std::string, Quantity>& m_vars;
    std::string_view m_unit;
    std::string* m_why;
};

}  // namespace

std::optional<Quantity> evaluateQuantity(const Expression& expression,
                                         const std::map<std::string, Quantity>& variables,
                                         std::string* why) {
    const auto result = evaluate(expression, variables, why);
    if (result && !std::isfinite(result->value)) return fail(why, "its value is not finite");
    return result;
}

std::unique_ptr<Expression> normalized(const Expression& expression,
                                       const std::map<std::string, Quantity>& variables,
                                       QuantityKind kind, LengthUnit unit, std::string* why) {
    Normalizer normalizer(variables, unit, why);
    auto tree = normalizer.rebuild(expression);
    if (!tree) return nullptr;
    const auto result = evaluateQuantity(*tree, variables, why);
    if (!result) return nullptr;
    switch (kind) {
        case QuantityKind::Length:
            if (result->pure()) {
                return std::make_unique<UnitExpr>(std::move(tree), std::string(symbolOf(unit)));
            }
            if (result->length != 1 || result->angle != 0) {
                fail(why, "it gives " + measureName(*result) + ", not a length");
                return nullptr;
            }
            return tree;
        case QuantityKind::Angle:
            if (result->pure()) return std::make_unique<UnitExpr>(std::move(tree), "deg");
            if (result->length != 0 || result->angle != 1) {
                fail(why, "it gives " + measureName(*result) + ", not an angle");
                return nullptr;
            }
            return tree;
        case QuantityKind::Number:
            if (!result->pure()) {
                fail(why, "it gives " + measureName(*result) + ", not a plain number");
                return nullptr;
            }
            return tree;
    }
    return nullptr;
}

std::string measureName(const Quantity& quantity) {
    if (quantity.pure()) return "a number";
    if (quantity.angle == 0) {
        if (quantity.length == 1) return "a length";
        if (quantity.length == 2) return "an area";
        if (quantity.length == 3) return "a volume";
    }
    if (quantity.length == 0 && quantity.angle == 1) return "an angle";
    return "length^" + std::to_string(quantity.length) + " angle^" + std::to_string(quantity.angle);
}

}  // namespace hz::math
