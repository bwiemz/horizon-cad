#include "horizon/math/Expression.h"

#include <cctype>
#include <cmath>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include "horizon/math/CharConv.h"

namespace hz::math {

// ===========================================================================
// Node evaluation / variables / toString implementations
// ===========================================================================

// --- LiteralExpr -----------------------------------------------------------

double LiteralExpr::evaluate(const std::map<std::string, double>& /*variables*/) const {
    return m_value;
}

std::set<std::string> LiteralExpr::variables() const {
    return {};
}

std::string LiteralExpr::toString() const {
    // Plain decimals, as the tokenizer reads them: the value exactly, never an
    // exponent ("5.0000000000000002e-05" did not read back), never a comma.
    // A value that is not finite is written as an expression that makes it.
    if (std::isnan(m_value)) return "(0 / 0)";
    if (std::isinf(m_value)) return m_value > 0.0 ? "(1 / 0)" : "(-(1 / 0))";
    if (std::signbit(m_value)) return "(-" + toDecimalString(-m_value) + ")";
    return toDecimalString(m_value);
}

nlohmann::json LiteralExpr::toJson() const {
    return {{"type", "literal"}, {"value", m_value}};
}

// --- VariableExpr ----------------------------------------------------------

double VariableExpr::evaluate(const std::map<std::string, double>& variables) const {
    auto it = variables.find(m_name);
    if (it != variables.end()) {
        return it->second;
    }
    return 0.0;
}

std::set<std::string> VariableExpr::variables() const {
    return {m_name};
}

std::string VariableExpr::toString() const {
    return m_name;
}

nlohmann::json VariableExpr::toJson() const {
    return {{"type", "variable"}, {"name", m_name}};
}

// --- BinaryOpExpr ----------------------------------------------------------

double BinaryOpExpr::evaluate(const std::map<std::string, double>& variables) const {
    double l = m_left->evaluate(variables);
    double r = m_right->evaluate(variables);
    switch (m_op) {
        case Op::Add:
            return l + r;
        case Op::Sub:
            return l - r;
        case Op::Mul:
            return l * r;
        case Op::Div:
            return l / r;
        case Op::Pow:
            return std::pow(l, r);
    }
    return 0.0;  // unreachable
}

std::set<std::string> BinaryOpExpr::variables() const {
    auto result = m_left->variables();
    auto rhs = m_right->variables();
    result.insert(rhs.begin(), rhs.end());
    return result;
}

std::string BinaryOpExpr::toString() const {
    char opChar = '+';
    switch (m_op) {
        case Op::Add:
            opChar = '+';
            break;
        case Op::Sub:
            opChar = '-';
            break;
        case Op::Mul:
            opChar = '*';
            break;
        case Op::Div:
            opChar = '/';
            break;
        case Op::Pow:
            opChar = '^';
            break;
    }
    // A chain of one precedence ("a + b - c", "a * b / c") is written as one
    // bracket, as parse() reads it back: each link in brackets of its own
    // nested the chain one bracket deeper per term, and past
    // kMaxNestingDepth terms the text no longer parsed.
    std::string left = m_left->toString();
    const auto* chained = dynamic_cast<const BinaryOpExpr*>(m_left.get());
    if (chained != nullptr && m_op != Op::Pow && precedence(chained->m_op) == precedence(m_op)) {
        left = left.substr(1, left.size() - 2);  // its own brackets
    }
    return "(" + left + " " + opChar + " " + m_right->toString() + ")";
}

int BinaryOpExpr::precedence(Op op) {
    switch (op) {
        case Op::Add:
        case Op::Sub:
            return 1;
        case Op::Mul:
        case Op::Div:
            return 2;
        case Op::Pow:
            return 3;
    }
    return 0;  // unreachable
}

nlohmann::json BinaryOpExpr::toJson() const {
    std::string opStr;
    switch (m_op) {
        case Op::Add:
            opStr = "+";
            break;
        case Op::Sub:
            opStr = "-";
            break;
        case Op::Mul:
            opStr = "*";
            break;
        case Op::Div:
            opStr = "/";
            break;
        case Op::Pow:
            opStr = "^";
            break;
    }
    return {{"type", "binary"},
            {"op", opStr},
            {"left", m_left->toJson()},
            {"right", m_right->toJson()}};
}

// --- UnaryOpExpr -----------------------------------------------------------

double UnaryOpExpr::evaluate(const std::map<std::string, double>& variables) const {
    double val = m_child->evaluate(variables);
    switch (m_op) {
        case Op::Negate:
            return -val;
    }
    return 0.0;  // unreachable
}

std::set<std::string> UnaryOpExpr::variables() const {
    return m_child->variables();
}

std::string UnaryOpExpr::toString() const {
    return "(-" + m_child->toString() + ")";
}

nlohmann::json UnaryOpExpr::toJson() const {
    std::string opStr;
    switch (m_op) {
        case Op::Negate:
            opStr = "-";
            break;
    }
    return {{"type", "unary"}, {"op", opStr}, {"child", m_child->toJson()}};
}

// --- FunctionCallExpr ------------------------------------------------------

double FunctionCallExpr::evaluate(const std::map<std::string, double>& variables) const {
    std::vector<double> vals;
    vals.reserve(m_args.size());
    for (const auto& arg : m_args) {
        vals.push_back(arg->evaluate(variables));
    }

    if (m_name == "sin" && vals.size() == 1) return std::sin(vals[0]);
    if (m_name == "cos" && vals.size() == 1) return std::cos(vals[0]);
    if (m_name == "tan" && vals.size() == 1) return std::tan(vals[0]);
    if (m_name == "sqrt" && vals.size() == 1) return std::sqrt(vals[0]);
    if (m_name == "abs" && vals.size() == 1) return std::abs(vals[0]);
    if (m_name == "asin" && vals.size() == 1) return std::asin(vals[0]);
    if (m_name == "acos" && vals.size() == 1) return std::acos(vals[0]);
    if (m_name == "atan" && vals.size() == 1) return std::atan(vals[0]);
    if (m_name == "atan2" && vals.size() == 2) return std::atan2(vals[0], vals[1]);

    return 0.0;  // unknown function
}

std::set<std::string> FunctionCallExpr::variables() const {
    std::set<std::string> result;
    for (const auto& arg : m_args) {
        auto argVars = arg->variables();
        result.insert(argVars.begin(), argVars.end());
    }
    return result;
}

std::string FunctionCallExpr::toString() const {
    std::string result = m_name + "(";
    for (size_t i = 0; i < m_args.size(); ++i) {
        if (i > 0) result += ", ";
        result += m_args[i]->toString();
    }
    result += ")";
    return result;
}

nlohmann::json FunctionCallExpr::toJson() const {
    nlohmann::json argsArray = nlohmann::json::array();
    for (const auto& arg : m_args) {
        argsArray.push_back(arg->toJson());
    }
    return {{"type", "function"}, {"name", m_name}, {"args", argsArray}};
}

// --- UnitExpr --------------------------------------------------------------

std::optional<double> UnitExpr::factor(std::string_view unit) {
    if (unit == "mm") return 1.0;
    if (unit == "cm") return 10.0;
    if (unit == "m") return 1000.0;
    if (unit == "in") return 25.4;
    if (unit == "ft") return 304.8;
    if (unit == "deg") return 3.14159265358979323846 / 180.0;
    if (unit == "rad") return 1.0;
    return std::nullopt;
}

bool UnitExpr::isAngle(std::string_view unit) {
    return unit == "deg" || unit == "rad";
}

double UnitExpr::evaluate(const std::map<std::string, double>& variables) const {
    return m_child->evaluate(variables) * factor(m_unit).value_or(1.0);
}

std::set<std::string> UnitExpr::variables() const {
    return m_child->variables();
}

std::string UnitExpr::toString() const {
    return "(" + m_child->toString() + " " + m_unit + ")";
}

nlohmann::json UnitExpr::toJson() const {
    return {{"type", "unit"}, {"unit", m_unit}, {"child", m_child->toJson()}};
}

// --- Expression::fromJson (static) -----------------------------------------

namespace {

/// The children @p j names, in order, in @p out; false when @p j is no node
/// fromJson reads (an unknown type, a field missing, an operator or unit it
/// does not know). Nothing is built.
bool childrenOf(const nlohmann::json& j, std::vector<const nlohmann::json*>& out) {
    if (!j.is_object() || !j.contains("type")) return false;
    const std::string type = j.at("type").get<std::string>();
    if (type == "literal") return j.contains("value");
    if (type == "variable") return j.contains("name");
    if (type == "binary") {
        if (!j.contains("op") || !j.contains("left") || !j.contains("right")) return false;
        const std::string op = j.at("op").get<std::string>();
        if (op != "+" && op != "-" && op != "*" && op != "/" && op != "^") return false;
        out = {&j.at("left"), &j.at("right")};
        return true;
    }
    if (type == "unary") {
        if (!j.contains("op") || !j.contains("child")) return false;
        if (j.at("op").get<std::string>() != "-") return false;
        out = {&j.at("child")};
        return true;
    }
    if (type == "function") {
        if (!j.contains("name") || !j.contains("args") || !j.at("args").is_array()) return false;
        for (const auto& arg : j.at("args")) out.push_back(&arg);
        return true;
    }
    if (type == "unit") {
        if (!j.contains("unit") || !j.contains("child")) return false;
        if (!UnitExpr::factor(j.at("unit").get<std::string>())) return false;
        out = {&j.at("child")};
        return true;
    }
    return false;  // unknown type
}

/// The node @p j describes, its children (as childrenOf named them) built.
std::unique_ptr<Expression> makeNode(const nlohmann::json& j,
                                     std::vector<std::unique_ptr<Expression>> children) {
    const std::string type = j.at("type").get<std::string>();
    if (type == "literal") return std::make_unique<LiteralExpr>(j.at("value").get<double>());
    if (type == "variable") return std::make_unique<VariableExpr>(j.at("name").get<std::string>());
    if (type == "binary") {
        const std::string op = j.at("op").get<std::string>();
        const BinaryOpExpr::Op kind = op == "+"   ? BinaryOpExpr::Op::Add
                                      : op == "-" ? BinaryOpExpr::Op::Sub
                                      : op == "*" ? BinaryOpExpr::Op::Mul
                                      : op == "/" ? BinaryOpExpr::Op::Div
                                                  : BinaryOpExpr::Op::Pow;
        return std::make_unique<BinaryOpExpr>(kind, std::move(children[0]), std::move(children[1]));
    }
    if (type == "unary") {
        return std::make_unique<UnaryOpExpr>(UnaryOpExpr::Op::Negate, std::move(children[0]));
    }
    if (type == "function") {
        return std::make_unique<FunctionCallExpr>(j.at("name").get<std::string>(),
                                                  std::move(children));
    }
    return std::make_unique<UnitExpr>(std::move(children[0]), j.at("unit").get<std::string>());
}

/// Expression::fromJson, with the tree bounded by Expression::kMaxNodes, as
/// parse() bounds it. Built from an explicit stack, not by recursing: a long
/// chain ("1+1+...+1", which parse() reads) is as deep as it is long, and a
/// file may nest deeper still before the count stops it. Recursing once per
/// level overflowed Windows' 1 MB stack on a debug build; a bound on the depth
/// alone (it was kMaxNestingDepth) refused the copy of a 66-term sum that
/// parse() had read, as parse() counts brackets, minus signs and powers, not
/// every level.
std::unique_ptr<Expression> fromJsonBounded(const nlohmann::json& root) {
    struct Pending {
        const nlohmann::json* node;
        std::vector<const nlohmann::json*> children;
        std::vector<std::unique_ptr<Expression>> built;
    };
    std::vector<Pending> stack;
    int nodes = 0;
    const auto visit = [&](const nlohmann::json& j) {
        if (++nodes > Expression::kMaxNodes) return false;
        Pending pending{&j, {}, {}};
        if (!childrenOf(j, pending.children)) return false;
        stack.push_back(std::move(pending));
        return true;
    };
    if (!visit(root)) return nullptr;
    while (true) {
        Pending& top = stack.back();
        if (top.built.size() < top.children.size()) {
            // Its next child first (visit may move the stack, and `top` with it).
            if (!visit(*top.children[top.built.size()])) return nullptr;
            continue;
        }
        auto made = makeNode(*top.node, std::move(top.built));
        stack.pop_back();
        if (stack.empty()) return made;
        stack.back().built.push_back(std::move(made));
    }
}

}  // namespace

std::unique_ptr<Expression> Expression::fromJson(const nlohmann::json& j) {
    // A wrong-typed field (a number where the operator belongs) makes
    // json::get throw; the contract is nullptr on any error.
    try {
        return fromJsonBounded(j);
    } catch (const nlohmann::json::exception&) {
        return nullptr;
    }
}

// ===========================================================================
// Tokenizer (private to this translation unit)
// ===========================================================================

namespace {

enum class TokenType {
    Number,
    Identifier,
    Plus,
    Minus,
    Star,
    Slash,
    Caret,
    LParen,
    RParen,
    Comma,
    End,
    Error
};

struct Token {
    TokenType type = TokenType::Error;
    std::string text;
    double numValue = 0.0;
};

class Tokenizer {
public:
    explicit Tokenizer(const std::string& input) : m_input(input), m_pos(0) {}

    Token next() {
        skipWhitespace();
        if (m_pos >= m_input.size()) {
            return {TokenType::End, "", 0.0};
        }

        char c = m_input[m_pos];

        // Single-character tokens
        switch (c) {
            case '+':
                ++m_pos;
                return {TokenType::Plus, "+", 0.0};
            case '-':
                ++m_pos;
                return {TokenType::Minus, "-", 0.0};
            case '*':
                ++m_pos;
                return {TokenType::Star, "*", 0.0};
            case '/':
                ++m_pos;
                return {TokenType::Slash, "/", 0.0};
            case '^':
                ++m_pos;
                return {TokenType::Caret, "^", 0.0};
            case '(':
                ++m_pos;
                return {TokenType::LParen, "(", 0.0};
            case ')':
                ++m_pos;
                return {TokenType::RParen, ")", 0.0};
            case ',':
                ++m_pos;
                return {TokenType::Comma, ",", 0.0};
            default:
                break;
        }

        // Number: [0-9]+ ('.' [0-9]+)?
        if (std::isdigit(static_cast<unsigned char>(c)) || c == '.') {
            return readNumber();
        }

        // Identifier: [a-zA-Z_][a-zA-Z_0-9]*
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            return readIdentifier();
        }

        return {TokenType::Error, std::string(1, c), 0.0};
    }

private:
    void skipWhitespace() {
        while (m_pos < m_input.size() && std::isspace(static_cast<unsigned char>(m_input[m_pos]))) {
            ++m_pos;
        }
    }

    Token readNumber() {
        size_t start = m_pos;
        while (m_pos < m_input.size() && std::isdigit(static_cast<unsigned char>(m_input[m_pos]))) {
            ++m_pos;
        }
        if (m_pos < m_input.size() && m_input[m_pos] == '.') {
            ++m_pos;
            while (m_pos < m_input.size() &&
                   std::isdigit(static_cast<unsigned char>(m_input[m_pos]))) {
                ++m_pos;
            }
        }
        std::string text = m_input.substr(start, m_pos - start);
        // In the C locale, whatever the user's: std::stod follows the one Qt
        // sets from the environment, and under de_DE read "1.5" as 1.
        double val = 0.0;
        const char* last = text.data() + text.size();
        const auto [end, error] = fromChars(text.data(), last, val);
        if (error != std::errc() || end != last) return {TokenType::Error, text, 0.0};
        return {TokenType::Number, text, val};
    }

    Token readIdentifier() {
        size_t start = m_pos;
        while (
            m_pos < m_input.size() &&
            (std::isalnum(static_cast<unsigned char>(m_input[m_pos])) || m_input[m_pos] == '_')) {
            ++m_pos;
        }
        std::string text = m_input.substr(start, m_pos - start);
        return {TokenType::Identifier, text, 0.0};
    }

    std::string m_input;
    size_t m_pos;
};

// ===========================================================================
// Recursive Descent Parser (private to this translation unit)
// ===========================================================================

class Parser {
public:
    explicit Parser(const std::string& input) : m_tokenizer(input), m_hasError(false) { advance(); }

    std::unique_ptr<Expression> parseExpression() {
        auto result = parseAddSub();
        if (m_hasError) return nullptr;
        if (m_current.type != TokenType::End) {
            // Unconsumed tokens -- error
            return nullptr;
        }
        return result;
    }

private:
    /// One level of recursion (a bracket, a minus sign, a power, a function's
    /// arguments). Past Expression::kMaxNestingDepth the parse fails.
    class Nesting {
    public:
        explicit Nesting(Parser& parser) : m_parser(parser) {
            if (++m_parser.m_depth > Expression::kMaxNestingDepth) m_parser.m_hasError = true;
        }
        ~Nesting() { --m_parser.m_depth; }
        Nesting(const Nesting&) = delete;
        Nesting& operator=(const Nesting&) = delete;

    private:
        Parser& m_parser;
    };

    /// Build a node, counting it against Expression::kMaxNodes. A long flat
    /// chain ("1+1+...+1") parses in a loop but still builds a tree as deep as
    /// it is long, which evaluation and destruction then recurse through.
    template <typename Node, typename... Args>
    std::unique_ptr<Expression> node(Args&&... args) {
        if (++m_nodes > Expression::kMaxNodes) m_hasError = true;
        return std::make_unique<Node>(std::forward<Args>(args)...);
    }

    void advance() {
        m_current = m_tokenizer.next();
        if (m_current.type == TokenType::Error) {
            m_hasError = true;
        }
    }

    bool expect(TokenType type) {
        if (m_current.type != type) {
            m_hasError = true;
            return false;
        }
        advance();
        return true;
    }

    // expression = term (('+' | '-') term)*
    std::unique_ptr<Expression> parseAddSub() {
        auto left = parseMulDiv();
        if (m_hasError || !left) return nullptr;

        while (m_current.type == TokenType::Plus || m_current.type == TokenType::Minus) {
            auto op =
                (m_current.type == TokenType::Plus) ? BinaryOpExpr::Op::Add : BinaryOpExpr::Op::Sub;
            advance();
            auto right = parseMulDiv();
            if (m_hasError || !right) return nullptr;
            left = node<BinaryOpExpr>(op, std::move(left), std::move(right));
        }
        return left;
    }

    // term = power (('*' | '/') power)*
    std::unique_ptr<Expression> parseMulDiv() {
        auto left = parsePower();
        if (m_hasError || !left) return nullptr;

        while (m_current.type == TokenType::Star || m_current.type == TokenType::Slash) {
            auto op =
                (m_current.type == TokenType::Star) ? BinaryOpExpr::Op::Mul : BinaryOpExpr::Op::Div;
            advance();
            auto right = parsePower();
            if (m_hasError || !right) return nullptr;
            left = node<BinaryOpExpr>(op, std::move(left), std::move(right));
        }
        return left;
    }

    // power = unary ('^' power)?   // right-associative
    std::unique_ptr<Expression> parsePower() {
        auto base = parseUnary();
        if (m_hasError || !base) return nullptr;

        if (m_current.type == TokenType::Caret) {
            advance();
            Nesting nesting(*this);
            if (m_hasError) return nullptr;
            auto exponent = parsePower();  // right-associative: recurse into parsePower
            if (m_hasError || !exponent) return nullptr;
            return node<BinaryOpExpr>(BinaryOpExpr::Op::Pow, std::move(base), std::move(exponent));
        }
        return base;
    }

    // unary = '-' unary | primary
    std::unique_ptr<Expression> parseUnary() {
        if (m_current.type == TokenType::Minus) {
            advance();
            Nesting nesting(*this);
            if (m_hasError) return nullptr;
            auto child = parseUnary();
            if (m_hasError || !child) return nullptr;
            return node<UnaryOpExpr>(UnaryOpExpr::Op::Negate, std::move(child));
        }
        return parsePrimary();
    }

    /// @p value in the unit named next, if one is ("2 in", "(a + b) mm",
    /// "wall in", "sqrt(2) in"); else as it is. Phase 155: a unit word after a
    /// number was an error (nothing multiplies without a sign), so no
    /// expression changes meaning; nor after a name or a call, which
    /// UnitExpr::toString writes for a plain number given the document's unit
    /// ("(count in)"), and which did not read back.
    std::unique_ptr<Expression> withUnit(std::unique_ptr<Expression> value) {
        if (m_hasError || !value || m_current.type != TokenType::Identifier ||
            !UnitExpr::factor(m_current.text)) {
            return value;
        }
        std::string unit = m_current.text;
        advance();
        return node<UnitExpr>(std::move(value), std::move(unit));
    }

    // primary = NUMBER unit? | IDENTIFIER '(' args ')' unit? | IDENTIFIER unit?
    //         | '(' expression ')' unit?
    std::unique_ptr<Expression> parsePrimary() {
        if (m_hasError) return nullptr;

        // NUMBER, and the unit after it ("2 in")
        if (m_current.type == TokenType::Number) {
            double val = m_current.numValue;
            advance();
            return withUnit(node<LiteralExpr>(val));
        }

        // IDENTIFIER (variable, constant, or function call)
        if (m_current.type == TokenType::Identifier) {
            std::string name = m_current.text;
            advance();

            // Check for function call: IDENTIFIER '(' args ')'
            if (m_current.type == TokenType::LParen) {
                advance();
                Nesting nesting(*this);
                if (m_hasError) return nullptr;
                std::vector<std::unique_ptr<Expression>> args;

                // Handle empty arg list (shouldn't happen for our functions, but be safe)
                if (m_current.type != TokenType::RParen) {
                    auto arg = parseAddSub();
                    if (m_hasError || !arg) return nullptr;
                    args.push_back(std::move(arg));

                    while (m_current.type == TokenType::Comma) {
                        advance();
                        arg = parseAddSub();
                        if (m_hasError || !arg) return nullptr;
                        args.push_back(std::move(arg));
                    }
                }

                if (!expect(TokenType::RParen)) return nullptr;
                return withUnit(node<FunctionCallExpr>(name, std::move(args)));
            }

            // Built-in constant: pi
            if (name == "pi") {
                return withUnit(node<LiteralExpr>(3.14159265358979323846));
            }

            // Variable reference
            return withUnit(node<VariableExpr>(name));
        }

        // '(' expression ')'
        if (m_current.type == TokenType::LParen) {
            advance();
            Nesting nesting(*this);
            if (m_hasError) return nullptr;
            auto inner = parseAddSub();
            if (m_hasError || !inner) return nullptr;
            if (!expect(TokenType::RParen)) return nullptr;
            return withUnit(std::move(inner));
        }

        // Unexpected token
        m_hasError = true;
        return nullptr;
    }

    Tokenizer m_tokenizer;
    Token m_current;
    bool m_hasError;
    int m_depth = 0;
    int m_nodes = 0;
};

}  // anonymous namespace

// ===========================================================================
// Expression::parse -- public entry point
// ===========================================================================

std::unique_ptr<Expression> Expression::parse(const std::string& input) {
    if (input.empty()) return nullptr;

    Parser parser(input);
    auto result = parser.parseExpression();
    return result;
}

}  // namespace hz::math
