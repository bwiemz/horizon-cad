#include "horizon/math/Expression.h"

#include <cctype>
#include <cmath>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

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
    std::ostringstream ss;
    ss << std::setprecision(17) << m_value;
    return ss.str();
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
        case Op::Add: return l + r;
        case Op::Sub: return l - r;
        case Op::Mul: return l * r;
        case Op::Div: return l / r;
        case Op::Pow: return std::pow(l, r);
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
        case Op::Add: opChar = '+'; break;
        case Op::Sub: opChar = '-'; break;
        case Op::Mul: opChar = '*'; break;
        case Op::Div: opChar = '/'; break;
        case Op::Pow: opChar = '^'; break;
    }
    return "(" + m_left->toString() + " " + opChar + " " + m_right->toString() + ")";
}

nlohmann::json BinaryOpExpr::toJson() const {
    std::string opStr;
    switch (m_op) {
        case Op::Add: opStr = "+"; break;
        case Op::Sub: opStr = "-"; break;
        case Op::Mul: opStr = "*"; break;
        case Op::Div: opStr = "/"; break;
        case Op::Pow: opStr = "^"; break;
    }
    return {{"type", "binary"}, {"op", opStr}, {"left", m_left->toJson()}, {"right", m_right->toJson()}};
}

// --- UnaryOpExpr -----------------------------------------------------------

double UnaryOpExpr::evaluate(const std::map<std::string, double>& variables) const {
    double val = m_child->evaluate(variables);
    switch (m_op) {
        case Op::Negate: return -val;
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
        case Op::Negate: opStr = "-"; break;
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

// --- Expression::fromJson (static) -----------------------------------------

std::unique_ptr<Expression> Expression::fromJson(const nlohmann::json& j) {
    if (!j.is_object() || !j.contains("type")) {
        return nullptr;
    }

    const std::string type = j.at("type").get<std::string>();

    if (type == "literal") {
        if (!j.contains("value")) return nullptr;
        return std::make_unique<LiteralExpr>(j.at("value").get<double>());
    }

    if (type == "variable") {
        if (!j.contains("name")) return nullptr;
        return std::make_unique<VariableExpr>(j.at("name").get<std::string>());
    }

    if (type == "binary") {
        if (!j.contains("op") || !j.contains("left") || !j.contains("right")) return nullptr;
        const std::string opStr = j.at("op").get<std::string>();
        BinaryOpExpr::Op op;
        if (opStr == "+") op = BinaryOpExpr::Op::Add;
        else if (opStr == "-") op = BinaryOpExpr::Op::Sub;
        else if (opStr == "*") op = BinaryOpExpr::Op::Mul;
        else if (opStr == "/") op = BinaryOpExpr::Op::Div;
        else if (opStr == "^") op = BinaryOpExpr::Op::Pow;
        else return nullptr;

        auto left = fromJson(j.at("left"));
        auto right = fromJson(j.at("right"));
        if (!left || !right) return nullptr;
        return std::make_unique<BinaryOpExpr>(op, std::move(left), std::move(right));
    }

    if (type == "unary") {
        if (!j.contains("op") || !j.contains("child")) return nullptr;
        const std::string opStr = j.at("op").get<std::string>();
        UnaryOpExpr::Op op;
        if (opStr == "-") op = UnaryOpExpr::Op::Negate;
        else return nullptr;

        auto child = fromJson(j.at("child"));
        if (!child) return nullptr;
        return std::make_unique<UnaryOpExpr>(op, std::move(child));
    }

    if (type == "function") {
        if (!j.contains("name") || !j.contains("args")) return nullptr;
        const std::string name = j.at("name").get<std::string>();
        const auto& argsJson = j.at("args");
        if (!argsJson.is_array()) return nullptr;

        std::vector<std::unique_ptr<Expression>> args;
        for (const auto& argJson : argsJson) {
            auto arg = fromJson(argJson);
            if (!arg) return nullptr;
            args.push_back(std::move(arg));
        }
        return std::make_unique<FunctionCallExpr>(name, std::move(args));
    }

    return nullptr;  // unknown type
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
            case '+': ++m_pos; return {TokenType::Plus, "+", 0.0};
            case '-': ++m_pos; return {TokenType::Minus, "-", 0.0};
            case '*': ++m_pos; return {TokenType::Star, "*", 0.0};
            case '/': ++m_pos; return {TokenType::Slash, "/", 0.0};
            case '^': ++m_pos; return {TokenType::Caret, "^", 0.0};
            case '(': ++m_pos; return {TokenType::LParen, "(", 0.0};
            case ')': ++m_pos; return {TokenType::RParen, ")", 0.0};
            case ',': ++m_pos; return {TokenType::Comma, ",", 0.0};
            default: break;
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
        while (m_pos < m_input.size() &&
               std::isspace(static_cast<unsigned char>(m_input[m_pos]))) {
            ++m_pos;
        }
    }

    Token readNumber() {
        size_t start = m_pos;
        while (m_pos < m_input.size() &&
               std::isdigit(static_cast<unsigned char>(m_input[m_pos]))) {
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
        double val = 0.0;
        try {
            val = std::stod(text);
        } catch (...) {
            return {TokenType::Error, text, 0.0};
        }
        return {TokenType::Number, text, val};
    }

    Token readIdentifier() {
        size_t start = m_pos;
        while (m_pos < m_input.size() &&
               (std::isalnum(static_cast<unsigned char>(m_input[m_pos])) ||
                m_input[m_pos] == '_')) {
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
    explicit Parser(const std::string& input) : m_tokenizer(input), m_hasError(false) {
        advance();
    }

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
            auto op = (m_current.type == TokenType::Plus) ? BinaryOpExpr::Op::Add
                                                          : BinaryOpExpr::Op::Sub;
            advance();
            auto right = parseMulDiv();
            if (m_hasError || !right) return nullptr;
            left = std::make_unique<BinaryOpExpr>(op, std::move(left), std::move(right));
        }
        return left;
    }

    // term = power (('*' | '/') power)*
    std::unique_ptr<Expression> parseMulDiv() {
        auto left = parsePower();
        if (m_hasError || !left) return nullptr;

        while (m_current.type == TokenType::Star || m_current.type == TokenType::Slash) {
            auto op = (m_current.type == TokenType::Star) ? BinaryOpExpr::Op::Mul
                                                          : BinaryOpExpr::Op::Div;
            advance();
            auto right = parsePower();
            if (m_hasError || !right) return nullptr;
            left = std::make_unique<BinaryOpExpr>(op, std::move(left), std::move(right));
        }
        return left;
    }

    // power = unary ('^' power)?   // right-associative
    std::unique_ptr<Expression> parsePower() {
        auto base = parseUnary();
        if (m_hasError || !base) return nullptr;

        if (m_current.type == TokenType::Caret) {
            advance();
            auto exponent = parsePower();  // right-associative: recurse into parsePower
            if (m_hasError || !exponent) return nullptr;
            return std::make_unique<BinaryOpExpr>(BinaryOpExpr::Op::Pow, std::move(base),
                                                  std::move(exponent));
        }
        return base;
    }

    // unary = '-' unary | primary
    std::unique_ptr<Expression> parseUnary() {
        if (m_current.type == TokenType::Minus) {
            advance();
            auto child = parseUnary();
            if (m_hasError || !child) return nullptr;
            return std::make_unique<UnaryOpExpr>(UnaryOpExpr::Op::Negate, std::move(child));
        }
        return parsePrimary();
    }

    // primary = NUMBER | IDENTIFIER '(' args ')' | IDENTIFIER | '(' expression ')'
    std::unique_ptr<Expression> parsePrimary() {
        if (m_hasError) return nullptr;

        // NUMBER
        if (m_current.type == TokenType::Number) {
            double val = m_current.numValue;
            advance();
            return std::make_unique<LiteralExpr>(val);
        }

        // IDENTIFIER (variable, constant, or function call)
        if (m_current.type == TokenType::Identifier) {
            std::string name = m_current.text;
            advance();

            // Check for function call: IDENTIFIER '(' args ')'
            if (m_current.type == TokenType::LParen) {
                advance();
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
                return std::make_unique<FunctionCallExpr>(name, std::move(args));
            }

            // Built-in constant: pi
            if (name == "pi") {
                return std::make_unique<LiteralExpr>(3.14159265358979323846);
            }

            // Variable reference
            return std::make_unique<VariableExpr>(name);
        }

        // '(' expression ')'
        if (m_current.type == TokenType::LParen) {
            advance();
            auto inner = parseAddSub();
            if (m_hasError || !inner) return nullptr;
            if (!expect(TokenType::RParen)) return nullptr;
            return inner;
        }

        // Unexpected token
        m_hasError = true;
        return nullptr;
    }

    Tokenizer m_tokenizer;
    Token m_current;
    bool m_hasError;
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
