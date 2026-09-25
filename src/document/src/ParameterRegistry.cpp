#include "horizon/document/ParameterRegistry.h"

#include <array>
#include <cctype>
#include <iomanip>
#include <locale>
#include <memory>
#include <sstream>
#include <string_view>

#include "horizon/math/Expression.h"

namespace hz::doc {

void ParameterRegistry::set(const std::string& name, double value) {
    m_engine.setLiteral(name, value);
}

void ParameterRegistry::setExpression(const std::string& name, const std::string& expr) {
    m_engine.setExpression(name, expr);
}

double ParameterRegistry::get(const std::string& name) const {
    return m_engine.getValue(name);
}

std::string ParameterRegistry::getExpression(const std::string& name) const {
    return m_engine.getExpression(name);
}

bool ParameterRegistry::has(const std::string& name) const {
    return m_engine.has(name);
}

bool ParameterRegistry::isExpression(const std::string& name) const {
    return m_engine.isExpression(name);
}

void ParameterRegistry::remove(const std::string& name) {
    m_engine.remove(name);
}

std::map<std::string, double> ParameterRegistry::all() const {
    return m_engine.allValues();
}

void ParameterRegistry::clear() {
    m_engine.clear();
}

std::map<std::string, std::string> ParameterRegistry::definitions() const {
    std::map<std::string, std::string> out;
    for (const auto& [name, value] : m_engine.allValues()) {
        if (m_engine.isExpression(name)) {
            out[name] = m_engine.getExpression(name);
        } else {
            std::ostringstream text;
            text.imbue(std::locale::classic());
            text << std::setprecision(17) << value;
            out[name] = text.str();
        }
    }
    return out;
}

void ParameterRegistry::setDefinitions(const std::map<std::string, std::string>& definitions) {
    m_engine.clear();
    for (const auto& [name, text] : definitions) m_engine.setExpression(name, text);
}

std::map<std::string, math::Quantity> ParameterRegistry::quantities(
    std::map<std::string, std::string>* errors) const {
    std::map<std::string, math::Quantity> out;
    const std::vector<std::string> order = m_engine.evaluationOrder();
    if (order.empty() && !m_engine.allValues().empty()) {
        // A loop: none of them can be worked out in order.
        if (errors != nullptr) {
            for (const auto& [name, value] : m_engine.allValues()) {
                (*errors)[name] = "it depends on itself: " + m_engine.describeCycle();
            }
        }
        return out;
    }
    for (const std::string& name : order) {
        if (!m_engine.isExpression(name)) {
            out[name] = math::Quantity{m_engine.getValue(name), 0, 0};
            continue;
        }
        const auto expression = math::Expression::parse(m_engine.getExpression(name));
        std::string why;
        const auto quantity =
            expression ? math::evaluateQuantity(*expression, out, &why) : std::nullopt;
        if (quantity) {
            out[name] = *quantity;
        } else if (errors != nullptr) {
            (*errors)[name] = expression ? why : "it is not an expression";
        }
    }
    return out;
}

bool ParameterRegistry::isName(const std::string& name) {
    if (name.empty() ||
        (std::isalpha(static_cast<unsigned char>(name[0])) == 0 && name[0] != '_')) {
        return false;
    }
    for (const char c : name) {
        if (std::isalnum(static_cast<unsigned char>(c)) == 0 && c != '_') return false;
    }
    static constexpr std::array<std::string_view, 10> kFunctions{
        "sin", "cos", "tan", "asin", "acos", "atan", "atan2", "sqrt", "abs", "pi"};
    for (const std::string_view reserved : kFunctions) {
        if (name == reserved) return false;
    }
    return !math::UnitExpr::factor(name).has_value();
}

bool ParameterRegistry::check(const std::map<std::string, std::string>& definitions,
                              std::string* why) {
    const auto fail = [why](std::string text) {
        if (why != nullptr) *why = std::move(text);
        return false;
    };
    for (const auto& [name, text] : definitions) {
        if (!isName(name)) {
            return fail("\"" + name +
                        "\" is not a name a variable can have: a letter or _, then letters, digits "
                        "and _, and not a unit, a function or pi");
        }
        if (!math::Expression::parse(text)) {
            return fail(name + ": \"" + text + "\" is not an expression");
        }
    }
    ParameterRegistry trial;
    trial.setDefinitions(definitions);
    if (trial.m_engine.hasCycle()) {
        return fail("the variables depend on each other in a loop: " +
                    trial.m_engine.describeCycle());
    }
    std::map<std::string, std::string> errors;
    (void)trial.quantities(&errors);
    if (!errors.empty()) return fail(errors.begin()->first + ": " + errors.begin()->second);
    return true;
}

}  // namespace hz::doc
