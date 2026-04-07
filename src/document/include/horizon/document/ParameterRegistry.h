#pragma once

#include "ExpressionEngine.h"

#include <map>
#include <string>

namespace hz::doc {

class ParameterRegistry {
public:
    ParameterRegistry() = default;

    /// Set a literal (non-expression) value (backward compatible).
    void set(const std::string& name, double value);

    /// Set an expression-driven value.
    void setExpression(const std::string& name, const std::string& expressionStr);

    [[nodiscard]] double get(const std::string& name) const;
    [[nodiscard]] std::string getExpression(const std::string& name) const;
    [[nodiscard]] bool has(const std::string& name) const;
    [[nodiscard]] bool isExpression(const std::string& name) const;
    void remove(const std::string& name);
    [[nodiscard]] std::map<std::string, double> all() const;
    void clear();

    /// Access the underlying expression engine.
    ExpressionEngine& engine() { return m_engine; }
    const ExpressionEngine& engine() const { return m_engine; }

private:
    ExpressionEngine m_engine;
};

}  // namespace hz::doc
