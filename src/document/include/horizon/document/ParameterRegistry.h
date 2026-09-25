#pragma once

#include <map>
#include <string>

#include "ExpressionEngine.h"
#include "horizon/math/Quantity.h"

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

    // --- Variables and equations (Phase 155) ---

    /// Every variable's definition as it is kept: its expression, or its
    /// value written as a number.
    [[nodiscard]] std::map<std::string, std::string> definitions() const;
    /// Every variable as @p definitions has it, and no other.
    void setDefinitions(const std::map<std::string, std::string>& definitions);

    /// Each variable's value and what it measures ("10 mm" a length, "4" a
    /// number), worked out in the order they depend on each other. One that
    /// cannot be is left out, with why in @p errors by its name; so are
    /// those in a loop.
    [[nodiscard]] std::map<std::string, math::Quantity> quantities(
        std::map<std::string, std::string>* errors = nullptr) const;

    /// Whether @p definitions can be kept: each name a name (a letter or _,
    /// then letters, digits and _; not a unit, a function or pi), each
    /// expression one, no variable depending on itself, and each worked
    /// out. Why not in @p why.
    [[nodiscard]] static bool check(const std::map<std::string, std::string>& definitions,
                                    std::string* why = nullptr);

    /// Whether @p name can name a variable (see check()).
    [[nodiscard]] static bool isName(const std::string& name);

    /// Access the underlying expression engine.
    ExpressionEngine& engine() { return m_engine; }
    const ExpressionEngine& engine() const { return m_engine; }

private:
    ExpressionEngine m_engine;
};

}  // namespace hz::doc
