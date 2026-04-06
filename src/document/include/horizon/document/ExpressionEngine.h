#pragma once

#include "horizon/math/Expression.h"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace hz::doc {

class ExpressionEngine {
public:
    ExpressionEngine() = default;

    void setLiteral(const std::string& name, double value);
    void setExpression(const std::string& name, const std::string& exprStr);

    [[nodiscard]] double getValue(const std::string& name) const;
    [[nodiscard]] std::string getExpression(const std::string& name) const;
    [[nodiscard]] bool has(const std::string& name) const;
    [[nodiscard]] bool isExpression(const std::string& name) const;

    void remove(const std::string& name);
    void clear();

    [[nodiscard]] std::map<std::string, double> allValues() const;
    [[nodiscard]] bool hasCycle() const;
    [[nodiscard]] std::string describeCycle() const;
    [[nodiscard]] std::vector<std::string> evaluationOrder() const;

private:
    struct Variable {
        double value = 0.0;
        std::string expressionStr;
        std::unique_ptr<math::Expression> expression;  // nullptr for literals
    };

    std::map<std::string, Variable> m_variables;

    void reevaluate();
    [[nodiscard]] std::map<std::string, std::set<std::string>> buildDependencyGraph() const;
    [[nodiscard]] std::vector<std::string> topologicalSort(
        const std::map<std::string, std::set<std::string>>& graph) const;
    [[nodiscard]] bool detectCycle(
        const std::map<std::string, std::set<std::string>>& graph,
        std::vector<std::string>& cyclePath) const;
};

}  // namespace hz::doc
