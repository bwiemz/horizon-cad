#include "horizon/document/ExpressionEngine.h"

#include <algorithm>
#include <queue>
#include <sstream>

namespace hz::doc {

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void ExpressionEngine::setLiteral(const std::string& name, double value) {
    auto& var = m_variables[name];
    var.value = value;
    var.expressionStr.clear();
    var.expression.reset();
    reevaluate();
}

void ExpressionEngine::setExpression(const std::string& name,
                                     const std::string& exprStr) {
    auto parsed = math::Expression::parse(exprStr);
    if (!parsed) {
        return;  // parse failure -- do nothing
    }
    auto& var = m_variables[name];
    var.expressionStr = exprStr;
    var.expression = std::move(parsed);
    // value will be filled in by reevaluate()
    reevaluate();
}

double ExpressionEngine::getValue(const std::string& name) const {
    auto it = m_variables.find(name);
    if (it == m_variables.end()) {
        return 0.0;
    }
    return it->second.value;
}

std::string ExpressionEngine::getExpression(const std::string& name) const {
    auto it = m_variables.find(name);
    if (it == m_variables.end()) {
        return {};
    }
    return it->second.expressionStr;
}

bool ExpressionEngine::has(const std::string& name) const {
    return m_variables.find(name) != m_variables.end();
}

bool ExpressionEngine::isExpression(const std::string& name) const {
    auto it = m_variables.find(name);
    if (it == m_variables.end()) {
        return false;
    }
    return it->second.expression != nullptr;
}

void ExpressionEngine::remove(const std::string& name) {
    m_variables.erase(name);
    reevaluate();
}

void ExpressionEngine::clear() {
    m_variables.clear();
}

std::map<std::string, double> ExpressionEngine::allValues() const {
    std::map<std::string, double> result;
    for (const auto& [name, var] : m_variables) {
        result[name] = var.value;
    }
    return result;
}

bool ExpressionEngine::hasCycle() const {
    auto graph = buildDependencyGraph();
    auto sorted = topologicalSort(graph);
    return sorted.size() < m_variables.size();
}

std::string ExpressionEngine::describeCycle() const {
    auto graph = buildDependencyGraph();
    std::vector<std::string> cyclePath;
    if (!detectCycle(graph, cyclePath)) {
        return {};
    }

    std::ostringstream oss;
    for (size_t i = 0; i < cyclePath.size(); ++i) {
        if (i > 0) {
            oss << " -> ";
        }
        oss << cyclePath[i];
    }
    return oss.str();
}

std::vector<std::string> ExpressionEngine::evaluationOrder() const {
    auto graph = buildDependencyGraph();
    auto sorted = topologicalSort(graph);
    if (sorted.size() < m_variables.size()) {
        return {};  // cycle -- no valid order
    }
    return sorted;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void ExpressionEngine::reevaluate() {
    auto graph = buildDependencyGraph();
    auto order = topologicalSort(graph);

    // If there is a cycle, leave values stale
    if (order.size() < m_variables.size()) {
        return;
    }

    // Build a mutable bindings map for evaluation
    std::map<std::string, double> bindings;
    for (const auto& [name, var] : m_variables) {
        bindings[name] = var.value;
    }

    // Evaluate in topological order
    for (const auto& name : order) {
        auto it = m_variables.find(name);
        if (it == m_variables.end()) {
            continue;
        }
        auto& var = it->second;
        if (var.expression) {
            var.value = var.expression->evaluate(bindings);
            bindings[name] = var.value;
        }
    }
}

std::map<std::string, std::set<std::string>>
ExpressionEngine::buildDependencyGraph() const {
    std::map<std::string, std::set<std::string>> graph;

    // Ensure every variable has an entry (even if no dependencies)
    for (const auto& [name, var] : m_variables) {
        graph[name];  // default-insert empty set
        if (var.expression) {
            auto deps = var.expression->variables();
            for (const auto& dep : deps) {
                // Only include dependencies on known variables
                if (m_variables.find(dep) != m_variables.end()) {
                    graph[name].insert(dep);
                }
            }
        }
    }

    return graph;
}

std::vector<std::string> ExpressionEngine::topologicalSort(
    const std::map<std::string, std::set<std::string>>& graph) const {
    // Kahn's algorithm
    // graph: node -> set of nodes it depends on (predecessors)
    // We need to invert this to get successors for in-degree computation.

    // Compute in-degree: how many nodes depend on each node?
    // Actually for Kahn's, in-degree = number of prerequisites a node has.
    std::map<std::string, int> inDegree;
    // Reverse graph: for each dependency edge A->B (A depends on B),
    // record B -> {A} so when B is processed, A's in-degree is decremented.
    std::map<std::string, std::vector<std::string>> reverseGraph;

    for (const auto& [node, deps] : graph) {
        if (inDegree.find(node) == inDegree.end()) {
            inDegree[node] = 0;
        }
        for (const auto& dep : deps) {
            if (inDegree.find(dep) == inDegree.end()) {
                inDegree[dep] = 0;
            }
        }
    }

    for (const auto& [node, deps] : graph) {
        inDegree[node] += static_cast<int>(deps.size());
        for (const auto& dep : deps) {
            reverseGraph[dep].push_back(node);
        }
    }

    std::queue<std::string> ready;
    for (const auto& [node, degree] : inDegree) {
        if (degree == 0) {
            ready.push(node);
        }
    }

    std::vector<std::string> result;
    while (!ready.empty()) {
        auto current = ready.front();
        ready.pop();
        result.push_back(current);

        for (const auto& dependent : reverseGraph[current]) {
            inDegree[dependent]--;
            if (inDegree[dependent] == 0) {
                ready.push(dependent);
            }
        }
    }

    return result;
}

bool ExpressionEngine::detectCycle(
    const std::map<std::string, std::set<std::string>>& graph,
    std::vector<std::string>& cyclePath) const {
    // DFS-based cycle detection
    enum class State { Unvisited, InProgress, Done };
    std::map<std::string, State> state;
    std::map<std::string, std::string> parent;

    for (const auto& [node, _] : graph) {
        state[node] = State::Unvisited;
    }

    // Iterative DFS with explicit stack
    for (const auto& [startNode, _] : graph) {
        if (state[startNode] != State::Unvisited) {
            continue;
        }

        // Stack of (node, iterator-index into deps)
        struct Frame {
            std::string node;
            std::vector<std::string> deps;
            size_t idx = 0;
        };
        std::vector<Frame> stack;
        state[startNode] = State::InProgress;
        Frame startFrame;
        startFrame.node = startNode;
        if (graph.find(startNode) != graph.end()) {
            const auto& s = graph.at(startNode);
            startFrame.deps.assign(s.begin(), s.end());
        }
        stack.push_back(std::move(startFrame));

        while (!stack.empty()) {
            auto& frame = stack.back();
            if (frame.idx < frame.deps.size()) {
                const auto& dep = frame.deps[frame.idx];
                frame.idx++;

                if (state.find(dep) == state.end()) {
                    continue;  // dependency on unknown variable
                }

                if (state[dep] == State::InProgress) {
                    // Found a cycle! Reconstruct it.
                    cyclePath.clear();
                    cyclePath.push_back(dep);
                    for (auto rit = stack.rbegin(); rit != stack.rend(); ++rit) {
                        cyclePath.push_back(rit->node);
                        if (rit->node == dep) {
                            break;
                        }
                    }
                    std::reverse(cyclePath.begin(), cyclePath.end());
                    return true;
                }

                if (state[dep] == State::Unvisited) {
                    state[dep] = State::InProgress;
                    parent[dep] = frame.node;
                    Frame newFrame;
                    newFrame.node = dep;
                    if (graph.find(dep) != graph.end()) {
                        const auto& s = graph.at(dep);
                        newFrame.deps.assign(s.begin(), s.end());
                    }
                    stack.push_back(std::move(newFrame));
                }
            } else {
                state[frame.node] = State::Done;
                stack.pop_back();
            }
        }
    }

    return false;
}

}  // namespace hz::doc
