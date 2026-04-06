#pragma once
#include <map>
#include <string>

namespace hz::doc {

class ParameterRegistry {
public:
    ParameterRegistry() = default;
    void set(const std::string& name, double value);
    [[nodiscard]] double get(const std::string& name) const;
    [[nodiscard]] bool has(const std::string& name) const;
    void remove(const std::string& name);
    [[nodiscard]] const std::map<std::string, double>& all() const;
    void clear();

private:
    std::map<std::string, double> m_variables;
};

}  // namespace hz::doc
