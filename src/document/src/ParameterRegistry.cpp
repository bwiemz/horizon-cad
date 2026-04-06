#include "horizon/document/ParameterRegistry.h"

namespace hz::doc {

void ParameterRegistry::set(const std::string& name, double value) { m_variables[name] = value; }

double ParameterRegistry::get(const std::string& name) const {
    auto it = m_variables.find(name);
    return (it != m_variables.end()) ? it->second : 0.0;
}

bool ParameterRegistry::has(const std::string& name) const { return m_variables.count(name) > 0; }

void ParameterRegistry::remove(const std::string& name) { m_variables.erase(name); }

const std::map<std::string, double>& ParameterRegistry::all() const { return m_variables; }

void ParameterRegistry::clear() { m_variables.clear(); }

}  // namespace hz::doc
