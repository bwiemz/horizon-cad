#include "horizon/document/ParameterRegistry.h"

namespace hz::doc {

void ParameterRegistry::set(const std::string& name, double value) {
    m_engine.setLiteral(name, value);
}

void ParameterRegistry::setExpression(const std::string& name, const std::string& expr) {
    m_engine.setExpression(name, expr);
}

double ParameterRegistry::get(const std::string& name) const { return m_engine.getValue(name); }

std::string ParameterRegistry::getExpression(const std::string& name) const {
    return m_engine.getExpression(name);
}

bool ParameterRegistry::has(const std::string& name) const { return m_engine.has(name); }

bool ParameterRegistry::isExpression(const std::string& name) const {
    return m_engine.isExpression(name);
}

void ParameterRegistry::remove(const std::string& name) { m_engine.remove(name); }

std::map<std::string, double> ParameterRegistry::all() const { return m_engine.allValues(); }

void ParameterRegistry::clear() { m_engine.clear(); }

}  // namespace hz::doc
