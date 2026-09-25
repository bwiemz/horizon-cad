#include "horizon/document/ConfigurationTable.h"

#include <algorithm>

namespace hz::doc {

void ConfigurationTable::setConfiguration(const std::string& name, const Overrides& overrides) {
    if (m_configs.find(name) == m_configs.end()) {
        m_order.push_back(name);
    }
    m_configs[name] = overrides;
}

bool ConfigurationTable::removeConfiguration(const std::string& name) {
    auto it = m_configs.find(name);
    if (it == m_configs.end()) return false;
    m_configs.erase(it);
    m_order.erase(std::remove(m_order.begin(), m_order.end(), name), m_order.end());
    if (m_active == name) m_active.clear();
    return true;
}

bool ConfigurationTable::hasConfiguration(const std::string& name) const {
    return m_configs.find(name) != m_configs.end();
}

ConfigurationTable::Overrides ConfigurationTable::overrides(const std::string& name) const {
    auto it = m_configs.find(name);
    return it == m_configs.end() ? Overrides{} : it->second;
}

ConfigurationTable::Overrides ConfigurationTable::overlay(const Overrides& definitions,
                                                          const std::string& name) const {
    Overrides out = definitions;
    const auto it = m_configs.find(name);
    if (it == m_configs.end()) return out;
    for (const auto& [variable, expression] : it->second) out[variable] = expression;
    return out;
}

bool ConfigurationTable::setActive(const std::string& name) {
    if (!name.empty() && !hasConfiguration(name)) return false;
    m_active = name;
    return true;
}

}  // namespace hz::doc
