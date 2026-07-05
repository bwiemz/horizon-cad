#include "horizon/document/ConfigurationTable.h"

#include <algorithm>

#include "horizon/document/ParameterRegistry.h"

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

bool ConfigurationTable::apply(const std::string& name, ParameterRegistry& params) const {
    auto it = m_configs.find(name);
    if (it == m_configs.end()) return false;
    for (const auto& [param, value] : it->second) {
        params.set(param, value);
    }
    return true;
}

bool ConfigurationTable::setActive(const std::string& name) {
    if (!hasConfiguration(name)) return false;
    m_active = name;
    return true;
}

}  // namespace hz::doc
