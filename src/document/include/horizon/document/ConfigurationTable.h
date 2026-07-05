#pragma once

#include <map>
#include <string>
#include <vector>

namespace hz::doc {

class ParameterRegistry;

/// A design table: named configurations that each override a set of document
/// parameters, so one feature tree can drive a whole family of part variants
/// (e.g. bolt sizes, plate thicknesses). Applying a configuration writes its
/// overrides into the document's ParameterRegistry before a rebuild.
class ConfigurationTable {
public:
    using Overrides = std::map<std::string, double>;

    /// Define or replace the configuration @p name with the given parameter
    /// @p overrides. Keeps insertion order for new names.
    void setConfiguration(const std::string& name, const Overrides& overrides);

    /// Remove configuration @p name. Returns true if it existed. Clears the
    /// active configuration if it was the one removed.
    bool removeConfiguration(const std::string& name);

    /// Whether a configuration with @p name exists.
    bool hasConfiguration(const std::string& name) const;

    /// Configuration names in the order they were first defined.
    const std::vector<std::string>& configurationNames() const { return m_order; }

    /// Number of configurations.
    std::size_t size() const { return m_order.size(); }

    /// The parameter overrides of configuration @p name (empty if it does not
    /// exist).
    Overrides overrides(const std::string& name) const;

    /// Write configuration @p name's overrides into @p params (via set()).
    /// Returns false if the configuration does not exist.
    bool apply(const std::string& name, ParameterRegistry& params) const;

    /// The active configuration name ("" if none).
    const std::string& active() const { return m_active; }

    /// Set the active configuration. Returns false (leaving the active
    /// configuration unchanged) if @p name does not exist.
    bool setActive(const std::string& name);

private:
    std::vector<std::string> m_order;
    std::map<std::string, Overrides> m_configs;
    std::string m_active;
};

}  // namespace hz::doc
