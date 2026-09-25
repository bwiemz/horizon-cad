#pragma once

#include <map>
#include <string>
#include <vector>

namespace hz::doc {

/// A design table: named configurations of one part, each giving some of
/// its variables other values (bolt sizes, plate thicknesses), so one
/// feature tree drives a family of variants (Phase 156).
///
/// A configuration overrides variables by expression ("diameter" = "8 mm").
/// The active one is laid over the document's own variables when the part
/// is built; none active, or a variable it leaves alone, is the document's
/// own. The variables themselves are never written: choosing another
/// configuration, or none, is all it takes to go back.
class ConfigurationTable {
public:
    /// Variable name to the expression it takes in the configuration.
    using Overrides = std::map<std::string, std::string>;

    /// Define or replace the configuration @p name with @p overrides. Keeps
    /// the order in which names were first defined.
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

    /// The overrides of configuration @p name (empty if it does not exist).
    Overrides overrides(const std::string& name) const;

    /// @p definitions, a document's own variables, with configuration
    /// @p name's overrides laid over them (as they are for no such name).
    Overrides overlay(const Overrides& definitions, const std::string& name) const;

    /// The active configuration's name; "" when none is: the variables as
    /// the document has them.
    const std::string& active() const { return m_active; }

    /// Make @p name the active configuration, or "" none. Returns false
    /// (leaving it unchanged) for a name that is not a configuration.
    bool setActive(const std::string& name);

    bool operator==(const ConfigurationTable& other) const = default;

private:
    std::vector<std::string> m_order;
    std::map<std::string, Overrides> m_configs;
    std::string m_active;
};

}  // namespace hz::doc
