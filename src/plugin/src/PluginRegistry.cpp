#include "horizon/plugin/PluginRegistry.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace hz::plugin {

namespace fs = std::filesystem;
using nlohmann::json;

namespace {

bool fail(std::string* error, const std::string& message) {
    if (error) *error = message;
    return false;
}

bool validName(const std::string& name) {
    if (name.size() < 3 || name.size() > 64) return false;
    if (!std::islower(static_cast<unsigned char>(name.front()))) return false;
    return std::all_of(name.begin(), name.end(), [](unsigned char ch) {
        return std::islower(ch) || std::isdigit(ch) || ch == '-';
    });
}

std::optional<Permission> permissionFromString(const std::string& text) {
    if (text == "filesystem") return Permission::Filesystem;
    if (text == "network") return Permission::Network;
    if (text == "ui") return Permission::UI;
    if (text == "document") return Permission::Document;
    if (text == "simulation") return Permission::Simulation;
    return std::nullopt;
}

/// The entry script must resolve to a regular file strictly inside the
/// plugin directory: relative, no absolute paths, no `..` escapes.
bool validEntry(const std::string& entry, const fs::path& pluginDir, std::string* error) {
    if (entry.empty()) return fail(error, "entry is required");
    const fs::path rel(entry);
    if (rel.is_absolute()) return fail(error, "entry must be a relative path");
    for (const auto& part : rel) {
        if (part == "..") return fail(error, "entry must not escape the plugin directory");
    }
    std::error_code ec;
    const fs::path full = pluginDir / rel;
    if (!fs::is_regular_file(full, ec)) return fail(error, "entry script not found: " + entry);
    return true;
}

}  // namespace

std::optional<std::array<int, 3>> PluginRegistry::parseSemver(const std::string& text) {
    std::array<int, 3> parts{};
    size_t pos = 0;
    for (int i = 0; i < 3; ++i) {
        if (pos >= text.size() || !std::isdigit(static_cast<unsigned char>(text[pos]))) {
            return std::nullopt;
        }
        size_t used = 0;
        long value = 0;
        try {
            value = std::stol(text.substr(pos), &used);
        } catch (const std::exception&) {
            return std::nullopt;
        }
        if (value < 0 || value > 1000000) return std::nullopt;
        parts[i] = static_cast<int>(value);
        pos += used;
        if (i < 2) {
            if (pos >= text.size() || text[pos] != '.') return std::nullopt;
            ++pos;
        }
    }
    if (pos != text.size()) return std::nullopt;  // no suffixes in this slice
    return parts;
}

std::optional<PluginManifest> PluginRegistry::parseManifest(const std::string& jsonText,
                                                            const fs::path& pluginDir,
                                                            std::string* error) {
    json root = json::parse(jsonText, nullptr, /*allow_exceptions=*/false);
    if (root.is_discarded() || !root.is_object()) {
        fail(error, "plugin.json is not a JSON object");
        return std::nullopt;
    }

    PluginManifest manifest;
    manifest.rootDir = pluginDir;
    manifest.name = root.value("name", "");
    manifest.version = root.value("version", "");
    manifest.entry = root.value("entry", "");
    manifest.description = root.value("description", "");
    manifest.author = root.value("author", "");
    manifest.minAppVersion = root.value("minAppVersion", "");

    if (!validName(manifest.name)) {
        fail(error, "name must match [a-z][a-z0-9-]{2,63}: '" + manifest.name + "'");
        return std::nullopt;
    }
    if (!parseSemver(manifest.version)) {
        fail(error, "version must be semver X.Y.Z: '" + manifest.version + "'");
        return std::nullopt;
    }
    if (!manifest.minAppVersion.empty() && !parseSemver(manifest.minAppVersion)) {
        fail(error, "minAppVersion must be semver X.Y.Z: '" + manifest.minAppVersion + "'");
        return std::nullopt;
    }
    if (!validEntry(manifest.entry, pluginDir, error)) return std::nullopt;

    // Permissions are fail-closed: an unknown string is an error, never a
    // silent no-op, so a typo cannot grant less scrutiny than intended.
    const json perms = root.value("permissions", json::array());
    if (!perms.is_array()) {
        fail(error, "permissions must be an array of strings");
        return std::nullopt;
    }
    for (const json& perm : perms) {
        if (!perm.is_string()) {
            fail(error, "permissions must be an array of strings");
            return std::nullopt;
        }
        const auto parsed = permissionFromString(perm.get<std::string>());
        if (!parsed) {
            fail(error, "unknown permission: '" + perm.get<std::string>() + "'");
            return std::nullopt;
        }
        if (std::find(manifest.permissions.begin(), manifest.permissions.end(), *parsed) ==
            manifest.permissions.end()) {
            manifest.permissions.push_back(*parsed);
        }
    }
    return manifest;
}

size_t PluginRegistry::discover(const fs::path& pluginsRoot) {
    std::error_code ec;
    if (!fs::is_directory(pluginsRoot, ec)) return 0;

    size_t added = 0;
    for (const auto& child : fs::directory_iterator(pluginsRoot, ec)) {
        if (!child.is_directory()) continue;
        const fs::path manifestPath = child.path() / "plugin.json";
        std::error_code fileEc;
        if (!fs::is_regular_file(manifestPath, fileEc)) continue;

        std::ifstream in(manifestPath, std::ios::binary);
        std::ostringstream text;
        text << in.rdbuf();
        if (!in) {
            m_errors.push_back({child.path(), "failed to read plugin.json"});
            continue;
        }

        std::string error;
        auto manifest = parseManifest(text.str(), child.path(), &error);
        if (!manifest) {
            m_errors.push_back({child.path(), error});
            continue;
        }
        if (find(manifest->name)) {
            m_errors.push_back({child.path(), "duplicate plugin name: '" + manifest->name + "'"});
            continue;
        }
        m_plugins.push_back(std::move(*manifest));
        ++added;
    }
    std::sort(m_plugins.begin(), m_plugins.end(),
              [](const PluginManifest& a, const PluginManifest& b) { return a.name < b.name; });
    return added;
}

const PluginManifest* PluginRegistry::find(const std::string& name) const {
    for (const PluginManifest& plugin : m_plugins) {
        if (plugin.name == name) return &plugin;
    }
    return nullptr;
}

bool PluginRegistry::setEnabled(const std::string& name, bool enabled) {
    if (!find(name)) return false;
    const auto it = std::find(m_enabled.begin(), m_enabled.end(), name);
    if (enabled && it == m_enabled.end()) m_enabled.push_back(name);
    if (!enabled && it != m_enabled.end()) m_enabled.erase(it);
    return true;
}

bool PluginRegistry::isEnabled(const std::string& name) const {
    return std::find(m_enabled.begin(), m_enabled.end(), name) != m_enabled.end();
}

bool PluginRegistry::isCompatible(const PluginManifest& manifest, const std::string& appVersion) {
    if (manifest.minAppVersion.empty()) return true;
    const auto minimum = parseSemver(manifest.minAppVersion);
    const auto app = parseSemver(appVersion);
    if (!minimum || !app) return false;  // unparseable gates fail closed
    return *minimum <= *app;
}

}  // namespace hz::plugin
