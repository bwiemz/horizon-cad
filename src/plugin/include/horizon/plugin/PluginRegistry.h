#pragma once

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace hz::plugin {

/// Capabilities a plugin must declare up front (Phase 79, roadmap §7.15
/// sandboxing: explicit permissions, fail-closed). Unknown permission
/// strings make the whole manifest invalid rather than being ignored.
enum class Permission {
    Filesystem,  ///< read/write outside the plugin's own directory
    Network,     ///< any network access
    UI,          ///< menus, toolbars, dock panels
    Document,    ///< mutate the open document/feature tree
    Simulation,  ///< run FEA/CAM jobs
};

/// Parsed and validated plugin.json.
struct PluginManifest {
    std::string name;     ///< ^[a-z][a-z0-9-]{2,63}$ — the unique id
    std::string version;  ///< semver "X.Y.Z"
    std::string entry;    ///< Python entry script, relative to rootDir
    std::string description;
    std::string author;
    std::string minAppVersion;  ///< optional semver gate ("" = any)
    std::vector<Permission> permissions;
    std::filesystem::path rootDir;  ///< directory the manifest came from
};

/// A manifest that failed validation, kept for diagnostics.
struct ManifestError {
    std::filesystem::path dir;
    std::string message;
};

/// Discovers and validates plugin manifests without executing any plugin
/// code (Phase 79 core slice). A plugin is a directory containing
/// `plugin.json` plus its entry script; discovery scans the immediate
/// subdirectories of a plugins root. Plugins are DISABLED by default —
/// running them is an explicit opt-in on top of this registry, and the
/// Python execution bridge itself is staged in hz::scripting.
class PluginRegistry {
public:
    /// Validate a manifest's JSON text against @p pluginDir (the entry
    /// script must exist inside it — no absolute paths, no `..` escapes).
    /// On failure returns std::nullopt and, when @p error is non-null,
    /// stores a human-readable reason.
    static std::optional<PluginManifest> parseManifest(const std::string& jsonText,
                                                       const std::filesystem::path& pluginDir,
                                                       std::string* error = nullptr);

    /// Scan `pluginsRoot/*/plugin.json`. Valid manifests join plugins()
    /// (sorted by name), invalid ones join errors(). A name that is
    /// already registered is an error (first one wins). Returns the number
    /// of plugins added. A missing root is not an error — returns 0.
    size_t discover(const std::filesystem::path& pluginsRoot);

    const std::vector<PluginManifest>& plugins() const { return m_plugins; }
    const std::vector<ManifestError>& errors() const { return m_errors; }
    const PluginManifest* find(const std::string& name) const;

    /// Enablement is per-name and defaults to OFF for every discovered
    /// plugin. Returns false when the name is unknown.
    bool setEnabled(const std::string& name, bool enabled);
    bool isEnabled(const std::string& name) const;

    /// True when @p manifest's minAppVersion gate admits @p appVersion.
    static bool isCompatible(const PluginManifest& manifest, const std::string& appVersion);

    /// Strict "X.Y.Z" semver parse; nullopt on anything else.
    static std::optional<std::array<int, 3>> parseSemver(const std::string& text);

private:
    std::vector<PluginManifest> m_plugins;
    std::vector<ManifestError> m_errors;
    std::vector<std::string> m_enabled;
};

}  // namespace hz::plugin
