#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "horizon/plugin/PluginRegistry.h"

using hz::plugin::Permission;
using hz::plugin::PluginManifest;
using hz::plugin::PluginRegistry;
namespace fs = std::filesystem;

namespace {

/// Fresh scratch root per test, removed on teardown (ASan-friendly).
class PluginRegistryTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Process id keeps the scratch root unique across parallel `ctest -j`
        // runs (gtest's random_seed defaults to 0 without --gtest_shuffle, so
        // it is not a reliable disambiguator).
        const unsigned long pid =
#ifdef _WIN32
            ::GetCurrentProcessId();
#else
            static_cast<unsigned long>(::getpid());
#endif
        m_root = fs::temp_directory_path() /
                 ("hz_plugin_test_" + std::to_string(pid) + "_" +
                  ::testing::UnitTest::GetInstance()->current_test_info()->name());
        fs::remove_all(m_root);
        fs::create_directories(m_root);
    }
    void TearDown() override { fs::remove_all(m_root); }

    /// Create plugins root subdir `name` with a plugin.json and entry file.
    fs::path makePlugin(const std::string& dirName, const std::string& manifestJson,
                        const std::string& entryFile = "main.py") {
        const fs::path dir = m_root / dirName;
        fs::create_directories(dir);
        if (!entryFile.empty()) {
            std::ofstream(dir / entryFile) << "# entry\n";
        }
        std::ofstream(dir / "plugin.json") << manifestJson;
        return dir;
    }

    fs::path m_root;
};

std::string validManifest(const std::string& name = "hole-wizard") {
    return R"({
        "name": ")" +
           name + R"(",
        "version": "1.2.3",
        "entry": "main.py",
        "description": "Parametric hole patterns",
        "author": "Jane Doe",
        "permissions": ["document", "ui"]
    })";
}

}  // namespace

TEST_F(PluginRegistryTest, ValidManifestParses) {
    const fs::path dir = makePlugin("hole-wizard", validManifest());
    std::string error;
    const auto manifest = PluginRegistry::parseManifest(validManifest(), dir, &error);
    ASSERT_TRUE(manifest.has_value()) << error;
    EXPECT_EQ(manifest->name, "hole-wizard");
    EXPECT_EQ(manifest->version, "1.2.3");
    EXPECT_EQ(manifest->entry, "main.py");
    EXPECT_EQ(manifest->author, "Jane Doe");
    ASSERT_EQ(manifest->permissions.size(), 2u);
    EXPECT_EQ(manifest->permissions[0], Permission::Document);
    EXPECT_EQ(manifest->permissions[1], Permission::UI);
    EXPECT_EQ(manifest->rootDir, dir);
}

TEST_F(PluginRegistryTest, NameRulesAreEnforced) {
    const fs::path dir = makePlugin("bad", "{}");
    std::string error;
    for (const std::string bad : {"ab", "Hole-Wizard", "hole_wizard", "9lives", "hole wizard"}) {
        const auto manifest = PluginRegistry::parseManifest(
            R"({"name": ")" + bad + R"(", "version": "1.0.0", "entry": "main.py"})", dir, &error);
        EXPECT_FALSE(manifest.has_value()) << bad;
        EXPECT_NE(error.find("name"), std::string::npos) << error;
    }
}

TEST_F(PluginRegistryTest, VersionMustBeStrictSemver) {
    const fs::path dir = makePlugin("versions", "{}");
    std::string error;
    for (const std::string bad : {"1.2", "1.2.3.4", "v1.2.3", "1.2.3-beta", "", "1..3"}) {
        const auto manifest = PluginRegistry::parseManifest(
            R"({"name": "my-plugin", "version": ")" + bad + R"(", "entry": "main.py"})", dir,
            &error);
        EXPECT_FALSE(manifest.has_value()) << bad;
    }
    EXPECT_TRUE(PluginRegistry::parseSemver("0.0.0").has_value());
    EXPECT_FALSE(PluginRegistry::parseSemver("01a.2.3").has_value());
}

TEST_F(PluginRegistryTest, EntryMustExistInsideThePluginDir) {
    const fs::path dir = makePlugin("entries", "{}");
    std::string error;

    // Missing file.
    auto manifest = PluginRegistry::parseManifest(
        R"({"name": "my-plugin", "version": "1.0.0", "entry": "nope.py"})", dir, &error);
    EXPECT_FALSE(manifest.has_value());
    EXPECT_NE(error.find("not found"), std::string::npos) << error;

    // Escape via .. is rejected even when the target exists.
    std::ofstream(m_root / "outside.py") << "# outside\n";
    manifest = PluginRegistry::parseManifest(
        R"({"name": "my-plugin", "version": "1.0.0", "entry": "../outside.py"})", dir, &error);
    EXPECT_FALSE(manifest.has_value());
    EXPECT_NE(error.find("escape"), std::string::npos) << error;

    // Absolute paths are rejected.
    const std::string abs = (m_root / "entries" / "main.py").generic_string();
    manifest = PluginRegistry::parseManifest(
        R"({"name": "my-plugin", "version": "1.0.0", "entry": ")" + abs + R"("})", dir, &error);
    EXPECT_FALSE(manifest.has_value());

    // Nested relative entries are fine.
    fs::create_directories(dir / "src");
    std::ofstream(dir / "src" / "run.py") << "# entry\n";
    manifest = PluginRegistry::parseManifest(
        R"({"name": "my-plugin", "version": "1.0.0", "entry": "src/run.py"})", dir, &error);
    EXPECT_TRUE(manifest.has_value()) << error;
}

TEST_F(PluginRegistryTest, UnknownPermissionsFailClosed) {
    const fs::path dir = makePlugin("perms", "{}");
    std::string error;
    const auto manifest = PluginRegistry::parseManifest(
        R"({"name": "my-plugin", "version": "1.0.0", "entry": "main.py",
            "permissions": ["document", "root-shell"]})",
        dir, &error);
    EXPECT_FALSE(manifest.has_value());
    EXPECT_NE(error.find("root-shell"), std::string::npos) << error;
}

TEST_F(PluginRegistryTest, MalformedJsonIsAnErrorNotACrash) {
    const fs::path dir = makePlugin("broken", "{ not json");
    std::string error;
    EXPECT_FALSE(PluginRegistry::parseManifest("{ not json", dir, &error).has_value());
    EXPECT_FALSE(PluginRegistry::parseManifest("[1, 2, 3]", dir, &error).has_value());
    EXPECT_FALSE(PluginRegistry::parseManifest("", dir, &error).has_value());
}

TEST_F(PluginRegistryTest, NonStringFieldsAreRejectedNotThrown) {
    const fs::path dir = makePlugin("typed", "{}");
    std::string error;
    // A present-but-wrong-typed field must be a validation error, not an
    // uncaught nlohmann type_error that aborts the scan.
    for (const std::string bad : {
             R"({"name": 123, "version": "1.0.0", "entry": "main.py"})",
             R"({"name": "my-plugin", "version": ["1", "0"], "entry": "main.py"})",
             R"({"name": "my-plugin", "version": "1.0.0", "entry": {"x": 1}})",
             R"({"name": "my-plugin", "version": "1.0.0", "entry": "main.py", "author": 5})",
         }) {
        EXPECT_FALSE(PluginRegistry::parseManifest(bad, dir, &error).has_value()) << bad;
    }

    // And discover() survives a directory full of wrong-typed manifests.
    makePlugin("numeric-name", R"({"name": 42, "version": "1.0.0", "entry": "main.py"})");
    PluginRegistry registry;
    EXPECT_NO_THROW(registry.discover(m_root));
    EXPECT_FALSE(registry.errors().empty());
}

TEST_F(PluginRegistryTest, RootAndDriveRelativeEntriesAreRejected) {
    const fs::path dir = makePlugin("winpaths", "{}");
    std::string error;
    // Forward-slash root ("/foo"), JSON-escaped backslash root ("\\foo"), and
    // Windows drive-relative ("C:foo") entries must all be refused: none is
    // caught by is_absolute() alone on every platform. (JSON strings escape a
    // backslash as "\\", i.e. the C++ raw literal R"(\\evil.py)".)
    for (const std::string bad : {R"(/evil.py)", R"(\\evil.py)", R"(C:evil.py)"}) {
        const auto manifest = PluginRegistry::parseManifest(
            R"({"name": "my-plugin", "version": "1.0.0", "entry": ")" + bad + R"("})", dir, &error);
        EXPECT_FALSE(manifest.has_value()) << bad;
    }
}

TEST_F(PluginRegistryTest, DiscoverScansValidatesAndSorts) {
    makePlugin("zeta", validManifest("zeta-tool"));
    makePlugin("alpha", validManifest("alpha-tool"));
    makePlugin("broken", "{ not json");
    fs::create_directories(m_root / "no-manifest");  // silently skipped

    PluginRegistry registry;
    EXPECT_EQ(registry.discover(m_root), 2u);
    ASSERT_EQ(registry.plugins().size(), 2u);
    EXPECT_EQ(registry.plugins()[0].name, "alpha-tool");
    EXPECT_EQ(registry.plugins()[1].name, "zeta-tool");
    ASSERT_EQ(registry.errors().size(), 1u);
    EXPECT_EQ(registry.errors()[0].dir.filename(), "broken");
    EXPECT_TRUE(registry.find("alpha-tool") != nullptr);
    EXPECT_TRUE(registry.find("missing") == nullptr);
}

TEST_F(PluginRegistryTest, DuplicateNamesKeepTheFirst) {
    makePlugin("a-dir", validManifest("same-name"));
    makePlugin("b-dir", validManifest("same-name"));

    PluginRegistry registry;
    EXPECT_EQ(registry.discover(m_root), 1u);
    EXPECT_EQ(registry.plugins().size(), 1u);
    ASSERT_EQ(registry.errors().size(), 1u);
    EXPECT_NE(registry.errors()[0].message.find("duplicate"), std::string::npos);
}

TEST_F(PluginRegistryTest, PluginsAreDisabledByDefault) {
    makePlugin("hole-wizard", validManifest());
    PluginRegistry registry;
    registry.discover(m_root);

    EXPECT_FALSE(registry.isEnabled("hole-wizard"));
    EXPECT_FALSE(registry.setEnabled("unknown", true));
    EXPECT_TRUE(registry.setEnabled("hole-wizard", true));
    EXPECT_TRUE(registry.isEnabled("hole-wizard"));
    EXPECT_TRUE(registry.setEnabled("hole-wizard", false));
    EXPECT_FALSE(registry.isEnabled("hole-wizard"));
}

TEST_F(PluginRegistryTest, MinAppVersionGatesCompatibility) {
    PluginManifest manifest;
    manifest.minAppVersion = "";
    EXPECT_TRUE(PluginRegistry::isCompatible(manifest, "0.1.0"));
    manifest.minAppVersion = "1.2.0";
    EXPECT_FALSE(PluginRegistry::isCompatible(manifest, "1.1.9"));
    EXPECT_TRUE(PluginRegistry::isCompatible(manifest, "1.2.0"));
    EXPECT_TRUE(PluginRegistry::isCompatible(manifest, "2.0.0"));
    EXPECT_FALSE(PluginRegistry::isCompatible(manifest, "garbage"));
}

TEST_F(PluginRegistryTest, MissingRootIsEmptyNotFatal) {
    PluginRegistry registry;
    EXPECT_EQ(registry.discover(m_root / "does-not-exist"), 0u);
    EXPECT_TRUE(registry.plugins().empty());
    EXPECT_TRUE(registry.errors().empty());
}

// -- Checked again at load (Phase 120) ----------------------------------------

// A loader gets the plugin's entry source only for a plugin the user enabled,
// and gets the bytes that were checked.
TEST_F(PluginRegistryTest, LoadingNeedsAnEnabledPlugin) {
    const fs::path dir = makePlugin("hole-wizard", validManifest());
    PluginRegistry registry;
    registry.discover(m_root);

    std::string error;
    EXPECT_FALSE(registry.prepareLoad("hole-wizard", "0.1.0", &error));
    EXPECT_NE(error.find("not enabled"), std::string::npos) << error;
    EXPECT_FALSE(registry.prepareLoad("no-such-plugin", "0.1.0", &error));
    EXPECT_NE(error.find("no plugin named"), std::string::npos) << error;

    ASSERT_TRUE(registry.setEnabled("hole-wizard", true));
    const auto plugin = registry.prepareLoad("hole-wizard", "0.1.0", &error);
    ASSERT_TRUE(plugin) << error;
    EXPECT_EQ(plugin->manifest.name, "hole-wizard");
    // The bytes on disk, whatever line ending the helper's text-mode write
    // gave them.
    std::ifstream in(dir / "main.py", std::ios::binary);
    const std::string onDisk{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    EXPECT_EQ(plugin->entrySource, onDisk);
    EXPECT_EQ(plugin->entrySource.rfind("# entry", 0), 0u);
    EXPECT_EQ(plugin->entryPath, fs::weakly_canonical(dir / "main.py"));
}

TEST_F(PluginRegistryTest, LoadingChecksCompatibility) {
    makePlugin("future", R"({"name": "future", "version": "1.0.0", "entry": "main.py",
                             "minAppVersion": "9.0.0"})");
    PluginRegistry registry;
    registry.discover(m_root);
    ASSERT_TRUE(registry.setEnabled("future", true));
    std::string error;
    EXPECT_FALSE(registry.prepareLoad("future", "0.1.0", &error));
    EXPECT_NE(error.find("9.0.0"), std::string::npos) << error;
    EXPECT_TRUE(registry.prepareLoad("future", "9.0.0", &error)) << error;
}

// A plugin that gains a permission after the user enabled it does not load
// with it: it has to be discovered and enabled again.
TEST_F(PluginRegistryTest, APluginThatChangedMustBeEnabledAgain) {
    const fs::path dir = makePlugin("hole-wizard", validManifest());
    PluginRegistry registry;
    registry.discover(m_root);
    ASSERT_TRUE(registry.setEnabled("hole-wizard", true));

    std::ofstream(dir / "plugin.json") << R"({"name": "hole-wizard", "version": "1.2.3",
        "entry": "main.py", "description": "Parametric hole patterns", "author": "Jane Doe",
        "permissions": ["document", "ui", "network"]})";
    std::string error;
    EXPECT_FALSE(registry.prepareLoad("hole-wizard", "0.1.0", &error));
    EXPECT_NE(error.find("changed"), std::string::npos) << error;

    // Listing the same permissions in another order is not a change.
    std::ofstream(dir / "plugin.json") << R"({"name": "hole-wizard", "version": "1.2.3",
        "entry": "main.py", "description": "Parametric hole patterns", "author": "Jane Doe",
        "permissions": ["ui", "document"]})";
    EXPECT_TRUE(registry.prepareLoad("hole-wizard", "0.1.0", &error)) << error;
}

// Files that disappear or break after discovery stop the load.
TEST_F(PluginRegistryTest, APluginBrokenAfterDiscoveryDoesNotLoad) {
    const fs::path dir = makePlugin("hole-wizard", validManifest());
    PluginRegistry registry;
    registry.discover(m_root);
    ASSERT_TRUE(registry.setEnabled("hole-wizard", true));
    std::string error;

    fs::remove(dir / "main.py");
    EXPECT_FALSE(registry.prepareLoad("hole-wizard", "0.1.0", &error));
    EXPECT_NE(error.find("no longer valid"), std::string::npos) << error;

    std::ofstream(dir / "main.py") << "# entry\n";
    std::ofstream(dir / "plugin.json") << "{ broken";
    EXPECT_FALSE(registry.prepareLoad("hole-wizard", "0.1.0", &error));

    fs::remove(dir / "plugin.json");
    EXPECT_FALSE(registry.prepareLoad("hole-wizard", "0.1.0", &error));
    EXPECT_NE(error.find("cannot read"), std::string::npos) << error;
}

#ifndef _WIN32
// The entry replaced after discovery by a link to a file outside the plugin
// (symbolic links need extra rights on Windows).
TEST_F(PluginRegistryTest, AnEntryThatNowEscapesDoesNotLoad) {
    const fs::path dir = makePlugin("hole-wizard", validManifest());
    PluginRegistry registry;
    registry.discover(m_root);
    ASSERT_TRUE(registry.setEnabled("hole-wizard", true));

    std::ofstream(m_root / "secret.txt") << "not a plugin";
    fs::remove(dir / "main.py");
    fs::create_symlink(m_root / "secret.txt", dir / "main.py");

    std::string error;
    EXPECT_FALSE(registry.prepareLoad("hole-wizard", "0.1.0", &error));
    EXPECT_NE(error.find("inside the plugin directory"), std::string::npos) << error;
}
#endif
