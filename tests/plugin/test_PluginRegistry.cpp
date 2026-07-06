#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

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
        m_root =
            fs::temp_directory_path() /
            ("hz_plugin_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
             "_" + ::testing::UnitTest::GetInstance()->current_test_info()->name());
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
