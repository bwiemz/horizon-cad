#include <gtest/gtest.h>

#include "horizon/document/ConfigurationTable.h"
#include "horizon/document/ParameterRegistry.h"

using hz::doc::ConfigurationTable;
using hz::doc::ParameterRegistry;

// Configurations are stored and listed in definition order.
TEST(ConfigurationTableTest, StoresConfigurationsInOrder) {
    ConfigurationTable table;
    table.setConfiguration("M6", {{"diameter", 6.0}, {"length", 20.0}});
    table.setConfiguration("M8", {{"diameter", 8.0}, {"length", 25.0}});
    table.setConfiguration("M10", {{"diameter", 10.0}, {"length", 30.0}});

    ASSERT_EQ(table.size(), 3u);
    EXPECT_TRUE(table.hasConfiguration("M8"));
    EXPECT_FALSE(table.hasConfiguration("M12"));
    const std::vector<std::string> expected = {"M6", "M8", "M10"};
    EXPECT_EQ(table.configurationNames(), expected);

    const auto ov = table.overrides("M8");
    EXPECT_EQ(ov.at("diameter"), 8.0);
    EXPECT_EQ(ov.at("length"), 25.0);
    EXPECT_TRUE(table.overrides("missing").empty());
}

// Applying a configuration writes its overrides into the parameter registry,
// leaving unrelated parameters untouched.
TEST(ConfigurationTableTest, ApplyDrivesParameters) {
    ParameterRegistry params;
    params.set("diameter", 6.0);
    params.set("length", 20.0);
    params.set("head_height", 4.0);  // not overridden by any configuration

    ConfigurationTable table;
    table.setConfiguration("M10", {{"diameter", 10.0}, {"length", 30.0}});

    ASSERT_TRUE(table.apply("M10", params));
    EXPECT_EQ(params.get("diameter"), 10.0);
    EXPECT_EQ(params.get("length"), 30.0);
    EXPECT_EQ(params.get("head_height"), 4.0);  // untouched

    EXPECT_FALSE(table.apply("missing", params));  // no such configuration
}

// Redefining a configuration replaces its overrides without reordering.
TEST(ConfigurationTableTest, RedefineReplacesInPlace) {
    ConfigurationTable table;
    table.setConfiguration("A", {{"x", 1.0}});
    table.setConfiguration("B", {{"x", 2.0}});
    table.setConfiguration("A", {{"x", 9.0}, {"y", 3.0}});

    EXPECT_EQ(table.size(), 2u);
    const std::vector<std::string> expected = {"A", "B"};
    EXPECT_EQ(table.configurationNames(), expected);
    EXPECT_EQ(table.overrides("A").at("x"), 9.0);
    EXPECT_EQ(table.overrides("A").at("y"), 3.0);
}

// The active configuration tracks a valid name and clears when removed.
TEST(ConfigurationTableTest, ActiveConfiguration) {
    ConfigurationTable table;
    table.setConfiguration("Small", {{"s", 1.0}});
    table.setConfiguration("Large", {{"s", 9.0}});

    EXPECT_EQ(table.active(), "");
    EXPECT_FALSE(table.setActive("Missing"));
    EXPECT_EQ(table.active(), "");

    EXPECT_TRUE(table.setActive("Large"));
    EXPECT_EQ(table.active(), "Large");

    // Removing the active configuration clears the active pointer.
    EXPECT_TRUE(table.removeConfiguration("Large"));
    EXPECT_EQ(table.active(), "");
    EXPECT_EQ(table.size(), 1u);
    EXPECT_FALSE(table.hasConfiguration("Large"));
}
