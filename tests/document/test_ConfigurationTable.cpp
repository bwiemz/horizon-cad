#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "horizon/document/ConfigurationTable.h"

using hz::doc::ConfigurationTable;

// Configurations are stored and listed in definition order.
TEST(ConfigurationTableTest, StoresConfigurationsInOrder) {
    ConfigurationTable table;
    table.setConfiguration("M6", {{"diameter", "6 mm"}, {"length", "20 mm"}});
    table.setConfiguration("M8", {{"diameter", "8 mm"}, {"length", "25 mm"}});
    table.setConfiguration("M10", {{"diameter", "10 mm"}, {"length", "30 mm"}});

    ASSERT_EQ(table.size(), 3u);
    EXPECT_TRUE(table.hasConfiguration("M8"));
    EXPECT_FALSE(table.hasConfiguration("M12"));
    const std::vector<std::string> expected = {"M6", "M8", "M10"};
    EXPECT_EQ(table.configurationNames(), expected);

    const auto ov = table.overrides("M8");
    EXPECT_EQ(ov.at("diameter"), "8 mm");
    EXPECT_EQ(ov.at("length"), "25 mm");
    EXPECT_TRUE(table.overrides("missing").empty());
}

// A configuration is laid over the document's own variables; what it
// leaves alone stays, and the variables themselves are not written.
TEST(ConfigurationTableTest, AConfigurationIsLaidOverTheVariables) {
    const ConfigurationTable::Overrides own{
        {"diameter", "6 mm"}, {"length", "20 mm"}, {"head", "4 mm"}};
    ConfigurationTable table;
    table.setConfiguration("M10", {{"diameter", "10 mm"}, {"length", "3 * diameter"}});

    const auto laid = table.overlay(own, "M10");
    EXPECT_EQ(laid.at("diameter"), "10 mm");
    EXPECT_EQ(laid.at("length"), "3 * diameter");
    EXPECT_EQ(laid.at("head"), "4 mm") << "left alone";
    EXPECT_EQ(table.overlay(own, "missing"), own);
    EXPECT_EQ(table.overlay(own, ""), own) << "none: the document's own";
}

// Redefining a configuration replaces its overrides without reordering.
TEST(ConfigurationTableTest, RedefineReplacesInPlace) {
    ConfigurationTable table;
    table.setConfiguration("A", {{"x", "1"}});
    table.setConfiguration("B", {{"x", "2"}});
    table.setConfiguration("A", {{"x", "9"}, {"y", "3"}});

    EXPECT_EQ(table.size(), 2u);
    const std::vector<std::string> expected = {"A", "B"};
    EXPECT_EQ(table.configurationNames(), expected);
    EXPECT_EQ(table.overrides("A").at("x"), "9");
    EXPECT_EQ(table.overrides("A").at("y"), "3");
}

// The active configuration is one of them, or none; it clears when removed.
TEST(ConfigurationTableTest, ActiveConfiguration) {
    ConfigurationTable table;
    table.setConfiguration("Small", {{"s", "1"}});
    table.setConfiguration("Large", {{"s", "9"}});

    EXPECT_EQ(table.active(), "");
    EXPECT_FALSE(table.setActive("Missing"));
    EXPECT_EQ(table.active(), "");

    EXPECT_TRUE(table.setActive("Large"));
    EXPECT_EQ(table.active(), "Large");
    EXPECT_TRUE(table.setActive("")) << "none: the document's own variables";
    EXPECT_EQ(table.active(), "");
    EXPECT_TRUE(table.setActive("Large"));

    // Removing the active configuration clears the active pointer.
    EXPECT_TRUE(table.removeConfiguration("Large"));
    EXPECT_EQ(table.active(), "");
    EXPECT_EQ(table.size(), 1u);
    EXPECT_FALSE(table.hasConfiguration("Large"));
}
