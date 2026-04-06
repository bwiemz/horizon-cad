#include <gtest/gtest.h>

#include "horizon/document/ParameterRegistry.h"

using namespace hz::doc;

TEST(ParameterRegistryTest, SetAndGet) {
    ParameterRegistry reg;
    reg.set("width", 50.0);
    EXPECT_DOUBLE_EQ(reg.get("width"), 50.0);
}

TEST(ParameterRegistryTest, GetNonExistentReturnsZero) {
    ParameterRegistry reg;
    EXPECT_DOUBLE_EQ(reg.get("missing"), 0.0);
}

TEST(ParameterRegistryTest, HasVariable) {
    ParameterRegistry reg;
    EXPECT_FALSE(reg.has("width"));
    reg.set("width", 10.0);
    EXPECT_TRUE(reg.has("width"));
}

TEST(ParameterRegistryTest, RemoveVariable) {
    ParameterRegistry reg;
    reg.set("width", 10.0);
    reg.remove("width");
    EXPECT_FALSE(reg.has("width"));
}

TEST(ParameterRegistryTest, AllVariables) {
    ParameterRegistry reg;
    reg.set("width", 10.0);
    reg.set("height", 20.0);
    auto vars = reg.all();
    EXPECT_EQ(vars.size(), 2u);
    EXPECT_DOUBLE_EQ(vars.at("width"), 10.0);
    EXPECT_DOUBLE_EQ(vars.at("height"), 20.0);
}

TEST(ParameterRegistryTest, Clear) {
    ParameterRegistry reg;
    reg.set("a", 1.0);
    reg.set("b", 2.0);
    reg.clear();
    EXPECT_EQ(reg.all().size(), 0u);
}
