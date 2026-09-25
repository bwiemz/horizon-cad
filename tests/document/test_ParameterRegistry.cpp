#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/ParameterRegistry.h"
#include "horizon/document/UndoStack.h"

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

// ---------------------------------------------------------------------------
// Phase 155: variables and equations
// ---------------------------------------------------------------------------

namespace {

const std::map<std::string, std::string> kRig{
    {"wall", "3 mm"}, {"width", "10 * wall"}, {"count", "4"}, {"tilt", "30 deg"}};

}  // namespace

TEST(VariablesTest, EachVariableIsWorkedOutWithWhatItMeasures) {
    hz::doc::ParameterRegistry variables;
    variables.setDefinitions(kRig);
    EXPECT_EQ(variables.definitions(), kRig) << "kept as typed";
    std::map<std::string, std::string> errors;
    const auto q = variables.quantities(&errors);
    EXPECT_TRUE(errors.empty());
    ASSERT_EQ(q.size(), 4u);
    EXPECT_DOUBLE_EQ(q.at("width").value, 30.0);
    EXPECT_EQ(q.at("width").length, 1);
    EXPECT_TRUE(q.at("count").pure());
    EXPECT_EQ(q.at("tilt").angle, 1);
    EXPECT_DOUBLE_EQ(variables.get("width"), 30.0) << "as constraints read it, in millimetres";

    // One that cannot be worked out is left out, with why; those after it too.
    variables.setDefinitions({{"a", "2 mm + 1"}, {"b", "a * 2"}, {"c", "5"}});
    errors.clear();
    const auto some = variables.quantities(&errors);
    EXPECT_EQ(some.count("c"), 1u);
    EXPECT_EQ(some.count("a"), 0u);
    EXPECT_EQ(errors.count("a"), 1u);
    EXPECT_EQ(errors.count("b"), 1u);
}

TEST(VariablesTest, WhatCannotBeAVariableIsRefusedWithWhy) {
    std::string why;
    EXPECT_TRUE(hz::doc::ParameterRegistry::check(kRig, &why)) << why;
    for (const auto& bad :
         std::vector<std::map<std::string, std::string>>{{{"2wide", "1"}},  // not a name
                                                         {{"in", "1"}},     // a unit
                                                         {{"sin", "1"}},    // a function
                                                         {{"pi", "3"}},     // a constant
                                                         {{"a", "2 +"}},    // not an expression
                                                         {{"a", "b + 1"}, {"b", "a"}},  // a loop
                                                         {{"a", "2 mm + 1"}},   // measures nothing
                                                         {{"a", "nosuch"}}}) {  // no such variable
        why.clear();
        EXPECT_FALSE(hz::doc::ParameterRegistry::check(bad, &why)) << bad.begin()->first;
        EXPECT_FALSE(why.empty()) << bad.begin()->first;
    }
}

// The variables as a whole are one undo step, and the part is built again.
TEST(VariablesTest, AChangeOfVariablesIsUndone) {
    hz::doc::Document document;
    document.parameterRegistry().setDefinitions({{"wall", "2 mm"}});
    const auto revision = document.featureTree().revision();
    document.undoStack().push(std::make_unique<hz::doc::SetVariablesCommand>(document, kRig));
    EXPECT_EQ(document.parameterRegistry().definitions(), kRig);
    EXPECT_GT(document.featureTree().revision(), revision) << "built again";
    document.undoStack().undo();
    EXPECT_EQ(document.parameterRegistry().definitions(),
              (std::map<std::string, std::string>{{"wall", "2 mm"}}));
    document.undoStack().redo();
    EXPECT_EQ(document.parameterRegistry().definitions(), kRig);
}

// A loop leaves out the variables in it, and those that depend on them;
// the others are worked out as ever.
TEST(VariablesTest, ALoopIsBlamedOnlyOnTheVariablesInIt) {
    hz::doc::ParameterRegistry variables;
    variables.setDefinitions(
        {{"wall", "3 mm"}, {"width", "10 * wall"}, {"a", "b + 1"}, {"b", "a * 2"}, {"c", "a + 1"}});
    std::map<std::string, std::string> errors;
    const auto q = variables.quantities(&errors);
    EXPECT_EQ(q.count("wall"), 1u);
    EXPECT_EQ(q.count("width"), 1u) << "no part of the loop";
    EXPECT_EQ(errors.count("wall"), 0u);
    EXPECT_EQ(errors.count("width"), 0u);
    EXPECT_EQ(errors.count("a"), 1u);
    EXPECT_EQ(errors.count("b"), 1u);
    EXPECT_EQ(errors.count("c"), 1u) << "it depends on the loop";
}
