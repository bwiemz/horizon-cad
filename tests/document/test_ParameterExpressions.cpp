// Phase 155: a feature's parameters as expressions of the document's
// variables, worked out each time the part is built.

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <memory>
#include <string>

#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/document/ConfigurationTable.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/ModelCommands.h"
#include "horizon/document/UndoStack.h"
#include "horizon/modeling/MassProperties.h"

using hz::doc::Document;
using hz::doc::PrimitiveFeature;

namespace {

double volumeOf(const Document& document) {
    return document.solid() != nullptr
               ? hz::model::MassPropertiesCalculator::compute(*document.solid()).volume
               : 0.0;
}

/// A part with one box, 10 x 10 x 10, and the variables @p variables.
std::unique_ptr<Document> boxPart(const std::map<std::string, std::string>& variables) {
    auto part = std::make_unique<Document>();
    part->setType(hz::doc::DocumentType::Part);
    part->parameterRegistry().setDefinitions(variables);
    part->featureTree().addFeature(PrimitiveFeature::makeBox(10, 10, 10));
    return part;
}

}  // namespace

TEST(ParameterExpressionsTest, AParameterFollowsTheVariablesItIsWrittenIn) {
    const auto owner = boxPart({{"wall", "3 mm"}});
    Document& part = *owner;
    part.featureTree().feature(0)->setParameterExpression("width", "((wall * 2) + (1 mm))");
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_DOUBLE_EQ(part.featureTree().feature(0)->parameters().at("width"), 7.0);
    EXPECT_NEAR(volumeOf(part), 700.0, 1e-9);

    // The variable changed, as one step, and the part built again follows.
    part.undoStack().push(std::make_unique<hz::doc::SetVariablesCommand>(
        part, std::map<std::string, std::string>{{"wall", "0.5 in"}}));
    EXPECT_TRUE(part.needsBuild());
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_NEAR(volumeOf(part), (2 * 12.7 + 1) * 100.0, 1e-9);
    part.undoStack().undo();
    ASSERT_TRUE(part.rebuildModel());
    EXPECT_NEAR(volumeOf(part), 700.0, 1e-9);
}

// One that cannot be worked out is its feature's failure, said; the part is
// as it stood before it.
TEST(ParameterExpressionsTest, AnExpressionThatCannotBeWorkedOutFailsItsFeature) {
    const auto owner = boxPart({{"wall", "3 mm"}, {"tilt", "30 deg"}});
    Document& part = *owner;
    part.featureTree().feature(0)->setParameterExpression("width", "(nosuch * 2)");
    EXPECT_FALSE(part.rebuildModel());
    EXPECT_EQ(part.failedFeatureIndex(), 0);
    EXPECT_NE(part.lastBuildMessage().find("cannot be worked out"), std::string::npos)
        << part.lastBuildMessage();
    EXPECT_NE(part.lastBuildMessage().find("nosuch"), std::string::npos);

    part.featureTree().feature(0)->setParameterExpression("width", "tilt");
    EXPECT_FALSE(part.rebuildModel());
    EXPECT_NE(part.lastBuildMessage().find("an angle, not a length"), std::string::npos)
        << part.lastBuildMessage();

    part.featureTree().feature(0)->setParameterExpression("width", "wall");
    EXPECT_TRUE(part.rebuildModel()) << "worked out again: the failure is gone";
    EXPECT_NEAR(volumeOf(part), 300.0, 1e-9);
}

// Given, changed and taken away by an edit, each one step to undo.
TEST(ParameterExpressionsTest, AnEditGivesAndTakesAnExpressionAsOneStep) {
    const auto owner = boxPart({{"wall", "3 mm"}});
    Document& part = *owner;
    const hz::doc::Feature* box = part.featureTree().feature(0);
    part.undoStack().push(std::make_unique<hz::doc::EditFeatureCommand>(
        part, box, std::map<std::string, double>{{"width", 6.0}}, std::nullopt,
        std::map<std::string, hz::math::Vec3>{},
        std::map<std::string, std::string>{{"width", "(wall * 2)"}}));
    EXPECT_EQ(box->parameterExpressions().at("width"), "(wall * 2)");
    part.undoStack().push(std::make_unique<hz::doc::EditFeatureCommand>(
        part, box, std::map<std::string, double>{{"width", 12.0}}, std::nullopt,
        std::map<std::string, hz::math::Vec3>{},
        std::map<std::string, std::string>{{"width", ""}}));
    EXPECT_TRUE(box->parameterExpressions().empty()) << "its number again";
    ASSERT_TRUE(part.rebuildModel());
    EXPECT_NEAR(volumeOf(part), 1200.0, 1e-9);
    part.undoStack().undo();
    EXPECT_EQ(box->parameterExpressions().at("width"), "(wall * 2)");
    part.undoStack().undo();
    EXPECT_TRUE(box->parameterExpressions().empty());
}

// Phase 156: a configuration laid over the variables drives the part; the
// document's own come back when none is active.
TEST(ConfigurationsTest, TheActiveConfigurationDrivesThePart) {
    const auto owner = boxPart({{"wall", "3 mm"}});
    Document& part = *owner;
    part.featureTree().feature(0)->setParameterExpression("width", "(wall * 2)");
    hz::doc::ConfigurationTable table;
    table.setConfiguration("Thin", {{"wall", "1 mm"}});
    table.setConfiguration("Thick", {{"wall", "5 mm"}});
    table.setActive("Thick");
    part.undoStack().push(std::make_unique<hz::doc::SetConfigurationsCommand>(part, table));
    EXPECT_TRUE(part.needsBuild());
    ASSERT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    EXPECT_NEAR(volumeOf(part), 1000.0, 1e-9) << "wall 5 mm: width 10";
    EXPECT_DOUBLE_EQ(part.variableResolver()("wall"), 5.0) << "as constraints read it";
    EXPECT_EQ(part.parameterRegistry().definitions().at("wall"), "3 mm") << "not written";

    table.setActive("");
    part.undoStack().push(std::make_unique<hz::doc::SetConfigurationsCommand>(part, table));
    ASSERT_TRUE(part.rebuildModel());
    EXPECT_NEAR(volumeOf(part), 600.0, 1e-9) << "its own: wall 3 mm";

    part.undoStack().undo();
    ASSERT_TRUE(part.rebuildModel());
    EXPECT_EQ(part.configurations().active(), "Thick");
    EXPECT_NEAR(volumeOf(part), 1000.0, 1e-9);
}

// A configuration that cannot be worked out (it names a variable that is
// gone) gives constraints no value: one tied to that variable keeps its own,
// never 0.
TEST(ConfigurationsTest, AConstraintKeepsItsValueWhenItsVariableCannotBeWorkedOut) {
    const auto owner = boxPart({{"wall", "3 mm"}, {"length", "wall * 5"}});
    Document& part = *owner;
    hz::doc::ConfigurationTable table;
    table.setConfiguration("Long", {{"length", "gone * 5"}});
    table.setActive("Long");
    part.undoStack().push(std::make_unique<hz::doc::SetConfigurationsCommand>(part, table));

    const auto resolver = part.variableResolver();
    EXPECT_TRUE(std::isnan(resolver("length"))) << "no value, not 0";
    EXPECT_TRUE(std::isnan(resolver("nosuch")));
    EXPECT_DOUBLE_EQ(resolver("wall"), 3.0) << "what it can work out, it gives";

    hz::cstr::ConstraintSystem constraints;
    const hz::cstr::GeometryRef a{1, hz::cstr::FeatureType::Point, 0};
    const hz::cstr::GeometryRef b{1, hz::cstr::FeatureType::Point, 1};
    auto tied = std::make_shared<hz::cstr::DistanceConstraint>(a, b, 7.0);
    tied->setVariableReference("length");
    auto wall = std::make_shared<hz::cstr::DistanceConstraint>(a, b, 1.0);
    wall->setVariableReference("wall");
    constraints.addConstraint(tied);
    constraints.addConstraint(wall);
    constraints.resolveVariables(resolver);
    EXPECT_DOUBLE_EQ(tied->dimensionalValue(), 7.0) << "left as it was";
    EXPECT_DOUBLE_EQ(wall->dimensionalValue(), 3.0);
}
