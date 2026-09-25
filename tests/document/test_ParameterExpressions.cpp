// Phase 155: a feature's parameters as expressions of the document's
// variables, worked out each time the part is built.

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>

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
