// Phase 155: a feature's parameter expressions saved beside their values,
// and built from a snapshot, as a worker builds.

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/MassProperties.h"

using hz::doc::Document;
using hz::doc::PrimitiveFeature;

namespace {

std::unique_ptr<Document> boxPart(const std::map<std::string, std::string>& variables) {
    auto part = std::make_unique<Document>();
    part->setType(hz::doc::DocumentType::Part);
    part->parameterRegistry().setDefinitions(variables);
    part->featureTree().addFeature(PrimitiveFeature::makeBox(10, 10, 10));
    return part;
}

}  // namespace

// Saved beside the value it last worked out to, and read back; a build from
// a snapshot (a worker's) works it out as well.
TEST(ExpressionsIOTest, AnExpressionIsSavedAndBuiltFromASnapshot) {
    const auto owner = boxPart({{"wall", "4 mm"}});
    Document& part = *owner;
    part.featureTree().feature(0)->setParameterExpression("depth", "(wall * 2)");
    ASSERT_TRUE(part.rebuildModel());
    const std::string json = hz::io::NativeFormat::documentToJson(part, false);
    Document copy;
    ASSERT_TRUE(hz::io::NativeFormat::documentFromJson(json, copy));
    ASSERT_EQ(copy.featureTree().featureCount(), 1u);
    EXPECT_EQ(copy.featureTree().feature(0)->parameterExpressions().at("depth"), "(wall * 2)");
    EXPECT_DOUBLE_EQ(copy.featureTree().feature(0)->parameters().at("depth"), 8.0)
        << "the value it worked out to, for a build that reads no expressions";
    copy.parameterRegistry().setDefinitions({{"wall", "5 mm"}});
    const auto result = copy.buildWithDiagnostics();
    ASSERT_NE(result.solid, nullptr) << result.failureMessage;
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*result.solid).volume, 1000.0, 1e-9);
}
