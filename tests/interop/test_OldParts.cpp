// Part files saved before the exact kernel (stage 2, Ruling 10): each holds its imported bodies as
// STEP the writer of its day wrote, which no exact form may hold. Such a part opens as it did: a
// body whose exact form fails is kept as the file has it, untouched by the exact pass, and builds
// as it built before.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::doc::Document;
using hz::doc::DocumentType;
using hz::doc::ImportedBodyFeature;
using hz::io::NativeFormat;
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

/// The old part bodies' folder (a function, not a static path: constructing one can throw, and
/// nothing would catch it before main).
fs::path oldPartDir() {
    return fs::path(HZ_OLD_PART_DIR);
}

std::string readText(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

/// A part file holding one imported body from @p source whose STEP is @p step, as a part saved
/// before stage 2 holds it: a part saved today, its body's text replaced by the old one.
std::string oldPartFile(const std::string& step, const std::string& source) {
    Document part;
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(std::make_unique<ImportedBodyFeature>(
        std::shared_ptr<const hz::topo::Solid>(hz::model::PrimitiveFactory::makeBox(1, 1, 1)),
        source));
    json root = json::parse(NativeFormat::documentToJson(part, false));
    for (json& feature : root.at("featureTree")) {
        if (feature.value("type", "") == "imported") feature["step"] = step;
    }
    return root.dump();
}

}  // namespace

// The corpus's Solid Edge part (tests/interop/step/solidedge_ercf_knife_holder.step), as a part
// file saved before stage 2 holds it: written by master's writer (54f72335), each face a B-spline
// patch. Its exact pass inserts a seam on one band and splits two rims, then fails; the body is
// kept as the file has it, not as the failed pass left it: its 90 edges, none a seam or a piece of
// a split rim, and it builds as on master, 3868 faces enclosing 684.444.
TEST(OldPartFile, ABodyKeptAsItWasIsTheFilesUntouchedByTheExactPass) {
    const std::string step = readText(oldPartDir() / "solidedge_ercf_knife_holder.body.step");
    ASSERT_FALSE(step.empty());

    Document opened;
    std::string error;
    hz::io::ImportReport report;
    ASSERT_TRUE(NativeFormat::documentFromJson(
        oldPartFile(step, "solidedge_ercf_knife_holder.step"), opened, &error, &report))
        << error;
    ASSERT_EQ(opened.featureTree().featureCount(), 1u);
    const auto* imported =
        dynamic_cast<const ImportedBodyFeature*>(opened.featureTree().feature(0));
    ASSERT_NE(imported, nullptr);
    ASSERT_NE(imported->solid(), nullptr);
    EXPECT_NE(imported->note().find("; kept as it was"), std::string::npos) << imported->note();

    const hz::topo::Solid& body = *imported->solid();
    EXPECT_EQ(body.edgeCount(), 90u) << "the file's edges, as it has them";
    for (const auto& e : body.edges()) {
        const std::string name = e.topoId.tag();
        EXPECT_EQ(name.find("/seam"), std::string::npos) << name;
        EXPECT_EQ(name.find("/piece:"), std::string::npos) << name;
    }

    ASSERT_TRUE(opened.rebuildModel()) << opened.lastBuildMessage();
    EXPECT_EQ(opened.failedFeatureIndex(), -1) << opened.lastBuildMessage();
    ASSERT_NE(opened.solid(), nullptr);
    EXPECT_EQ(opened.solid()->faceCount(), 3868u);
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*opened.solid()).volume, 684.444,
                5e-4);
    EXPECT_EQ(opened.exactSolid(), nullptr) << "no exact form holds it";
}
