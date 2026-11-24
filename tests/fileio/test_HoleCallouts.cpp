// Hole callouts on a drawing sheet: the hole's size as its feature says, how
// many the part has, drawn on a leader, saved with the sheet.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/Naming.h"

using hz::doc::Document;
using hz::io::DrawingDocumentIO;
using hz::io::DrawingDocumentSpec;
using hz::math::Vec3;

namespace {

/// A 60 x 20 x 10 plate with three counterbored holes through it, 20 apart:
/// one hole feature and a pattern of it.
struct Plate {
    Document part;
    std::string hole;
    Plate() {
        part.setType(hz::doc::DocumentType::Part);
        auto box = hz::doc::PrimitiveFeature::makeBox(60, 20, 10);
        const std::string top = box->featureID() + "/top";
        part.featureTree().addFeature(std::move(box));
        auto drill = hz::doc::HoleFeature::make(top, Vec3(10, 10, 99), 6.6, 10);
        EXPECT_TRUE(drill->setParameter("type", 1.0));    // counterbore
        EXPECT_TRUE(drill->setParameter("extent", 1.0));  // through all
        EXPECT_TRUE(drill->setParameter("boreDiameter", 11.0));
        EXPECT_TRUE(drill->setParameter("boreDepth", 6.4));
        hole = drill->featureID();
        part.featureTree().addFeature(std::move(drill));
        auto three = hz::doc::PatternFeature::makeLinear(Vec3(1, 0, 0), 20.0, 3);
        three->setTargets({hole});
        part.featureTree().addFeature(std::move(three));
        EXPECT_TRUE(part.rebuildModel()) << part.lastBuildMessage();
    }

    /// The first hole's rim at the top, as a callout names it.
    std::string rim() const {
        for (const auto& e : part.solid()->edges()) {
            const std::string name =
                hz::model::withoutPiece(hz::model::logicalEdge(e.topoId.tag()));
            if (name.rfind("edge:" + hole + "/bore|", 0) == 0) return name;
        }
        return {};
    }

    /// The standard sheet, with the top view's index.
    DrawingDocumentSpec sheet(std::size_t* top) const {
        DrawingDocumentSpec spec;
        spec.partPath = "plate.hzpart";
        spec.views = DrawingDocumentIO::viewsOf(
            hz::model::DrawingGenerator::sheetLayout(*part.solid(), spec.sheet, spec.titleBlock));
        for (std::size_t i = 0; i < spec.views.size(); ++i) {
            if (spec.views[i].kind == hz::model::StandardView::Top) *top = i;
        }
        return spec;
    }
};

std::vector<std::string> textsOn(const Document& sheet, const std::string& layer) {
    std::vector<std::string> texts;
    for (const auto& entity : sheet.draftDocument().entities()) {
        if (entity->layer() != layer) continue;
        if (const auto* t = dynamic_cast<const hz::draft::DraftText*>(entity.get())) {
            texts.push_back(t->text());
        }
    }
    return texts;
}

bool anyStarts(const std::vector<std::string>& texts, const std::string& start) {
    for (const auto& t : texts) {
        if (t.rfind(start, 0) == 0) return true;
    }
    return false;
}

}  // namespace

TEST(HoleCalloutsTest, ACalloutSaysTheHoleAndHowMany) {
    Plate plate;
    const std::string rim = plate.rim();
    ASSERT_FALSE(rim.empty());
    std::size_t top = 99;
    DrawingDocumentSpec spec = plate.sheet(&top);
    ASSERT_LT(top, spec.views.size());
    spec.views[top].holeCallouts.push_back(rim);

    const auto holes = DrawingDocumentIO::holesOf(plate.part);
    ASSERT_EQ(holes.size(), 1u);
    std::vector<std::string> lost;
    const auto drawing = DrawingDocumentIO::build(*plate.part.solid(), spec, &lost, &holes);
    EXPECT_TRUE(lost.empty());
    ASSERT_EQ(drawing.views[top].holeCallouts.size(), 1u);
    const auto& callout = drawing.views[top].holeCallouts[0];
    EXPECT_EQ(callout.count, 3) << "the hole and its pattern's two copies";
    EXPECT_EQ(callout.kind, hz::model::HoleCallout::Kind::Counterbore);
    EXPECT_EQ(callout.depth, hz::model::HoleCallout::Depth::Through);
    EXPECT_DOUBLE_EQ(callout.diameter, 6.6);
    EXPECT_DOUBLE_EQ(callout.boreDiameter, 11.0);

    Document sheet;
    hz::io::DrawingExport::populate(sheet, drawing);
    const auto texts = textsOn(sheet, "Dimensions");
    EXPECT_TRUE(anyStarts(texts,
                          "3\xC3\x97 \xE2\x8C\x80"
                          "6.6"))
        << ::testing::PrintToString(texts);
    EXPECT_TRUE(anyStarts(texts,
                          "CBORE \xE2\x8C\x80"
                          "11"))
        << ::testing::PrintToString(texts);
}

TEST(HoleCalloutsTest, ACalloutWhoseHoleIsGoneIsSaid) {
    Plate plate;
    std::size_t top = 99;
    DrawingDocumentSpec spec = plate.sheet(&top);
    spec.views[top].holeCallouts.push_back(plate.rim());
    std::vector<std::string> lost;
    // The part's holes not known (an assembly's sheet), or the hole gone.
    const auto drawing = DrawingDocumentIO::build(*plate.part.solid(), spec, &lost, nullptr);
    EXPECT_TRUE(drawing.views[top].holeCallouts.empty());
    ASSERT_EQ(lost.size(), 1u);
    EXPECT_NE(lost[0].find("hole callout"), std::string::npos);
}

TEST(HoleCalloutsTest, SavedAsVersion4OnlyWithCallouts) {
    Plate plate;
    std::size_t top = 99;
    DrawingDocumentSpec spec = plate.sheet(&top);
    const auto dir = std::filesystem::temp_directory_path();
    const std::string plain = (dir / "hz_test_callouts_none.hzdwg").string();
    const std::string called = (dir / "hz_test_callouts.hzdwg").string();
    ASSERT_TRUE(DrawingDocumentIO::save(plain, spec));
    spec.views[top].holeCallouts.push_back(plate.rim());
    ASSERT_TRUE(DrawingDocumentIO::save(called, spec));
    const auto versionOf = [](const std::string& path) {
        std::ifstream in(path);
        return nlohmann::json::parse(in).at("version").get<int>();
    };
    EXPECT_EQ(versionOf(plain), 3) << "still read by a version that reads 3";
    EXPECT_EQ(versionOf(called), 4);

    DrawingDocumentSpec back;
    std::string why;
    ASSERT_TRUE(DrawingDocumentIO::readSpec(called, back, &why)) << why;
    ASSERT_EQ(back.views.size(), spec.views.size());
    EXPECT_EQ(back.views[top].holeCallouts, spec.views[top].holeCallouts);
    std::filesystem::remove(plain);
    std::filesystem::remove(called);
}

TEST(HoleCalloutsTest, AnEdgeIsTheHoleThatNamesIt) {
    std::vector<hz::io::HoleNote> holes(2);
    holes[0].feature = "hole_1";
    holes[1].feature = "hole_11";
    EXPECT_EQ(DrawingDocumentIO::holeOfEdge(holes, "edge:hole_11/wall|primitive_1/top"), &holes[1]);
    EXPECT_EQ(DrawingDocumentIO::holeOfEdge(holes, "edge:hole_1/bore|primitive_1/top"), &holes[0]);
    EXPECT_EQ(DrawingDocumentIO::holeOfEdge(holes, "hole_1/edge:bore|boreFloor"), &holes[0]);
    EXPECT_EQ(DrawingDocumentIO::holeOfEdge(holes, "edge:primitive_1/top|xhole_1/wall"), nullptr);
    EXPECT_EQ(DrawingDocumentIO::holeOfEdge(holes, "edge:primitive_1/top|primitive_1/front"),
              nullptr);
}
