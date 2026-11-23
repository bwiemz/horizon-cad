// A base flange's flat pattern: the blank a shop cuts, its bends marked, at
// full size in a DXF.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/FlatPattern.h"
#include "horizon/modeling/SheetMetal.h"

using hz::doc::SheetMetalFeature;
using hz::math::Vec2;

namespace {

constexpr double kPi = 3.14159265358979323846;

std::shared_ptr<hz::doc::Sketch> zSketch() {
    auto sketch = std::make_shared<hz::doc::Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(40, 0)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(40, 0), Vec2(40, 25)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(40, 25), Vec2(70, 25)));
    return sketch;
}

}  // namespace

// A Z of two bends, one each way: the blank is the developed length by the
// width, and each bend is marked where its allowance is centred, up or down.
TEST(FlatPatternTest, AZUnfoldsToItsBlank) {
    SheetMetalFeature flange(zSketch(), 2.0, 1.0, 30.0);
    std::string why;
    const auto pattern = hz::io::flatPatternOf(flange, &why);
    ASSERT_TRUE(pattern.has_value()) << why;
    const hz::model::SheetMetalParams p = flange.params();
    const double ba = hz::model::bendAllowance(kPi / 2, p);
    // Line 1 toward the material bends up, so its surface is the outside
    // (setback 3); the second bends down (setback 1).
    const double flat1 = 40.0 - 3.0;
    const double flat2 = 25.0 - 3.0 - 1.0;
    const double flat3 = 30.0 - 1.0;
    EXPECT_NEAR(pattern->length, flat1 + flat2 + flat3 + 2.0 * ba, 1e-9);
    EXPECT_DOUBLE_EQ(pattern->width, 30.0);
    EXPECT_DOUBLE_EQ(pattern->thickness, 2.0);
    ASSERT_EQ(pattern->bends.size(), 2u);
    EXPECT_NEAR(pattern->bends[0].at, flat1 + ba / 2.0, 1e-9);
    EXPECT_NEAR(pattern->bends[0].degrees, 90.0, 1e-9);
    EXPECT_TRUE(pattern->bends[0].up);
    EXPECT_NEAR(pattern->bends[1].at, flat1 + ba + flat2 + ba / 2.0, 1e-9);
    EXPECT_FALSE(pattern->bends[1].up);
}

// Drawn and written: the outline and the bend lines on their layers, with
// their notes, and read back from the DXF at full size.
TEST(FlatPatternTest, ItsDxfHasTheOutlineAndBendLines) {
    SheetMetalFeature flange(zSketch(), 2.0, 1.0, 30.0);
    const auto pattern = hz::io::flatPatternOf(flange, nullptr);
    ASSERT_TRUE(pattern.has_value());
    const std::string path =
        (std::filesystem::temp_directory_path() / "hz_test_flat_pattern.dxf").string();
    std::string why;
    ASSERT_TRUE(hz::io::saveFlatPatternDxf(path, *pattern, &why)) << why;

    hz::doc::Document read;
    ASSERT_TRUE(hz::io::DxfFormat::load(path, read));
    int outline = 0;
    int bendLines = 0;
    std::vector<std::string> notes;
    double right = 0.0;
    for (const auto& entity : read.draftDocument().entities()) {
        if (const auto* line = dynamic_cast<const hz::draft::DraftLine*>(entity.get())) {
            if (entity->layer() == "Flat Pattern") ++outline;
            if (entity->layer() == "Bend Lines") ++bendLines;
            right = std::max({right, line->start().x, line->end().x});
        } else if (const auto* text = dynamic_cast<const hz::draft::DraftText*>(entity.get())) {
            notes.push_back(text->text());
        }
    }
    EXPECT_EQ(outline, 4);
    EXPECT_EQ(bendLines, 2);
    EXPECT_NEAR(right, pattern->length, 1e-6) << "at full size";
    ASSERT_EQ(notes.size(), 2u);
    EXPECT_EQ(notes[0], "UP 90\xC2\xB0");
    EXPECT_EQ(notes[1], "DOWN 90\xC2\xB0");
    std::filesystem::remove(path);
}

// A flange that does not fold has no flat pattern, and says why.
TEST(FlatPatternTest, NoneForAFlangeThatDoesNotFold) {
    auto closed = zSketch();
    closed->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(70, 25), Vec2(0, 0)));
    SheetMetalFeature flange(closed, 2.0, 1.0, 30.0);
    std::string why;
    EXPECT_FALSE(hz::io::flatPatternOf(flange, &why).has_value());
    EXPECT_NE(why.find("closed"), std::string::npos) << why;
}
