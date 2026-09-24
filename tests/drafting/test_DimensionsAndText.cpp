// Dimension units and multi-line text (Phase 129): a style shows lengths in
// its unit, and a text's lines are laid out, bounded and plotted one below
// the other.

#include <gtest/gtest.h>

#include <cmath>
#include <locale>
#include <memory>
#include <stdexcept>
#include <string>

#include "horizon/drafting/DimensionStyle.h"
#include "horizon/drafting/DraftAngularDimension.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/drafting/DraftRadialDimension.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/drafting/PlotScene.h"
#include "horizon/math/Constants.h"

using hz::draft::DimensionStyle;
using hz::draft::DraftLinearDimension;
using hz::draft::DraftText;
using hz::math::Vec2;

namespace {

bool near(const Vec2& a, const Vec2& b, double tol = 1e-9) {
    return (a - b).length() <= tol;
}

DimensionStyle styleIn(const std::string& unit, bool showUnits, int precision = 2) {
    DimensionStyle s;
    s.unit = unit;
    s.showUnits = showUnits;
    s.precision = precision;
    return s;
}

}  // namespace

TEST(DimensionUnitsTest, ALengthIsShownInTheStylesUnit) {
    EXPECT_EQ(styleIn("mm", false).formatLength(25.4), "25.40");
    EXPECT_EQ(styleIn("mm", true).formatLength(25.4), "25.40 mm");
    EXPECT_EQ(styleIn("cm", true, 1).formatLength(25.0), "2.5 cm");
    EXPECT_EQ(styleIn("m", true, 3).formatLength(1250.0), "1.250 m");
    EXPECT_EQ(styleIn("in", false).formatLength(25.4), "1.00");
    EXPECT_EQ(styleIn("in", true).formatLength(50.8), "2.00\"") << "inches by their mark";
    EXPECT_EQ(styleIn("ft", true, 1).formatLength(457.2), "1.5'") << "feet by theirs";
    EXPECT_EQ(styleIn("mm", false, 0).formatLength(12.5), "12");
}

// A unit the style does not know (from a hand-edited file, say) is millimetres,
// and says so, rather than printing a millimetre value as something else.
TEST(DimensionUnitsTest, AnUnknownUnitIsMillimetres) {
    EXPECT_FALSE(hz::draft::isDimensionUnit("furlong"));
    EXPECT_DOUBLE_EQ(hz::draft::millimetresPerUnit("furlong"), 1.0);
    EXPECT_EQ(styleIn("furlong", true).formatLength(10.0), "10.00 mm");
    for (const char* unit : {"mm", "cm", "m", "in", "ft"}) {
        EXPECT_TRUE(hz::draft::isDimensionUnit(unit)) << unit;
    }
}

// Whatever the program's locale, a dimension is written with a point: the
// value is what the drawing says, not how the user's desktop writes numbers.
TEST(DimensionUnitsTest, TheValueIsWrittenTheSameInEveryLocale) {
    std::locale comma;
    try {
        comma = std::locale("de_DE.UTF-8");
    } catch (const std::runtime_error&) {
        GTEST_SKIP() << "no de_DE locale here";
    }
    const hz::draft::DraftAngularDimension angle(Vec2(0, 0), Vec2(10, 0), Vec2(0, 10), 5.0);
    const std::locale previous = std::locale::global(comma);
    const std::string length = styleIn("mm", false).formatLength(25.4);
    const std::string degrees = angle.displayText(DimensionStyle{});
    std::locale::global(previous);
    EXPECT_EQ(length, "25.40");
    EXPECT_EQ(degrees, "90.00\xC2\xB0");
}

TEST(DimensionUnitsTest, DimensionsShowTheirValueInTheUnit) {
    const DraftLinearDimension inch(Vec2(0, 0), Vec2(25.4, 0), Vec2(12.7, 10),
                                    DraftLinearDimension::Orientation::Horizontal);
    EXPECT_EQ(inch.displayText(styleIn("in", true, 3)), "1.000\"");
    EXPECT_EQ(inch.displayText(DimensionStyle{}), "25.40") << "millimetres, as before";

    const hz::draft::DraftRadialDimension diameter(Vec2(0, 0), 10.0, Vec2(20, 0), true);
    EXPECT_EQ(diameter.displayText(styleIn("cm", true)),
              "\xE2\x8C\x80"
              "2.00 cm");
    const hz::draft::DraftRadialDimension radius(Vec2(0, 0), 10.0, Vec2(20, 0), false);
    EXPECT_EQ(radius.displayText(styleIn("cm", false)), "R1.00");

    DraftLinearDimension overridden = inch;
    overridden.setTextOverride("TYP");
    EXPECT_EQ(overridden.displayText(styleIn("in", true)), "TYP") << "an override is as typed";
}

// An angle is not a length: a unit does not change it.
TEST(DimensionUnitsTest, AnglesAreNotConverted) {
    const hz::draft::DraftAngularDimension angle(Vec2(0, 0), Vec2(10, 0), Vec2(0, 10), 5.0);
    const std::string mm = angle.displayText(DimensionStyle{});
    EXPECT_EQ(angle.displayText(styleIn("in", true)), mm);
    EXPECT_NE(mm.find("90"), std::string::npos) << mm;
}

TEST(MultiLineTextTest, TextSplitsIntoItsLines) {
    EXPECT_EQ(DraftText(Vec2(0, 0), "one").lines(), std::vector<std::string>{"one"});
    EXPECT_EQ(DraftText(Vec2(0, 0), "a\nbb\nccc").lines(),
              (std::vector<std::string>{"a", "bb", "ccc"}));
    EXPECT_EQ(DraftText(Vec2(0, 0), "a\n\nb").lines(), (std::vector<std::string>{"a", "", "b"}))
        << "an empty line keeps its place";
    EXPECT_EQ(DraftText(Vec2(0, 0), "").lines(), std::vector<std::string>{""});
}

TEST(MultiLineTextTest, LinesGoDownThePageTurnedWithTheText) {
    DraftText text(Vec2(10, 20), "a\nb\nc", 3.0);
    const double pitch = DraftText::kLinePitch * 3.0;
    EXPECT_TRUE(near(text.lineBaseline(0), Vec2(10, 20)));
    EXPECT_TRUE(near(text.lineBaseline(2), Vec2(10, 20 - 2 * pitch)));

    // Turned a quarter anticlockwise, the text reads upwards and its lines go
    // to the right.
    text.setRotation(hz::math::kPi / 2);
    EXPECT_TRUE(near(text.lineBaseline(1), Vec2(10 + pitch, 20)));
}

// The bounds cover every line, down to the last one's descenders, and as wide
// as the widest line: a box select or a zoom-to-fit takes it all.
TEST(MultiLineTextTest, TheBoundsCoverEveryLine) {
    const DraftText one(Vec2(0, 0), "abcdef", 2.0);
    const DraftText three(Vec2(0, 0), "ab\nabcdef\nc", 2.0);
    const auto b1 = one.boundingBox();
    const auto b3 = three.boundingBox();
    EXPECT_NEAR(b3.max().x - b3.min().x, b1.max().x - b1.min().x, 1e-9) << "the widest line's";
    EXPECT_NEAR(b3.max().y, b1.max().y, 1e-9);
    EXPECT_NEAR(b3.min().y, b1.min().y - 2 * DraftText::kLinePitch * 2.0, 1e-9);

    const Vec2 onLastLine = three.lineBaseline(2) + Vec2(0.5, 0.5);
    EXPECT_TRUE(three.hitTest(onLastLine, 0.1)) << "the last line can be picked";
    EXPECT_FALSE(one.hitTest(onLastLine, 0.1));
}

TEST(MultiLineTextTest, EachLinePlotsAtItsBaseline) {
    hz::draft::DraftDocument drawing;
    hz::draft::LayerManager layers;
    auto text = std::make_shared<DraftText>(Vec2(5, 5), "first\nsecond", 2.0);
    text->setRotation(0.3);
    drawing.addEntity(text);
    const auto scene = hz::draft::buildPlotScene(drawing, layers, DimensionStyle{});
    ASSERT_EQ(scene.texts.size(), 2u);
    EXPECT_EQ(scene.texts[0].text, "first");
    EXPECT_EQ(scene.texts[1].text, "second");
    EXPECT_TRUE(near(scene.texts[0].position, text->lineBaseline(0)));
    EXPECT_TRUE(near(scene.texts[1].position, text->lineBaseline(1)));
    EXPECT_NEAR(scene.texts[1].rotation, 0.3, 1e-12);
    EXPECT_LT(scene.bounds.min().y, text->lineBaseline(1).y) << "the second line is in bounds";
}
