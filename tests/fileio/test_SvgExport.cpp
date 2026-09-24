// A drawing plotted to SVG (Phase 127): the page in millimetres, each stroke a
// polyline with its weight and dashes, text escaped, numbers in the C locale.

#include <gtest/gtest.h>

#include <clocale>
#include <string>

#include "horizon/drafting/PlotScene.h"
#include "horizon/fileio/SvgExport.h"

using hz::draft::PlotScene;
using hz::math::Vec2;

namespace {

PlotScene scene() {
    PlotScene s;
    s.strokes.push_back({{Vec2(0, 0), Vec2(10, 0)}, false, 0xFFFFFFFF, 1.0, 1});
    s.strokes.push_back({{Vec2(0, 5), Vec2(10, 5), Vec2(10, 8)}, true, 0xFF3366CC, 0.5, 2});
    s.texts.push_back(
        {Vec2(5, 2.5), "A & <B>", 2.5, 0.0, hz::draft::TextAlignment::Center, 0xFF000000});
    s.bounds.expand(hz::math::Vec3(0, 0, 0));
    s.bounds.expand(hz::math::Vec3(10, 8, 0));
    return s;
}

int count(const std::string& text, const std::string& part) {
    int n = 0;
    for (size_t at = text.find(part); at != std::string::npos; at = text.find(part, at + 1)) ++n;
    return n;
}

}  // namespace

TEST(SvgExportTest, APlotIsAPageOfStrokesAndText) {
    hz::draft::PlotLayout layout;  // A4 landscape
    layout.scale = 1.0;            // 1:1
    const std::string svg = hz::io::SvgExport::toString(scene(), layout);
    EXPECT_NE(svg.find("width=\"297mm\" height=\"210mm\" viewBox=\"0 0 297 210\""),
              std::string::npos);
    EXPECT_EQ(count(svg, "<polyline "), 1);
    EXPECT_EQ(count(svg, "<polygon "), 1);
    // The first line, 10 long at 1:1, centred: from x 143.5 to 153.5, at y
    // 105 + 4 (the drawing's centre is 4 above it).
    EXPECT_NE(svg.find("points=\"143.5,109 153.5,109\""), std::string::npos) << svg;
    EXPECT_NE(svg.find("stroke=\"#000000\" stroke-width=\"0.25\""), std::string::npos)
        << "white plots black, the default width at 0.25 mm";
    EXPECT_NE(svg.find("stroke=\"#3366cc\" stroke-width=\"0.5\" stroke-dasharray=\"5 3\""),
              std::string::npos);
    EXPECT_NE(svg.find(">A &amp; &lt;B&gt;</text>"), std::string::npos) << "escaped";
    EXPECT_NE(svg.find("text-anchor=\"middle\""), std::string::npos);
}

TEST(SvgExportTest, MonochromeIsAllBlack) {
    hz::draft::PlotLayout layout;
    layout.monochrome = true;
    const std::string svg = hz::io::SvgExport::toString(scene(), layout);
    EXPECT_EQ(svg.find("#3366cc"), std::string::npos);
}

// Numbers are written with a point whatever the locale: a comma would make
// "143,5,109" of a coordinate pair.
TEST(SvgExportTest, NumbersIgnoreACommaDecimalLocale) {
    const char* previous = std::setlocale(LC_NUMERIC, nullptr);
    const std::string saved = previous ? previous : "C";
    if (!std::setlocale(LC_NUMERIC, "de_DE.UTF-8") && !std::setlocale(LC_NUMERIC, "de_DE")) {
        GTEST_SKIP() << "no de_DE locale here";
    }
    hz::draft::PlotLayout layout;
    layout.scale = 1.0;
    const std::string svg = hz::io::SvgExport::toString(scene(), layout);
    std::setlocale(LC_NUMERIC, saved.c_str());
    EXPECT_NE(svg.find("points=\"143.5,109 153.5,109\""), std::string::npos);
}
