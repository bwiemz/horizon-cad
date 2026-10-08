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

// Text from a DXF can hold what XML has no character for: \U+0001, a raw
// 0x0B or 0x0C, U+FFFE, a byte that is not UTF-8. Each made the file not
// well-formed, and a browser showed none of it. A control character is left
// out, and the rest is U+FFFD; a tab, a line break and every real character
// stay as they were.
TEST(SvgExportTest, TextHoldsOnlyWhatXmlHasCharactersFor) {
    PlotScene s = scene();
    s.texts.front().text = std::string(
                               "a\x01"
                               "b\x0B\x0C"
                               "c\t"
                               "d\x7F") +
                           "\xEF\xBF\xBE"                   // U+FFFE
                           "\xFF"                           // not UTF-8
                           "\xED\xA0\x80"                   // a surrogate, U+D800
                           "\xE5\x9B\xB3\xF0\x9F\x93\x90";  // "図📐"
    const std::string svg = hz::io::SvgExport::toString(s, hz::draft::PlotLayout{});
    const std::string replaced = "\xEF\xBF\xBD";
    EXPECT_NE(svg.find(">abc\td\x7F" + replaced + replaced + replaced + replaced + replaced +
                       "\xE5\x9B\xB3\xF0\x9F\x93\x90</text>"),
              std::string::npos)
        << svg;
    for (const char c : svg) {
        const auto byte = static_cast<unsigned char>(c);
        EXPECT_TRUE(byte >= 0x20 || c == '\t' || c == '\n' || c == '\r') << int{byte};
    }
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
