// DXF text and metadata read as the file meant them, and written so they
// read back (Phase 108b): MTEXT chunks and codes, %% codes, \U+ escapes,
// TEXT alignment, the full colour index and true colour, $DWGCODEPAGE,
// $INSUNITS, and values that could break the file. Fixtures are written to
// the DXF reference's group codes.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/math/Constants.h"

using hz::draft::DraftText;
using hz::draft::TextAlignment;
using hz::io::ImportReport;
using hz::math::Vec2;

namespace {

/// A DXF file with `header` groups in its HEADER, `tables` in TABLES,
/// `blocks` in BLOCKS and `entities` in ENTITIES.
std::string dxf(const std::string& entities, const std::string& header = "",
                const std::string& blocks = "", const std::string& tables = "") {
    return "0\nSECTION\n2\nHEADER\n" + header + "0\nENDSEC\n0\nSECTION\n2\nTABLES\n" + tables +
           "0\nENDSEC\n0\nSECTION\n2\nBLOCKS\n" + blocks + "0\nENDSEC\n0\nSECTION\n2\nENTITIES\n" +
           entities + "0\nENDSEC\n0\nEOF\n";
}

/// A TEXT entity with these groups after its layer.
std::string text(const std::string& groups) {
    return "0\nTEXT\n8\n0\n" + groups;
}

/// An MTEXT entity at (x, y), height h, with these further groups.
std::string mtext(double x, double y, double h, const std::string& groups) {
    std::ostringstream out;
    out << "0\nMTEXT\n8\n0\n10\n" << x << "\n20\n" << y << "\n40\n" << h << "\n" << groups;
    return out.str();
}

struct Loaded {
    hz::doc::Document doc;
    ImportReport report;
    std::string error;
    bool ok = false;
    explicit Loaded(const std::string& file) {
        ok = hz::io::DxfFormat::loadFromString(file, doc, &error, &report);
    }
    std::vector<const DraftText*> texts() const {
        std::vector<const DraftText*> out;
        for (const auto& e : doc.draftDocument().entities()) {
            if (const auto* t = dynamic_cast<const DraftText*>(e.get())) out.push_back(t);
        }
        return out;
    }
    uint32_t firstColor() const { return doc.draftDocument().entities().at(0)->color(); }
};

/// What DxfFormat::save writes for `doc`.
std::string saved(const hz::doc::Document& doc) {
    std::random_device rd;
    const auto path =
        std::filesystem::temp_directory_path() / ("hz_dxftext_" + std::to_string(rd()) + ".dxf");
    std::string error;
    EXPECT_TRUE(hz::io::DxfFormat::save(path.string(), doc, &error)) << error;
    std::ifstream in(path, std::ios::binary);
    std::stringstream text;
    text << in.rdbuf();
    in.close();
    std::filesystem::remove(path);
    return text.str();
}

bool near(const Vec2& a, const Vec2& b, double tol = 1e-9) {
    return (a - b).length() <= tol;
}

bool contains(const std::vector<std::string>& lines, const std::string& part) {
    for (const auto& line : lines) {
        if (line.find(part) != std::string::npos) return true;
    }
    return false;
}

const char* const kDegree = "\xC2\xB0";
const char* const kPlusMinus = "\xC2\xB1";
const char* const kDiameter = "\xE2\x8C\x80";

}  // namespace

// ---------------------------------------------------------------------------
// MTEXT
// ---------------------------------------------------------------------------

TEST(DxfTextTest, MtextChunksComeInTheOrderWritten) {
    // Chunks (3) come first, in order, and the last part (1) ends the text.
    // They used to be put together back to front.
    Loaded in(dxf(mtext(0, 0, 2, "3\nAAA\n3\nBBB\n1\nCCC\n")));
    ASSERT_TRUE(in.ok) << in.error;
    const auto texts = in.texts();
    ASSERT_EQ(texts.size(), 1u);
    EXPECT_EQ(texts[0]->text(), "AAABBBCCC");
}

TEST(DxfTextTest, MtextParagraphsBecomeOneTextOfSeveralLines) {
    // Top left (71 = 1): the first line hangs below the point, and each
    // next line is 5/3 of the height lower, as a text's own lines are.
    Loaded in(dxf(mtext(10, 20, 2, "71\n1\n1\nFirst\\PSecond\n")));
    ASSERT_TRUE(in.ok) << in.error;
    const auto texts = in.texts();
    ASSERT_EQ(texts.size(), 1u);
    EXPECT_EQ(texts[0]->text(), "First\nSecond");
    EXPECT_TRUE(near(texts[0]->lineBaseline(0), Vec2(10, 18)));
    EXPECT_TRUE(near(texts[0]->lineBaseline(1), Vec2(10, 18 - 10.0 / 3.0)));
    EXPECT_EQ(texts[0]->alignment(), TextAlignment::Left);
    EXPECT_TRUE(in.report.approximated.empty()) << "nothing about it was changed";
}

// Blank lines at either end are not part of the text; the first line with
// something on it is where it starts. Blank lines between are kept.
TEST(DxfTextTest, MtextBlankLinesAtTheEndsAreDropped) {
    Loaded in(dxf(mtext(0, 0, 3, "71\n1\n1\n\\P  \\PA\\P\\PB\\P\n")));
    ASSERT_TRUE(in.ok) << in.error;
    const auto texts = in.texts();
    ASSERT_EQ(texts.size(), 1u);
    EXPECT_EQ(texts[0]->text(), "A\n\nB");
    EXPECT_TRUE(near(texts[0]->position(), Vec2(0, -3 - 2 * 5.0)));
}

// Lines spaced other than a text's own are placed where the file has them,
// one text each, grouped, and the report says so.
TEST(DxfTextTest, MtextWithOtherSpacingBecomesOneTextPerLine) {
    Loaded in(dxf(mtext(10, 20, 2, "71\n1\n44\n1.5\n1\nFirst\\PSecond\n")));
    ASSERT_TRUE(in.ok) << in.error;
    const auto texts = in.texts();
    ASSERT_EQ(texts.size(), 2u);
    EXPECT_EQ(texts[0]->text(), "First");
    EXPECT_EQ(texts[1]->text(), "Second");
    EXPECT_TRUE(near(texts[0]->position(), Vec2(10, 18)));
    EXPECT_TRUE(near(texts[1]->position(), Vec2(10, 18 - 5.0)));
    EXPECT_NE(texts[0]->groupId(), 0u) << "the lines stay together";
    EXPECT_EQ(texts[0]->groupId(), texts[1]->groupId());
    EXPECT_TRUE(contains(in.report.approximated, "1 MTEXT entity: lines spaced other than usual"));
}

TEST(DxfTextTest, MtextFormattingCodesAreReadNotShown) {
    Loaded in(dxf(mtext(0, 0, 2,
                        "1\n{\\fArial|b1|i0;Bold} \\H2.5x;big\\~and \\\\ \\{x\\}"
                        "\\pxi-3,l3;para \\S1/2;%%d\\U+00B1\n")));
    ASSERT_TRUE(in.ok) << in.error;
    const auto texts = in.texts();
    ASSERT_EQ(texts.size(), 1u);
    EXPECT_EQ(texts[0]->text(),
              std::string("Bold big\xC2\xA0") + "and \\ {x}para 1/2" + kDegree + kPlusMinus);
    EXPECT_TRUE(contains(in.report.approximated, "stacked fractions written inline"));
    EXPECT_TRUE(contains(in.report.approximated, "colour or size changes inside it not kept"));
}

TEST(DxfTextTest, MtextAttachmentPointPlacesItsLines) {
    // Middle centre (5), one line: centred on the point both ways.
    Loaded middle(dxf(mtext(0, 0, 2, "71\n5\n1\nOne\n")));
    ASSERT_TRUE(middle.ok) << middle.error;
    ASSERT_EQ(middle.texts().size(), 1u);
    EXPECT_TRUE(near(middle.texts()[0]->position(), Vec2(0, -1)));
    EXPECT_EQ(middle.texts()[0]->alignment(), TextAlignment::Center);

    // Bottom right (9), two lines: the last one stands on the point.
    Loaded bottom(dxf(mtext(0, 0, 3, "71\n9\n1\nUp\\PDown\n")));
    ASSERT_TRUE(bottom.ok) << bottom.error;
    const auto texts = bottom.texts();
    ASSERT_EQ(texts.size(), 1u);
    EXPECT_TRUE(near(texts[0]->lineBaseline(0), Vec2(0, 5)));
    EXPECT_TRUE(near(texts[0]->lineBaseline(1), Vec2(0, 0)));
    EXPECT_EQ(texts[0]->alignment(), TextAlignment::Right);
}

TEST(DxfTextTest, MtextTakesItsDirectionOrItsAngleInDegrees) {
    Loaded byDirection(dxf(mtext(0, 0, 2, "11\n0\n21\n1\n1\nUp\n")));
    ASSERT_TRUE(byDirection.ok) << byDirection.error;
    EXPECT_NEAR(byDirection.texts().at(0)->rotation(), hz::math::kPi / 2.0, 1e-12);

    Loaded byAngle(dxf(mtext(0, 0, 2, "50\n90\n1\nUp\n")));
    ASSERT_TRUE(byAngle.ok) << byAngle.error;
    EXPECT_NEAR(byAngle.texts().at(0)->rotation(), hz::math::kPi / 2.0, 1e-12);
}

TEST(DxfTextTest, AnEmptyMtextIsReportedNotAddedBlank) {
    Loaded in(dxf(mtext(0, 0, 2, "1\n{\\fArial;}\n")));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_TRUE(in.texts().empty());
    EXPECT_TRUE(contains(in.report.skipped, "1 MTEXT entity (it has no text) not read"));
}

// ---------------------------------------------------------------------------
// TEXT
// ---------------------------------------------------------------------------

TEST(DxfTextTest, PercentCodesBecomeTheirSigns) {
    Loaded in(dxf(text("10\n0\n20\n0\n40\n1\n1\n45%%d %%p0.1 %%C10 100%%% %%176\n")));
    ASSERT_TRUE(in.ok) << in.error;
    ASSERT_EQ(in.texts().size(), 1u);
    EXPECT_EQ(in.texts()[0]->text(), std::string("45") + kDegree + " " + kPlusMinus + "0.1 " +
                                         kDiameter + "10 100% " + kDegree);
    EXPECT_TRUE(in.report.approximated.empty());

    Loaded underlined(dxf(text("10\n0\n20\n0\n40\n1\n1\n%%uNote%%u\n")));
    ASSERT_TRUE(underlined.ok) << underlined.error;
    EXPECT_EQ(underlined.texts().at(0)->text(), "Note");
    EXPECT_TRUE(contains(underlined.report.approximated, "1 TEXT entity: underline"));
}

TEST(DxfTextTest, UnicodeEscapesAndCaretCodesAreRead) {
    Loaded in(dxf(text("10\n0\n20\n0\n40\n1\n1\n\\U+00E9t\\U+00E9 a^ b^Ic 2^N\n")));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_EQ(in.texts().at(0)->text(), "\xC3\xA9t\xC3\xA9 a^b\tc 2^N")
        << "a caret before anything but a space, I, J or M is a caret";
}

TEST(DxfTextTest, TextKeepsItsSpaces) {
    Loaded in(dxf(text("10\n0\n20\n0\n40\n1\n1\n  indented\r\n")));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_EQ(in.texts().at(0)->text(), "  indented");
}

TEST(DxfTextTest, AlignedTextStandsAtItsSecondPoint) {
    // Centred (72 = 1): the second point is where it stands. The first point
    // is AutoCAD's own computation, and other writers leave it at 0, 0.
    Loaded centred(dxf(text("10\n0\n20\n0\n11\n5\n21\n5\n40\n1\n72\n1\n1\nMid\n")));
    ASSERT_TRUE(centred.ok) << centred.error;
    EXPECT_TRUE(near(centred.texts().at(0)->position(), Vec2(5, 5)));
    EXPECT_EQ(centred.texts().at(0)->alignment(), TextAlignment::Center);

    // Top right (72 = 2, 73 = 3), height 2: the baseline is 2 below the point.
    Loaded topRight(dxf(text("10\n0\n20\n0\n11\n10\n21\n10\n40\n2\n72\n2\n73\n3\n1\nTR\n")));
    ASSERT_TRUE(topRight.ok) << topRight.error;
    EXPECT_TRUE(near(topRight.texts().at(0)->position(), Vec2(10, 8)));
    EXPECT_EQ(topRight.texts().at(0)->alignment(), TextAlignment::Right);

    // Left on the baseline: the second point, if written, means nothing.
    Loaded left(dxf(text("10\n3\n20\n4\n11\n0\n21\n0\n40\n1\n1\nL\n")));
    ASSERT_TRUE(left.ok) << left.error;
    EXPECT_TRUE(near(left.texts().at(0)->position(), Vec2(3, 4)));
}

// ---------------------------------------------------------------------------
// Code pages
// ---------------------------------------------------------------------------

TEST(DxfTextTest, CodePageBytesBecomeUtf8) {
    const std::string western = "9\n$DWGCODEPAGE\n3\nANSI_1252\n";
    Loaded in(dxf("0\nTEXT\n8\n\xE9tage\n10\n0\n20\n0\n40\n1\n1\nCaf\xE9\n", western));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_EQ(in.texts().at(0)->text(), "Caf\xC3\xA9");
    EXPECT_EQ(in.texts().at(0)->layer(), "\xC3\xA9tage") << "names are read the same way";

    const std::string cyrillic = "9\n$DWGCODEPAGE\n3\nANSI_1251\n";
    Loaded ru(dxf(text("10\n0\n20\n0\n40\n1\n1\n\xC6\xE0\n"), cyrillic));
    ASSERT_TRUE(ru.ok) << ru.error;
    EXPECT_EQ(ru.texts().at(0)->text(), "\xD0\x96\xD0\xB0");  // "Жа"

    // UTF-8, as every file from AutoCAD 2007 on is written, is kept.
    Loaded utf8(dxf(text("10\n0\n20\n0\n40\n1\n1\nCaf\xC3\xA9\n"), western));
    ASSERT_TRUE(utf8.ok) << utf8.error;
    EXPECT_EQ(utf8.texts().at(0)->text(), "Caf\xC3\xA9");

    // With no header, Windows-1252.
    Loaded bare(dxf(text("10\n0\n20\n0\n40\n1\n1\nCaf\xE9\n")));
    ASSERT_TRUE(bare.ok) << bare.error;
    EXPECT_EQ(bare.texts().at(0)->text(), "Caf\xC3\xA9");
    EXPECT_TRUE(bare.report.empty());
}

TEST(DxfTextTest, ACodePageThatCannotBeReadIsReported) {
    Loaded in(dxf(text("10\n0\n20\n0\n40\n1\n1\n\x82\xA0\n"), "9\n$DWGCODEPAGE\n3\nANSI_932\n"));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_EQ(in.texts().at(0)->text(), "\xEF\xBF\xBD\xEF\xBF\xBD");
    EXPECT_TRUE(contains(in.report.approximated, "2 characters in code page ANSI_932"));
}

// ---------------------------------------------------------------------------
// Colour
// ---------------------------------------------------------------------------

TEST(DxfTextTest, EveryColourIndexHasItsColour) {
    // Index 12 used to come in white, like everything past 9.
    const auto colorOf = [](int aci) {
        Loaded in(
            dxf("0\nLINE\n8\n0\n62\n" + std::to_string(aci) + "\n10\n0\n20\n0\n11\n1\n21\n0\n"));
        EXPECT_TRUE(in.ok) << in.error;
        return in.firstColor();
    };
    EXPECT_EQ(colorOf(1), 0xFFFF0000u);
    EXPECT_EQ(colorOf(9), 0xFFC0C0C0u);
    EXPECT_EQ(colorOf(11), 0xFFFF7F7Fu);   // red, pale
    EXPECT_EQ(colorOf(12), 0xFFA50000u);   // red, darker
    EXPECT_EQ(colorOf(23), 0xFFA56752u);   // orange-red, darker and pale
    EXPECT_EQ(colorOf(100), 0xFF00FF3Fu);  // green, a quarter toward cyan
    EXPECT_EQ(colorOf(240), 0xFFFF003Fu);  // red, a quarter toward magenta
    EXPECT_EQ(colorOf(250), 0xFF333333u);  // greys
    EXPECT_EQ(colorOf(254), 0xFFBEBEBEu);
}

TEST(DxfTextTest, AnOutOfRangeColourIndexInheritsInsteadOfCrashing) {
    // -2147483648 cannot be negated as an int: it used to index the colour
    // table far out of range.
    for (const char* index : {"-2147483648", "2147483647", "256", "-300"}) {
        Loaded in(dxf("0\nLINE\n8\n0\n62\n" + std::string(index) + "\n10\n0\n20\n0\n11\n1\n21\n0\n",
                      "", "",
                      "0\nTABLE\n2\nLAYER\n70\n1\n0\nLAYER\n2\nOdd\n70\n0\n62\n" +
                          std::string(index) + "\n6\nCONTINUOUS\n0\nENDTAB\n"));
        ASSERT_TRUE(in.ok) << in.error;
        EXPECT_EQ(in.firstColor(), 0u) << index;
        ASSERT_NE(in.doc.layerManager().getLayer("Odd"), nullptr) << index;
    }
}

TEST(DxfTextTest, TrueColourWinsOverTheIndex) {
    for (const std::string& groups :
         {std::string("62\n1\n420\n1193046\n"), std::string("420\n1193046\n62\n1\n")}) {
        Loaded in(dxf("0\nLINE\n8\n0\n" + groups + "10\n0\n20\n0\n11\n1\n21\n0\n"));
        ASSERT_TRUE(in.ok) << in.error;
        EXPECT_EQ(in.firstColor(), 0xFF123456u) << groups;
    }
    Loaded layer(dxf("", "", "",
                     "0\nTABLE\n2\nLAYER\n70\n1\n0\nLAYER\n2\nSteel\n70\n0\n62\n5\n420\n1193046\n"
                     "6\nCONTINUOUS\n0\nENDTAB\n"));
    ASSERT_TRUE(layer.ok) << layer.error;
    ASSERT_NE(layer.doc.layerManager().getLayer("Steel"), nullptr);
    EXPECT_EQ(layer.doc.layerManager().getLayer("Steel")->color, 0xFF123456u);
}

TEST(DxfTextTest, ColoursRoundTripExactly) {
    hz::doc::Document doc;
    for (const uint32_t color : {0xFF123456u, 0xFF00FF00u, 0xFF000000u, 0xFFA50000u}) {
        auto line = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0));
        line->setColor(color);
        doc.draftDocument().addEntity(line);
    }
    hz::draft::LayerProperties steel;
    steel.name = "Steel";
    steel.color = 0xFF654321u;
    doc.layerManager().addLayer(steel);

    Loaded in(saved(doc));
    ASSERT_TRUE(in.ok) << in.error;
    const auto& entities = in.doc.draftDocument().entities();
    ASSERT_EQ(entities.size(), 4u);
    EXPECT_EQ(entities[0]->color(), 0xFF123456u);
    EXPECT_EQ(entities[1]->color(), 0xFF00FF00u);
    EXPECT_EQ(entities[2]->color(), 0xFF000000u) << "black is not white";
    EXPECT_EQ(entities[3]->color(), 0xFFA50000u);
    ASSERT_NE(in.doc.layerManager().getLayer("Steel"), nullptr);
    EXPECT_EQ(in.doc.layerManager().getLayer("Steel")->color, 0xFF654321u);
}

// ---------------------------------------------------------------------------
// Writing text
// ---------------------------------------------------------------------------

TEST(DxfTextTest, TextIsWrittenSoItReadsBack) {
    const std::vector<std::string> samples = {
        std::string(kDiameter) + "10 " + kPlusMinus + "0.1 45" + kDegree,
        "50% off, %%d written out, 100%",
        "a^J caret, a^ space, x^2",
        "tab\there",
        "  spaced  ",
        "caf\xC3\xA9 \xD0\x96",
    };
    hz::doc::Document doc;
    for (size_t k = 0; k < samples.size(); ++k) {
        doc.draftDocument().addEntity(
            std::make_shared<DraftText>(Vec2(0, 5.0 * static_cast<double>(k)), samples[k], 2.0));
    }
    const std::string file = saved(doc);
    EXPECT_NE(file.find("%%c10 %%p0.1 45%%d"), std::string::npos)
        << "the signs go as %% codes, which every reader knows";

    Loaded in(file);
    ASSERT_TRUE(in.ok) << in.error;
    const auto texts = in.texts();
    ASSERT_EQ(texts.size(), samples.size());
    for (size_t k = 0; k < samples.size(); ++k) EXPECT_EQ(texts[k]->text(), samples[k]);
}

// A text of several lines is written as one MTEXT and reads back as the same
// text, where it was, turned and aligned as it was; MTEXT's own escapes and a
// value too long for one group come back too.
TEST(DxfTextTest, TextOfSeveralLinesIsWrittenAsMtextAndReadsBack) {
    const std::string longLine(300, 'x');
    const std::vector<std::string> samples = {
        "First\nSecond\nThird",
        "back\\slash\n{braces}\n" + std::string(kDiameter) + "12",
        "a\n\nafter a blank line",
        longLine + "\nend",
    };
    const std::vector<TextAlignment> aligned = {TextAlignment::Left, TextAlignment::Center,
                                                TextAlignment::Right, TextAlignment::Left};
    hz::doc::Document doc;
    std::vector<std::shared_ptr<DraftText>> written;
    for (size_t k = 0; k < samples.size(); ++k) {
        auto text =
            std::make_shared<DraftText>(Vec2(3.0, 40.0 * static_cast<double>(k)), samples[k], 2.5);
        text->setRotation(0.25 * static_cast<double>(k));
        text->setAlignment(aligned[k]);
        written.push_back(text);
        doc.draftDocument().addEntity(text);
    }
    const std::string file = saved(doc);
    size_t mtexts = 0;
    for (size_t at = file.find("\nMTEXT\n"); at != std::string::npos;
         at = file.find("\nMTEXT\n", at + 1)) {
        ++mtexts;
    }
    EXPECT_EQ(mtexts, samples.size());

    Loaded in(file);
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_TRUE(in.report.approximated.empty());
    const auto texts = in.texts();
    ASSERT_EQ(texts.size(), samples.size());
    for (size_t k = 0; k < samples.size(); ++k) {
        SCOPED_TRACE(k);
        EXPECT_EQ(texts[k]->text(), samples[k]);
        EXPECT_TRUE(near(texts[k]->position(), written[k]->position(), 1e-6));
        EXPECT_TRUE(near(texts[k]->lineBaseline(2), written[k]->lineBaseline(2), 1e-6));
        EXPECT_NEAR(texts[k]->rotation(), written[k]->rotation(), 1e-6);
        EXPECT_EQ(texts[k]->alignment(), aligned[k]);
        EXPECT_DOUBLE_EQ(texts[k]->textHeight(), 2.5);
    }
}

TEST(DxfTextTest, ALineBreakInANameCannotShiftTheFile) {
    hz::doc::Document doc;
    auto line = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0));
    line->setLayer("two\nlines");
    doc.draftDocument().addEntity(line);
    doc.draftDocument().addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(0, 1)));

    Loaded in(saved(doc));
    ASSERT_TRUE(in.ok) << in.error;
    const auto& entities = in.doc.draftDocument().entities();
    ASSERT_EQ(entities.size(), 2u);
    EXPECT_EQ(entities[0]->layer(), "two lines");
}

// ---------------------------------------------------------------------------
// Units and the file itself
// ---------------------------------------------------------------------------

TEST(DxfTextTest, AnInchDrawingIsScaledIntoMillimetres) {
    const std::string inches = "9\n$INSUNITS\n70\n1\n";
    const std::string block =
        "0\nBLOCK\n8\n0\n2\nPin\n70\n0\n10\n1\n20\n0\n"
        "0\nLINE\n8\n0\n10\n1\n20\n0\n11\n2\n21\n0\n0\nENDBLK\n8\n0\n";
    Loaded in(dxf("0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1\n21\n0\n" +
                      text("10\n1\n20\n1\n40\n0.1\n1\nNote\n") +
                      "0\nINSERT\n8\n0\n2\nPin\n10\n2\n20\n0\n",
                  inches, block));
    ASSERT_TRUE(in.ok) << in.error;
    const auto& entities = in.doc.draftDocument().entities();
    ASSERT_EQ(entities.size(), 3u);

    const auto* line = dynamic_cast<const hz::draft::DraftLine*>(entities[0].get());
    ASSERT_NE(line, nullptr);
    EXPECT_NEAR((line->end() - line->start()).length(), 25.4, 1e-9);

    const auto* note = dynamic_cast<const DraftText*>(entities[1].get());
    ASSERT_NE(note, nullptr);
    EXPECT_TRUE(near(note->position(), Vec2(25.4, 25.4)));
    EXPECT_NEAR(note->textHeight(), 2.54, 1e-9);

    // The block is scaled once, in its definition; its insert only moves.
    const auto* pin = dynamic_cast<const hz::draft::DraftBlockRef*>(entities[2].get());
    ASSERT_NE(pin, nullptr);
    EXPECT_TRUE(near(pin->insertPos(), Vec2(50.8, 0)));
    EXPECT_DOUBLE_EQ(pin->uniformScale(), 1.0);
    const auto def = in.doc.draftDocument().blockTable().findBlock("Pin");
    ASSERT_NE(def, nullptr);
    EXPECT_TRUE(near(def->basePoint, Vec2(25.4, 0)));
    const auto* inner = dynamic_cast<const hz::draft::DraftLine*>(def->entities.at(0).get());
    ASSERT_NE(inner, nullptr);
    EXPECT_TRUE(near(inner->end(), Vec2(50.8, 0)));

    EXPECT_TRUE(in.report.empty()) << "a conversion loses nothing";
    EXPECT_TRUE(contains(in.report.converted, "drawn in inches, scaled by 25.4 into millimetres"));
}

TEST(DxfTextTest, AUnitlessOrMillimetreDrawingIsNotScaled) {
    for (const char* units : {"0", "4"}) {
        Loaded in(dxf("0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1\n21\n0\n",
                      std::string("9\n$INSUNITS\n70\n") + units + "\n"));
        ASSERT_TRUE(in.ok) << in.error;
        const auto* line = dynamic_cast<const hz::draft::DraftLine*>(
            in.doc.draftDocument().entities().at(0).get());
        ASSERT_NE(line, nullptr);
        EXPECT_NEAR((line->end() - line->start()).length(), 1.0, 1e-12) << units;
        EXPECT_TRUE(in.report.converted.empty());
    }
}

TEST(DxfTextTest, ASavedDrawingSaysItIsInMillimetres) {
    hz::doc::Document doc;
    doc.draftDocument().addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0)));
    const std::string file = saved(doc);
    EXPECT_NE(file.find("$INSUNITS\n  70\n4\n"), std::string::npos);

    Loaded in(file);
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_TRUE(in.report.converted.empty()) << "millimetres come back as they are";
}

TEST(DxfTextTest, ABinaryDxfIsRefusedWithAReason) {
    Loaded in(std::string("AutoCAD Binary DXF\r\n\x1a", 21) + std::string(8, '\0'));
    EXPECT_FALSE(in.ok);
    EXPECT_NE(in.error.find("binary DXF"), std::string::npos) << in.error;
}

TEST(DxfTextTest, AByteOrderMarkIsSkipped) {
    Loaded in("\xEF\xBB\xBF" + dxf("0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1\n21\n0\n"));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_EQ(in.doc.draftDocument().entities().size(), 1u);
}
