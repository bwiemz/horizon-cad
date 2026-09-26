// Every input here once crashed, hung, or silently misread the app. Each is
// now a reported failure, a clamped value, or — for one malformed item inside
// an otherwise readable document — that item skipped. (Telling the user what
// was skipped is Phase 107's import report.)

#include <gtest/gtest.h>

#include <clocale>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/Vec3.h"

using hz::doc::Document;
using hz::io::DxfFormat;
using hz::io::NativeFormat;
using json = nlohmann::json;

namespace {

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

/// Load `text` as a native document, requiring that it neither throws nor
/// succeeds; returns the reason.
std::string rejectedReason(const std::string& text) {
    Document doc;
    std::string error;
    bool loaded = true;
    EXPECT_NO_THROW(loaded = NativeFormat::documentFromJson(text, doc, &error));
    EXPECT_FALSE(loaded) << text;
    EXPECT_FALSE(error.empty());
    return error;
}

/// A document's envelope with feature `index` edited by `edit`.
template <typename Edit>
std::string editedFeature(const Document& doc, size_t index, Edit edit) {
    json root = json::parse(NativeFormat::documentToJson(doc, false));
    edit(root.at("featureTree").at(index));
    return root.dump();
}

std::string rejectedDxf(const std::string& text) {
    Document doc;
    std::string error;
    bool loaded = true;
    EXPECT_NO_THROW(loaded = DxfFormat::loadFromString(text, doc, &error));
    EXPECT_FALSE(loaded);
    EXPECT_FALSE(error.empty());
    return error;
}

}  // namespace

// ---------------------------------------------------------------------------
// Native documents
// ---------------------------------------------------------------------------

TEST(MalformedInputTest, EmptyPullDirectionSkipsTheFeatureNotACrash) {
    // Indexed [0] on an empty array: an assertion in Debug, undefined in Release.
    Document doc;
    std::string error;
    bool loaded = false;
    ASSERT_NO_THROW(loaded = NativeFormat::documentFromJson(
                        R"({"version":16,"entities":[],"featureTree":[)"
                        R"({"type":"draft","pullDir":[]},{"type":"primitive","kind":"box",)"
                        R"("p0":1,"p1":1,"p2":1}]})",
                        doc, &error));
    EXPECT_TRUE(loaded) << error;
    EXPECT_LE(doc.featureTree().featureCount(), 1u) << "the malformed draft feature is skipped";
}

TEST(MalformedInputTest, MissingRequiredFieldSkipsTheEntityNotACrash) {
    // A line with no end point: operator[] on a const object for a missing key.
    Document doc;
    std::string error;
    bool loaded = false;
    ASSERT_NO_THROW(loaded = NativeFormat::documentFromJson(
                        R"({"version":16,"entities":[{"type":"line","start":{"x":0,"y":0}},)"
                        R"({"type":"line","start":{"x":0,"y":0},"end":{"x":1,"y":1}}]})",
                        doc, &error));
    EXPECT_TRUE(loaded) << error;
    EXPECT_EQ(doc.draftDocument().entities().size(), 1u) << "only the complete line loads";
}

TEST(MalformedInputTest, NewerFormatIsRefusedAndSaysWhich) {
    // Loading it would drop what this build does not know; saving would then
    // destroy it.
    const std::string error = rejectedReason(R"({"version":99,"entities":[]})");
    EXPECT_TRUE(contains(error, "newer version")) << error;
    EXPECT_TRUE(contains(error, "99")) << error;
}

/// A two-entity document whose first line carries `badField`; the second line
/// is well-formed and must survive.
std::string oneBadLine(const std::string& badField) {
    return R"({"version":16,"entities":[{"type":"line","start":{"x":0,"y":0},)"
           R"("end":{"x":1,"y":0},)" +
           badField + R"(},{"type":"line","start":{"x":0,"y":1},"end":{"x":1,"y":1}}]})";
}

size_t entitiesLoadedFrom(const std::string& text) {
    Document doc;
    std::string error;
    bool loaded = false;
    EXPECT_NO_THROW(loaded = NativeFormat::documentFromJson(text, doc, &error));
    EXPECT_TRUE(loaded) << error;
    return doc.draftDocument().entities().size();
}

TEST(MalformedInputTest, EnumOutOfRangeSkipsOnlyItsEntity) {
    // A line type indexes the renderer's dash-pattern table. The bad entity
    // goes; the rest of the document does not go with it.
    EXPECT_EQ(entitiesLoadedFrom(oneBadLine(R"("lineType":999)")), 1u);
}

TEST(MalformedInputTest, HugeFloatWhereAnIntegerBelongsSkipsOnlyItsEntity) {
    // nlohmann converts a float to int with a plain static_cast: undefined
    // behaviour for 1e300.
    EXPECT_EQ(entitiesLoadedFrom(oneBadLine(R"("lineType":1e300)")), 1u);
    EXPECT_EQ(entitiesLoadedFrom(oneBadLine(R"("lineType":"dashed")")), 1u);
}

// Colours, ids and groups are whole numbers too, and were cast as the line
// type was: the fuzzer found a layer's colour of 4.29e119.
TEST(MalformedInputTest, AColourOrIdThatIsNotAWholeNumberSkipsOnlyItsEntity) {
    for (const char* field :
         {R"("color":4.29e119)", R"("color":-1)", R"("color":1.5)", R"("color":4294967296)",
          R"("id":1e300)", R"("id":-3)", R"("groupId":1e300)"}) {
        EXPECT_EQ(entitiesLoadedFrom(oneBadLine(field)), 1u) << field;
    }
    // Every colour a colour can be still is one.
    Document doc;
    std::string error;
    ASSERT_TRUE(NativeFormat::documentFromJson(oneBadLine(R"("color":4294967295)"), doc, &error))
        << error;
    ASSERT_EQ(doc.draftDocument().entities().size(), 2u);
    EXPECT_EQ(doc.draftDocument().entities().front()->color(), 0xFFFFFFFFu);
}

TEST(MalformedInputTest, ALayersColourThatIsNotAWholeNumberIsDamaged) {
    const std::string error =
        rejectedReason(R"({"version":16,"entities":[],"layers":[{"name":"A","color":4.29e119}]})");
    EXPECT_TRUE(contains(error, "color")) << error;
}

TEST(MalformedInputTest, BadPatternCountSkipsOnlyItsFeature) {
    Document doc;
    std::string error;
    ASSERT_TRUE(NativeFormat::documentFromJson(
        R"({"version":16,"type":"hzpart","entities":[],"featureTree":[)"
        R"({"type":"primitive","kind":"box","p0":1,"p1":1,"p2":1},)"
        R"({"type":"pattern","kind":"linear","vecA":[1,0,0],"vecB":[0,0,0],"scalar":1,)"
        R"("count":"many"}]})",
        doc, &error))
        << error;
    EXPECT_EQ(doc.featureTree().featureCount(), 1u) << "the box survives the broken pattern";
}

TEST(MalformedInputTest, BadDocumentLevelFieldIsDamaged) {
    // Outside any single item there is nothing smaller to skip.
    const std::string error =
        rejectedReason(R"({"version":16,"entities":[],"dimensionStyle":{"precision":1e300}})");
    EXPECT_TRUE(contains(error, "precision")) << error;
}

// The dimension style's unit is saved; a file from before it had one, or
// with a unit nothing knows, shows millimetres.
TEST(MalformedInputTest, TheDimensionUnitIsKeptAndAnUnknownOneIsMillimetres) {
    Document doc;
    auto style = doc.draftDocument().dimensionStyle();
    style.unit = "in";
    style.showUnits = true;
    doc.draftDocument().setDimensionStyle(style);
    Document back;
    std::string error;
    ASSERT_TRUE(
        NativeFormat::documentFromJson(NativeFormat::documentToJson(doc, false), back, &error))
        << error;
    EXPECT_EQ(back.draftDocument().dimensionStyle().unit, "in");
    EXPECT_TRUE(back.draftDocument().dimensionStyle().showUnits);

    for (const char* unit : {R"("furlong")", "7", "null"}) {
        Document odd;
        ASSERT_TRUE(NativeFormat::documentFromJson(
            std::string(R"({"version":16,"entities":[],"dimensionStyle":{"unit":)") + unit + "}}",
            odd, &error))
            << error;
        EXPECT_EQ(odd.draftDocument().dimensionStyle().unit, "mm") << unit;
    }
}

TEST(MalformedInputTest, AbsurdPatternCountIsClamped) {
    Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    doc.featureTree().addFeature(hz::doc::PrimitiveFeature::makeBox(1.0, 1.0, 1.0));
    doc.featureTree().addFeature(
        hz::doc::PatternFeature::makeLinear(hz::math::Vec3(1, 0, 0), 2.0, 3, {}));
    const std::string text =
        editedFeature(doc, 1, [](json& f) { f["count"] = 2.0e9; });  // would allocate 2e9 copies

    Document loaded;
    std::string error;
    ASSERT_TRUE(NativeFormat::documentFromJson(text, loaded, &error)) << error;
    const auto* pattern =
        dynamic_cast<const hz::doc::PatternFeature*>(loaded.featureTree().feature(1));
    ASSERT_NE(pattern, nullptr);
    EXPECT_EQ(pattern->count(), hz::doc::kMaxPatternCount);
}

TEST(MalformedInputTest, AbsurdFacetCountIsClamped) {
    Document doc;
    doc.setType(hz::doc::DocumentType::Part);
    doc.featureTree().addFeature(hz::doc::PrimitiveFeature::makeCylinder(1.0, 2.0));
    const std::string text = editedFeature(doc, 0, [](json& f) { f["segments"] = 1.0e9; });

    Document loaded;
    std::string error;
    ASSERT_TRUE(NativeFormat::documentFromJson(text, loaded, &error)) << error;
    const auto* cylinder =
        dynamic_cast<const hz::doc::PrimitiveFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(cylinder, nullptr);
    EXPECT_EQ(cylinder->segments(), hz::doc::kMaxFacetSegments);
}

TEST(MalformedInputTest, InfiniteParameterIsRefusedByTheFeature) {
    // static_cast<int>(inf) was undefined behaviour behind a ">= 3" check.
    auto cylinder = hz::doc::PrimitiveFeature::makeCylinder(1.0, 2.0);
    const int before = cylinder->segments();
    EXPECT_FALSE(cylinder->setParameter("segments", std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(cylinder->setParameter("segments", std::numeric_limits<double>::quiet_NaN()));
    EXPECT_EQ(cylinder->segments(), before);
}

// ---------------------------------------------------------------------------
// DXF
// ---------------------------------------------------------------------------

TEST(MalformedInputTest, DxfTruncatedAfterAnEntityTypeDoesNotHang) {
    // Used to re-parse "LINE" forever, adding a line each time.
    const std::string error = rejectedDxf("0\nSECTION\n2\nENTITIES\n0\nLINE\n");
    EXPECT_TRUE(contains(error, "truncated")) << error;
}

TEST(MalformedInputTest, DxfTruncatedInsideAnEntityDoesNotHang) {
    // Used to spin at 100% CPU on the last group.
    const std::string error = rejectedDxf("0\nSECTION\n2\nENTITIES\n0\nLINE\n10\n1.0\n");
    EXPECT_TRUE(contains(error, "truncated")) << error;
}

TEST(MalformedInputTest, DxfCutInTheMiddleOfAGroupSaysSo) {
    const std::string error = rejectedDxf("0\nSECTION\n2\nENTITIES\n0\nLINE\n10\n");
    EXPECT_TRUE(contains(error, "middle of a group")) << error;
}

TEST(MalformedInputTest, DxfBadGroupCodeNamesTheLine) {
    const std::string error = rejectedDxf("0\nSECTION\n2\nENTITIES\n0\nLINE\nxx\n1.0\n");
    EXPECT_TRUE(contains(error, "line 7")) << error;
}

TEST(MalformedInputTest, DxfTruncatedBlockOrTableDoesNotHang) {
    rejectedDxf("0\nSECTION\n2\nBLOCKS\n0\nBLOCK\n2\nB1\n0\nLINE\n10\n0\n");
    rejectedDxf("0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLAYER\n0\nLAYER\n2\nWalls\n");
}

TEST(MalformedInputTest, DxfWithoutAnEofMarkerStillLoads) {
    // Some writers omit "0 EOF"; a complete ENTITIES section is enough.
    Document doc;
    std::string error;
    ASSERT_TRUE(DxfFormat::loadFromString(
        "0\nSECTION\n2\nENTITIES\n0\nLINE\n8\n0\n10\n0\n20\n0\n11\n3\n21\n4\n0\nENDSEC\n", doc,
        &error))
        << error;
    ASSERT_EQ(doc.draftDocument().entities().size(), 1u);
}

TEST(MalformedInputTest, DxfNumbersIgnoreACommaDecimalLocale) {
    const char* previous = std::setlocale(LC_NUMERIC, nullptr);
    const std::string saved = previous ? previous : "C";
    const char* comma = nullptr;
    for (const char* name : {"de_DE.UTF-8", "de_DE.utf8", "fr_FR.UTF-8", "ru_RU.UTF-8"}) {
        if (std::setlocale(LC_NUMERIC, name) != nullptr) {
            comma = name;
            break;
        }
    }
    if (comma == nullptr) GTEST_SKIP() << "no comma-decimal locale installed";

    Document doc;
    const bool loaded = DxfFormat::loadFromString(
        "0\nSECTION\n2\nENTITIES\n0\nLINE\n10\n1.5\n20\n0\n11\n2.25\n21\n0\n0\nENDSEC\n0\nEOF\n",
        doc);
    std::setlocale(LC_NUMERIC, saved.c_str());

    ASSERT_TRUE(loaded);
    ASSERT_EQ(doc.draftDocument().entities().size(), 1u);
    const auto* line =
        dynamic_cast<const hz::draft::DraftLine*>(doc.draftDocument().entities()[0].get());
    ASSERT_NE(line, nullptr);
    EXPECT_DOUBLE_EQ(line->start().x, 1.5) << "std::stod read 1.5 as 1 under de_DE";
    EXPECT_DOUBLE_EQ(line->end().x, 2.25);
}

// Two entities under one ID (a damaged or hand-edited file). The later one
// used to hide the earlier from every lookup: unselectable, undeletable. It
// is loaded under a new ID, and the report says so.
TEST(MalformedInputTest, ADuplicateEntityIdIsRenumbered) {
    hz::io::ImportReport report;
    Document doc;
    std::string error;
    ASSERT_TRUE(NativeFormat::documentFromJson(
        R"({"version":16,"entities":[)"
        R"({"type":"line","id":7,"start":{"x":0,"y":0},"end":{"x":1,"y":0}},)"
        R"({"type":"line","id":7,"start":{"x":0,"y":5},"end":{"x":1,"y":5}}]})",
        doc, &error, &report))
        << error;
    const auto& entities = doc.draftDocument().entities();
    ASSERT_EQ(entities.size(), 2u);
    EXPECT_EQ(entities[0]->id(), 7u);
    EXPECT_NE(entities[1]->id(), 7u);
    EXPECT_EQ(doc.draftDocument().findEntity(entities[0]->id()), entities[0].get());
    EXPECT_EQ(doc.draftDocument().findEntity(entities[1]->id()), entities[1].get());
    bool noted = false;
    for (const auto& line : report.approximated) noted = noted || contains(line, "earlier entity");
    EXPECT_TRUE(noted);
}

// A duplicate given a new ID must not take one a later entity holds in the
// file. With IDs a, a, a+2 (making the second entity takes a+1 before its
// saved ID is set), the duplicate was given a+2, and the third entity, whose
// ID it was, was then renumbered as a duplicate too.
TEST(MalformedInputTest, ARenumberedDuplicateTakesNoLaterEntitysId) {
    const uint64_t a = hz::draft::DraftEntity::newId() + 10;
    const std::string line = R"({"type":"line","id":%ID%,"start":{"x":0,"y":0},)"
                             R"("end":{"x":1,"y":0}})";
    const auto withId = [&line](uint64_t id) {
        std::string out = line;
        out.replace(out.find("%ID%"), 4, std::to_string(id));
        return out;
    };
    hz::io::ImportReport report;
    Document doc;
    std::string error;
    ASSERT_TRUE(NativeFormat::documentFromJson(
        R"({"version":16,"entities":[)" + withId(a) + "," + withId(a) + "," + withId(a + 2) + "]}",
        doc, &error, &report))
        << error;
    const auto& entities = doc.draftDocument().entities();
    ASSERT_EQ(entities.size(), 3u);
    EXPECT_EQ(entities[0]->id(), a);
    EXPECT_EQ(entities[2]->id(), a + 2) << "the third keeps the ID it had";
    EXPECT_NE(entities[1]->id(), a);
    EXPECT_NE(entities[1]->id(), a + 2);
    int noted = 0;
    for (const auto& l : report.approximated) noted += contains(l, "earlier entity") ? 1 : 0;
    EXPECT_EQ(noted, 1) << "one duplicate";
}
