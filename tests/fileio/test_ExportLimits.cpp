// What a drawing holds that a file cannot (Phase 168): layer names DXF
// refuses, entities with no size, values that are not numbers. A file a
// reader refuses is worse than one with an entity left out and said so.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/NativeFormat.h"

using hz::doc::Document;
using hz::io::DxfFormat;
using hz::io::DxfWriteReport;
using hz::io::ImportReport;
using hz::io::NativeFormat;
using hz::math::Vec2;

namespace {

/// A file of the test's own, removed when it goes.
struct TempFile {
    explicit TempFile(const std::string& name)
        : path(std::filesystem::temp_directory_path() / ("hz_export_limits_" + name)) {}
    ~TempFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
    std::string str() const { return path.string(); }
    std::string text() const {
        std::ifstream in(path, std::ios::binary);
        return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    }
    std::filesystem::path path;
};

std::shared_ptr<hz::draft::DraftLine> lineOn(const std::string& layer) {
    auto line = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 1));
    line->setLayer(layer);
    return line;
}

}  // namespace

// Layer names DXF refuses (its forbidden characters, nothing at all, a
// trailing space) are written under the nearest name it takes, apart from
// every other name whatever their case; the names it takes are kept as they
// are, and every entity goes on its layer under the name it was given.
TEST(ExportLimitsTest, LayerNamesAreWrittenAsDxfTakesThem) {
    Document doc;
    const std::vector<std::string> names = {
        "Walls", "walls",     "Wände/Ebene 1", "a:b;c", "",
        " ",     "trailing ", "tab\there",     "Layer", std::string(300, 'x')};
    for (const auto& name : names) {
        hz::draft::LayerProperties props;
        props.name = name;
        doc.layerManager().addLayer(props);
        doc.draftDocument().addEntity(lineOn(name));
    }
    TempFile file("names.dxf");
    DxfWriteReport report;
    std::string error;
    ASSERT_TRUE(DxfFormat::save(file.str(), doc, &error, &report)) << error;

    // Read back: every name one DXF takes, and none twice whatever the case.
    std::set<std::string> folded;
    std::vector<std::string> table;
    Document back;
    ImportReport read;
    ASSERT_TRUE(DxfFormat::load(file.str(), back, &error, &read)) << error;
    for (const auto& name : back.layerManager().layerNames()) {
        EXPECT_EQ(name.find_first_of("<>/\\\":;?*|=`\t"), std::string::npos) << name;
        EXPECT_FALSE(name.empty());
        EXPECT_NE(name.back(), ' ') << name;
        EXPECT_LE(name.size(), 255u);
        std::string lower = name;
        for (char& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        EXPECT_TRUE(folded.insert(lower).second) << "two layers named " << name;
        table.push_back(name);
    }
    // Every layer, and layer 0.
    EXPECT_EQ(table.size(), names.size() + 1);
    // Names DXF takes are kept: the first of two that differ only in case,
    // and a layer named as the empty name's stand-in.
    const auto has = [&back](const std::string& name) {
        return back.layerManager().getLayer(name) != nullptr;
    };
    EXPECT_TRUE(has("Layer"));
    EXPECT_TRUE(has("Wände_Ebene 1"));
    EXPECT_TRUE(has("a_b_c"));
    EXPECT_TRUE(has("tab_here"));
    EXPECT_TRUE(has("trailing"));
    // Each entity on the layer its own was written as.
    ASSERT_EQ(back.draftDocument().entities().size(), names.size());
    for (const auto& entity : back.draftDocument().entities()) {
        EXPECT_TRUE(has(entity->layer())) << entity->layer();
    }
    // Every renamed layer said, and only those.
    EXPECT_TRUE(report.leftOut.empty());
    EXPECT_EQ(report.renamed.size(), 8u) << "all but \"Layer\" and the first of the Walls";
}

// Entities DXF cannot hold as they are (a circle or arc with no radius, a
// polyline or spline of under two points, an ellipse with no width) and any
// value not a finite number, which readers take for a damaged file: left
// out, each said, and the rest written. The file has no "nan" or "inf" in it.
TEST(ExportLimitsTest, WhatDxfCannotHoldIsLeftOutAndSaid) {
    Document doc;
    auto& drawing = doc.draftDocument();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    drawing.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(5, 0)));
    drawing.addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(0, 0), 0.0));
    drawing.addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(0, 0), -2.0));
    drawing.addEntity(std::make_shared<hz::draft::DraftPolyline>(std::vector<Vec2>{{1, 1}}, true));
    drawing.addEntity(std::make_shared<hz::draft::DraftSpline>(std::vector<Vec2>{{1, 1}}));
    drawing.addEntity(std::make_shared<hz::draft::DraftEllipse>(Vec2(0, 0), 3.0, 0.0));
    drawing.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(nan, 0), Vec2(5, 0)));
    drawing.addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(0, 0), inf));
    drawing.addEntity(std::make_shared<hz::draft::DraftText>(Vec2(0, 0), "x", nan));
    drawing.addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(9, 9), 2.0));

    TempFile file("left-out.dxf");
    DxfWriteReport report;
    std::string error;
    ASSERT_TRUE(DxfFormat::save(file.str(), doc, &error, &report)) << error;
    EXPECT_EQ(report.leftOut.size(), 8u);
    for (const auto& why : report.leftOut) EXPECT_EQ(why.rfind("entity ", 0), 0u) << why;
    const std::string text = file.text();
    for (const char* bad : {"nan", "inf", "NaN", "Inf"}) {
        EXPECT_EQ(text.find(std::string("\n") + bad), std::string::npos) << bad;
        EXPECT_EQ(text.find(std::string("\n-") + bad), std::string::npos) << bad;
    }

    Document back;
    ImportReport read;
    ASSERT_TRUE(DxfFormat::load(file.str(), back, &error, &read)) << error;
    EXPECT_EQ(back.draftDocument().entities().size(), 2u) << "the line and the circle";
    EXPECT_TRUE(read.skipped.empty()) << "nothing written that the reader takes for damaged";
}

// A value that is not a finite number cannot be written as JSON, which has
// none: it goes as nothing, and the load leaves the entity out saying why,
// not with JSON's own words.
TEST(ExportLimitsTest, ANumberSavedAsNothingIsExplained) {
    Document doc;
    doc.draftDocument().addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 1)));
    doc.draftDocument().addEntity(std::make_shared<hz::draft::DraftLine>(
        Vec2(std::numeric_limits<double>::quiet_NaN(), 0), Vec2(1, 1)));
    TempFile file("nan.hcad");
    std::string error;
    ASSERT_TRUE(NativeFormat::save(file.str(), doc, &error)) << error;
    Document back;
    ImportReport report;
    ASSERT_TRUE(NativeFormat::load(file.str(), back, &error, &report)) << error;
    EXPECT_EQ(back.draftDocument().entities().size(), 1u);
    ASSERT_EQ(report.skipped.size(), 1u);
    EXPECT_NE(report.skipped[0].find("not a finite number"), std::string::npos)
        << report.skipped[0];
}

// Many layers and many entities go out and come back whole, in time: ten
// thousand layers and a hundred thousand lines, natively and as DXF.
TEST(ExportLimitsTest, ManyLayersAndEntitiesComeBackWhole) {
    Document doc;
    constexpr int kLayers = 10'000;
    constexpr int kLines = 100'000;
    for (int i = 0; i < kLayers; ++i) {
        hz::draft::LayerProperties props;
        props.name = "L" + std::to_string(i);
        doc.layerManager().addLayer(props);
    }
    for (int i = 0; i < kLines; ++i) {
        auto line = std::make_shared<hz::draft::DraftLine>(Vec2(i, 0), Vec2(i, 1));
        line->setLayer("L" + std::to_string(i % kLayers));
        doc.draftDocument().addEntity(line);
    }
    TempFile native("many.hcad");
    TempFile dxf("many.dxf");
    std::string error;
    const auto start = std::chrono::steady_clock::now();
    ASSERT_TRUE(NativeFormat::save(native.str(), doc, &error)) << error;
    ASSERT_TRUE(DxfFormat::save(dxf.str(), doc, &error)) << error;
    for (const auto& path : {native.str(), dxf.str()}) {
        Document back;
        const bool ok = path == native.str() ? NativeFormat::load(path, back, &error)
                                             : DxfFormat::load(path, back, &error);
        ASSERT_TRUE(ok) << error;
        EXPECT_EQ(back.layerManager().layerNames().size(), static_cast<size_t>(kLayers + 1));
        EXPECT_EQ(back.draftDocument().entities().size(), static_cast<size_t>(kLines));
    }
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    EXPECT_LT(seconds, 60.0) << "a generous bound, for an unoptimized build";
}
