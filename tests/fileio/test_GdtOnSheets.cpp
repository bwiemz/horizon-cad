// Geometric tolerances and datum feature symbols on a drawing sheet: saved
// with it, read back as the standard allows them, and found on the edges the
// part has when the sheet is built.

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/GeometricTolerance.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::io::DrawingDatumSpec;
using hz::io::DrawingDocumentIO;
using hz::io::DrawingDocumentSpec;
using hz::io::DrawingToleranceSpec;
using hz::model::GeometricCharacteristic;
using hz::model::MaterialCondition;
using nlohmann::json;

namespace {

const std::unique_ptr<hz::topo::Solid>& block() {
    static const auto solid = hz::model::PrimitiveFactory::makeBox(40.0, 30.0, 20.0);
    return solid;
}

/// The standard sheet of the block.
DrawingDocumentSpec sheet() {
    DrawingDocumentSpec spec;
    spec.partPath = "block.hzpart";
    spec.views = DrawingDocumentIO::viewsOf(
        hz::model::DrawingGenerator::sheetLayout(*block(), spec.sheet, spec.titleBlock));
    return spec;
}

DrawingToleranceSpec position(const std::string& edge) {
    DrawingToleranceSpec t;
    t.edge = edge;
    t.frame.characteristic = GeometricCharacteristic::Position;
    t.frame.tolerance = 0.2;
    t.frame.diameterZone = true;
    t.frame.modifier = MaterialCondition::MMC;
    t.frame.datumRefs = {"A", "B"};
    return t;
}

std::string temp(const std::string& name) {
    return (std::filesystem::temp_directory_path() / name).string();
}

int versionOf(const std::string& path) {
    std::ifstream in(path);
    return json::parse(in).at("version").get<int>();
}

}  // namespace

// A sheet with frames or datums is format 5, and reads back as it was.
TEST(GdtOnSheetsTest, SavedAsVersion5AndReadBack) {
    DrawingDocumentSpec spec = sheet();
    ASSERT_FALSE(spec.views.empty());
    spec.views[0].tolerances.push_back(position("edge:hole_1/wall|block/top"));
    DrawingToleranceSpec flat;
    flat.edge = "edge:block/top|block/front";
    flat.frame.characteristic = GeometricCharacteristic::Flatness;
    flat.frame.tolerance = 0.05;
    spec.views[0].tolerances.push_back(flat);
    spec.views[0].datums.push_back({"edge:block/bottom|block/front", "A"});
    spec.views.back().datums.push_back({"edge:block/left|block/top", "B"});

    const std::string path = temp("hz_test_gdt.hzdwg");
    ASSERT_TRUE(DrawingDocumentIO::save(path, spec));
    EXPECT_EQ(versionOf(path), 5);

    DrawingDocumentSpec back;
    std::string why;
    ASSERT_TRUE(DrawingDocumentIO::readSpec(path, back, &why)) << why;
    EXPECT_TRUE(back.leftOut.empty()) << ::testing::PrintToString(back.leftOut);
    ASSERT_EQ(back.views.size(), spec.views.size());
    ASSERT_EQ(back.views[0].tolerances.size(), 2u);
    const auto& frame = back.views[0].tolerances[0];
    EXPECT_EQ(frame.edge, "edge:hole_1/wall|block/top");
    EXPECT_EQ(frame.frame.characteristic, GeometricCharacteristic::Position);
    EXPECT_DOUBLE_EQ(frame.frame.tolerance, 0.2);
    EXPECT_TRUE(frame.frame.diameterZone);
    EXPECT_EQ(frame.frame.modifier, MaterialCondition::MMC);
    EXPECT_EQ(frame.frame.datumRefs, (std::vector<std::string>{"A", "B"}));
    EXPECT_EQ(back.views[0].tolerances[1].frame.characteristic, GeometricCharacteristic::Flatness);
    ASSERT_EQ(back.views[0].datums.size(), 1u);
    EXPECT_EQ(back.views[0].datums[0].label, "A");
    ASSERT_EQ(back.views.back().datums.size(), 1u);
    EXPECT_EQ(back.views.back().datums[0].edge, "edge:block/left|block/top");

    // Without them, the sheet is the format it was.
    for (auto& v : spec.views) {
        v.tolerances.clear();
        v.datums.clear();
    }
    ASSERT_TRUE(DrawingDocumentIO::save(path, spec));
    EXPECT_EQ(versionOf(path), 3);
    std::filesystem::remove(path);
}

// What the standard does not allow is left out of the sheet, each with why:
// the sheet itself still reads.
TEST(GdtOnSheetsTest, WhatTheStandardRefusesIsLeftOutAndSaid) {
    DrawingDocumentSpec spec = sheet();
    const std::string path = temp("hz_test_gdt_refused.hzdwg");
    ASSERT_TRUE(DrawingDocumentIO::save(path, spec));
    json root;
    {
        std::ifstream in(path);
        root = json::parse(in);
    }
    const json good = {{"edge", "edge:a"},
                       {"characteristic", "perpendicularity"},
                       {"tolerance", 0.05},
                       {"condition", "none"},
                       {"datums", {"A"}}};
    json frames = json::array();
    frames.push_back(good);
    json noEdge = good;
    noEdge["edge"] = "";
    frames.push_back(noEdge);
    json wobble = good;
    wobble["characteristic"] = "wobble";
    frames.push_back(wobble);
    json formWithDatum = good;
    formWithDatum["characteristic"] = "flatness";
    frames.push_back(formWithDatum);
    json noDatum = good;
    noDatum["datums"] = json::array();
    frames.push_back(noDatum);
    json rfs = good;
    rfs["condition"] = "rfs";
    frames.push_back(rfs);
    json notLetters = good;
    notLetters["datums"] = "A";
    frames.push_back(notLetters);
    json zero = good;
    zero["tolerance"] = 0.0;
    frames.push_back(zero);
    frames.push_back("position");
    root["views"][0]["tolerances"] = frames;
    root["views"][0]["datums"] = json::array(
        {json{{"edge", "edge:a"}, {"label", "A"}}, json{{"edge", "edge:b"}, {"label", "I"}},
         json{{"edge", "edge:c"}, {"label", "A"}}, json{{"label", "B"}}, 7});
    root["version"] = 5;
    {
        std::ofstream out(path);
        out << root.dump();
    }

    DrawingDocumentSpec back;
    std::string why;
    ASSERT_TRUE(DrawingDocumentIO::readSpec(path, back, &why)) << why;
    ASSERT_EQ(back.views[0].tolerances.size(), 1u);
    EXPECT_EQ(back.views[0].tolerances[0].edge, "edge:a");
    ASSERT_EQ(back.views[0].datums.size(), 1u);
    EXPECT_EQ(back.views[0].datums[0].edge, "edge:a");
    EXPECT_EQ(back.leftOut.size(), 8u + 4u) << ::testing::PrintToString(back.leftOut);
    const auto said = [&back](const std::string& what) {
        return std::any_of(back.leftOut.begin(), back.leftOut.end(), [&what](const std::string& s) {
            return s.find(what) != std::string::npos;
        });
    };
    EXPECT_TRUE(said("tolerance 2: it names no edge"));
    EXPECT_TRUE(said("tolerance 3: not a characteristic"));
    EXPECT_TRUE(said("form tolerance"));
    EXPECT_TRUE(said("needs a datum"));
    EXPECT_TRUE(said("not a material condition"));
    EXPECT_TRUE(said("not datum letters"));
    EXPECT_TRUE(said("more than 0"));
    EXPECT_TRUE(said("tolerance 9: it is not a tolerance"));
    EXPECT_TRUE(said("datum 2: its letter"));
    EXPECT_TRUE(said("datum A is on the sheet already"));
    EXPECT_TRUE(said("datum 4: it names no edge"));
    std::filesystem::remove(path);
}

// Built, each is on the edge it names, wherever the view shows it; one on an
// edge the view does not show is said.
TEST(GdtOnSheetsTest, BuiltOnTheEdgesTheViewShows) {
    DrawingDocumentSpec spec = sheet();
    const hz::model::Drawing laid = DrawingDocumentIO::build(*block(), spec);
    ASSERT_FALSE(laid.views.empty());
    const std::string edge = hz::model::logicalEdge(laid.views[0].edges.front().sourceEdge.tag());
    spec.views[0].tolerances.push_back(position(edge));
    spec.views[0].tolerances.push_back(position("edge:gone"));
    spec.views[0].datums.push_back({edge, "A"});
    spec.views[0].datums.push_back({"edge:gone", "B"});

    std::vector<std::string> lost;
    const hz::model::Drawing drawing = DrawingDocumentIO::build(*block(), spec, &lost);
    const hz::model::DrawingView& view = drawing.views[0];
    ASSERT_EQ(view.tolerances.size(), 1u);
    EXPECT_EQ(hz::model::logicalEdge(view.tolerances[0].feature.tag()), edge);
    EXPECT_DOUBLE_EQ(view.tolerances[0].tolerance, 0.2);
    EXPECT_EQ(view.tolerances[0].datumRefs, (std::vector<std::string>{"A", "B"}));
    ASSERT_EQ(view.datums.size(), 1u);
    EXPECT_EQ(view.datums[0].label, "A");
    EXPECT_EQ(lost, (std::vector<std::string>{"view 1: a tolerance frame", "view 1: datum B"}));
}

// As many as a view's entry is read with is what the workbench lets a view
// hold: a sheet with that many reads back; one more is refused.
TEST(GdtOnSheetsTest, AViewHoldsAsManyAsItIsReadWith) {
    DrawingDocumentSpec spec = sheet();
    for (std::size_t i = 0; i < DrawingDocumentIO::kMaxPerView; ++i) {
        DrawingToleranceSpec t = position("edge:a");
        t.frame.tolerance = 0.001 * static_cast<double>(i + 1);
        spec.views[0].tolerances.push_back(t);
    }
    const std::string path = temp("hz_test_gdt_full.hzdwg");
    ASSERT_TRUE(DrawingDocumentIO::save(path, spec));
    DrawingDocumentSpec back;
    std::string why;
    ASSERT_TRUE(DrawingDocumentIO::readSpec(path, back, &why)) << why;
    EXPECT_EQ(back.views[0].tolerances.size(), DrawingDocumentIO::kMaxPerView);

    spec.views[0].tolerances.push_back(position("edge:b"));
    ASSERT_TRUE(DrawingDocumentIO::save(path, spec));
    EXPECT_FALSE(DrawingDocumentIO::readSpec(path, back, &why));
    EXPECT_NE(why.find("tolerances"), std::string::npos) << why;
    std::filesystem::remove(path);
}
