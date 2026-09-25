#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/drafting/DimensionStyle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftLinearDimension.h"
#include "horizon/fileio/DrawingDimensionRenderer.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/fileio/NativeFormat.h"

using hz::doc::Document;
using hz::doc::DocumentType;
using hz::doc::PrimitiveFeature;
using hz::io::DrawingDocumentIO;
using hz::io::DrawingDocumentSpec;
using hz::io::NativeFormat;
using hz::model::Drawing;

namespace {
std::string tempPath(const std::string& name) {
    return (std::filesystem::temp_directory_path() / name).string();
}
}  // namespace

// A .hzdwg references a part and regenerates its views on load, so the drawing
// tracks the model rather than storing a stale snapshot.
TEST(DrawingDocumentIOTest, ReferencesPartAndRegeneratesViews) {
    // 1. Author and save a part.
    const std::string partPath = tempPath("hz_test_dwg_part.hzpart");
    {
        Document part;
        part.setType(DocumentType::Part);
        part.featureTree().addFeature(PrimitiveFeature::makeBox(4.0, 3.0, 2.0));
        ASSERT_TRUE(part.rebuildModel());
        ASSERT_TRUE(NativeFormat::save(partPath, part));
    }

    // 2. Save a drawing document referencing the part.
    const std::string dwgPath = tempPath("hz_test_drawing.hzdwg");
    DrawingDocumentSpec spec;
    spec.partPath = partPath;
    spec.gap = 12.0;
    ASSERT_TRUE(DrawingDocumentIO::save(dwgPath, spec));

    // 3. Load the drawing: it re-opens the part and regenerates the views.
    DrawingDocumentSpec loadedSpec;
    Drawing drawing;
    ASSERT_TRUE(DrawingDocumentIO::load(dwgPath, loadedSpec, drawing));

    EXPECT_EQ(loadedSpec.partPath, partPath);
    EXPECT_DOUBLE_EQ(loadedSpec.gap, 12.0);
    ASSERT_EQ(drawing.views.size(), 4u);  // front, top, right, isometric
    for (const auto& v : drawing.views) {
        EXPECT_FALSE(v.edges.empty());
    }

    std::remove(dwgPath.c_str());
    std::remove(partPath.c_str());
}

// A drawing document whose part reference is missing fails to load rather than
// producing an empty drawing.
TEST(DrawingDocumentIOTest, MissingPartReferenceFailsToLoad) {
    const std::string dwgPath = tempPath("hz_test_drawing_missing.hzdwg");
    DrawingDocumentSpec spec;
    spec.partPath = tempPath("hz_test_nonexistent_part.hzpart");
    ASSERT_TRUE(DrawingDocumentIO::save(dwgPath, spec));

    DrawingDocumentSpec loadedSpec;
    Drawing drawing;
    EXPECT_FALSE(DrawingDocumentIO::load(dwgPath, loadedSpec, drawing));

    std::remove(dwgPath.c_str());
}

// ---------------------------------------------------------------------------
// Version 2 (Phase 148): the sheet, title block and views are kept.
// ---------------------------------------------------------------------------

namespace {

/// A 4 x 3 x 2 box part saved at @p path.
void saveBox(const std::string& path) {
    Document part;
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(PrimitiveFeature::makeBox(4.0, 3.0, 2.0));
    ASSERT_TRUE(part.rebuildModel());
    ASSERT_TRUE(NativeFormat::save(path, part));
}

void write(const std::string& path, const std::string& content) {
    std::ofstream(path) << content;
}

}  // namespace

// A drawing keeps its sheet, title block and each view's direction, scale and
// placement: it was saved as a part and a gap, and read back as four views
// at 1:1 wherever they had been.
TEST(DrawingDocumentIOTest, ADrawingKeepsItsSheetAndViews) {
    const auto dir = std::filesystem::temp_directory_path() / "hz_dwg_v2";
    std::filesystem::create_directories(dir);
    const std::string partPath = (dir / "box.hzpart").string();
    saveBox(partPath);

    DrawingDocumentSpec spec;
    spec.partPath = partPath;
    spec.sheet.size = hz::model::PaperSize::A4;
    spec.sheet.orientation = hz::model::Orientation::Portrait;
    spec.titleBlock.title = "Bracket";
    spec.titleBlock.drawnBy = "HZ";
    {
        Document part;
        ASSERT_TRUE(NativeFormat::load(partPath, part));
        ASSERT_TRUE(part.rebuildModel());
        double scale = 0.0;
        auto laid = hz::model::DrawingGenerator::sheetLayout(*part.solid(), spec.sheet,
                                                             spec.titleBlock, 10.0, &scale);
        laid.views.pop_back();  // no isometric
        laid.views[0].placement = {33.0, 77.0};
        laid.views[1].showHidden = false;
        spec.views = DrawingDocumentIO::viewsOf(laid);
        spec.titleBlock.scale = hz::model::DrawingGenerator::scaleName(scale);
    }
    const std::string dwgPath = (dir / "bracket.hzdwg").string();
    ASSERT_TRUE(DrawingDocumentIO::save(dwgPath, spec));

    DrawingDocumentSpec back;
    Drawing drawing;
    std::string error;
    ASSERT_TRUE(DrawingDocumentIO::load(dwgPath, back, drawing, &error)) << error;
    EXPECT_EQ(back.sheet.size, hz::model::PaperSize::A4);
    EXPECT_EQ(back.sheet.orientation, hz::model::Orientation::Portrait);
    EXPECT_EQ(back.titleBlock.title, "Bracket");
    EXPECT_EQ(back.titleBlock.drawnBy, "HZ");
    EXPECT_EQ(back.titleBlock.scale, spec.titleBlock.scale);
    ASSERT_EQ(drawing.views.size(), 3u);
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(drawing.views[i].kind, spec.views[i].kind);
        EXPECT_DOUBLE_EQ(drawing.views[i].scale, spec.views[i].scale);
        EXPECT_DOUBLE_EQ(drawing.views[i].placement.x, spec.views[i].placement.x);
        EXPECT_DOUBLE_EQ(drawing.views[i].placement.y, spec.views[i].placement.y);
        EXPECT_FALSE(drawing.views[i].edges.empty()) << "projected again from the part";
    }
    EXPECT_DOUBLE_EQ(drawing.views[0].placement.x, 33.0);
    EXPECT_FALSE(drawing.views[1].showHidden);
    std::filesystem::remove_all(dir);
}

// The part beside a drawing is saved relative to it, so the two can move
// together; it was saved as given, and read from the working folder.
TEST(DrawingDocumentIOTest, APartBesideTheDrawingMovesWithIt) {
    const auto base = std::filesystem::temp_directory_path() / "hz_dwg_move";
    std::filesystem::remove_all(base);
    const auto first = base / "first";
    std::filesystem::create_directories(first / "parts");
    saveBox((first / "parts" / "box.hzpart").string());
    DrawingDocumentSpec spec;
    spec.partPath = (first / "parts" / "box.hzpart").string();
    ASSERT_TRUE(DrawingDocumentIO::save((first / "box.hzdwg").string(), spec));

    const auto second = base / "second";
    std::filesystem::rename(first, second);
    DrawingDocumentSpec back;
    Drawing drawing;
    std::string error;
    ASSERT_TRUE(DrawingDocumentIO::load((second / "box.hzdwg").string(), back, drawing, &error))
        << error;
    EXPECT_EQ(std::filesystem::path(back.partPath), second / "parts" / "box.hzpart");
    EXPECT_EQ(drawing.views.size(), 4u) << "no views saved: the sheet layout";
    std::filesystem::remove_all(base);
}

// A drawing file with fields of the wrong kind is read or refused with a
// reason; it does not throw. A part named by a number threw from the reader.
TEST(DrawingDocumentIOTest, AHostileDrawingIsReadOrRefusedNotThrown) {
    const auto dir = std::filesystem::temp_directory_path() / "hz_dwg_hostile";
    std::filesystem::create_directories(dir);
    saveBox((dir / "box.hzpart").string());
    const std::string dwg = (dir / "d.hzdwg").string();
    DrawingDocumentSpec spec;
    Drawing drawing;
    std::string error;

    write(dwg, R"({"part": 5, "version": 2})");
    EXPECT_FALSE(DrawingDocumentIO::load(dwg, spec, drawing, &error));
    EXPECT_FALSE(error.empty());

    write(dwg, R"([1, 2, 3])");
    EXPECT_FALSE(DrawingDocumentIO::load(dwg, spec, drawing, &error));

    write(dwg, R"({"part": "box.hzpart", "version": 1e300, "gap": "wide",
                   "sheet": {"paper": 3, "margin": -4},
                   "titleBlock": {"title": 7, "width": "x"},
                   "views": [{"kind": 1, "direction": [0, 0], "scale": -2,
                              "placement": ["a", 1]}, 5, {"direction": [1e308, 1e308, 1e308]}]})");
    ASSERT_TRUE(DrawingDocumentIO::load(dwg, spec, drawing, &error)) << error;
    EXPECT_DOUBLE_EQ(spec.gap, 10.0);
    EXPECT_DOUBLE_EQ(spec.sheet.margin, 10.0);
    ASSERT_EQ(drawing.views.size(), 2u);
    for (const auto& v : drawing.views) {
        EXPECT_DOUBLE_EQ(v.scale, 1.0);
        EXPECT_FALSE(v.edges.empty());
        for (const auto& e : v.edges) {
            EXPECT_TRUE(std::isfinite(e.a.x) && std::isfinite(e.a.y) && std::isfinite(e.b.x) &&
                        std::isfinite(e.b.y))
                << "a direction too long to normalize is the view's standard one";
        }
    }
    std::filesystem::remove_all(dir);
}

// A view drawn at a scale draws its edges at that scale, and states its
// dimensions at 1:1; it leaves out the hidden and tangent edges it says to.
TEST(DrawingDocumentIOTest, AScaledViewDrawsAtScaleAndStatesTrueLengths) {
    hz::model::DrawingView view;
    hz::model::ProjectedEdge edge;
    edge.a = {0.0, 0.0};
    edge.b = {40.0, 0.0};
    edge.sourceEdge = hz::topo::TopologyID::make("box", "edge0");
    view.edges.push_back(edge);
    hz::model::ProjectedEdge hidden = edge;
    hidden.a = {0.0, 10.0};
    hidden.b = {40.0, 10.0};
    hidden.visibility = hz::model::ProjectedEdge::Visibility::Hidden;
    hidden.sourceEdge = hz::topo::TopologyID::make("box", "edge1");
    view.edges.push_back(hidden);
    hz::model::ProjectedEdge tangent = edge;
    tangent.a = {0.0, 20.0};
    tangent.b = {40.0, 20.0};
    tangent.kind = hz::model::ProjectedEdge::Kind::Tangent;
    view.edges.push_back(tangent);
    view.boundsMin = {0.0, 0.0};
    view.boundsMax = {40.0, 20.0};
    view.placement = {100.0, 50.0};
    view.scale = 0.5;
    view.showHidden = false;
    view.showTangentEdges = false;

    hz::model::LinearDimension dim;
    dim.edge = edge.sourceEdge;
    const auto drafted = hz::io::DrawingDimensionRenderer::render(view, dim, 5.0);
    ASSERT_NE(drafted, nullptr);
    EXPECT_EQ(drafted->textOverride(), hz::draft::DimensionStyle{}.formatLength(40.0))
        << "40 long, drawn 20 long at 1:2";

    // In the document's style: a drawing in inches states inches.
    hz::draft::DimensionStyle inches;
    inches.unit = "in";
    inches.precision = 3;
    inches.showUnits = true;
    const auto styled = hz::io::DrawingDimensionRenderer::render(view, dim, 5.0, inches);
    ASSERT_NE(styled, nullptr);
    EXPECT_EQ(styled->textOverride(), inches.formatLength(40.0));
    EXPECT_NE(styled->textOverride(), drafted->textOverride());

    Document sheet;
    Drawing drawing;
    drawing.views.push_back(view);
    hz::io::DrawingExport::populate(sheet, drawing);
    int lines = 0;
    for (const auto& e : sheet.draftDocument().entities()) {
        const auto* line = dynamic_cast<const hz::draft::DraftLine*>(e.get());
        if (line == nullptr) continue;
        ++lines;
        EXPECT_NEAR((line->end() - line->start()).length(), 20.0, 1e-9) << "at 1:2";
        EXPECT_NEAR(line->start().x, 100.0, 1e-9);
    }
    EXPECT_EQ(lines, 1) << "the hidden and the tangent edge are left out";
}

// A file asking for thousands of views is refused, not read: each is a
// projection of the whole part, and the file's size did not bound them.
TEST(DrawingDocumentIOTest, TooManyViewsAreRefused) {
    const auto dir = std::filesystem::temp_directory_path() / "hz_dwg_views";
    std::filesystem::create_directories(dir);
    saveBox((dir / "box.hzpart").string());
    std::string views = "[";
    for (int i = 0; i < 5000; ++i) views += i == 0 ? "{}" : ",{}";
    views += "]";
    const std::string dwg = (dir / "d.hzdwg").string();
    write(dwg, R"({"part": "box.hzpart", "version": 2, "views": )" + views + "}");
    DrawingDocumentSpec spec;
    Drawing drawing;
    std::string error;
    EXPECT_FALSE(DrawingDocumentIO::load(dwg, spec, drawing, &error));
    EXPECT_NE(error.find("views"), std::string::npos) << error;
    std::filesystem::remove_all(dir);
}
