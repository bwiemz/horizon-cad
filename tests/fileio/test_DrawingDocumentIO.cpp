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
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/DrawingDimensionRenderer.h"
#include "horizon/fileio/DrawingDocumentIO.h"
#include "horizon/fileio/DrawingExport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/Solid.h"

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

// The scale chosen for the sheet is kept; none, or one that is no scale,
// reads as the largest that fits. The spec reads without its part, which
// need not be there yet.
TEST(DrawingDocumentIOTest, TheSheetScaleIsKeptAndReadWithoutThePart) {
    const auto dir = std::filesystem::temp_directory_path() / "hz_dwg_scale";
    std::filesystem::create_directories(dir);
    const std::string dwg = (dir / "d.hzdwg").string();
    DrawingDocumentSpec spec;
    spec.partPath = (dir / "not-yet.hzpart").string();
    spec.scale = 0.5;
    ASSERT_TRUE(DrawingDocumentIO::save(dwg, spec));
    DrawingDocumentSpec read;
    std::string error;
    ASSERT_TRUE(DrawingDocumentIO::readSpec(dwg, read, &error)) << error;
    EXPECT_DOUBLE_EQ(read.scale, 0.5);
    EXPECT_EQ(read.version, 3);
    EXPECT_TRUE(
        std::filesystem::equivalent(dir, std::filesystem::path(read.partPath).parent_path()));
    Drawing drawing;
    EXPECT_FALSE(DrawingDocumentIO::load(dwg, read, drawing, &error)) << "load needs the part";

    for (const char* scale : {"-2", "0", "1e300", "\"big\""}) {
        write(dwg, std::string(R"({"part": "p.hzpart", "version": 2, "scale": )") + scale + "}");
        ASSERT_TRUE(DrawingDocumentIO::readSpec(dwg, read)) << scale;
        EXPECT_DOUBLE_EQ(read.scale, 0.0) << scale;
    }
    std::filesystem::remove_all(dir);
}

// ---------------------------------------------------------------------------
// Version 3 (Phase 149): sections and details.
// ---------------------------------------------------------------------------

namespace {

/// The box part at @p path, read into @p part and rebuilt: its solid.
const hz::topo::Solid* boxSolid(const std::string& path, Document& part) {
    EXPECT_TRUE(NativeFormat::load(path, part));
    EXPECT_TRUE(part.rebuildModel());
    return part.solid();
}

/// A sheet of the box: its standard views, section A-A cut through the
/// Front view's middle, and detail B of the Front view at twice its scale.
DrawingDocumentSpec sectionedSpec(const std::string& partPath, const hz::topo::Solid& solid) {
    DrawingDocumentSpec spec;
    spec.partPath = partPath;
    const Drawing laid =
        hz::model::DrawingGenerator::sheetLayout(solid, spec.sheet, spec.titleBlock);
    spec.views = DrawingDocumentIO::viewsOf(laid);
    const auto& front = laid.views[0];
    hz::math::BoundingBox box;
    for (const auto& v : solid.vertices()) box.expand(v.point);

    hz::io::DrawingViewSpec section;
    section.role = hz::model::ViewRole::Section;
    section.label = "A";
    section.source = 0;
    section.projection.origin = box.center();
    section.projection.dir = {-1.0, 0.0, 0.0};  // Front's right: a vertical cut, looking left
    section.scale = front.scale;
    section.placement = {20.0, 200.0};
    spec.views.push_back(section);

    hz::io::DrawingViewSpec detail;
    detail.role = hz::model::ViewRole::Detail;
    detail.label = "B";
    detail.source = 0;
    detail.detailCenter = {front.boundsMin.x, front.boundsMin.y};  // a corner
    detail.detailRadius = 1.0;
    detail.scale = 2.0 * front.scale;
    detail.placement = {20.0, 60.0};
    spec.views.push_back(detail);
    return spec;
}

}  // namespace

// A section and a detail are saved with their role, label and source, and
// built again from the part: the section cut and hatched, the detail the
// part of its source inside its circle.
TEST(DrawingDocumentIOTest, SectionsAndDetailsAreKept) {
    const auto dir = std::filesystem::temp_directory_path() / "hz_dwg_v3";
    std::filesystem::create_directories(dir);
    const std::string partPath = (dir / "box.hzpart").string();
    saveBox(partPath);
    Document part;
    const hz::topo::Solid* solid = boxSolid(partPath, part);
    ASSERT_NE(solid, nullptr);
    const DrawingDocumentSpec spec = sectionedSpec(partPath, *solid);

    const std::string dwg = (dir / "d.hzdwg").string();
    ASSERT_TRUE(DrawingDocumentIO::save(dwg, spec));
    DrawingDocumentSpec read;
    std::string error;
    ASSERT_TRUE(DrawingDocumentIO::readSpec(dwg, read, &error)) << error;
    EXPECT_EQ(read.version, 3);
    ASSERT_EQ(read.views.size(), 6u);
    EXPECT_EQ(read.views[4].role, hz::model::ViewRole::Section);
    EXPECT_EQ(read.views[4].label, "A");
    EXPECT_EQ(read.views[4].source, 0);
    EXPECT_EQ(read.views[5].role, hz::model::ViewRole::Detail);
    EXPECT_EQ(read.views[5].label, "B");
    EXPECT_EQ(read.views[5].source, 0);
    EXPECT_DOUBLE_EQ(read.views[5].detailRadius, 1.0);
    EXPECT_DOUBLE_EQ(read.views[5].detailCenter.x, spec.views[5].detailCenter.x);

    const Drawing built = DrawingDocumentIO::build(*solid, read);
    ASSERT_EQ(built.views.size(), 6u);
    const auto& section = built.views[4];
    EXPECT_EQ(section.role, hz::model::ViewRole::Section);
    EXPECT_FALSE(section.sectionLoops.empty()) << "cut through the middle";
    EXPECT_FALSE(section.sectionHatch.empty());
    const auto& detail = built.views[5];
    EXPECT_EQ(detail.source, 0);
    ASSERT_FALSE(detail.edges.empty());
    for (const auto& e : detail.edges) {
        for (const auto& p : {e.a, e.b}) {
            EXPECT_LE(std::hypot(p.x - detail.detailCenter.x, p.y - detail.detailCenter.y),
                      1.0 + 1e-9)
                << "inside its circle, not enlarged";
        }
    }
    EXPECT_DOUBLE_EQ(detail.scale, 2.0 * built.views[0].scale);
    std::filesystem::remove_all(dir);
}

// A section or detail taken from nothing, from itself or a later view, or
// from another section is refused with a reason; a detail needs a circle; a label is
// kept to a few characters on one line.
TEST(DrawingDocumentIOTest, ABrokenSectionOrDetailIsRefused) {
    const auto dir = std::filesystem::temp_directory_path() / "hz_dwg_v3_bad";
    std::filesystem::create_directories(dir);
    const std::string dwg = (dir / "d.hzdwg").string();
    const std::string front = R"({"kind": "Front"})";
    const auto file = [&](const std::string& views) {
        write(dwg, R"({"part": "box.hzpart", "version": 3, "views": [)" + views + "]}");
    };
    DrawingDocumentSpec spec;
    std::string error;

    file(front + R"(, {"role": "detail", "source": 2, "detailCenter": [0, 0], "detailRadius": 1})");
    EXPECT_FALSE(DrawingDocumentIO::readSpec(dwg, spec, &error));
    EXPECT_NE(error.find("view 2"), std::string::npos) << error;

    file(front + R"(, {"role": "detail", "source": 0, "detailCenter": [0, 0], "detailRadius": 0})");
    EXPECT_FALSE(DrawingDocumentIO::readSpec(dwg, spec, &error));
    EXPECT_NE(error.find("circle"), std::string::npos) << error;

    file(front + R"(, {"role": "section", "source": 0, "direction": [1, 0, 0]},)" +
         R"({"role": "section", "source": 1, "direction": [0, 0, 1]})");
    EXPECT_FALSE(DrawingDocumentIO::readSpec(dwg, spec, &error));
    EXPECT_NE(error.find("not a projection"), std::string::npos) << error;

    // A section says where it cuts, on a view before it: none, itself or a
    // later one is refused. It was read: a caption marking nothing.
    for (const std::string& source :
         {std::string(), std::string(R"("source": -1, )"), std::string(R"("source": 1, )"),
          std::string(R"("source": 5, )")}) {
        file(front + R"(, {"role": "section", )" + source + R"("direction": [1, 0, 0]})");
        EXPECT_FALSE(DrawingDocumentIO::readSpec(dwg, spec, &error)) << source;
        EXPECT_NE(error.find("section of no view"), std::string::npos) << error;
    }

    file(front + R"(, {"role": "section", "source": 0, "direction": [1, 0, 0],)" +
         R"( "label": "A\nBCDEFGHIJKLMNOP"})");
    ASSERT_TRUE(DrawingDocumentIO::readSpec(dwg, spec, &error)) << error;
    EXPECT_EQ(spec.views[1].label, "ABCDEFGH");
    std::filesystem::remove_all(dir);
}

// Dimensions are kept by their edges' names and measured again from the
// part; one whose edge the part no longer has is said, not drawn; the
// centre-line switch is kept. A file asking for thousands is refused.
TEST(DrawingDocumentIOTest, DimensionsAreKeptByTheirEdges) {
    const auto dir = std::filesystem::temp_directory_path() / "hz_dwg_dims";
    std::filesystem::create_directories(dir);
    const std::string partPath = (dir / "box.hzpart").string();
    saveBox(partPath);
    Document part;
    const hz::topo::Solid* solid = boxSolid(partPath, part);
    ASSERT_NE(solid, nullptr);

    DrawingDocumentSpec spec;
    spec.partPath = partPath;
    spec.views = DrawingDocumentIO::viewsOf(
        hz::model::DrawingGenerator::sheetLayout(*solid, spec.sheet, spec.titleBlock));
    const Drawing laid = DrawingDocumentIO::build(*solid, spec);
    // Front's first edge seen along its length.
    std::string edge;
    for (const auto& e : laid.views[0].edges) {
        if ((e.b - e.a).length() > 1e-6 && !e.sourceEdge.tag().empty()) {
            edge = e.sourceEdge.tag();
            break;
        }
    }
    ASSERT_FALSE(edge.empty());
    spec.views[0].dimensions.push_back({edge, hz::io::DrawingDimensionSpec::Kind::Length});
    spec.views[0].dimensions.push_back(
        {"box/no-such-edge", hz::io::DrawingDimensionSpec::Kind::Length});
    spec.views[1].showCentreLines = false;

    const std::string dwg = (dir / "d.hzdwg").string();
    ASSERT_TRUE(DrawingDocumentIO::save(dwg, spec));
    DrawingDocumentSpec read;
    std::string error;
    ASSERT_TRUE(DrawingDocumentIO::readSpec(dwg, read, &error)) << error;
    ASSERT_EQ(read.views[0].dimensions.size(), 2u);
    EXPECT_EQ(read.views[0].dimensions[0].edge, edge);
    EXPECT_FALSE(read.views[1].showCentreLines);
    EXPECT_TRUE(read.views[0].showCentreLines);

    std::vector<std::string> lost;
    const Drawing built = DrawingDocumentIO::build(*solid, read, &lost);
    ASSERT_EQ(built.views[0].dimensions.size(), 1u) << "the edge the part has";
    EXPECT_GT(built.views[0].dimensions[0].value, 0.0) << "measured from the part";
    ASSERT_EQ(lost.size(), 1u);
    EXPECT_NE(lost[0].find("view 1"), std::string::npos) << lost[0];
    EXPECT_FALSE(built.views[1].showCentreLines);

    std::string many = "[";
    for (int i = 0; i < 300; ++i)
        many += std::string(i == 0 ? "" : ",") + R"({"edge": "e", "kind": "length"})";
    many += "]";
    write(dwg,
          R"({"part": "box.hzpart", "version": 3, "views": [{"kind": "Front", "dimensions": )" +
              many + "}]}");
    EXPECT_FALSE(DrawingDocumentIO::readSpec(dwg, read, &error));
    EXPECT_NE(error.find("dimensions"), std::string::npos) << error;
    write(dwg, R"({"part": "box.hzpart", "version": 3, "views": [{"kind": "Front", "dimensions": )"
               R"([{"edge": "e", "kind": "wingspan"}, {"kind": "length"}, 7]}]})");
    ASSERT_TRUE(DrawingDocumentIO::readSpec(dwg, read, &error)) << error;
    EXPECT_TRUE(read.views[0].dimensions.empty()) << "none it could not read";
    std::filesystem::remove_all(dir);
}

// A circle is one edge of many chords: a dimension names the circle, and is
// measured through its chords and drawn on all of it.
TEST(DrawingDocumentIOTest, ACircleIsDimensionedByItsOwnName) {
    Document part;
    part.setType(DocumentType::Part);
    part.featureTree().addFeature(PrimitiveFeature::makeCylinder(10.0, 30.0));
    ASSERT_TRUE(part.rebuildModel());
    std::string rim;
    for (const auto& e : part.solid()->edges()) {
        const std::string logical = hz::model::logicalEdge(e.topoId.tag());
        if (logical != e.topoId.tag()) {
            rim = logical;
            break;
        }
    }
    ASSERT_FALSE(rim.empty()) << "a rim of chords";

    DrawingDocumentSpec spec;
    spec.views = DrawingDocumentIO::viewsOf(
        hz::model::DrawingGenerator::sheetLayout(*part.solid(), spec.sheet, spec.titleBlock));
    spec.views[1].dimensions.push_back({rim, hz::io::DrawingDimensionSpec::Kind::Diameter});
    std::vector<std::string> lost;
    const Drawing built = DrawingDocumentIO::build(*part.solid(), spec, &lost);
    EXPECT_TRUE(lost.empty());
    ASSERT_EQ(built.views[1].radialDimensions.size(), 1u);
    EXPECT_NEAR(built.views[1].radialDimensions[0].value, 10.0, 1e-9);
    EXPECT_TRUE(built.views[1].radialDimensions[0].diameter);

    Document sheet;
    hz::io::DrawingExport::populate(sheet, built);
    bool drawn = false;
    for (const auto& e : sheet.draftDocument().entities()) {
        const auto* t = dynamic_cast<const hz::draft::DraftText*>(e.get());
        drawn = drawn || (t != nullptr && e->layer() == "Dimensions" &&
                          t->text().find("20.00") != std::string::npos);
    }
    EXPECT_TRUE(drawn) << "fitted to the whole circle";
}
