// DXF geometry read as the file meant it (Phase 108a): polyline arcs,
// mirrored object coordinate systems, the old POLYLINE form, partial
// ellipses, inserts with unequal or mirrored scales, and blocks inside
// blocks. Fixtures are written to the DXF reference's group codes.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/ImportReport.h"

using hz::io::ImportReport;
using hz::math::Vec2;

namespace {

/// A DXF file: `blocks` go in the BLOCKS section, `entities` in ENTITIES.
std::string dxf(const std::string& entities, const std::string& blocks = "") {
    return "0\nSECTION\n2\nBLOCKS\n" + blocks + "0\nENDSEC\n0\nSECTION\n2\nENTITIES\n" + entities +
           "0\nENDSEC\n0\nEOF\n";
}

struct Loaded {
    hz::doc::Document doc;
    ImportReport report;
    std::string error;
    bool ok = false;
    explicit Loaded(const std::string& text) {
        ok = hz::io::DxfFormat::loadFromString(text, doc, &error, &report);
    }
    const auto& entities() const { return doc.draftDocument().entities(); }
    template <typename T>
    std::vector<const T*> all() const {
        std::vector<const T*> out;
        for (const auto& e : entities()) {
            if (const auto* t = dynamic_cast<const T*>(e.get())) out.push_back(t);
        }
        return out;
    }
};

bool near(const Vec2& a, const Vec2& b, double tol = 1e-9) {
    return (a - b).length() <= tol;
}

bool contains(const std::vector<std::string>& lines, const std::string& part) {
    for (const auto& line : lines) {
        if (line.find(part) != std::string::npos) return true;
    }
    return false;
}

}  // namespace

TEST(DxfFidelityTest, PolylineArcsComeInAsArcs) {
    // A 10 x 2 slot: bulge 1 is a half circle, counterclockwise.
    Loaded in(
        dxf("0\nLWPOLYLINE\n8\n0\n90\n4\n70\n1\n"
            "10\n0\n20\n0\n"
            "10\n10\n20\n0\n42\n1\n"
            "10\n10\n20\n2\n"
            "10\n0\n20\n2\n42\n1\n"));
    ASSERT_TRUE(in.ok) << in.error;
    ASSERT_EQ(in.all<hz::draft::DraftLine>().size(), 2u);
    const auto arcs = in.all<hz::draft::DraftArc>();
    ASSERT_EQ(arcs.size(), 2u) << "the ends are arcs, not straight lines across";
    for (const auto* arc : arcs) EXPECT_NEAR(arc->radius(), 1.0, 1e-12);
    EXPECT_TRUE(near(arcs[0]->center(), Vec2(10, 1)));
    EXPECT_TRUE(near(arcs[1]->center(), Vec2(0, 1)));
    // The right end bulges out, past x = 10.
    EXPECT_NEAR(arcs[0]->midPoint().x, 11.0, 1e-9);
    const auto group = in.entities().front()->groupId();
    EXPECT_NE(group, 0u);
    for (const auto& e : in.entities()) EXPECT_EQ(e->groupId(), group) << "kept together";
    EXPECT_TRUE(contains(in.report.approximated, "1 LWPOLYLINE entity: arcs brought in as lines"));
}

TEST(DxfFidelityTest, AMirroredCoordinateSystemIsMirrored) {
    // Extrusion (0, 0, -1): the entity's x runs the other way.
    Loaded in(
        dxf("0\nARC\n8\n0\n10\n10\n20\n0\n40\n2\n50\n0\n51\n90\n210\n0\n220\n0\n230\n-1\n"
            "0\nCIRCLE\n8\n0\n10\n5\n20\n3\n40\n1\n210\n0\n220\n0\n230\n-1\n"));
    ASSERT_TRUE(in.ok) << in.error;
    const auto arcs = in.all<hz::draft::DraftArc>();
    ASSERT_EQ(arcs.size(), 1u);
    EXPECT_TRUE(near(arcs[0]->center(), Vec2(-10, 0)));
    // Its quarter from 0 to 90 degrees becomes the quarter from 90 to 180.
    EXPECT_TRUE(near(arcs[0]->startPoint(), Vec2(-10, 2)));
    EXPECT_TRUE(near(arcs[0]->endPoint(), Vec2(-12, 0)));
    const auto circles = in.all<hz::draft::DraftCircle>();
    ASSERT_EQ(circles.size(), 1u);
    EXPECT_TRUE(near(circles[0]->center(), Vec2(-5, 3)));
}

TEST(DxfFidelityTest, AnEntityOutOfThePlaneIsReportedNotFlattened) {
    Loaded in(dxf("0\nCIRCLE\n8\n0\n10\n0\n20\n0\n40\n1\n210\n1\n220\n0\n230\n0\n"));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_TRUE(in.entities().empty());
    EXPECT_TRUE(
        contains(in.report.skipped, "1 CIRCLE entity (not in the drawing's plane) not read"));
}

TEST(DxfFidelityTest, TheOldPolylineFormIsRead) {
    Loaded in(
        dxf("0\nPOLYLINE\n8\n0\n66\n1\n70\n1\n"
            "0\nVERTEX\n8\n0\n10\n0\n20\n0\n"
            "0\nVERTEX\n8\n0\n10\n4\n20\n0\n"
            "0\nVERTEX\n8\n0\n10\n4\n20\n3\n"
            "0\nVERTEX\n8\n0\n10\n0\n20\n3\n"
            "0\nSEQEND\n8\n0\n"
            "0\nPOLYLINE\n8\n0\n66\n1\n70\n8\n"
            "0\nVERTEX\n8\n0\n10\n0\n20\n0\n30\n1\n"
            "0\nVERTEX\n8\n0\n10\n1\n20\n0\n30\n2\n"
            "0\nSEQEND\n8\n0\n"));
    ASSERT_TRUE(in.ok) << in.error;
    const auto polys = in.all<hz::draft::DraftPolyline>();
    ASSERT_EQ(polys.size(), 1u);
    EXPECT_TRUE(polys[0]->closed());
    ASSERT_EQ(polys[0]->points().size(), 4u);
    EXPECT_TRUE(near(polys[0]->points()[2], Vec2(4, 3)));
    EXPECT_TRUE(contains(in.report.skipped, "1 POLYLINE entity (a 3D polyline or mesh) not read"));
    EXPECT_FALSE(contains(in.report.skipped, "VERTEX")) << "its vertices are not strays";
}

TEST(DxfFidelityTest, APartialEllipseIsItsArcNotTheWholeEllipse) {
    // Major axis 4 along x, ratio 0.5; parameters 0 to pi: the upper half.
    Loaded in(dxf(
        "0\nELLIPSE\n8\n0\n10\n0\n20\n0\n11\n4\n21\n0\n40\n0.5\n41\n0\n42\n3.14159265358979\n"
        "0\nELLIPSE\n8\n0\n10\n20\n20\n0\n11\n4\n21\n0\n40\n0.5\n41\n0\n42\n6.28318530717959\n"));
    ASSERT_TRUE(in.ok) << in.error;
    const auto polys = in.all<hz::draft::DraftPolyline>();
    ASSERT_EQ(polys.size(), 1u);
    EXPECT_TRUE(near(polys[0]->points().front(), Vec2(4, 0), 1e-9));
    EXPECT_TRUE(near(polys[0]->points().back(), Vec2(-4, 0), 1e-6));
    for (const auto& p : polys[0]->points()) {
        EXPECT_GE(p.y, -1e-9) << "the upper half only";
        EXPECT_NEAR(p.x * p.x / 16.0 + p.y * p.y / 4.0, 1.0, 1e-9) << "on the ellipse";
    }
    EXPECT_EQ(in.all<hz::draft::DraftEllipse>().size(), 1u) << "a whole one stays an ellipse";
    EXPECT_TRUE(contains(in.report.approximated, "1 ELLIPSE entity: partial"));
}

TEST(DxfFidelityTest, InsertsKeepTheirScalesExactly) {
    const std::string block =
        "0\nBLOCK\n8\n0\n2\nTick\n70\n0\n10\n0\n20\n0\n"
        "0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1\n21\n1\n"
        "0\nCIRCLE\n8\n0\n10\n3\n20\n0\n40\n1\n"
        "0\nENDBLK\n8\n0\n";
    Loaded in(
        dxf("0\nINSERT\n8\n0\n2\nTick\n10\n10\n20\n0\n41\n2\n42\n1\n"
            "0\nINSERT\n8\n0\n2\nTick\n10\n0\n20\n10\n41\n-1\n42\n1\n"
            "0\nINSERT\n8\n0\n2\nTick\n10\n0\n20\n20\n41\n2\n42\n2\n",
            block));
    ASSERT_TRUE(in.ok) << in.error;

    // Unequal scales (2, 1): exploded, the line stretched and the circle an
    // ellipse — the scales used to be averaged to 1.5.
    const auto lines = in.all<hz::draft::DraftLine>();
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(10, 0)));
    EXPECT_TRUE(near(lines[0]->end(), Vec2(12, 1)));
    const auto ellipses = in.all<hz::draft::DraftEllipse>();
    ASSERT_EQ(ellipses.size(), 1u);
    EXPECT_TRUE(near(ellipses[0]->center(), Vec2(16, 0)));
    EXPECT_NEAR(ellipses[0]->semiMajor(), 2.0, 1e-12);
    EXPECT_NEAR(ellipses[0]->semiMinor(), 1.0, 1e-12);

    // Mirrored (-1, 1): exploded, mirrored — it used to lose the mirror.
    EXPECT_TRUE(near(lines[1]->start(), Vec2(0, 10)));
    EXPECT_TRUE(near(lines[1]->end(), Vec2(-1, 11)));
    const auto circles = in.all<hz::draft::DraftCircle>();
    ASSERT_EQ(circles.size(), 1u);
    EXPECT_TRUE(near(circles[0]->center(), Vec2(-3, 10)));

    // Equal positive scales stay a block reference.
    const auto refs = in.all<hz::draft::DraftBlockRef>();
    ASSERT_EQ(refs.size(), 1u);
    EXPECT_DOUBLE_EQ(refs[0]->uniformScale(), 2.0);
    EXPECT_TRUE(contains(in.report.approximated, "2 INSERT entities: unequal or mirrored scales"));
}

TEST(DxfFidelityTest, ABlockInsideABlockIsFlattenedIntoIt) {
    // "Frame" inserts "Corner", which is defined after it.
    const std::string blocks =
        "0\nBLOCK\n8\n0\n2\nFrame\n70\n0\n10\n0\n20\n0\n"
        "0\nLINE\n8\n0\n10\n0\n20\n0\n11\n10\n21\n0\n"
        "0\nINSERT\n8\n0\n2\nCorner\n10\n10\n20\n0\n"
        "0\nENDBLK\n8\n0\n"
        "0\nBLOCK\n8\n0\n2\nCorner\n70\n0\n10\n0\n20\n0\n"
        "0\nLINE\n8\n0\n10\n0\n20\n0\n11\n0\n21\n1\n"
        "0\nENDBLK\n8\n0\n";
    Loaded in(dxf("0\nINSERT\n8\n0\n2\nFrame\n10\n0\n20\n0\n", blocks));
    ASSERT_TRUE(in.ok) << in.error;
    const auto frame = in.doc.draftDocument().blockTable().findBlock("Frame");
    ASSERT_NE(frame, nullptr);
    ASSERT_EQ(frame->entities.size(), 2u);
    const auto* corner = dynamic_cast<const hz::draft::DraftLine*>(frame->entities[1].get());
    ASSERT_NE(corner, nullptr);
    EXPECT_TRUE(near(corner->start(), Vec2(10, 0)));
    EXPECT_TRUE(near(corner->end(), Vec2(10, 1)));
    EXPECT_NE(in.doc.draftDocument().blockTable().findBlock("Corner"), nullptr);
    EXPECT_TRUE(contains(in.report.approximated, "1 INSERT entity: inside a block, flattened"));
}

TEST(DxfFidelityTest, AnOutOfPlanePolylineWithoutSeqendDoesNotSwallowWhatFollows) {
    Loaded in(
        dxf("0\nPOLYLINE\n8\n0\n66\n1\n70\n0\n210\n1\n220\n0\n230\n0\n"
            "0\nVERTEX\n8\n0\n10\n0\n20\n0\n"
            "0\nVERTEX\n8\n0\n10\n1\n20\n0\n"
            "0\nLINE\n8\n0\n10\n0\n20\n0\n11\n5\n21\n5\n"));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_EQ(in.all<hz::draft::DraftLine>().size(), 1u) << "the line after it is kept";
    EXPECT_TRUE(contains(in.report.skipped, "1 POLYLINE entity (not in the drawing's plane)"));
}

TEST(DxfFidelityTest, AnExplodedInsertsNamedLayerContentKeepsItsLayerColour) {
    // A red insert with unequal scales: the content on layer 0 takes red; the
    // content on layer STEEL keeps STEEL's colour (ByLayer), not red.
    const std::string block =
        "0\nBLOCK\n8\n0\n2\nWidget\n70\n0\n10\n0\n20\n0\n"
        "0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1\n21\n0\n"
        "0\nLINE\n8\nSTEEL\n10\n0\n20\n1\n11\n1\n21\n1\n"
        "0\nENDBLK\n8\n0\n";
    Loaded in(dxf("0\nINSERT\n8\nPARTS\n62\n1\n2\nWidget\n10\n0\n20\n0\n41\n2\n42\n1\n", block));
    ASSERT_TRUE(in.ok) << in.error;
    const auto lines = in.all<hz::draft::DraftLine>();
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_EQ(lines[0]->layer(), "PARTS");
    EXPECT_NE(lines[0]->color(), 0u) << "layer 0 content takes the insert's colour";
    EXPECT_EQ(lines[1]->layer(), "STEEL");
    EXPECT_EQ(lines[1]->color(), 0u) << "named-layer content stays ByLayer";
}

TEST(DxfFidelityTest, AnEllipsesCenterIsInWorldCoordinatesWhateverItsExtrusion) {
    // The DXF reference puts an ELLIPSE's center (10) and major axis (11) in
    // world coordinates; a reversed extrusion only reverses which way its
    // parameter runs. So the center stays at (5, 0), and the half from
    // parameter 0 to pi runs below the axis instead of above it.
    Loaded in(
        dxf("0\nELLIPSE\n8\n0\n10\n5\n20\n0\n11\n2\n21\n0\n40\n0.5\n41\n0\n42\n6.28318530717959\n"
            "210\n0\n220\n0\n230\n-1\n"
            "0\nELLIPSE\n8\n0\n10\n0\n20\n0\n11\n2\n21\n0\n40\n0.5\n41\n0\n42\n3.14159265358979\n"
            "210\n0\n220\n0\n230\n-1\n"));
    ASSERT_TRUE(in.ok) << in.error;
    const auto ellipses = in.all<hz::draft::DraftEllipse>();
    ASSERT_EQ(ellipses.size(), 1u);
    EXPECT_TRUE(near(ellipses[0]->center(), Vec2(5, 0)));
    const auto polys = in.all<hz::draft::DraftPolyline>();
    ASSERT_EQ(polys.size(), 1u);
    for (const auto& p : polys[0]->points()) EXPECT_LE(p.y, 1e-9) << "below the axis";
}

// -- Hostile files, round 2 (Phase 124) ----------------------------------------

namespace {

double area(const std::vector<Vec2>& pts) {
    double a = 0.0;
    for (size_t k = 0; k < pts.size(); ++k) {
        a += pts[k].x * pts[(k + 1) % pts.size()].y - pts[(k + 1) % pts.size()].x * pts[k].y;
    }
    return 0.5 * a;
}

// The flattening budget, set for one test and put back after it.
struct FlattenBudget {
    size_t saved = hz::io::DxfFormat::maxFlattenedEntities();
    explicit FlattenBudget(size_t limit) { hz::io::DxfFormat::setMaxFlattenedEntities(limit); }
    ~FlattenBudget() { hz::io::DxfFormat::setMaxFlattenedEntities(saved); }
};

}  // namespace

// A hatch of a square with a square island, and a seed point after them.
// Reading every 10/20 as one polygon merged the elevation, both paths and the
// seed into one garbled outline. The outer path is kept, and the island is
// reported.
TEST(DxfFidelityTest, AHatchKeepsItsOuterBoundaryAndReportsItsIslands) {
    Loaded in(dxf(
        "0\nHATCH\n8\n0\n10\n0\n20\n0\n30\n0\n210\n0\n220\n0\n230\n1\n2\nSOLID\n70\n1\n71\n0\n"
        "91\n2\n"
        "92\n3\n72\n0\n73\n1\n93\n4\n10\n0\n20\n0\n10\n10\n20\n0\n10\n10\n20\n10\n10\n0\n20\n10\n"
        "97\n0\n"
        "92\n2\n72\n0\n73\n1\n93\n4\n10\n4\n20\n4\n10\n6\n20\n4\n10\n6\n20\n6\n10\n4\n20\n6\n"
        "97\n0\n"
        "75\n1\n76\n1\n98\n1\n10\n5\n20\n5\n"));
    ASSERT_TRUE(in.ok) << in.error;
    const auto hatches = in.all<hz::draft::DraftHatch>();
    ASSERT_EQ(hatches.size(), 1u);
    const auto& b = hatches[0]->boundary();
    ASSERT_EQ(b.size(), 4u) << "the outer square only";
    EXPECT_NEAR(std::abs(area(b)), 100.0, 1e-9);
    EXPECT_TRUE(contains(in.report.approximated, "islands inside it left out"));
}

// A boundary of edges: a line and a half circle. The arc is followed, in
// segments, and reported as such.
TEST(DxfFidelityTest, AHatchBoundaryOfEdgesFollowsItsArc) {
    Loaded in(
        dxf("0\nHATCH\n8\n0\n10\n0\n20\n0\n30\n0\n2\nANSI31\n70\n0\n71\n0\n91\n1\n"
            "92\n1\n93\n2\n"
            "72\n1\n10\n-5\n20\n0\n11\n5\n21\n0\n"
            "72\n2\n10\n0\n20\n0\n40\n5\n50\n0\n51\n180\n73\n1\n"
            "97\n0\n75\n1\n76\n1\n52\n45\n41\n1\n77\n0\n78\n0\n98\n0\n"));
    ASSERT_TRUE(in.ok) << in.error;
    const auto hatches = in.all<hz::draft::DraftHatch>();
    ASSERT_EQ(hatches.size(), 1u);
    const auto& b = hatches[0]->boundary();
    for (const Vec2& q : b) {
        EXPECT_LE(q.length(), 5.0 + 1e-9);
        EXPECT_GE(q.y, -1e-9);
    }
    EXPECT_NEAR(std::abs(area(b)), 12.5 * 3.14159265358979, 0.5) << "a half disc";
    EXPECT_TRUE(contains(in.report.approximated, "curved boundary brought in as segments"));
}

// Blocks that insert blocks, ten at a time, six deep: a million lines from a
// few kilobytes. The import stops flattening at its budget and says so.
TEST(DxfFidelityTest, NestedBlocksThatMultiplyAreCutAtTheBudget) {
    const FlattenBudget budget(1000);
    std::string blocks;
    for (int level = 0; level < 6; ++level) {
        blocks += "0\nBLOCK\n8\n0\n2\nL" + std::to_string(level) + "\n70\n0\n10\n0\n20\n0\n";
        for (int k = 0; k < 10; ++k) {
            blocks += "0\nINSERT\n8\n0\n2\nL" + std::to_string(level + 1) + "\n10\n" +
                      std::to_string(k) + "\n20\n0\n";
        }
        blocks += "0\nENDBLK\n8\n0\n";
    }
    blocks +=
        "0\nBLOCK\n8\n0\n2\nL6\n70\n0\n10\n0\n20\n0\n"
        "0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1\n21\n0\n0\nENDBLK\n8\n0\n";
    Loaded in(dxf("0\nINSERT\n8\n0\n2\nL0\n10\n0\n20\n0\n41\n1\n42\n2\n", blocks));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_LE(in.entities().size(), 1000u);
    EXPECT_TRUE(contains(in.report.skipped, "the rest were left out"));
}

// DXF takes only 24 lineweights (hundredths of a millimetre). A width of 1.5
// was written as 150; it is now the nearest, 158. A layer's width is written
// too, and read back.
TEST(DxfFidelityTest, LineweightsAreOnesADxfReaderAccepts) {
    hz::doc::Document doc;
    hz::draft::LayerProperties thick;
    thick.name = "Thick";
    thick.lineWidth = 0.35;
    doc.layerManager().addLayer(thick);
    auto line = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0));
    line->setLineWidth(1.5);
    doc.draftDocument().addEntity(line);

    const auto path = std::filesystem::temp_directory_path() / "hz_dxf_lineweights.dxf";
    std::string error;
    ASSERT_TRUE(hz::io::DxfFormat::save(path.string(), doc, &error)) << error;
    std::ifstream file(path);
    std::vector<std::string> lines;
    for (std::string l; std::getline(file, l);) lines.push_back(l);
    static const std::vector<int> valid = {-3, -2, -1,  0,   5,   9,   13,  15,  18,
                                           20, 25, 30,  35,  40,  50,  53,  60,  70,
                                           80, 90, 100, 106, 120, 140, 158, 200, 211};
    int seen = 0;
    for (size_t k = 0; k + 1 < lines.size(); k += 2) {  // group code, then its value
        if (std::stoi(lines[k]) != 370) continue;
        const int value = std::stoi(lines[k + 1]);
        EXPECT_NE(std::find(valid.begin(), valid.end(), value), valid.end()) << value;
        ++seen;
    }
    EXPECT_GE(seen, 2) << "the entity's and the layers'";

    hz::doc::Document back;
    ASSERT_TRUE(hz::io::DxfFormat::load(path.string(), back, &error)) << error;
    ASSERT_NE(back.layerManager().getLayer("Thick"), nullptr);
    EXPECT_NEAR(back.layerManager().getLayer("Thick")->lineWidth, 0.35, 1e-12);
    ASSERT_NE(back.layerManager().getLayer("0"), nullptr);
    EXPECT_NEAR(back.layerManager().getLayer("0")->lineWidth, 1.0, 1e-12) << "the default";
    ASSERT_FALSE(back.draftDocument().entities().empty());
    EXPECT_NEAR(back.draftDocument().entities().front()->lineWidth(), 1.58, 1e-12);
    std::filesystem::remove(path);
}
