// DXF geometry read as the file meant it (Phase 108a): polyline arcs,
// mirrored object coordinate systems, the old POLYLINE form, partial
// ellipses, inserts with unequal or mirrored scales, and blocks inside
// blocks. Fixtures are written to the DXF reference's group codes.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <clocale>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/BlockTable.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftHatch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftSpline.h"
#include "horizon/drafting/DraftText.h"
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

/// The lines of the file at @p path. The file is closed on return: Windows
/// will not remove a file that is still open.
std::vector<std::string> linesOf(const std::filesystem::path& path) {
    std::ifstream file(path);
    std::vector<std::string> lines;
    for (std::string l; std::getline(file, l);) lines.push_back(l);
    return lines;
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
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(10, 0)));
    EXPECT_TRUE(near(lines[0]->end(), Vec2(12, 1)));
    const auto ellipses = in.all<hz::draft::DraftEllipse>();
    ASSERT_EQ(ellipses.size(), 1u);
    EXPECT_TRUE(near(ellipses[0]->center(), Vec2(16, 0)));
    EXPECT_NEAR(ellipses[0]->semiMajor(), 2.0, 1e-12);
    EXPECT_NEAR(ellipses[0]->semiMinor(), 1.0, 1e-12);

    // Mirrored (-1, 1): a mirrored block reference, its content placed
    // mirrored — it was once read without the mirror, then exploded.
    const auto refs = in.all<hz::draft::DraftBlockRef>();
    ASSERT_EQ(refs.size(), 2u);
    EXPECT_TRUE(refs[0]->mirrored());
    EXPECT_DOUBLE_EQ(refs[0]->uniformScale(), 1.0);
    EXPECT_TRUE(near(refs[0]->transformPoint(Vec2(0, 0)), Vec2(0, 10)));
    EXPECT_TRUE(near(refs[0]->transformPoint(Vec2(1, 1)), Vec2(-1, 11)));
    EXPECT_TRUE(near(refs[0]->transformPoint(Vec2(3, 0)), Vec2(-3, 10)));

    // Equal positive scales stay a block reference, as they were.
    EXPECT_FALSE(refs[1]->mirrored());
    EXPECT_DOUBLE_EQ(refs[1]->uniformScale(), 2.0);
    EXPECT_TRUE(contains(in.report.approximated, "1 INSERT entity: unequal scales"));
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

// An insert of a block the drawing already has, holding a reference to another
// block, places that reference's pieces by both: its own scale and mirror
// inside the block, then the insert's. The insert's x scale was replaced by
// the inner reference's.
TEST(DxfFidelityTest, ABlockAlreadyInTheDrawingPlacesTheBlocksInsideIt) {
    Loaded in(dxf(""));
    ASSERT_TRUE(in.ok) << in.error;
    auto& blocks = in.doc.draftDocument().blockTable();
    auto gear = std::make_shared<hz::draft::BlockDefinition>();
    gear->name = "GEAR";
    gear->entities.push_back(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0)));
    blocks.addBlock(gear);
    auto widget = std::make_shared<hz::draft::BlockDefinition>();
    widget->name = "WIDGET";
    widget->entities.push_back(
        std::make_shared<hz::draft::DraftBlockRef>(gear, Vec2(0, 0), 0.0, 3.0));
    auto mirroredGear = std::make_shared<hz::draft::DraftBlockRef>(gear, Vec2(0, 5), 0.0, 3.0);
    mirroredGear->setMirrored(true);
    widget->entities.push_back(mirroredGear);
    blocks.addBlock(widget);

    // Scaled (2, 1): unequal, so placed piece by piece.
    std::string error;
    hz::io::ImportReport report;
    ASSERT_TRUE(hz::io::DxfFormat::loadFromString(
        dxf("0\nINSERT\n8\n0\n2\nWIDGET\n10\n0\n20\n0\n41\n2\n42\n1\n"), in.doc, &error, &report))
        << error;
    const auto lines = in.all<hz::draft::DraftLine>();
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_TRUE(near(lines[0]->end(), Vec2(6, 0))) << "3 inside the block, then 2 across";
    EXPECT_TRUE(near(lines[1]->start(), Vec2(0, 5)));
    EXPECT_TRUE(near(lines[1]->end(), Vec2(-6, 5))) << "mirrored inside, then 2 across";
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

// An arc edge whose angles are absurd still ends, and stays on its circle.
// A sweep of -1e88 degrees had a turn added until it was positive, which
// never happened (the turn is below its last digit), and one of +1e89 made
// a step count no int holds (undefined behaviour; found by fuzzing).
TEST(DxfFidelityTest, AHatchArcEdgeWithAbsurdAnglesEndsOnItsCircle) {
    for (const char* angles : {"50\n1e90\n51\n0\n", "50\n0\n51\n1e89\n"}) {
        Loaded in(dxf(std::string("0\nHATCH\n8\n0\n10\n0\n20\n0\n30\n0\n2\nANSI31\n70\n0\n71\n0\n"
                                  "91\n1\n92\n1\n93\n2\n"
                                  "72\n1\n10\n-5\n20\n0\n11\n5\n21\n0\n"
                                  "72\n2\n10\n0\n20\n0\n40\n5\n") +
                      angles + "73\n1\n97\n0\n75\n1\n76\n1\n52\n45\n41\n1\n77\n0\n78\n0\n98\n0\n"));
        ASSERT_TRUE(in.ok) << in.error;
        for (const auto* hatch : in.all<hz::draft::DraftHatch>()) {
            for (const Vec2& q : hatch->boundary()) EXPECT_LE(q.length(), 5.0 + 1e-9) << angles;
        }
    }
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

// A spline edge followed by more edges. The spline's fit data (97, written
// even when it is 0) was left unread, which stopped the path there: the
// lines after it were lost and the hatch dropped as damaged.
TEST(DxfFidelityTest, AHatchSplineEdgeWithFitDataDoesNotEndItsPath) {
    Loaded in(
        dxf("0\nHATCH\n8\n0\n10\n0\n20\n0\n30\n0\n2\nSOLID\n70\n1\n71\n0\n91\n1\n"
            "92\n1\n93\n3\n"
            "72\n4\n94\n2\n73\n0\n74\n0\n95\n6\n96\n3\n"
            "40\n0\n40\n0\n40\n0\n40\n1\n40\n1\n40\n1\n"
            "10\n0\n20\n0\n10\n10\n20\n0\n10\n10\n20\n10\n"
            "97\n1\n11\n8\n21\n2\n12\n1\n22\n0\n13\n0\n23\n1\n"
            "72\n1\n10\n10\n20\n10\n11\n0\n21\n10\n"
            "72\n1\n10\n0\n20\n10\n11\n0\n21\n0\n"
            "97\n0\n75\n1\n76\n1\n98\n0\n"));
    ASSERT_TRUE(in.ok) << in.error;
    const auto hatches = in.all<hz::draft::DraftHatch>();
    ASSERT_EQ(hatches.size(), 1u) << "the hatch is not dropped";
    EXPECT_NEAR(std::abs(area(hatches[0]->boundary())), 100.0, 1e-9)
        << "the spline's control polygon and both lines";
}

// A block already in the drawing, made only of inserts, ten to a level and
// nine levels deep, over an empty block: a billion placements with nothing
// placed. Only placed entities were counted, so none of it met the budget,
// and importing a DXF that inserts it inside a block did not end.
TEST(DxfFidelityTest, NestedInsertsWithNothingInThemAreCutAtTheBudget) {
    const FlattenBudget budget(1000);
    hz::doc::Document doc;
    auto below = std::make_shared<hz::draft::BlockDefinition>();
    below->name = "C0";
    doc.draftDocument().blockTable().addBlock(below);
    for (int level = 1; level <= 9; ++level) {
        auto def = std::make_shared<hz::draft::BlockDefinition>();
        def->name = "C" + std::to_string(level);
        for (int k = 0; k < 10; ++k) {
            def->entities.push_back(std::make_shared<hz::draft::DraftBlockRef>(below, Vec2(k, 0)));
        }
        doc.draftDocument().blockTable().addBlock(def);
        below = def;
    }
    const std::string text = dxf("0\nINSERT\n8\n0\n2\nOUTER\n10\n0\n20\n0\n",
                                 "0\nBLOCK\n8\n0\n2\nOUTER\n70\n0\n10\n0\n20\n0\n"
                                 "0\nINSERT\n8\n0\n2\nC9\n10\n0\n20\n0\n0\nENDBLK\n8\n0\n");
    ImportReport report;
    std::string error;
    const auto start = std::chrono::steady_clock::now();
    ASSERT_TRUE(hz::io::DxfFormat::loadFromString(text, doc, &error, &report)) << error;
    EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(),
              10.0);
    EXPECT_TRUE(contains(report.skipped, "the rest were left out"));
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
    const std::vector<std::string> lines = linesOf(path);
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

// A thin width was written as 0, the hairline weight, which reads back as no
// width: ByLayer on an entity. It is written as the thinnest weight, 5. A
// layer with no width reads as the default, and is written as the default.
TEST(DxfFidelityTest, AThinWidthIsNotWrittenAsNoWidth) {
    hz::doc::Document doc;
    hz::draft::LayerProperties none;
    none.name = "None";
    none.lineWidth = 0.0;
    doc.layerManager().addLayer(none);
    auto line = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0));
    line->setLineWidth(0.02);
    doc.draftDocument().addEntity(line);

    const auto path = std::filesystem::temp_directory_path() / "hz_dxf_thin_width.dxf";
    std::string error;
    ASSERT_TRUE(hz::io::DxfFormat::save(path.string(), doc, &error)) << error;
    const std::vector<std::string> lines = linesOf(path);
    for (size_t k = 0; k + 1 < lines.size(); k += 2) {
        if (std::stoi(lines[k]) == 370) {
            EXPECT_NE(std::stoi(lines[k + 1]), 0);
        }
    }

    hz::doc::Document back;
    ASSERT_TRUE(hz::io::DxfFormat::load(path.string(), back, &error)) << error;
    ASSERT_FALSE(back.draftDocument().entities().empty());
    EXPECT_NEAR(back.draftDocument().entities().front()->lineWidth(), 0.05, 1e-12) << "not ByLayer";
    ASSERT_NE(back.layerManager().getLayer("None"), nullptr);
    EXPECT_NEAR(back.layerManager().getLayer("None")->lineWidth, 1.0, 1e-12);
    std::filesystem::remove(path);
}

namespace {

/// @p doc saved as a DXF file named @p name in the temporary directory.
std::filesystem::path savedAs(const hz::doc::Document& doc, const std::string& name) {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::string error;
    EXPECT_TRUE(hz::io::DxfFormat::save(path.string(), doc, &error)) << error;
    return path;
}

using Groups = std::vector<std::pair<int, std::string>>;

/// The groups of each entity or table entry of type @p type in the file at
/// @p path, in the order written.
std::vector<Groups> recordsIn(const std::filesystem::path& path, const std::string& type) {
    const std::vector<std::string> lines = linesOf(path);
    std::vector<Groups> out;
    bool inside = false;
    for (size_t k = 0; k + 1 < lines.size(); k += 2) {  // group code, then its value
        const int code = std::stoi(lines[k]);
        if (code == 0) {
            inside = lines[k + 1] == type;
            if (inside) out.emplace_back();
        } else if (inside) {
            out.back().emplace_back(code, lines[k + 1]);
        }
    }
    return out;
}

/// A SPLINE as written: what another program draws from.
struct WrittenSpline {
    int flags = 0;
    int degree = 0;
    int knotCount = 0;
    int pointCount = 0;
    std::vector<double> knots;
    std::vector<double> weights;
    std::vector<Vec2> points;
};

WrittenSpline readSpline(const Groups& groups) {
    WrittenSpline s;
    for (const auto& [code, value] : groups) {
        if (code == 70) s.flags = std::stoi(value);
        if (code == 71) s.degree = std::stoi(value);
        if (code == 72) s.knotCount = std::stoi(value);
        if (code == 73) s.pointCount = std::stoi(value);
        if (code == 40) s.knots.push_back(std::stod(value));
        if (code == 41) s.weights.push_back(std::stod(value));
        if (code == 10) s.points.emplace_back(std::stod(value), 0.0);
        if (code == 20 && !s.points.empty()) s.points.back().y = std::stod(value);
    }
    return s;
}

/// The point at @p u of the B-spline of degree @p p on knots @p U, with
/// control points @p P and weights @p W, by de Boor's algorithm: the curve
/// any program draws from a SPLINE's groups. It is worked out here, apart
/// from DraftSpline's own evaluation, which is the curve the drawing shows.
Vec2 deBoor(int p, const std::vector<double>& U, const std::vector<Vec2>& P,
            const std::vector<double>& W, double u) {
    const int n = static_cast<int>(P.size());
    int k = p;  // the span: U[k] <= u < U[k + 1], the last one closed
    while (k < n - 1 && u >= U[k + 1]) ++k;
    std::vector<std::array<double, 3>> d(p + 1);
    for (int j = 0; j <= p; ++j) {
        const int i = j + k - p;
        d[j] = {P[i].x * W[i], P[i].y * W[i], W[i]};
    }
    for (int r = 1; r <= p; ++r) {
        for (int j = p; j >= r; --j) {
            const double a = (u - U[j + k - p]) / (U[j + 1 + k - r] - U[j + k - p]);
            for (int c = 0; c < 3; ++c) d[j][c] = (1.0 - a) * d[j - 1][c] + a * d[j][c];
        }
    }
    return {d[p][0] / d[p][2], d[p][1] / d[p][2]};
}

/// Splines of each kind the drawing has: open and closed, weighted or not,
/// and with too few points for a cubic, which are drawn straight.
std::vector<std::shared_ptr<hz::draft::DraftSpline>> splinesOfEachKind() {
    using hz::draft::DraftSpline;
    const std::vector<Vec2> six = {{0, 0}, {1, 3}, {4, 3}, {5, 0}, {8, -2}, {10, 1}};
    const std::vector<Vec2> three(six.begin(), six.begin() + 3);
    std::vector<std::shared_ptr<DraftSpline>> out = {
        std::make_shared<DraftSpline>(six, false),
        std::make_shared<DraftSpline>(six, true),
        std::make_shared<DraftSpline>(three, true),
        std::make_shared<DraftSpline>(three, false),
        std::make_shared<DraftSpline>(std::vector<Vec2>(six.begin(), six.begin() + 2), false),
    };
    auto weighted = std::make_shared<DraftSpline>(six, false);
    weighted->setWeights({1, 2, 0.5, 1, 3, 1});
    out.push_back(weighted);
    auto weightedClosed = std::make_shared<DraftSpline>(six, true);
    weightedClosed->setWeights({1, 0.5, 2, 1, 1, 4});
    out.push_back(weightedClosed);
    return out;
}

/// Whether @p a and @p b are the same points, exactly.
bool samePoints(const std::vector<Vec2>& a, const std::vector<Vec2>& b) {
    const auto same = [](const Vec2& p, const Vec2& q) { return p.x == q.x && p.y == q.y; };
    return std::equal(a.begin(), a.end(), b.begin(), b.end(), same);
}

}  // namespace

// Reals were written with six decimals. A whole ellipse's end parameter,
// 2 pi, read back as 6.283185, short of a turn: every ellipse saved came back
// as an open polyline, reported as partial. Reals are now written so they
// read back as the same double.
TEST(DxfFidelityTest, ASavedEllipseComesBackWholeAndRealsExactly) {
    hz::doc::Document doc;
    doc.draftDocument().addEntity(
        std::make_shared<hz::draft::DraftEllipse>(Vec2(8, 8), 3.0, 1.5, 0.4));
    const Vec2 start(0.1 + 0.2, 1e-7);
    const Vec2 end(123456.789012345678, -2.0 / 3.0);
    doc.draftDocument().addEntity(std::make_shared<hz::draft::DraftLine>(start, end));
    const auto path = savedAs(doc, "hz_dxf_reals.dxf");

    Loaded in(dxf(""));
    std::string error;
    ASSERT_TRUE(hz::io::DxfFormat::load(path.string(), in.doc, &error, &in.report)) << error;
    std::filesystem::remove(path);
    const auto ellipses = in.all<hz::draft::DraftEllipse>();
    ASSERT_EQ(ellipses.size(), 1u) << "not an open polyline";
    EXPECT_TRUE(in.all<hz::draft::DraftPolyline>().empty());
    EXPECT_TRUE(in.report.approximated.empty());
    EXPECT_NEAR(ellipses[0]->semiMajor(), 3.0, 1e-12);
    EXPECT_NEAR(ellipses[0]->semiMinor(), 1.5, 1e-12);
    EXPECT_NEAR(ellipses[0]->rotation(), 0.4, 1e-12);
    const auto lines = in.all<hz::draft::DraftLine>();
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0]->start().x, start.x);
    EXPECT_EQ(lines[0]->start().y, start.y) << "1e-7 was written as 0.000000";
    EXPECT_EQ(lines[0]->end().x, end.x);
    EXPECT_EQ(lines[0]->end().y, end.y);
}

// Another program may write a whole turn with six decimals too, or leave the
// end parameter out, when it is 2 pi. Left out, it defaulted to
// std::to_string(2 pi): six decimals again, and under a comma-decimal C
// locale, which Qt sets from the environment, "6,283185", read as 6.
TEST(DxfFidelityTest, AnEllipseOfAWholeTurnToSixDecimalsOrLeftOutIsWhole) {
    const std::string text =
        dxf("0\nELLIPSE\n8\n0\n10\n0\n20\n0\n11\n4\n21\n0\n40\n0.5\n41\n0\n42\n6.283185\n"
            "0\nELLIPSE\n8\n0\n10\n20\n20\n0\n11\n4\n21\n0\n40\n0.5\n41\n0\n");
    {
        Loaded in(text);
        ASSERT_TRUE(in.ok) << in.error;
        EXPECT_EQ(in.all<hz::draft::DraftEllipse>().size(), 2u);
        EXPECT_TRUE(in.report.approximated.empty());
    }
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
    Loaded in(text);
    std::setlocale(LC_NUMERIC, saved.c_str());
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_EQ(in.all<hz::draft::DraftEllipse>().size(), 2u) << "under " << comma;
}

// 20,000 blocks, each inserting the next: 1.9 MB. A block is built inside
// the one that inserts it, a call deeper each time, and the stack ran out (a
// worker's stack is smaller still). Blocks are built 64 deep, and an insert
// deeper than that is reported.
TEST(DxfFidelityTest, BlocksNestedTooDeepAreReportedNotRecursedInto) {
    const auto chain = [](int count) {
        const auto name = [](int k) {
            char text[16];
            std::snprintf(text, sizeof text, "B%05d", k);
            return std::string(text);
        };
        std::string blocks;
        for (int k = 0; k < count; ++k) {
            blocks += "0\nBLOCK\n8\n0\n2\n" + name(k) + "\n70\n0\n10\n0\n20\n0\n";
            if (k + 1 < count) {
                blocks += "0\nINSERT\n8\n0\n2\n" + name(k + 1) + "\n10\n1\n20\n0\n";
            } else {
                blocks += "0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1\n21\n0\n";
            }
            blocks += "0\nENDBLK\n8\n0\n";
        }
        return dxf("0\nINSERT\n8\n0\n2\nB00000\n10\n0\n20\n0\n", blocks);
    };
    {
        Loaded in(chain(64));
        ASSERT_TRUE(in.ok) << in.error;
        const auto top = in.doc.draftDocument().blockTable().findBlock("B00000");
        ASSERT_NE(top, nullptr);
        ASSERT_EQ(top->entities.size(), 1u) << "64 deep is read whole";
        const auto* line = dynamic_cast<const hz::draft::DraftLine*>(top->entities[0].get());
        ASSERT_NE(line, nullptr);
        EXPECT_TRUE(near(line->start(), Vec2(63, 0)));
        EXPECT_TRUE(in.report.skipped.empty());
    }
    Loaded in(chain(20000));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_TRUE(contains(in.report.skipped, "(blocks nested more than 64 deep) not read"));
    EXPECT_EQ(in.all<hz::draft::DraftBlockRef>().size(), 1u);
}

// Every block named "*..." was skipped as a layout (*Model_Space,
// *Paper_Space). Anonymous blocks are named so too: a dynamic block's
// instance or an array (*U), a dimension's picture (*D). An insert of one
// was reported with its block missing. Only the layouts are left out now;
// an anonymous block is read when something inserts it.
TEST(DxfFidelityTest, AnAnonymousBlockIsReadAndTheLayoutsAreNot) {
    const std::string blocks =
        "0\nBLOCK\n8\n0\n2\n*MODEL_SPACE\n70\n0\n10\n0\n20\n0\n0\nENDBLK\n8\n0\n"
        "0\nBLOCK\n8\n0\n2\n*Paper_Space\n70\n0\n10\n0\n20\n0\n"
        "0\nLINE\n8\n0\n67\n1\n10\n0\n20\n0\n11\n420\n21\n0\n0\nENDBLK\n8\n0\n"
        "0\nBLOCK\n8\n0\n2\n*Paper_Space0\n70\n0\n10\n0\n20\n0\n0\nENDBLK\n8\n0\n"
        "0\nBLOCK\n8\n0\n2\n$PAPER_SPACE\n70\n0\n10\n0\n20\n0\n0\nENDBLK\n8\n0\n"
        "0\nBLOCK\n8\n0\n2\n*U1\n70\n1\n10\n0\n20\n0\n"
        "0\nCIRCLE\n8\n0\n10\n0\n20\n0\n40\n2\n0\nENDBLK\n8\n0\n"
        "0\nBLOCK\n8\n0\n2\n*U2\n70\n1\n10\n0\n20\n0\n"
        "0\nINSERT\n8\n0\n2\n*U1\n10\n5\n20\n0\n0\nENDBLK\n8\n0\n"
        "0\nBLOCK\n8\n0\n2\n*D1\n70\n1\n10\n0\n20\n0\n"
        "0\nLINE\n8\n0\n10\n0\n20\n0\n11\n9\n21\n0\n0\nENDBLK\n8\n0\n";
    Loaded in(
        dxf("0\nINSERT\n8\n0\n2\n*U1\n10\n10\n20\n0\n"
            "0\nINSERT\n8\n0\n2\n*U2\n10\n0\n20\n10\n",
            blocks));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_FALSE(contains(in.report.skipped, "missing"));
    ASSERT_EQ(in.all<hz::draft::DraftBlockRef>().size(), 2u);
    const auto& table = in.doc.draftDocument().blockTable();
    const auto u1 = table.findBlock("*U1");
    ASSERT_NE(u1, nullptr);
    ASSERT_EQ(u1->entities.size(), 1u);
    EXPECT_NE(dynamic_cast<const hz::draft::DraftCircle*>(u1->entities[0].get()), nullptr);
    const auto u2 = table.findBlock("*U2");
    ASSERT_NE(u2, nullptr);
    ASSERT_EQ(u2->entities.size(), 1u);
    const auto* inner = dynamic_cast<const hz::draft::DraftCircle*>(u2->entities[0].get());
    ASSERT_NE(inner, nullptr);
    EXPECT_TRUE(near(inner->center(), Vec2(5, 0)));
    for (const char* layout : {"*MODEL_SPACE", "*Paper_Space", "*Paper_Space0", "$PAPER_SPACE"}) {
        EXPECT_EQ(table.findBlock(layout), nullptr) << layout;
    }
    EXPECT_EQ(table.findBlock("*D1"), nullptr) << "nothing read inserts it";

    // Written back as anonymous blocks (70 = 1), which a "*" name must be.
    const auto path = savedAs(in.doc, "hz_dxf_anonymous.dxf");
    int anonymous = 0;
    for (const auto& block : recordsIn(path, "BLOCK")) {
        const auto name =
            std::find(block.begin(), block.end(), std::make_pair(2, std::string("*U1")));
        if (name == block.end()) continue;
        for (const auto& [code, value] : block) {
            if (code == 70) anonymous = std::stoi(value) & 1;
        }
    }
    EXPECT_EQ(anonymous, 1);
    hz::doc::Document back;
    std::string error;
    ImportReport report;
    ASSERT_TRUE(hz::io::DxfFormat::load(path.string(), back, &error, &report)) << error;
    std::filesystem::remove(path);
    EXPECT_TRUE(report.skipped.empty());
    EXPECT_NE(back.draftDocument().blockTable().findBlock("*U1"), nullptr);
}

// From R2000 on, the ENTITIES section holds the active layout's paper space
// entities too, marked 67 = 1: a sheet's border, title block and viewports.
// They were read into the model, a title block drawn over it. They are left
// out now, and reported.
TEST(DxfFidelityTest, PaperSpaceEntitiesAreReportedNotDrawnOverTheModel) {
    Loaded in(
        dxf("0\nLINE\n8\n0\n10\n0\n20\n0\n11\n5\n21\n5\n"
            "0\nLINE\n8\n0\n67\n1\n10\n0\n20\n0\n11\n420\n21\n0\n"
            "0\nLWPOLYLINE\n8\n0\n67\n1\n90\n2\n70\n0\n10\n0\n20\n0\n10\n1\n20\n1\n"
            "0\nPOLYLINE\n8\n0\n67\n1\n66\n1\n70\n0\n"
            "0\nVERTEX\n8\n0\n67\n1\n10\n0\n20\n0\n"
            "0\nVERTEX\n8\n0\n67\n1\n10\n1\n20\n0\n"
            "0\nSEQEND\n8\n0\n67\n1\n"
            "0\nVIEWPORT\n8\n0\n67\n1\n10\n0\n20\n0\n"
            "0\nTEXT\n8\n0\n67\n0\n10\n0\n20\n0\n40\n2.5\n1\nmodel\n"));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_EQ(in.all<hz::draft::DraftLine>().size(), 1u);
    EXPECT_TRUE(in.all<hz::draft::DraftPolyline>().empty());
    EXPECT_EQ(in.all<hz::draft::DraftText>().size(), 1u) << "67 = 0 is the model";
    EXPECT_TRUE(contains(in.report.skipped, "4 paper space entities"));
    EXPECT_FALSE(contains(in.report.skipped, "VERTEX")) << "a polyline's vertices go with it";
}

// The drawing's spline is a uniform cubic B-spline: it starts at
// (P0 + 4 P1 + P2) / 6, not at P0. It was written with clamped knots, which
// another program draws through P0 and the last point: another curve. A
// closed one was written as an open one, and one of fewer than four points
// with more knots than it said. Each spline written is drawn here from its
// groups by de Boor's algorithm, as another program draws it, and must be the
// curve the drawing shows, at the same parameters.
TEST(DxfFidelityTest, ASavedSplineIsTheCurveTheDrawingShows) {
    auto splines = splinesOfEachKind();
    splines.push_back(std::make_shared<hz::draft::DraftSpline>(std::vector<Vec2>{{0, 0}, {3, 1}},
                                                               true));  // drawn as one segment
    hz::doc::Document doc;
    for (const auto& spline : splines) doc.draftDocument().addEntity(spline);
    const auto path = savedAs(doc, "hz_dxf_spline_curve.dxf");
    const std::vector<Groups> written = recordsIn(path, "SPLINE");
    std::filesystem::remove(path);
    ASSERT_EQ(written.size(), splines.size());
    for (size_t s = 0; s < splines.size(); ++s) {
        SCOPED_TRACE("spline " + std::to_string(s));
        const WrittenSpline w = readSpline(written[s]);
        const int m = static_cast<int>(w.points.size());
        ASSERT_GE(w.degree, 1);
        ASSERT_EQ(w.pointCount, m);
        ASSERT_EQ(w.knotCount, static_cast<int>(w.knots.size()));
        ASSERT_EQ(w.knotCount, m + w.degree + 1) << "a knot for each point, and the order";
        std::vector<double> weights = w.weights;
        if ((w.flags & 4) != 0) {
            ASSERT_EQ(static_cast<int>(weights.size()), m);
        } else {
            EXPECT_TRUE(weights.empty()) << "weights only on a rational spline";
            weights.assign(m, 1.0);
        }
        // The curve runs from knot U[p] to U[m], a span between each two;
        // the drawing samples each span evenly.
        const int perSpan = w.degree == 1 ? 1 : 8;
        std::vector<Vec2> other;
        for (int k = w.degree; k < m; ++k) {
            for (int j = 0; j < perSpan; ++j) {
                const double u = w.knots[k] + (w.knots[k + 1] - w.knots[k]) * j / perSpan;
                other.push_back(deBoor(w.degree, w.knots, w.points, weights, u));
            }
        }
        other.push_back(deBoor(w.degree, w.knots, w.points, weights, w.knots[m]));
        const std::vector<Vec2> drawn = splines[s]->evaluate(8);
        ASSERT_EQ(other.size(), drawn.size());
        for (size_t i = 0; i < drawn.size(); ++i) {
            EXPECT_TRUE(near(other[i], drawn[i], 1e-9))
                << i << ": (" << other[i].x << ", " << other[i].y << ") drawn at (" << drawn[i].x
                << ", " << drawn[i].y << ")";
        }
    }
}

// A spline saved reads back as it was: its points, whether closed, its
// weights. A closed one is written with its first points again at its end,
// which the reader takes off.
TEST(DxfFidelityTest, ASavedSplineReadsBackAsItWas) {
    const auto splines = splinesOfEachKind();
    hz::doc::Document doc;
    for (const auto& spline : splines) doc.draftDocument().addEntity(spline);
    const auto path = savedAs(doc, "hz_dxf_spline_back.dxf");
    Loaded in(dxf(""));
    std::string error;
    ASSERT_TRUE(hz::io::DxfFormat::load(path.string(), in.doc, &error, &in.report)) << error;
    std::filesystem::remove(path);
    EXPECT_TRUE(in.report.approximated.empty());
    const auto back = in.all<hz::draft::DraftSpline>();
    ASSERT_EQ(back.size(), splines.size());
    for (size_t s = 0; s < splines.size(); ++s) {
        SCOPED_TRACE("spline " + std::to_string(s));
        EXPECT_EQ(back[s]->closed(), splines[s]->closed());
        EXPECT_TRUE(samePoints(back[s]->controlPoints(), splines[s]->controlPoints()));
        EXPECT_EQ(back[s]->weights(), splines[s]->weights());
    }
}

// A SPLINE the drawing cannot hold as it is, clamped as most programs write
// one, comes in as a uniform cubic on its control points: another curve,
// and the report says so. One of degree 1 is the polyline through its points.
TEST(DxfFidelityTest, ASplineThatIsNotAUniformCubicIsReported) {
    Loaded in(dxf(
        "0\nSPLINE\n8\n0\n70\n8\n71\n3\n72\n8\n73\n4\n74\n0\n"
        "40\n0\n40\n0\n40\n0\n40\n0\n40\n1\n40\n1\n40\n1\n40\n1\n"
        "10\n0\n20\n0\n30\n0\n10\n1\n20\n2\n30\n0\n10\n3\n20\n2\n30\n0\n10\n4\n20\n0\n30\n0\n"
        "0\nSPLINE\n8\n0\n70\n8\n71\n1\n72\n6\n73\n4\n74\n0\n"
        "40\n0\n40\n0\n40\n1\n40\n2\n40\n3\n40\n3\n"
        "10\n0\n20\n0\n30\n0\n10\n1\n20\n2\n30\n0\n10\n3\n20\n2\n30\n0\n10\n4\n20\n0\n30\n0\n"));
    ASSERT_TRUE(in.ok) << in.error;
    EXPECT_EQ(in.all<hz::draft::DraftSpline>().size(), 1u);
    EXPECT_TRUE(contains(in.report.approximated, "1 SPLINE entity: not a uniform cubic"));
    const auto polys = in.all<hz::draft::DraftPolyline>();
    ASSERT_EQ(polys.size(), 1u) << "degree 1";
    EXPECT_TRUE(samePoints(polys[0]->points(), {{0, 0}, {1, 2}, {3, 2}, {4, 0}}));
}

// A bulge of 1e200, squared, is infinite: the arc it made had a centre of NaN,
// which nothing can draw or pick. Such a segment comes in straight.
TEST(DxfFidelityTest, ABulgeTooLargeForAnArcIsAStraightSegment) {
    Loaded in(
        dxf("0\nLWPOLYLINE\n8\n0\n90\n3\n70\n0\n"
            "10\n0\n20\n0\n42\n1e200\n10\n10\n20\n0\n42\n1\n10\n10\n20\n2\n"));
    ASSERT_TRUE(in.ok) << in.error;
    const auto lines = in.all<hz::draft::DraftLine>();
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_TRUE(near(lines[0]->start(), Vec2(0, 0)));
    EXPECT_TRUE(near(lines[0]->end(), Vec2(10, 0)));
    const auto arcs = in.all<hz::draft::DraftArc>();
    ASSERT_EQ(arcs.size(), 1u);
    EXPECT_TRUE(near(arcs[0]->center(), Vec2(10, 1)));
}
