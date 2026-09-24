// DXF geometry read as the file meant it (Phase 108a): polyline arcs,
// mirrored object coordinate systems, the old POLYLINE form, partial
// ellipses, inserts with unequal or mirrored scales, and blocks inside
// blocks. Fixtures are written to the DXF reference's group codes.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
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
