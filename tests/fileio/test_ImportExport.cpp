// Getting work in and out (Phase 107): what a load leaves out is reported,
// imported bodies live in the part, and a part exports as binary STL.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StlExport.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::io::ImportReport;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

double volumeOf(const hz::topo::Solid& solid) {
    return hz::model::MassPropertiesCalculator::compute(solid).volume;
}

}  // namespace

TEST(ImportReportTest, ANativeLoadSaysWhatItLeftOut) {
    // A file with one good line and four items this build cannot use.
    const std::string text = R"({"version": 16, "type": "hzpart",
        "entities": [
            {"type": "line", "id": 1, "start": {"x": 0, "y": 0}, "end": {"x": 1, "y": 0}},
            {"type": "line", "id": 2, "start": {"x": 0, "y": 0}},
            {"type": "hologram", "id": 3}
        ],
        "sketches": [],
        "featureTree": [
            {"type": "teleport", "featureID": "teleport_1"},
            {"type": "extrude", "featureID": "extrude_1", "sketchId": 999, "distance": 1}
        ]})";
    hz::doc::Document doc;
    std::string error;
    ImportReport report;
    ASSERT_TRUE(hz::io::NativeFormat::documentFromJson(text, doc, &error, &report)) << error;
    EXPECT_EQ(doc.draftDocument().entities().size(), 1u) << "the good line is kept";

    ASSERT_EQ(report.skipped.size(), 4u);
    EXPECT_TRUE(contains(report.skipped[0], "entity 2 (line)")) << report.skipped[0];
    EXPECT_TRUE(contains(report.skipped[1], "entity 3 (hologram)")) << report.skipped[1];
    EXPECT_TRUE(contains(report.skipped[1], "not a kind of entity")) << report.skipped[1];
    EXPECT_TRUE(contains(report.skipped[2], "feature 1 (teleport)")) << report.skipped[2];
    EXPECT_TRUE(contains(report.skipped[2], "not a kind of feature")) << report.skipped[2];
    EXPECT_TRUE(contains(report.skipped[3], "feature 2 (extrude): its sketch is missing"))
        << report.skipped[3];
    EXPECT_EQ(report.summary(), "4 items were left out.");
}

TEST(ImportReportTest, AFileReadWholeReportsNothing) {
    hz::doc::Document original;
    original.draftDocument().addEntity(
        std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 1)));
    hz::doc::Document loaded;
    ImportReport report;
    ASSERT_TRUE(hz::io::NativeFormat::documentFromJson(
        hz::io::NativeFormat::documentToJson(original, false), loaded, nullptr, &report));
    EXPECT_TRUE(report.empty());
    EXPECT_EQ(report.summary(), "");
}

TEST(ImportReportTest, ADxfLoadCountsWhatItDidNotRead) {
    const std::string dxf =
        "0\nSECTION\n2\nENTITIES\n"
        "0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1\n21\n1\n"
        "0\n3DFACE\n8\n0\n"
        "0\n3DFACE\n8\n0\n"
        "0\nSOLID\n8\n0\n"
        "0\nENDSEC\n0\nEOF\n";
    hz::doc::Document doc;
    std::string error;
    ImportReport report;
    ASSERT_TRUE(hz::io::DxfFormat::loadFromString(dxf, doc, &error, &report)) << error;
    EXPECT_EQ(doc.draftDocument().entities().size(), 1u);
    ASSERT_EQ(report.skipped.size(), 2u);
    EXPECT_EQ(report.skipped[0], "2 3DFACE entities not read");
    EXPECT_EQ(report.skipped[1], "1 SOLID entity not read");
}

TEST(StlExportTest, ABoxIsTwelveTrianglesWithUnitNormals) {
    auto box = hz::model::PrimitiveFactory::makeBox(1, 2, 3);
    const std::string stl = hz::io::StlExport::toBinary(*box);
    ASSERT_GE(stl.size(), 84u);
    EXPECT_NE(stl.compare(0, 5, "solid"), 0) << "an ASCII STL's first word";

    uint32_t count = 0;
    std::memcpy(&count, stl.data() + 80, sizeof count);
    EXPECT_EQ(count, 12u);
    ASSERT_EQ(stl.size(), 84u + 50u * count);

    // Each facet: a unit normal pointing out, and the three corners; together
    // they enclose the box's volume.
    double volume = 0.0;
    for (uint32_t t = 0; t < count; ++t) {
        float f[12];
        std::memcpy(f, stl.data() + 84 + 50 * t, sizeof f);
        const Vec3 n(f[0], f[1], f[2]);
        const Vec3 a(f[3], f[4], f[5]), b(f[6], f[7], f[8]), c(f[9], f[10], f[11]);
        EXPECT_NEAR(n.length(), 1.0, 1e-6);
        EXPECT_GT(n.dot((b - a).cross(c - a)), 0.0) << "the normal agrees with the winding";
        volume += a.dot(b.cross(c)) / 6.0;
    }
    EXPECT_NEAR(volume, 6.0, 1e-5);
}

TEST(ImportedBodyTest, AnImportedBodyTravelsWithThePartAndTakesACut) {
    hz::doc::Document original;
    original.setType(hz::doc::DocumentType::Part);
    std::shared_ptr<const hz::topo::Solid> box = hz::model::PrimitiveFactory::makeBox(10, 10, 10);
    original.featureTree().addFeature(
        std::make_unique<hz::doc::ImportedBodyFeature>(box, "block.step"));

    // A 2 x 2 cut through it.
    auto hole = std::make_shared<hz::doc::Sketch>();
    for (auto [a, b] : {std::pair{Vec2(4, 4), Vec2(6, 4)}, std::pair{Vec2(6, 4), Vec2(6, 6)},
                        std::pair{Vec2(6, 6), Vec2(4, 6)}, std::pair{Vec2(4, 6), Vec2(4, 4)}}) {
        hole->addEntity(std::make_shared<hz::draft::DraftLine>(a, b));
    }
    original.addSketch(hole);
    auto cut = std::make_unique<hz::doc::ExtrudeFeature>(hole, Vec3(0, 0, 1), 10.0);
    cut->setOperation(hz::doc::BodyOperation::Cut);
    original.featureTree().addFeature(std::move(cut));
    ASSERT_TRUE(original.rebuildModel()) << original.lastBuildMessage();
    EXPECT_NEAR(volumeOf(*original.solid()), 1000.0 - 40.0, 1e-6);

    hz::doc::Document loaded;
    std::string error;
    ASSERT_TRUE(hz::io::NativeFormat::documentFromJson(
        hz::io::NativeFormat::documentToJson(original, false), loaded, &error))
        << error;
    const auto* imported =
        dynamic_cast<const hz::doc::ImportedBodyFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(imported, nullptr);
    EXPECT_EQ(imported->source(), "block.step");
    EXPECT_EQ(imported->featureID(), original.featureTree().feature(0)->featureID());
    ASSERT_TRUE(loaded.rebuildModel()) << loaded.lastBuildMessage();
    EXPECT_NEAR(volumeOf(*loaded.solid()), 1000.0 - 40.0, 1e-6) << "without the STEP file";
}

TEST(ImportReportTest, ALoftOrSweepThatCannotFindItsSketchesIsReported) {
    const std::string text = R"({"version": 16, "type": "hzpart", "entities": [],
        "sketches": [],
        "featureTree": [
            {"type": "loft", "featureID": "loft_1", "sketchIds": [998, 999]},
            {"type": "sweep", "featureID": "sweep_1", "sketchId": 998, "pathSketchId": 999}
        ]})";
    hz::doc::Document doc;
    ImportReport report;
    ASSERT_TRUE(hz::io::NativeFormat::documentFromJson(text, doc, nullptr, &report));
    ASSERT_EQ(report.skipped.size(), 2u) << "they used to vanish without a word";
    EXPECT_TRUE(contains(report.skipped[0], "feature 1 (loft): it needs two or more section"))
        << report.skipped[0];
    EXPECT_TRUE(contains(report.skipped[1], "feature 2 (sweep): its profile sketch is missing"))
        << report.skipped[1];
}
