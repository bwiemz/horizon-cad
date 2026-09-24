#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "horizon/constraint/Constraint.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/Solid.h"

using namespace hz::doc;
using hz::io::NativeFormat;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

std::shared_ptr<Sketch> makeRectSketch(double w, double h) {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(w, 0)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(w, 0), Vec2(w, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(w, h), Vec2(0, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, h), Vec2(0, 0)));
    return sketch;
}

std::string tempPath(const std::string& name) {
    return (std::filesystem::temp_directory_path() / name).string();
}

// Build a part document: rectangle sketch + extrude along a NON-default
// direction so the round-trip test catches direction loss.
void buildBoxPart(Document& doc, const Vec3& direction, double distance) {
    doc.setType(DocumentType::Part);
    auto sketch = makeRectSketch(10.0, 5.0);
    sketch->setName("Profile");
    doc.addSketch(sketch);
    doc.featureTree().addFeature(std::make_unique<ExtrudeFeature>(sketch, direction, distance));
    doc.rebuildModel();
}

}  // namespace

// ---------------------------------------------------------------------------
// PartRoundTripPreservesTypeAndFeatureTree
// ---------------------------------------------------------------------------

TEST(PartFormatTest, PartRoundTripPreservesTypeAndFeatureTree) {
    Document original;
    buildBoxPart(original, Vec3(0, 0, -1), 7.5);
    ASSERT_NE(original.solid(), nullptr);

    std::string path = tempPath("hz_test_part_roundtrip.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));

    EXPECT_EQ(loaded.type(), DocumentType::Part);
    ASSERT_EQ(loaded.featureTree().featureCount(), 1u);

    const auto* ext = dynamic_cast<const ExtrudeFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(ext, nullptr);
    EXPECT_DOUBLE_EQ(ext->distance(), 7.5);
    EXPECT_DOUBLE_EQ(ext->direction().x, 0.0);
    EXPECT_DOUBLE_EQ(ext->direction().y, 0.0);
    EXPECT_DOUBLE_EQ(ext->direction().z, -1.0);

    // The feature references the real profile sketch, not an arbitrary index.
    ASSERT_NE(ext->sketch(), nullptr);
    EXPECT_EQ(ext->sketch()->name(), "Profile");
    EXPECT_EQ(ext->sketch()->entities().size(), 4u);

    // The loaded feature tree must replay successfully.
    EXPECT_TRUE(loaded.rebuildModel());
    EXPECT_NE(loaded.solid(), nullptr);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// RevolveRoundTripPreservesAxis
// ---------------------------------------------------------------------------

TEST(PartFormatTest, RevolveRoundTripPreservesAxis) {
    Document original;
    original.setType(DocumentType::Part);

    auto sketch = std::make_shared<Sketch>();
    sketch->setName("RevProfile");
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(5, 0), Vec2(10, 0)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(10, 0), Vec2(10, 5)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(10, 5), Vec2(5, 5)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(5, 5), Vec2(5, 0)));
    original.addSketch(sketch);

    original.featureTree().addFeature(
        std::make_unique<RevolveFeature>(sketch, Vec3(1, 2, 3), Vec3(0, 0, 1), 3.14159));

    std::string path = tempPath("hz_test_revolve_roundtrip.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.featureTree().featureCount(), 1u);

    const auto* rev = dynamic_cast<const RevolveFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(rev, nullptr);
    EXPECT_DOUBLE_EQ(rev->angle(), 3.14159);
    EXPECT_DOUBLE_EQ(rev->axisPoint().x, 1.0);
    EXPECT_DOUBLE_EQ(rev->axisPoint().y, 2.0);
    EXPECT_DOUBLE_EQ(rev->axisPoint().z, 3.0);
    EXPECT_DOUBLE_EQ(rev->axisDir().z, 1.0);
    ASSERT_NE(rev->sketch(), nullptr);
    EXPECT_EQ(rev->sketch()->name(), "RevProfile");

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// TessellationCacheIsWrittenForBuiltParts
// ---------------------------------------------------------------------------

TEST(PartFormatTest, TessellationCacheIsWrittenForBuiltParts) {
    Document doc;
    buildBoxPart(doc, Vec3(0, 0, 1), 3.0);
    ASSERT_NE(doc.solid(), nullptr);

    std::string path = tempPath("hz_test_part_cache.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, doc));

    auto mesh = NativeFormat::loadPartMesh(path);
    ASSERT_NE(mesh, nullptr);
    EXPECT_FALSE(mesh->positions.empty());
    EXPECT_FALSE(mesh->normals.empty());
    EXPECT_FALSE(mesh->indices.empty());
    EXPECT_EQ(mesh->positions.size() % 3, 0u);
    EXPECT_EQ(mesh->indices.size() % 3, 0u);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// LoadPartMeshReturnsNullWithoutCache
// ---------------------------------------------------------------------------

TEST(PartFormatTest, LoadPartMeshReturnsNullWithoutCache) {
    // A part that was never rebuilt has no solid, hence no cache.
    Document doc;
    doc.setType(DocumentType::Part);

    std::string path = tempPath("hz_test_part_nocache.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, doc));

    EXPECT_EQ(NativeFormat::loadPartMesh(path), nullptr);
    EXPECT_EQ(NativeFormat::loadPartMesh("/nonexistent/file.hzpart"), nullptr);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// DrawingsKeepHcadTypeTag
// ---------------------------------------------------------------------------

TEST(PartFormatTest, DrawingsKeepHcadTypeTag) {
    Document doc;  // default type: Drawing

    std::string path = tempPath("hz_test_drawing.hcad");
    ASSERT_TRUE(NativeFormat::save(path, doc));

    {
        std::ifstream in(path);
        std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        EXPECT_NE(content.find("\"hcad\""), std::string::npos);
    }

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    EXPECT_EQ(loaded.type(), DocumentType::Drawing);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// LegacyV15FileStillLoads
// ---------------------------------------------------------------------------

TEST(PartFormatTest, LegacyV15FileStillLoads) {
    // Minimal v15 file with the legacy sketchIndex feature reference.
    const char* v15 = R"({
        "version": 15,
        "type": "hcad",
        "entities": [],
        "sketches": [
            {
                "id": 1,
                "name": "Default Sketch",
                "plane": {"origin": [0,0,0], "normal": [0,0,1], "xAxis": [1,0,0]},
                "entities": [
                    {"type": "line", "layer": "0", "start": {"x":0,"y":0}, "end": {"x":4,"y":0}},
                    {"type": "line", "layer": "0", "start": {"x":4,"y":0}, "end": {"x":4,"y":4}},
                    {"type": "line", "layer": "0", "start": {"x":4,"y":4}, "end": {"x":0,"y":4}},
                    {"type": "line", "layer": "0", "start": {"x":0,"y":4}, "end": {"x":0,"y":0}}
                ]
            }
        ],
        "featureTree": [
            {"featureID": "extrude_1", "type": "extrude", "distance": 2.0, "sketchIndex": 0}
        ]
    })";

    std::string path = tempPath("hz_test_v15_compat.hcad");
    {
        std::ofstream out(path);
        out << v15;
    }

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    EXPECT_EQ(loaded.type(), DocumentType::Drawing);
    ASSERT_EQ(loaded.featureTree().featureCount(), 1u);

    const auto* ext = dynamic_cast<const ExtrudeFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(ext, nullptr);
    EXPECT_DOUBLE_EQ(ext->distance(), 2.0);
    // Legacy files carry no direction — the default must apply.
    EXPECT_DOUBLE_EQ(ext->direction().z, 1.0);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// MalformedTypeFieldDoesNotThrow
// ---------------------------------------------------------------------------

TEST(PartFormatTest, MalformedTypeFieldDoesNotThrow) {
    std::string path = tempPath("hz_test_bad_type.hcad");
    {
        std::ofstream out(path);
        out << R"({"version": 16, "type": null, "entities": []})";
    }

    Document loaded;
    // Must not throw; the malformed tag falls back to Drawing.
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    EXPECT_EQ(loaded.type(), DocumentType::Drawing);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// LoadPartMeshRejectsInvalidIndices
// ---------------------------------------------------------------------------

TEST(PartFormatTest, LoadPartMeshRejectsInvalidIndices) {
    // Out-of-range index (only 3 vertices, index 999) and negative index
    // must both be rejected instead of producing a mesh that crashes the
    // renderer.
    std::string outOfRange = tempPath("hz_test_bad_indices.hzpart");
    {
        std::ofstream out(outOfRange);
        out << R"({"version": 16, "type": "hzpart", "entities": [],
                   "tessellationCache": {
                       "positions": [0,0,0, 1,0,0, 0,1,0],
                       "normals": [0,0,1, 0,0,1, 0,0,1],
                       "indices": [0, 1, 999]}})";
    }
    EXPECT_EQ(NativeFormat::loadPartMesh(outOfRange), nullptr);

    std::string negative = tempPath("hz_test_neg_indices.hzpart");
    {
        std::ofstream out(negative);
        out << R"({"version": 16, "type": "hzpart", "entities": [],
                   "tessellationCache": {
                       "positions": [0,0,0, 1,0,0, 0,1,0],
                       "normals": [0,0,1, 0,0,1, 0,0,1],
                       "indices": [0, 1, -1]}})";
    }
    EXPECT_EQ(NativeFormat::loadPartMesh(negative), nullptr);

    // A well-formed cache still loads.
    std::string good = tempPath("hz_test_good_indices.hzpart");
    {
        std::ofstream out(good);
        out << R"({"version": 16, "type": "hzpart", "entities": [],
                   "tessellationCache": {
                       "positions": [0,0,0, 1,0,0, 0,1,0],
                       "normals": [0,0,1, 0,0,1, 0,0,1],
                       "indices": [0, 1, 2]}})";
    }
    auto mesh = NativeFormat::loadPartMesh(good);
    ASSERT_NE(mesh, nullptr);
    EXPECT_EQ(mesh->indices.size(), 3u);

    std::remove(outOfRange.c_str());
    std::remove(negative.c_str());
    std::remove(good.c_str());
}

// ---------------------------------------------------------------------------
// LoftFeatureRoundTrip
// ---------------------------------------------------------------------------

static std::shared_ptr<Sketch> squareOnPlane(double s, double z) {
    const double h = s * 0.5;
    auto sketch = std::make_shared<Sketch>(
        hz::draft::SketchPlane(Vec3(0, 0, z), Vec3(0, 0, 1), Vec3(1, 0, 0)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(-h, -h), Vec2(h, -h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(h, -h), Vec2(h, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(h, h), Vec2(-h, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(-h, h), Vec2(-h, -h)));
    return sketch;
}

TEST(PartFormatTest, LoftFeatureRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    auto s0 = squareOnPlane(6.0, 0.0);
    auto s1 = squareOnPlane(3.0, 10.0);
    original.addSketch(s0);
    original.addSketch(s1);
    original.featureTree().addFeature(
        std::make_unique<LoftFeature>(std::vector<std::shared_ptr<Sketch>>{s0, s1}));
    ASSERT_TRUE(original.rebuildModel());

    std::string path = tempPath("hz_test_loft_roundtrip.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.featureTree().featureCount(), 1u);

    const auto* loft = dynamic_cast<const LoftFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(loft, nullptr);
    EXPECT_EQ(loft->sections().size(), 2u);
    EXPECT_TRUE(loaded.rebuildModel());
    ASSERT_NE(loaded.solid(), nullptr);
    EXPECT_EQ(loaded.solid()->faceCount(), 6u);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// SweepFeatureRoundTrip
// ---------------------------------------------------------------------------

TEST(PartFormatTest, SweepFeatureRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    auto profile = squareOnPlane(4.0, 0.0);
    auto pathSketch = std::make_shared<Sketch>(
        hz::draft::SketchPlane(Vec3(0, 0, 0), Vec3(0, 1, 0), Vec3(1, 0, 0)));
    pathSketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(0, 10)));
    original.addSketch(profile);
    original.addSketch(pathSketch);
    original.featureTree().addFeature(std::make_unique<SweepFeature>(profile, pathSketch));
    ASSERT_TRUE(original.rebuildModel());

    std::string path = tempPath("hz_test_sweep_roundtrip.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.featureTree().featureCount(), 1u);

    const auto* sweep = dynamic_cast<const SweepFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(sweep, nullptr);
    ASSERT_NE(sweep->profile(), nullptr);
    ASSERT_NE(sweep->path(), nullptr);
    EXPECT_TRUE(loaded.rebuildModel());
    ASSERT_NE(loaded.solid(), nullptr);
    EXPECT_EQ(loaded.solid()->faceCount(), 6u);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// ShellAndDraftFeatureRoundTrip
// ---------------------------------------------------------------------------

TEST(PartFormatTest, ShellFeatureRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    auto sketch = makeRectSketch(10.0, 8.0);
    original.addSketch(sketch);
    original.featureTree().addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 5.0));
    std::string extId = original.featureTree().feature(0)->featureID();
    original.featureTree().addFeature(std::make_unique<ShellFeature>(
        1.0, std::vector<hz::topo::TopologyID>{hz::topo::TopologyID::make(extId, "cap_top")}));
    ASSERT_TRUE(original.rebuildModel());
    ASSERT_NE(original.solid(), nullptr);

    std::string path = tempPath("hz_test_shell_roundtrip.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.featureTree().featureCount(), 2u);

    const auto* shell = dynamic_cast<const ShellFeature*>(loaded.featureTree().feature(1));
    ASSERT_NE(shell, nullptr);
    EXPECT_DOUBLE_EQ(shell->thickness(), 1.0);
    ASSERT_EQ(shell->removedFaceIds().size(), 1u);
    EXPECT_EQ(shell->removedFaceIds()[0].tag(), extId + "/cap_top");

    EXPECT_TRUE(loaded.rebuildModel());
    ASSERT_NE(loaded.solid(), nullptr);
    EXPECT_EQ(loaded.solid()->faceCount(), 14u);

    std::remove(path.c_str());
}

TEST(PartFormatTest, DraftFeatureRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    auto sketch = makeRectSketch(10.0, 10.0);
    original.addSketch(sketch);
    original.featureTree().addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 5.0));
    original.featureTree().addFeature(
        std::make_unique<DraftFeature>(Vec3(0, 0, 1), Vec3(0, 0, 0), 0.15));
    ASSERT_TRUE(original.rebuildModel());

    std::string path = tempPath("hz_test_draft_roundtrip.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.featureTree().featureCount(), 2u);

    const auto* draft = dynamic_cast<const DraftFeature*>(loaded.featureTree().feature(1));
    ASSERT_NE(draft, nullptr);
    EXPECT_DOUBLE_EQ(draft->angle(), 0.15);
    EXPECT_DOUBLE_EQ(draft->pullDir().z, 1.0);
    EXPECT_TRUE(loaded.rebuildModel());
    ASSERT_NE(loaded.solid(), nullptr);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// PatternFeatureRoundTrip
// ---------------------------------------------------------------------------

TEST(PartFormatTest, LinearPatternRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    auto sketch = makeRectSketch(2.0, 2.0);
    original.addSketch(sketch);
    original.featureTree().addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 2.0));
    original.featureTree().addFeature(PatternFeature::makeLinear(Vec3(1, 0, 0), 5.0, 4, {2}));
    ASSERT_TRUE(original.rebuildModel());

    std::string path = tempPath("hz_test_pattern_roundtrip.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.featureTree().featureCount(), 2u);

    const auto* pat = dynamic_cast<const PatternFeature*>(loaded.featureTree().feature(1));
    ASSERT_NE(pat, nullptr);
    EXPECT_EQ(pat->kind(), PatternFeature::Kind::Linear);
    EXPECT_EQ(pat->count(), 4);
    EXPECT_DOUBLE_EQ(pat->scalar(), 5.0);
    ASSERT_EQ(pat->suppressedInstances().size(), 1u);
    EXPECT_EQ(pat->suppressedInstances()[0], 2);

    EXPECT_TRUE(loaded.rebuildModel());
    ASSERT_NE(loaded.solid(), nullptr);
    // 4 instances minus 1 suppressed = 3 bodies.
    EXPECT_EQ(loaded.solid()->shellCount(), 3u);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// DatumFeatureRoundTrip — reference geometry persists and stays transparent
// ---------------------------------------------------------------------------

TEST(PartFormatTest, DatumFeatureRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    auto sketch = makeRectSketch(3.0, 3.0);
    original.addSketch(sketch);
    // Datum plane, then a solid; the datum must survive save/load and not
    // disturb the rebuilt body.
    original.featureTree().addFeature(DatumFeature::makePlane(
        hz::model::DatumPlane{Vec3(0, 0, 7), Vec3(0, 0, 1), Vec3(1, 0, 0)}));
    original.featureTree().addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 2.0));
    ASSERT_TRUE(original.rebuildModel());

    std::string path = tempPath("hz_test_datum_roundtrip.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.featureTree().featureCount(), 2u);

    const auto* datum = dynamic_cast<const DatumFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(datum, nullptr);
    EXPECT_EQ(datum->datumKind(), DatumFeature::DatumKind::Plane);
    EXPECT_TRUE(datum->isConstruction());
    EXPECT_DOUBLE_EQ(datum->origin().z, 7.0);
    EXPECT_DOUBLE_EQ(datum->dirA().z, 1.0);
    EXPECT_DOUBLE_EQ(datum->dirB().x, 1.0);

    EXPECT_TRUE(loaded.rebuildModel());
    ASSERT_NE(loaded.solid(), nullptr);
    EXPECT_EQ(loaded.solid()->faceCount(), 6u);  // just the box

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// PrimitiveFeatureRoundTrip — parametric primitives persist and rebuild
// ---------------------------------------------------------------------------

TEST(PartFormatTest, PrimitiveFeatureRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    original.featureTree().addFeature(PrimitiveFeature::makeCone(4.0, 2.0, 6.0));
    ASSERT_TRUE(original.rebuildModel());

    std::string path = tempPath("hz_test_primitive_roundtrip.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.featureTree().featureCount(), 1u);

    const auto* prim = dynamic_cast<const PrimitiveFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(prim, nullptr);
    EXPECT_EQ(prim->kind(), PrimitiveFeature::Kind::Cone);
    EXPECT_DOUBLE_EQ(prim->p0(), 4.0);
    EXPECT_DOUBLE_EQ(prim->p1(), 2.0);
    EXPECT_DOUBLE_EQ(prim->p2(), 6.0);

    EXPECT_TRUE(loaded.rebuildModel());
    ASSERT_NE(loaded.solid(), nullptr);
    EXPECT_TRUE(loaded.solid()->isValid());

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// FilletFeatureRoundTrip — parametric fillet persists (edge tags + radius)
// ---------------------------------------------------------------------------

TEST(PartFormatTest, FilletFeatureRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    original.featureTree().addFeature(PrimitiveFeature::makeBox(10.0, 10.0, 10.0));
    ASSERT_TRUE(original.rebuildModel());
    ASSERT_NE(original.solid(), nullptr);
    ASSERT_FALSE(original.solid()->edges().empty());
    const auto edgeId = original.solid()->edges().front().topoId;
    original.featureTree().addFeature(
        std::make_unique<FilletFeature>(std::vector<hz::topo::TopologyID>{edgeId}, 1.5));
    ASSERT_TRUE(original.rebuildModel());

    std::string path = tempPath("hz_test_fillet_roundtrip.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.featureTree().featureCount(), 2u);

    const auto* fillet = dynamic_cast<const FilletFeature*>(loaded.featureTree().feature(1));
    ASSERT_NE(fillet, nullptr);
    EXPECT_DOUBLE_EQ(fillet->radius(), 1.5);
    ASSERT_EQ(fillet->edgeIds().size(), 1u);
    // The stored edge tag round-trips and still resolves against the rebuilt box.
    EXPECT_TRUE(loaded.rebuildModel());
    ASSERT_NE(loaded.solid(), nullptr);
    EXPECT_TRUE(loaded.solid()->isValid());

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// ChamferFeatureRoundTrip — parametric chamfer persists (edge tags + distance)
// ---------------------------------------------------------------------------

TEST(PartFormatTest, ChamferFeatureRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    original.featureTree().addFeature(PrimitiveFeature::makeBox(10.0, 10.0, 10.0));
    ASSERT_TRUE(original.rebuildModel());
    ASSERT_NE(original.solid(), nullptr);
    ASSERT_FALSE(original.solid()->edges().empty());
    const auto edgeId = original.solid()->edges().front().topoId;
    original.featureTree().addFeature(
        std::make_unique<ChamferFeature>(std::vector<hz::topo::TopologyID>{edgeId}, 0.8));
    ASSERT_TRUE(original.rebuildModel());

    std::string path = tempPath("hz_test_chamfer_roundtrip.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.featureTree().featureCount(), 2u);

    const auto* chamfer = dynamic_cast<const ChamferFeature*>(loaded.featureTree().feature(1));
    ASSERT_NE(chamfer, nullptr);
    EXPECT_DOUBLE_EQ(chamfer->distance(), 0.8);
    ASSERT_EQ(chamfer->edgeIds().size(), 1u);
    EXPECT_TRUE(loaded.rebuildModel());
    ASSERT_NE(loaded.solid(), nullptr);
    EXPECT_TRUE(loaded.solid()->isValid());

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// BooleanFeatureRoundTrip — Boolean operation type persists across save/load
// ---------------------------------------------------------------------------

TEST(PartFormatTest, BooleanFeatureRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    original.featureTree().addFeature(PrimitiveFeature::makeBox(10.0, 10.0, 10.0));
    original.featureTree().addFeature(PrimitiveFeature::makeBox(6.0, 6.0, 6.0));
    original.featureTree().addFeature(
        std::make_unique<BooleanFeature>(hz::model::BooleanType::Subtract));

    std::string path = tempPath("hz_test_boolean_roundtrip.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.featureTree().featureCount(), 3u);

    const auto* boolean = dynamic_cast<const BooleanFeature*>(loaded.featureTree().feature(2));
    ASSERT_NE(boolean, nullptr);
    EXPECT_EQ(boolean->booleanType(), hz::model::BooleanType::Subtract);
    EXPECT_EQ(boolean->name(), "Boolean Subtract");

    // The multi-body build folds the two boxes through the persisted Boolean.
    EXPECT_EQ(loaded.featureTree().buildBodies().size(), 1u);

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// Faceting resolution survives a round trip (Phase 88)
//
// The resolution decides how close a faceted solid's volume is to the exact
// one, so losing it on reload silently changes the part. Files written before
// it was a feature property carry no such field and must still load.
// ---------------------------------------------------------------------------

TEST(PartFormatTest, PrimitiveSegmentsRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    auto cylinder = PrimitiveFeature::makeCylinder(5.0, 10.0);
    ASSERT_TRUE(cylinder->setParameter("segments", 96.0));
    original.featureTree().addFeature(std::move(cylinder));
    ASSERT_TRUE(original.rebuildModel());
    const size_t originalFaces = original.solid()->faceCount();

    std::string path = tempPath("hz_test_primitive_segments.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    const auto* prim = dynamic_cast<const PrimitiveFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(prim, nullptr);
    EXPECT_EQ(prim->segments(), 96);

    // The reloaded feature must rebuild the same solid, not the default one.
    ASSERT_TRUE(loaded.rebuildModel());
    EXPECT_EQ(loaded.solid()->faceCount(), originalFaces);

    std::remove(path.c_str());
}

TEST(PartFormatTest, RevolveSegmentsRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(5, 0), Vec2(10, 0)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(10, 0), Vec2(10, 5)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(10, 5), Vec2(5, 5)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(5, 5), Vec2(5, 0)));
    original.addSketch(sketch);
    auto revolve = std::make_unique<RevolveFeature>(sketch, Vec3::Zero, Vec3::UnitY, 3.14159265);
    ASSERT_TRUE(revolve->setParameter("segments", 48.0));
    original.featureTree().addFeature(std::move(revolve));
    ASSERT_TRUE(original.rebuildModel());
    const size_t originalFaces = original.solid()->faceCount();

    std::string path = tempPath("hz_test_revolve_segments.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    const auto* rev = dynamic_cast<const RevolveFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(rev, nullptr);
    EXPECT_EQ(rev->segments(), 48);

    ASSERT_TRUE(loaded.rebuildModel());
    EXPECT_EQ(loaded.solid()->faceCount(), originalFaces);

    std::remove(path.c_str());
}

TEST(PartFormatTest, SweepSegmentsRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    auto profile = squareOnPlane(2.0, 0.0);
    auto pathSketch = std::make_shared<Sketch>(
        hz::draft::SketchPlane(Vec3(0, 0, 0), Vec3(0, -1, 0), Vec3(1, 0, 0)));
    pathSketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(0, 10)));
    pathSketch->addEntity(
        std::make_shared<hz::draft::DraftArc>(Vec2(10, 10), 10.0, 3.14159265 * 0.5, 3.14159265));
    original.addSketch(profile);
    original.addSketch(pathSketch);
    auto sweep = std::make_unique<SweepFeature>(profile, pathSketch);
    ASSERT_TRUE(sweep->setParameter("segments", 64.0));
    original.featureTree().addFeature(std::move(sweep));
    ASSERT_TRUE(original.rebuildModel());
    const size_t originalFaces = original.solid()->faceCount();

    std::string path = tempPath("hz_test_sweep_segments.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    const auto* reloaded = dynamic_cast<const SweepFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(reloaded, nullptr);
    EXPECT_EQ(reloaded->segments(), 64);

    ASSERT_TRUE(loaded.rebuildModel());
    EXPECT_EQ(loaded.solid()->faceCount(), originalFaces);

    std::remove(path.c_str());
}

TEST(PartFormatTest, ExtrudeResolutionRoundTrips) {
    Document original;
    original.setType(DocumentType::Part);
    auto disc = std::make_shared<Sketch>();
    disc->addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(0, 0), 5.0));
    original.addSketch(disc);
    auto extrude = std::make_unique<ExtrudeFeature>(disc, Vec3(0, 0, 1), 10.0);
    ASSERT_TRUE(extrude->setParameter("segments", 72.0));
    original.featureTree().addFeature(std::move(extrude));
    ASSERT_TRUE(original.rebuildModel());
    EXPECT_EQ(original.solid()->faceCount(), 74u);

    std::string path = tempPath("hz_test_extrude_segments.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    const auto* ext = dynamic_cast<const ExtrudeFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(ext, nullptr);
    EXPECT_EQ(ext->segments(), 72);
    ASSERT_TRUE(loaded.rebuildModel());
    EXPECT_EQ(loaded.solid()->faceCount(), 74u);

    std::remove(path.c_str());
}

TEST(PartFormatTest, FilletArcSegmentsRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    original.featureTree().addFeature(PrimitiveFeature::makeBox(10.0, 10.0, 10.0));
    ASSERT_TRUE(original.rebuildModel());
    const auto edgeId = original.solid()->edges().front().topoId;

    auto fillet = std::make_unique<FilletFeature>(std::vector<hz::topo::TopologyID>{edgeId}, 1.0);
    ASSERT_TRUE(fillet->setParameter("arcSegments", 24.0));
    original.featureTree().addFeature(std::move(fillet));
    ASSERT_TRUE(original.rebuildModel());
    const size_t originalFaces = original.solid()->faceCount();

    std::string path = tempPath("hz_test_fillet_arcsegments.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    const auto* reloaded = dynamic_cast<const FilletFeature*>(loaded.featureTree().feature(1));
    ASSERT_NE(reloaded, nullptr);
    EXPECT_EQ(reloaded->arcSegments(), 24);

    ASSERT_TRUE(loaded.rebuildModel());
    EXPECT_EQ(loaded.solid()->faceCount(), originalFaces);

    std::remove(path.c_str());
}

TEST(PartFormatTest, ChordToleranceRoundTrips) {
    Document original;
    original.setType(DocumentType::Part);
    auto cylinder = PrimitiveFeature::makeCylinder(20.0, 5.0);
    ASSERT_TRUE(cylinder->setParameter("chordTolerance", 0.002));
    original.featureTree().addFeature(std::move(cylinder));
    ASSERT_TRUE(original.rebuildModel());
    const size_t originalFaces = original.solid()->faceCount();

    std::string path = tempPath("hz_test_chord_tolerance.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    auto* prim = dynamic_cast<PrimitiveFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(prim, nullptr);
    EXPECT_DOUBLE_EQ(prim->chordTolerance(), 0.002);
    ASSERT_TRUE(loaded.rebuildModel());
    EXPECT_EQ(loaded.solid()->faceCount(), originalFaces);

    // It reloads as a tolerance, not as the count it happened to produce: a
    // radius edit after reload still re-derives the count.
    const int before = prim->segments();
    ASSERT_TRUE(prim->setParameter("radius", 40.0));
    EXPECT_GT(prim->segments(), before);

    std::remove(path.c_str());
}

TEST(PartFormatTest, FilesWithoutResolutionFieldsLoadAtTheFeatureDefaults) {
    // A part envelope as written before the resolution became a property: the
    // primitive and fillet entries carry no "segments"/"arcSegments" key.
    const std::string legacy = R"({
      "version": 16,
      "type": "hzpart",
      "entities": [],
      "sketches": [],
      "featureTree": [
        {"type": "primitive", "primitiveKind": "cylinder", "p0": 5.0, "p1": 10.0, "p2": 1.0,
         "featureId": "primitive_1"}
      ]
    })";

    std::string path = tempPath("hz_test_legacy_resolution.hzpart");
    {
        std::ofstream out(path);
        out << legacy;
    }

    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    ASSERT_EQ(loaded.featureTree().featureCount(), 1u);
    const auto* prim = dynamic_cast<const PrimitiveFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(prim, nullptr);
    EXPECT_EQ(prim->segments(), hz::model::PrimitiveFactory::kDefaultSegments);
    EXPECT_TRUE(loaded.rebuildModel());

    std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// Body operations (Phase 102)
// ---------------------------------------------------------------------------

TEST(PartFormatTest, BodyOperationsRoundTrip) {
    Document original;
    original.setType(DocumentType::Part);
    auto plate = makeRectSketch(10.0, 10.0);
    auto hole = std::make_shared<Sketch>();
    hole->addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(5, 5), 2.0));
    original.addSketch(plate);
    original.addSketch(hole);
    original.featureTree().addFeature(std::make_unique<ExtrudeFeature>(plate, Vec3(0, 0, 1), 2.0));
    auto cut = std::make_unique<ExtrudeFeature>(hole, Vec3(0, 0, 1), 2.0);
    cut->setOperation(BodyOperation::Cut);
    original.featureTree().addFeature(std::move(cut));
    ASSERT_TRUE(original.rebuildModel()) << original.lastBuildMessage();

    std::string path = tempPath("hz_test_body_operations.hzpart");
    ASSERT_TRUE(NativeFormat::save(path, original));
    Document loaded;
    ASSERT_TRUE(NativeFormat::load(path, loaded));
    std::remove(path.c_str());

    ASSERT_EQ(loaded.featureTree().featureCount(), 2u);
    EXPECT_EQ(loaded.featureTree().feature(0)->operation(), BodyOperation::NewBody);
    EXPECT_EQ(loaded.featureTree().feature(1)->operation(), BodyOperation::Cut);
    ASSERT_TRUE(loaded.rebuildModel()) << loaded.lastBuildMessage();
    EXPECT_NEAR(hz::model::MassPropertiesCalculator::compute(*loaded.solid()).volume,
                hz::model::MassPropertiesCalculator::compute(*original.solid()).volume, 1e-9)
        << "the reloaded part still has its hole";
}

TEST(PartFormatTest, FilesWrittenBeforeBodyOperationsLoadAsSeparateBodies) {
    // Before Phase 102 the rebuild kept only the last created body; loading
    // such a file as New body shows every body it describes.
    Document original;
    original.setType(DocumentType::Part);
    auto a = makeRectSketch(10.0, 10.0);
    original.addSketch(a);
    original.featureTree().addFeature(std::make_unique<ExtrudeFeature>(a, Vec3(0, 0, 1), 1.0));
    original.featureTree().addFeature(std::make_unique<ExtrudeFeature>(a, Vec3(0, 0, -1), 1.0));
    for (size_t i = 0; i < 2; ++i) {
        original.featureTree().feature(i)->setOperation(BodyOperation::Join);
    }

    nlohmann::json root = nlohmann::json::parse(NativeFormat::documentToJson(original, false));
    for (auto& feature : root.at("featureTree")) feature.erase("bodyOperation");

    Document loaded;
    std::string error;
    ASSERT_TRUE(NativeFormat::documentFromJson(root.dump(), loaded, &error)) << error;
    ASSERT_EQ(loaded.featureTree().featureCount(), 2u);
    EXPECT_EQ(loaded.featureTree().feature(0)->operation(), BodyOperation::NewBody);
    EXPECT_EQ(loaded.featureTree().feature(1)->operation(), BodyOperation::NewBody);
}

// ---------------------------------------------------------------------------
// Feature suppression (Phase 104)
// ---------------------------------------------------------------------------

TEST(PartFormatTest, ASuppressedFeatureStaysSuppressed) {
    Document original;
    original.setType(DocumentType::Part);
    auto plate = makeRectSketch(10.0, 10.0);
    original.addSketch(plate);
    original.featureTree().addFeature(std::make_unique<ExtrudeFeature>(plate, Vec3(0, 0, 1), 2.0));
    original.featureTree().addFeature(PatternFeature::makeLinear(Vec3(1, 0, 0), 20.0, 3, {1}));
    original.featureTree().feature(1)->setSuppressed(true);

    nlohmann::json root = nlohmann::json::parse(NativeFormat::documentToJson(original, false));
    EXPECT_GE(root.at("version").get<int>(), 17)
        << "an older build would build the suppressed feature: it must refuse the file";
    const auto& pattern = root.at("featureTree").at(1);
    EXPECT_TRUE(pattern.at("featureSuppressed").get<bool>());
    EXPECT_EQ(pattern.at("suppressed"), nlohmann::json::array({1}))
        << "the pattern's own instance list keeps its key";
    EXPECT_FALSE(root.at("featureTree").at(0).contains("featureSuppressed"))
        << "only written when set";

    Document loaded;
    std::string error;
    ASSERT_TRUE(NativeFormat::documentFromJson(root.dump(), loaded, &error)) << error;
    ASSERT_EQ(loaded.featureTree().featureCount(), 2u);
    EXPECT_FALSE(loaded.featureTree().feature(0)->isSuppressed());
    EXPECT_TRUE(loaded.featureTree().feature(1)->isSuppressed());
    ASSERT_TRUE(loaded.rebuildModel()) << loaded.lastBuildMessage();
    EXPECT_EQ(loaded.solid()->shellCount(), 1u) << "the suppressed pattern adds nothing";
}

TEST(PartFormatTest, AFeatureSuppressionOfTheWrongTypeSkipsOnlyThatFeature) {
    Document original;
    original.setType(DocumentType::Part);
    auto plate = makeRectSketch(10.0, 10.0);
    original.addSketch(plate);
    original.featureTree().addFeature(std::make_unique<ExtrudeFeature>(plate, Vec3(0, 0, 1), 2.0));
    original.featureTree().addFeature(std::make_unique<ExtrudeFeature>(plate, Vec3(0, 0, 1), 3.0));
    nlohmann::json root = nlohmann::json::parse(NativeFormat::documentToJson(original, false));
    root.at("featureTree").at(1)["featureSuppressed"] = "yes";

    Document loaded;
    std::string error;
    ASSERT_TRUE(NativeFormat::documentFromJson(root.dump(), loaded, &error)) << error;
    EXPECT_EQ(loaded.featureTree().featureCount(), 1u);
}

// ---------------------------------------------------------------------------
// Sketches keep their constraints; the old empty default sketch goes
// ---------------------------------------------------------------------------

// A sketch's constraints are saved with it. They were lost: only the
// top-level drawing's were written.
TEST(PartFormatTest, ASketchKeepsItsConstraints) {
    Document doc;
    auto sketch = std::make_shared<Sketch>();
    sketch->setName("Bracket");
    auto a = std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(5, 0.1));
    sketch->addEntity(a);
    hz::cstr::GeometryRef ref{a->id(), hz::cstr::FeatureType::Line, 0};
    sketch->constraintSystem().addConstraint(
        std::make_shared<hz::cstr::HorizontalConstraint>(ref, ref));
    doc.addSketch(sketch);

    Document back;
    std::string error;
    ASSERT_TRUE(
        NativeFormat::documentFromJson(NativeFormat::documentToJson(doc, false), back, &error))
        << error;
    ASSERT_EQ(back.sketches().size(), 1u);
    EXPECT_EQ(back.sketches()[0]->name(), "Bracket");
    EXPECT_EQ(back.sketches()[0]->constraintSystem().constraints().size(), 1u);
    EXPECT_TRUE(back.constraintSystem().empty()) << "not moved to the top level";
}

// Every document used to have an empty "Default Sketch", saved in every file.
// One nothing uses is dropped on reading; one a feature uses, or with
// entities, stays.
TEST(PartFormatTest, AnOldEmptyDefaultSketchIsDropped) {
    const auto load = [](const std::string& sketchEntities, const std::string& features) {
        auto doc = std::make_unique<Document>();
        std::string error;
        EXPECT_TRUE(NativeFormat::documentFromJson(
            R"({"version": 18, "type": "hcad", "entities": [], "sketches": [{"id": 900,)"
            R"( "name": "Default Sketch", "plane": {"origin": [0,0,0], "normal": [0,0,1],)"
            R"( "xAxis": [1,0,0]}, "entities": [)" +
                sketchEntities + R"(]}], "featureTree": [)" + features + "]}",
            *doc, &error))
            << error;
        return doc;
    };
    EXPECT_TRUE(load("", "")->sketches().empty());
    EXPECT_EQ(load(R"({"type": "line", "layer": "0", "start": {"x":0,"y":0},)"
                   R"( "end": {"x":4,"y":0}})",
                   "")
                  ->sketches()
                  .size(),
              1u)
        << "a default sketch with something in it is kept";
    const auto used = load("", R"({"featureID": "extrude_1", "type": "extrude", "distance": 2.0,)"
                               R"( "sketchId": 900})");
    EXPECT_EQ(used->sketches().size(), 1u) << "one a feature uses is kept";
}
