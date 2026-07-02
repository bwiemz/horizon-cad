#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/NativeFormat.h"

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

    std::ifstream in(path);
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("\"hcad\""), std::string::npos);

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
