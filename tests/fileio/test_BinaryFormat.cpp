#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/Sketch.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/fileio/BinaryFormat.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/SolidTessellator.h"

using namespace hz::doc;
using hz::io::BinaryFormat;
using hz::io::NativeFormat;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

std::string tempPath(const std::string& name) {
    return (std::filesystem::temp_directory_path() / name).string();
}

std::shared_ptr<Sketch> makeRectSketch(double w, double h) {
    auto sketch = std::make_shared<Sketch>();
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(w, 0)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(w, 0), Vec2(w, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(w, h), Vec2(0, h)));
    sketch->addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, h), Vec2(0, 0)));
    return sketch;
}

void buildBoxPart(Document& doc) {
    doc.setType(DocumentType::Part);
    auto sketch = makeRectSketch(10.0, 5.0);
    sketch->setName("Profile");
    doc.addSketch(sketch);
    doc.featureTree().addFeature(std::make_unique<ExtrudeFeature>(sketch, Vec3(0, 0, 1), 4.0));
    doc.rebuildModel();
}

struct TempFile {
    explicit TempFile(std::string p) : path(std::move(p)) {}
    ~TempFile() { std::filesystem::remove(path); }
    std::string path;
};

}  // namespace

TEST(BinaryFormatTest, PartRoundTripPreservesFeatureTree) {
    Document original;
    buildBoxPart(original);
    ASSERT_NE(original.solid(), nullptr);

    TempFile file(tempPath("hz_test_binary_part.hzpart"));
    ASSERT_TRUE(BinaryFormat::save(file.path, original));

    Document loaded;
    ASSERT_TRUE(BinaryFormat::load(file.path, loaded));

    EXPECT_EQ(loaded.type(), DocumentType::Part);
    ASSERT_EQ(loaded.featureTree().featureCount(), 1u);
    const auto* ext = dynamic_cast<const ExtrudeFeature*>(loaded.featureTree().feature(0));
    ASSERT_NE(ext, nullptr);
    EXPECT_DOUBLE_EQ(ext->distance(), 4.0);
    ASSERT_NE(ext->sketch(), nullptr);
    EXPECT_EQ(ext->sketch()->name(), "Profile");

    loaded.rebuildModel();
    ASSERT_NE(loaded.solid(), nullptr);
    EXPECT_EQ(loaded.solid()->faceCount(), original.solid()->faceCount());
}

TEST(BinaryFormatTest, LightweightMeshLoadMatchesTessellation) {
    Document doc;
    buildBoxPart(doc);
    ASSERT_NE(doc.solid(), nullptr);
    const auto direct = hz::model::SolidTessellator::tessellate(*doc.solid());

    TempFile file(tempPath("hz_test_binary_mesh.hzpart"));
    ASSERT_TRUE(BinaryFormat::save(file.path, doc));

    auto mesh = BinaryFormat::loadPartMesh(file.path);
    ASSERT_NE(mesh, nullptr);
    EXPECT_EQ(mesh->positions.size(), direct.positions.size());
    EXPECT_EQ(mesh->normals.size(), direct.normals.size());
    EXPECT_EQ(mesh->indices.size(), direct.indices.size());
}

TEST(BinaryFormatTest, AssemblyRoundTripPreservesComponentsAndMates) {
    AssemblyDocument original;
    ComponentInstance comp;
    comp.id = 7;
    comp.name = "bracket";
    comp.partPath = tempPath("bracket.hzpart");
    original.addComponent(std::move(comp));

    Mate mate;
    mate.id = 3;
    mate.type = MateType::Distance;
    mate.value = 12.5;
    mate.a.componentId = 7;
    original.addMate(std::move(mate));

    TempFile file(tempPath("hz_test_binary_asm.hzasm"));
    ASSERT_TRUE(BinaryFormat::saveAssembly(file.path, original));

    AssemblyDocument loaded;
    ASSERT_TRUE(BinaryFormat::loadAssembly(file.path, loaded));
    ASSERT_EQ(loaded.components().size(), 1u);
    EXPECT_EQ(loaded.components()[0].id, 7u);
    EXPECT_EQ(loaded.components()[0].name, "bracket");
    ASSERT_EQ(loaded.mates().size(), 1u);
    EXPECT_EQ(loaded.mates()[0].type, MateType::Distance);
    EXPECT_DOUBLE_EQ(loaded.mates()[0].value, 12.5);
}

TEST(BinaryFormatTest, IsBinaryFileDistinguishesJsonAndBinary) {
    Document doc;
    buildBoxPart(doc);

    TempFile binFile(tempPath("hz_test_binary_sniff.hzpart"));
    TempFile jsonFile(tempPath("hz_test_json_sniff.hzpart"));
    ASSERT_TRUE(BinaryFormat::save(binFile.path, doc));
    ASSERT_TRUE(NativeFormat::save(jsonFile.path, doc));

    EXPECT_TRUE(BinaryFormat::isBinaryFile(binFile.path));
    EXPECT_FALSE(BinaryFormat::isBinaryFile(jsonFile.path));
    EXPECT_FALSE(BinaryFormat::isBinaryFile(tempPath("hz_missing_file.hzpart")));
}

/// Files shorter than the FlatBuffers minimum (8 bytes) must be rejected
/// without reading past the buffer — regression test for a heap over-read
/// when BufferHasIdentifier was pre-checked on tiny buffers.
TEST(BinaryFormatTest, RejectsTinyFilesSafely) {
    for (size_t size = 1; size <= 7; ++size) {
        TempFile tiny(tempPath("hz_test_binary_tiny_" + std::to_string(size) + ".hzpart"));
        {
            std::ofstream out(tiny.path, std::ios::binary);
            out << std::string(size, 'H');
        }
        Document doc;
        EXPECT_FALSE(BinaryFormat::load(tiny.path, doc)) << "size " << size;
        EXPECT_EQ(BinaryFormat::loadPartMesh(tiny.path), nullptr) << "size " << size;
        AssemblyDocument asmDoc;
        EXPECT_FALSE(BinaryFormat::loadAssembly(tiny.path, asmDoc)) << "size " << size;
        EXPECT_FALSE(BinaryFormat::isBinaryFile(tiny.path)) << "size " << size;
    }
}

TEST(BinaryFormatTest, RejectsCorruptAndTruncatedFiles) {
    TempFile garbage(tempPath("hz_test_binary_garbage.hzpart"));
    {
        std::ofstream out(garbage.path, std::ios::binary);
        out << "this is definitely not a flatbuffer";
    }
    Document doc;
    EXPECT_FALSE(BinaryFormat::load(garbage.path, doc));
    EXPECT_EQ(BinaryFormat::loadPartMesh(garbage.path), nullptr);

    // Truncate a valid file mid-buffer: the verifier must reject it.
    Document source;
    buildBoxPart(source);
    TempFile truncated(tempPath("hz_test_binary_truncated.hzpart"));
    ASSERT_TRUE(BinaryFormat::save(truncated.path, source));
    const auto fullSize = std::filesystem::file_size(truncated.path);
    std::filesystem::resize_file(truncated.path, fullSize / 2);

    Document loaded;
    EXPECT_FALSE(BinaryFormat::load(truncated.path, loaded));
    EXPECT_EQ(BinaryFormat::loadPartMesh(truncated.path), nullptr);
}

TEST(BinaryFormatTest, BinaryAndJsonLoadsAgree) {
    Document doc;
    buildBoxPart(doc);

    TempFile binFile(tempPath("hz_test_binary_eq.hzpart"));
    TempFile jsonFile(tempPath("hz_test_json_eq.hzpart"));
    ASSERT_TRUE(BinaryFormat::save(binFile.path, doc));
    ASSERT_TRUE(NativeFormat::save(jsonFile.path, doc));

    Document fromBinary;
    Document fromJson;
    ASSERT_TRUE(BinaryFormat::load(binFile.path, fromBinary));
    ASSERT_TRUE(NativeFormat::load(jsonFile.path, fromJson));

    EXPECT_EQ(fromBinary.type(), fromJson.type());
    EXPECT_EQ(fromBinary.featureTree().featureCount(), fromJson.featureTree().featureCount());
    EXPECT_EQ(fromBinary.sketches().size(), fromJson.sketches().size());
}
