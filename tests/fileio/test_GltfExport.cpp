#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

#include "horizon/fileio/GltfExport.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/render/MaterialLibrary.h"

using hz::io::GltfExport;
using hz::model::PrimitiveFactory;
using hz::model::SolidTessellator;
using nlohmann::json;

namespace {

uint32_t readU32(const std::vector<uint8_t>& bytes, size_t at) {
    uint32_t v = 0;
    std::memcpy(&v, bytes.data() + at, 4);
    return v;
}

/// Split a GLB into its parsed JSON root and BIN chunk length.
void parseGlb(const std::vector<uint8_t>& glb, json& outRoot, size_t& outBinLength) {
    ASSERT_GE(glb.size(), 20u);
    EXPECT_EQ(readU32(glb, 0), 0x46546C67u);  // "glTF"
    EXPECT_EQ(readU32(glb, 4), 2u);
    EXPECT_EQ(readU32(glb, 8), glb.size()) << "declared GLB length mismatch";

    const uint32_t jsonLen = readU32(glb, 12);
    EXPECT_EQ(readU32(glb, 16), 0x4E4F534Au);  // "JSON"
    EXPECT_EQ(jsonLen % 4, 0u);
    outRoot = json::parse(glb.begin() + 20, glb.begin() + 20 + jsonLen);

    const size_t binHeader = 20 + jsonLen;
    ASSERT_GE(glb.size(), binHeader + 8);
    outBinLength = readU32(glb, binHeader);
    EXPECT_EQ(readU32(glb, binHeader + 4), 0x004E4942u);  // "BIN\0"
    EXPECT_EQ(binHeader + 8 + outBinLength, glb.size());
}

GltfExport::Item boxItem(const std::string& material = "Polished Steel") {
    auto box = PrimitiveFactory::makeBox(10.0, 6.0, 4.0);
    GltfExport::Item item;
    item.mesh = SolidTessellator::tessellate(*box);
    item.material = hz::render::MaterialLibrary::find(material);
    item.name = "box";
    return item;
}

}  // namespace

TEST(GltfExportTest, GlbStructureIsValid) {
    const GltfExport::Item item = boxItem();
    const auto glb = GltfExport::toGlb({item});
    ASSERT_FALSE(glb.empty());

    json root;
    size_t binLength = 0;
    parseGlb(glb, root, binLength);

    EXPECT_EQ(root["asset"]["version"], "2.0");
    ASSERT_EQ(root["buffers"].size(), 1u);
    EXPECT_EQ(root["buffers"][0]["byteLength"].get<size_t>(), binLength);
    EXPECT_EQ(root["scene"], 0);
    ASSERT_EQ(root["meshes"].size(), 1u);
    EXPECT_EQ(root["meshes"][0]["name"], "box");

    // Every buffer view is 4-byte aligned and inside the BIN chunk.
    for (const auto& view : root["bufferViews"]) {
        EXPECT_EQ(view["byteOffset"].get<size_t>() % 4, 0u);
        EXPECT_LE(view["byteOffset"].get<size_t>() + view["byteLength"].get<size_t>(), binLength);
    }
}

TEST(GltfExportTest, AccessorsMatchTessellation) {
    const GltfExport::Item item = boxItem();
    const size_t vertexCount = item.mesh.positions.size() / 3;
    const size_t indexCount = item.mesh.indices.size();

    const auto glb = GltfExport::toGlb({item});
    json root;
    size_t binLength = 0;
    parseGlb(glb, root, binLength);

    const auto& prim = root["meshes"][0]["primitives"][0];
    const auto& pos = root["accessors"][prim["attributes"]["POSITION"].get<int>()];
    const auto& idx = root["accessors"][prim["indices"].get<int>()];

    EXPECT_EQ(pos["count"].get<size_t>(), vertexCount);
    EXPECT_EQ(pos["type"], "VEC3");
    EXPECT_EQ(idx["count"].get<size_t>(), indexCount);
    EXPECT_EQ(idx["componentType"], 5125);  // UNSIGNED_INT

    // Position bounds match the 10×6×4 box.
    EXPECT_NEAR(pos["min"][0].get<double>(), 0.0, 1e-5);
    EXPECT_NEAR(pos["max"][0].get<double>(), 10.0, 1e-5);
    EXPECT_NEAR(pos["max"][1].get<double>(), 6.0, 1e-5);
    EXPECT_NEAR(pos["max"][2].get<double>(), 4.0, 1e-5);

    // Normals present with matching count.
    ASSERT_TRUE(prim["attributes"].contains("NORMAL"));
    EXPECT_EQ(root["accessors"][prim["attributes"]["NORMAL"].get<int>()]["count"].get<size_t>(),
              vertexCount);
}

TEST(GltfExportTest, MaterialPassthrough) {
    const auto glb = GltfExport::toGlb({boxItem("Polished Steel")});
    json root;
    size_t binLength = 0;
    parseGlb(glb, root, binLength);

    const auto& pbr = root["materials"][0]["pbrMetallicRoughness"];
    EXPECT_FLOAT_EQ(pbr["metallicFactor"].get<float>(), 1.0f);
    EXPECT_NEAR(pbr["roughnessFactor"].get<double>(), 0.12, 1e-6);
    EXPECT_FALSE(root["materials"][0].contains("alphaMode"));

    // Transparent materials switch to BLEND.
    const auto glassGlb = GltfExport::toGlb({boxItem("Glass")});
    json glassRoot;
    parseGlb(glassGlb, glassRoot, binLength);
    EXPECT_EQ(glassRoot["materials"][0]["alphaMode"], "BLEND");
    EXPECT_LT(glassRoot["materials"][0]["pbrMetallicRoughness"]["baseColorFactor"][3].get<double>(),
              1.0);
}

TEST(GltfExportTest, ZUpToYUpRootNode) {
    const auto glb = GltfExport::toGlb({boxItem()});
    json root;
    size_t binLength = 0;
    parseGlb(glb, root, binLength);

    const int rootNode = root["scenes"][0]["nodes"][0].get<int>();
    const auto& node = root["nodes"][rootNode];
    ASSERT_TRUE(node.contains("matrix"));
    // Column-major -90° about X: Y column = (0,0,-1), Z column = (0,1,0).
    EXPECT_EQ(node["matrix"][6].get<int>(), -1);
    EXPECT_EQ(node["matrix"][9].get<int>(), 1);
    ASSERT_EQ(node["children"].size(), 1u);
}

TEST(GltfExportTest, MultipleItemsShareOneBuffer) {
    auto cyl = PrimitiveFactory::makeCylinder(3.0, 8.0);
    GltfExport::Item second;
    second.mesh = SolidTessellator::tessellate(*cyl);
    second.material = hz::render::MaterialLibrary::find("Rubber");
    second.name = "cyl";

    const auto glb = GltfExport::toGlb({boxItem(), second});
    json root;
    size_t binLength = 0;
    parseGlb(glb, root, binLength);

    EXPECT_EQ(root["meshes"].size(), 2u);
    EXPECT_EQ(root["materials"].size(), 2u);
    EXPECT_EQ(root["buffers"].size(), 1u);
    EXPECT_EQ(root["nodes"].size(), 3u);  // two mesh nodes + the Z-up root
}

TEST(GltfExportTest, SaveSolidWritesFile) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    const auto path = std::filesystem::temp_directory_path() / "hz_gltf_test.glb";
    ASSERT_TRUE(
        GltfExport::saveSolid(path.string(), *box, hz::render::MaterialLibrary::find("Wood")));
    EXPECT_GT(std::filesystem::file_size(path), 500u);
    std::filesystem::remove(path);
}

TEST(GltfExportTest, EmptyInputFails) {
    EXPECT_TRUE(GltfExport::toGlb({}).empty());
    GltfExport::Item empty;
    empty.name = "nothing";
    EXPECT_TRUE(GltfExport::toGlb({empty}).empty());
}
