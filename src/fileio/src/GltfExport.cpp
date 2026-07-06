#include "horizon/fileio/GltfExport.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>

#include "horizon/modeling/SolidTessellator.h"
#include "horizon/topology/Solid.h"

namespace hz::io {

using nlohmann::json;

namespace {

constexpr uint32_t kGlbMagic = 0x46546C67;  // "glTF"
constexpr uint32_t kGlbVersion = 2;
constexpr uint32_t kChunkJson = 0x4E4F534A;  // "JSON"
constexpr uint32_t kChunkBin = 0x004E4942;   // "BIN\0"

void appendU32(std::vector<uint8_t>& out, uint32_t value) {
    const size_t at = out.size();
    out.resize(at + 4);
    std::memcpy(out.data() + at, &value, 4);
}

void padTo4(std::vector<uint8_t>& buffer, uint8_t fill) {
    while (buffer.size() % 4 != 0) buffer.push_back(fill);
}

}  // namespace

std::vector<uint8_t> GltfExport::toGlb(const std::vector<Item>& items) {
    // -- BIN chunk: positions, normals, indices per item, 4-byte aligned ------
    std::vector<uint8_t> bin;
    json bufferViews = json::array();
    json accessors = json::array();
    json meshes = json::array();
    json materials = json::array();
    json nodes = json::array();
    json meshNodeIndices = json::array();

    auto addView = [&](const void* data, size_t bytes, int target) -> int {
        const int viewIndex = static_cast<int>(bufferViews.size());
        json view;
        view["buffer"] = 0;
        view["byteOffset"] = bin.size();
        view["byteLength"] = bytes;
        view["target"] = target;
        bufferViews.push_back(view);
        const size_t at = bin.size();
        bin.resize(at + bytes);
        std::memcpy(bin.data() + at, data, bytes);
        padTo4(bin, 0);
        return viewIndex;
    };

    bool anyTriangles = false;
    for (const Item& item : items) {
        const render::MeshData& mesh = item.mesh;
        if (mesh.positions.empty() || mesh.indices.size() < 3) continue;
        anyTriangles = true;

        const size_t vertexCount = mesh.positions.size() / 3;

        // Positions accessor needs min/max per the glTF spec.
        float mn[3] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max()};
        float mx[3] = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest()};
        for (size_t v = 0; v < vertexCount; ++v) {
            for (int c = 0; c < 3; ++c) {
                mn[c] = std::min(mn[c], mesh.positions[3 * v + c]);
                mx[c] = std::max(mx[c], mesh.positions[3 * v + c]);
            }
        }

        const int posView = addView(mesh.positions.data(), mesh.positions.size() * sizeof(float),
                                    34962 /*ARRAY_BUFFER*/);
        const int posAccessor = static_cast<int>(accessors.size());
        accessors.push_back({{"bufferView", posView},
                             {"componentType", 5126 /*FLOAT*/},
                             {"count", vertexCount},
                             {"type", "VEC3"},
                             {"min", {mn[0], mn[1], mn[2]}},
                             {"max", {mx[0], mx[1], mx[2]}}});

        int normalAccessor = -1;
        if (mesh.normals.size() == mesh.positions.size()) {
            const int nView =
                addView(mesh.normals.data(), mesh.normals.size() * sizeof(float), 34962);
            normalAccessor = static_cast<int>(accessors.size());
            accessors.push_back({{"bufferView", nView},
                                 {"componentType", 5126},
                                 {"count", vertexCount},
                                 {"type", "VEC3"}});
        }

        const int idxView = addView(mesh.indices.data(), mesh.indices.size() * sizeof(uint32_t),
                                    34963 /*ELEMENT_ARRAY_BUFFER*/);
        const int idxAccessor = static_cast<int>(accessors.size());
        accessors.push_back({{"bufferView", idxView},
                             {"componentType", 5125 /*UNSIGNED_INT*/},
                             {"count", mesh.indices.size()},
                             {"type", "SCALAR"}});

        // Metallic-roughness maps 1:1 from the viewport material.
        const render::Material& m = item.material;
        json material;
        material["name"] = item.name + "_material";
        material["pbrMetallicRoughness"] = {
            {"baseColorFactor", {m.color.x, m.color.y, m.color.z, m.alpha}},
            {"metallicFactor", m.metallic},
            {"roughnessFactor", m.roughness}};
        if (m.alpha < 1.0f) {
            material["alphaMode"] = "BLEND";
        }
        const int materialIndex = static_cast<int>(materials.size());
        materials.push_back(material);

        json attributes;
        attributes["POSITION"] = posAccessor;
        if (normalAccessor >= 0) attributes["NORMAL"] = normalAccessor;
        json primitive;
        primitive["attributes"] = attributes;
        primitive["indices"] = idxAccessor;
        primitive["material"] = materialIndex;
        primitive["mode"] = 4;  // TRIANGLES

        const int meshIndex = static_cast<int>(meshes.size());
        meshes.push_back({{"name", item.name}, {"primitives", json::array({primitive})}});

        const int nodeIndex = static_cast<int>(nodes.size());
        nodes.push_back({{"name", item.name}, {"mesh", meshIndex}});
        meshNodeIndices.push_back(nodeIndex);
    }
    if (!anyTriangles) return {};

    // Root node: Horizon is Z-up, glTF is Y-up — rotate -90° about X
    // (column-major matrix).
    const int rootIndex = static_cast<int>(nodes.size());
    nodes.push_back({{"name", "zUpToYUp"},
                     {"children", meshNodeIndices},
                     {"matrix", {1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1}}});

    json root;
    root["asset"] = {{"version", "2.0"}, {"generator", "Horizon CAD"}};
    root["buffers"] = json::array({{{"byteLength", bin.size()}}});
    root["bufferViews"] = bufferViews;
    root["accessors"] = accessors;
    root["materials"] = materials;
    root["meshes"] = meshes;
    root["nodes"] = nodes;
    root["scenes"] = json::array({{{"nodes", {rootIndex}}}});
    root["scene"] = 0;

    // -- GLB container ------------------------------------------------------------
    std::vector<uint8_t> jsonChunk;
    {
        const std::string text = root.dump();
        jsonChunk.assign(text.begin(), text.end());
        padTo4(jsonChunk, ' ');  // JSON chunks pad with spaces
    }
    padTo4(bin, 0);

    std::vector<uint8_t> glb;
    appendU32(glb, kGlbMagic);
    appendU32(glb, kGlbVersion);
    appendU32(glb, static_cast<uint32_t>(12 + 8 + jsonChunk.size() + 8 + bin.size()));
    appendU32(glb, static_cast<uint32_t>(jsonChunk.size()));
    appendU32(glb, kChunkJson);
    glb.insert(glb.end(), jsonChunk.begin(), jsonChunk.end());
    appendU32(glb, static_cast<uint32_t>(bin.size()));
    appendU32(glb, kChunkBin);
    glb.insert(glb.end(), bin.begin(), bin.end());
    return glb;
}

bool GltfExport::save(const std::string& path, const std::vector<Item>& items) {
    const std::vector<uint8_t> glb = toGlb(items);
    if (glb.empty()) return false;
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) return false;
    file.write(reinterpret_cast<const char*>(glb.data()), static_cast<std::streamsize>(glb.size()));
    file.close();
    return !file.fail();
}

bool GltfExport::saveSolid(const std::string& path, const topo::Solid& solid,
                           const render::Material& material, const std::string& name) {
    Item item;
    item.mesh = model::SolidTessellator::tessellate(solid);
    item.material = material;
    item.name = name;
    return save(path, {item});
}

}  // namespace hz::io
