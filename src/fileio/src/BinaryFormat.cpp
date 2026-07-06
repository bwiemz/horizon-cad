#include "horizon/fileio/BinaryFormat.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/modeling/SolidTessellator.h"
#include "horizon/render/SceneGraph.h"
#include "hzbinary_generated.h"

namespace hz::io {

namespace {

constexpr uint32_t kBinaryVersion = 1;

std::vector<uint8_t> readAllBytes(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) return {};
    const std::streamsize size = file.tellg();
    if (size <= 0) return {};
    std::vector<uint8_t> bytes(static_cast<size_t>(size));
    file.seekg(0);
    file.read(reinterpret_cast<char*>(bytes.data()), size);
    if (!file.good()) return {};
    return bytes;
}

bool writeAllBytes(const std::string& filePath, const uint8_t* data, size_t size) {
    std::ofstream file(filePath, std::ios::binary);
    if (!file.is_open()) return false;
    file.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
    // Close explicitly so buffered data is flushed while we can still observe
    // the failure — the destructor swallows flush errors (e.g. disk full).
    file.close();
    return !file.fail();
}

/// Verify the buffer and return the root, or nullptr when invalid.
///
/// The generated verifier checks the minimum buffer size AND the "HZBF"
/// identifier bounds-safely; do NOT pre-check with BufferHasIdentifier here —
/// it reads bytes 4..7 unconditionally, which over-reads buffers shorter than
/// 8 bytes (truncated/corrupt files).
const fb::BinaryDocument* verifyAndGetRoot(const std::vector<uint8_t>& bytes) {
    if (bytes.empty()) return nullptr;
    flatbuffers::Verifier verifier(bytes.data(), bytes.size());
    if (!fb::VerifyBinaryDocumentBuffer(verifier)) return nullptr;
    return fb::GetBinaryDocument(bytes.data());
}

flatbuffers::Offset<fb::Mesh> buildMesh(flatbuffers::FlatBufferBuilder& fbb,
                                        const render::MeshData& mesh) {
    const auto positions = fbb.CreateVector(mesh.positions);
    const auto normals = fbb.CreateVector(mesh.normals);
    const auto indices = fbb.CreateVector(mesh.indices);
    return fb::CreateMesh(fbb, positions, normals, indices);
}

bool writeDocumentBuffer(const std::string& filePath, const std::string& kind,
                         const std::string& payload, const render::MeshData* mesh) {
    flatbuffers::FlatBufferBuilder fbb;
    std::vector<flatbuffers::Offset<fb::Mesh>> meshOffsets;
    if (mesh != nullptr) meshOffsets.push_back(buildMesh(fbb, *mesh));

    const auto root =
        fb::CreateBinaryDocument(fbb, kBinaryVersion, fbb.CreateString(kind),
                                 fbb.CreateString(payload), fbb.CreateVector(meshOffsets));
    fbb.Finish(root, fb::BinaryDocumentIdentifier());
    return writeAllBytes(filePath, fbb.GetBufferPointer(), fbb.GetSize());
}

}  // namespace

bool BinaryFormat::save(const std::string& filePath, const doc::Document& doc) {
    // The JSON payload omits the tessellation cache — it lives in typed
    // vectors where lightweight loaders can reach it without a JSON parse.
    const std::string payload = NativeFormat::documentToJson(doc, /*includeTessellation=*/false);
    const std::string kind = doc.type() == doc::DocumentType::Part ? "hzpart" : "hcad";

    render::MeshData mesh;
    const render::MeshData* meshPtr = nullptr;
    if (doc.solid() != nullptr) {
        mesh = model::SolidTessellator::tessellate(*doc.solid());
        if (!mesh.positions.empty()) meshPtr = &mesh;
    }
    return writeDocumentBuffer(filePath, kind, payload, meshPtr);
}

bool BinaryFormat::load(const std::string& filePath, doc::Document& doc) {
    const std::vector<uint8_t> bytes = readAllBytes(filePath);
    const fb::BinaryDocument* root = verifyAndGetRoot(bytes);
    if (root == nullptr || root->payload() == nullptr) return false;
    return NativeFormat::documentFromJson(root->payload()->str(), doc);
}

bool BinaryFormat::saveAssembly(const std::string& filePath, const doc::AssemblyDocument& asmDoc) {
    const std::string payload = NativeFormat::assemblyToJson(asmDoc, filePath);
    return writeDocumentBuffer(filePath, "hzasm", payload, nullptr);
}

bool BinaryFormat::loadAssembly(const std::string& filePath, doc::AssemblyDocument& asmDoc) {
    const std::vector<uint8_t> bytes = readAllBytes(filePath);
    const fb::BinaryDocument* root = verifyAndGetRoot(bytes);
    if (root == nullptr || root->payload() == nullptr) return false;
    return NativeFormat::assemblyFromJson(root->payload()->str(), asmDoc, filePath);
}

std::shared_ptr<render::MeshData> BinaryFormat::loadPartMesh(const std::string& filePath) {
    const std::vector<uint8_t> bytes = readAllBytes(filePath);
    const fb::BinaryDocument* root = verifyAndGetRoot(bytes);
    if (root == nullptr || root->meshes() == nullptr || root->meshes()->size() == 0) {
        return nullptr;
    }

    // Zero-copy access into the buffer; only the final MeshData copy touches
    // the bytes. The JSON payload is never parsed on this path.
    const fb::Mesh* m = root->meshes()->Get(0);
    if (m->positions() == nullptr || m->indices() == nullptr) return nullptr;
    if (m->positions()->size() == 0 || m->positions()->size() % 3 != 0 ||
        m->indices()->size() % 3 != 0) {
        return nullptr;
    }
    const uint32_t vertexCount = m->positions()->size() / 3;
    if (m->normals() != nullptr && m->normals()->size() != 0 &&
        m->normals()->size() != m->positions()->size()) {
        return nullptr;
    }
    for (uint32_t i = 0; i < m->indices()->size(); ++i) {
        if (m->indices()->Get(i) >= vertexCount) return nullptr;
    }

    auto mesh = std::make_shared<render::MeshData>();
    mesh->positions.assign(m->positions()->begin(), m->positions()->end());
    if (m->normals() != nullptr) mesh->normals.assign(m->normals()->begin(), m->normals()->end());
    mesh->indices.assign(m->indices()->begin(), m->indices()->end());
    return mesh;
}

bool BinaryFormat::isBinaryFile(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary);
    if (!file.is_open()) return false;
    char header[8] = {};
    file.read(header, sizeof(header));
    if (file.gcount() < static_cast<std::streamsize>(sizeof(header))) return false;
    // The FlatBuffers file identifier sits at bytes 4..7.
    return std::memcmp(header + 4, fb::BinaryDocumentIdentifier(), 4) == 0;
}

}  // namespace hz::io
