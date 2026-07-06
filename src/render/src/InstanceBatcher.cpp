#include "horizon/render/InstanceBatcher.h"

#include <cstring>
#include <unordered_map>

namespace hz::render {

namespace {

uint64_t fnv1a(const void* data, size_t bytes, uint64_t hash) {
    const auto* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < bytes; ++i) {
        hash ^= p[i];
        hash *= 1099511628211ULL;
    }
    return hash;
}

}  // namespace

uint64_t InstanceBatcher::meshContentHash(const MeshData& mesh) {
    uint64_t hash = 1469598103934665603ULL;
    hash = fnv1a(mesh.positions.data(), mesh.positions.size() * sizeof(float), hash);
    hash = fnv1a(mesh.normals.data(), mesh.normals.size() * sizeof(float), hash);
    hash = fnv1a(mesh.indices.data(), mesh.indices.size() * sizeof(uint32_t), hash);
    return hash;
}

std::vector<InstanceBatch> InstanceBatcher::batch(const SceneGraph& scene) {
    std::vector<InstanceBatch> batches;
    std::unordered_map<uint64_t, size_t> byHash;  // contentHash → batch index

    for (const SceneNode* node : scene.collectVisibleMeshNodes()) {
        if (!node->hasMesh()) continue;
        const MeshData& mesh = node->mesh();
        if (mesh.positions.empty() || mesh.indices.empty()) continue;

        const uint64_t hash = meshContentHash(mesh);
        auto it = byHash.find(hash);
        bool matched = false;
        if (it != byHash.end()) {
            // Guard hash collisions: buffer sizes must agree too.
            const InstanceBatch& candidate = batches[it->second];
            matched = candidate.mesh->positions.size() == mesh.positions.size() &&
                      candidate.mesh->indices.size() == mesh.indices.size() &&
                      candidate.mesh->normals.size() == mesh.normals.size();
        }

        if (!matched) {
            InstanceBatch fresh;
            fresh.mesh = &mesh;
            fresh.contentHash = hash;
            byHash[hash] = batches.size();
            batches.push_back(std::move(fresh));
            it = byHash.find(hash);
        }

        InstanceBatch& batch = batches[it->second];
        batch.transforms.push_back(node->worldTransform());
        batch.materials.push_back(node->material());
        batch.nodes.push_back(node);
    }
    return batches;
}

}  // namespace hz::render
