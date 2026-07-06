#include "horizon/render/InstanceBatcher.h"

#include <cstring>
#include <unordered_map>
#include <vector>

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

// Byte-exact equality of two meshes' buffers. Only ever called on a hash
// match, so this runs once per genuine duplicate (cheap) and turns the FNV
// hash into an exact identity — a hash collision can never merge two
// different meshes into the same instanced draw.
bool sameGeometry(const MeshData& a, const MeshData& b) {
    return a.positions == b.positions && a.normals == b.normals && a.indices == b.indices;
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
    // A hash can map to several batches (distinct meshes that collide, or the
    // future case of two equal hashes for different content); each candidate
    // is confirmed by an exact buffer compare, so a collision never merges
    // unlike geometry and never orphans an existing batch.
    std::unordered_map<uint64_t, std::vector<size_t>> byHash;

    for (const SceneNode* node : scene.collectVisibleMeshNodes()) {
        if (!node->hasMesh()) continue;
        const MeshData& mesh = node->mesh();
        if (mesh.positions.empty() || mesh.indices.empty()) continue;

        const uint64_t hash = meshContentHash(mesh);
        std::vector<size_t>& bucket = byHash[hash];

        size_t target = batches.size();  // sentinel: no match yet
        for (size_t candidate : bucket) {
            if (sameGeometry(*batches[candidate].mesh, mesh)) {
                target = candidate;
                break;
            }
        }
        if (target == batches.size()) {
            InstanceBatch fresh;
            fresh.mesh = &mesh;
            fresh.contentHash = hash;
            bucket.push_back(batches.size());
            batches.push_back(std::move(fresh));
        }

        InstanceBatch& batch = batches[target];
        batch.transforms.push_back(node->worldTransform());
        batch.materials.push_back(node->material());
        batch.nodes.push_back(node);
    }
    return batches;
}

}  // namespace hz::render
