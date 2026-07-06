#pragma once

#include <cstdint>
#include <vector>

#include "horizon/math/Mat4.h"
#include "horizon/render/SceneGraph.h"

namespace hz::render {

/// One instanced draw batch: a shared mesh plus per-instance data. A batch
/// with N instances renders as a single instanced draw call instead of N
/// separate ones — the large-assembly path for "1000 identical bolts".
struct InstanceBatch {
    const MeshData* mesh = nullptr;       ///< shared geometry (first-seen node's mesh)
    uint64_t contentHash = 0;             ///< identity of the shared geometry
    std::vector<math::Mat4> transforms;   ///< per-instance world transforms
    std::vector<Material> materials;      ///< per-instance materials
    std::vector<const SceneNode*> nodes;  ///< source nodes, same order
};

/// Groups scene nodes with byte-identical meshes into instanced batches
/// (Phase 78, roadmap §7.14 large-assembly optimization).
///
/// Identity is the content hash of the mesh buffers, verified by buffer
/// sizes to guard against collisions. Batch order is deterministic
/// (first-seen order), so downstream GPU uploads are stable frame to frame.
class InstanceBatcher {
public:
    /// Batch every visible mesh node of @p scene.
    static std::vector<InstanceBatch> batch(const SceneGraph& scene);

    /// FNV-1a over the mesh's position/normal/index buffers.
    static uint64_t meshContentHash(const MeshData& mesh);
};

}  // namespace hz::render
