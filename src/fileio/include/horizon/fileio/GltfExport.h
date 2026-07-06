#pragma once

#include <string>
#include <vector>

#include "horizon/render/SceneGraph.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::io {

/// glTF 2.0 export (Phase 76, roadmap §7.12) — the web/AR/VR interchange
/// format whose metallic-roughness materials map 1:1 onto Horizon's PBR
/// model. Output is a self-contained binary GLB (12-byte header + JSON
/// chunk + BIN chunk), so a single file carries geometry and materials.
///
/// Horizon is Z-up; glTF is Y-up. The exporter adds a root node with the
/// -90° X rotation so models arrive upright in other viewers.
class GltfExport {
public:
    /// One exported object: a tessellated mesh with its material.
    struct Item {
        render::MeshData mesh;
        render::Material material;
        std::string name;
    };

    /// Serialize items to GLB bytes. Empty result when @p items is empty or
    /// contains no triangles.
    static std::vector<uint8_t> toGlb(const std::vector<Item>& items);

    /// Write items to a .glb file. False on I/O failure or empty geometry.
    static bool save(const std::string& path, const std::vector<Item>& items);

    /// Convenience: tessellate @p solid and export it with @p material.
    static bool saveSolid(const std::string& path, const topo::Solid& solid,
                          const render::Material& material, const std::string& name = "solid");
};

}  // namespace hz::io
