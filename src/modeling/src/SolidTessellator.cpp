#include "horizon/modeling/SolidTessellator.h"

#include "horizon/geometry/surfaces/NurbsSurface.h"

namespace hz::model {

render::MeshData SolidTessellator::tessellate(const topo::Solid& solid, double tolerance) {
    render::MeshData result;
    uint32_t vertexOffset = 0;

    for (const auto& face : solid.faces()) {
        if (!face.surface) continue;

        auto faceMesh = face.surface->tessellate(tolerance);

        // Append positions (3 floats per vertex).
        result.positions.insert(result.positions.end(), faceMesh.positions.begin(),
                                faceMesh.positions.end());

        // Append normals (3 floats per vertex).
        result.normals.insert(result.normals.end(), faceMesh.normals.begin(),
                              faceMesh.normals.end());

        // Append indices with offset to account for previously added vertices.
        for (uint32_t idx : faceMesh.indices) {
            result.indices.push_back(idx + vertexOffset);
        }

        vertexOffset += static_cast<uint32_t>(faceMesh.positions.size() / 3);
    }

    return result;
}

}  // namespace hz::model
