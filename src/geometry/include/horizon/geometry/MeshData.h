#pragma once

#include <cstdint>
#include <vector>

namespace hz::geo {

/// CPU-side triangle mesh (positions + normals + indices).
///
/// The neutral exchange type between the modeling kernel and its consumers
/// (render, fileio, document).  Lives in geometry so the kernel does not
/// depend on the render module; render aliases it as render::MeshData.
struct MeshData {
    std::vector<float> positions;   ///< 3 floats per vertex
    std::vector<float> normals;     ///< 3 floats per vertex
    std::vector<uint32_t> indices;  ///< triangle list
};

}  // namespace hz::geo
