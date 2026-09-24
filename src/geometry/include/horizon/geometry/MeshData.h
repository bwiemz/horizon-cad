#pragma once

#include <cstdint>
#include <string>
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

    // What the mesh shows of the solid it was made from, for picking and
    // for drawing the part's edges. Empty for a mesh that was not made from
    // a solid's faces (one read from a file's cache, or imported).

    /// The face each triangle belongs to, as an index into faceTags: one
    /// entry per triangle, or none.
    std::vector<uint32_t> triangleFaces;
    /// The faces' persistent names (topo::TopologyID tags).
    std::vector<std::string> faceTags;

    /// One of the solid's edges as the part shows it.
    struct Edge {
        std::string tag;            ///< its persistent name (a TopologyID tag)
        std::vector<float> points;  ///< a polyline, 3 floats per point
    };
    /// The part's edges: not the seams between two facets of one curved
    /// surface, nor between two pieces of one flat face.
    std::vector<Edge> edges;

    /// Whether triangleFaces names a face for every triangle.
    bool hasFaces() const { return !indices.empty() && triangleFaces.size() == indices.size() / 3; }
};

}  // namespace hz::geo
