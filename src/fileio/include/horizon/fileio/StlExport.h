#pragma once

#include <string>

#include "horizon/geometry/MeshData.h"
#include "horizon/topology/Solid.h"

namespace hz::io {

/// Binary STL: the triangles of a solid's tessellation, each with its facet
/// normal, for printing and for tools that read nothing richer.
class StlExport {
public:
    /// The chord tolerance save() meshes a solid at, in model units: the display's least
    /// (model::SolidTessellator::kDisplayTolerance), whatever the part's size.
    static constexpr double kTolerance = 0.1;

    /// The file's bytes. @p tolerance is the tessellation's chord tolerance.
    /// An exact face that cannot be meshed is left out, and said in
    /// @p problem when given (model::SolidTessellator::tessellate).
    static std::string toBinary(const topo::Solid& solid, double tolerance = kTolerance,
                                std::string* problem = nullptr);
    /// The file's bytes for @p mesh's triangles, as they are: a mesh already
    /// made (the one an exact part's build kept, Document::exactMesh), not
    /// meshed again. A triangle that names a corner the mesh does not have is
    /// left out.
    static std::string toBinary(const geo::MeshData& mesh);

    /// Write the solid to @p path, atomically, meshed at kTolerance. False,
    /// with why in @p error, when the solid has no triangles, a face of it
    /// cannot be meshed, or the file cannot be written.
    static bool save(const std::string& path, const topo::Solid& solid,
                     std::string* error = nullptr);
    /// Write @p mesh to @p path, atomically, its triangles as they are. False,
    /// with why in @p error, when it has no triangles or the file cannot be
    /// written.
    static bool save(const std::string& path, const geo::MeshData& mesh,
                     std::string* error = nullptr);
};

}  // namespace hz::io
