#pragma once

#include <string>

#include "horizon/topology/Solid.h"

namespace hz::io {

/// Binary STL: the triangles of a solid's tessellation, each with its facet
/// normal, for printing and for tools that read nothing richer.
class StlExport {
public:
    /// The file's bytes. @p tolerance is the tessellation's chord tolerance.
    static std::string toBinary(const topo::Solid& solid, double tolerance = 0.1);

    /// Write the solid to @p path, atomically. False, with why in @p error,
    /// when the solid has no triangles or the file cannot be written.
    static bool save(const std::string& path, const topo::Solid& solid,
                     std::string* error = nullptr);
};

}  // namespace hz::io
