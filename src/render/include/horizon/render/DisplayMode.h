#pragma once

namespace hz::render {

/// How solids are drawn.
enum class DisplayMode {
    Shaded,           ///< faces only
    ShadedWithEdges,  ///< faces, and the part's edges over them
    Wireframe,        ///< the part's edges only
};

}  // namespace hz::render
