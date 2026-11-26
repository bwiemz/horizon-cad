#pragma once

#include <atomic>
#include <string>

#include "horizon/geometry/MeshData.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// Converts a B-Rep Solid into a triangle mesh (geo::MeshData).
///
/// Planar faces are triangulated from their trimmed vertex loops — their
/// stored surfaces are bounding-rectangle patches that over-cover any
/// non-rectangular face (see Extrude).  Curved faces fall back to NURBS
/// surface tessellation for smooth shading. A face with an exact surface
/// (topo::Face::exactSurface, as read from a STEP file) is meshed at the
/// read point (BoundaryMesh::extractFacePolygons) at the display's angle,
/// shaded by its surface's normals.
class SolidTessellator {
public:
    /// The chord tolerance a part is drawn at: the viewport's for curved faces, in model units.
    /// The part's exact counterpart (doc::BuildResult::exact) is meshed at it once.
    static constexpr double kDisplayTolerance = 0.1;
    /// The angle an exact face's mesh is drawn at (MeshOptions::maxAngle, between neighbouring
    /// triangles' normals), in radians: the owner's choice, coarser than the read point's 0.35.
    /// The mesh is cached in the part file and loaded for every part of an assembly, so it is
    /// kept light; at 0.5 rad (about 29 degrees) it still shades smoothly by the surface's
    /// normals. Where the angle, not the tolerance, sets a round edge's chords, they turn by half
    /// of it: 26 a turn, where 0.35 made 36.
    static constexpr double kDisplayMaxAngle = 0.5;

    /// Tessellate all faces of @p solid and return a merged mesh: one entry of faceTags for each
    /// face, which each of its triangles names (triangleFaces), and the edges the part shows.
    /// @param tolerance  Tessellation chord-height tolerance (curved faces; an exact face's
    ///                   mesh and its edges, absolute). An exact face is meshed at
    ///                   kDisplayMaxAngle, whatever the tolerance.
    /// An exact face is meshed through the read point's problem overload, so nothing is thrown
    /// for a face that cannot be meshed: it is left out of the mesh, and with @p problem given,
    /// it says which and why ("<tag>: <reason>"; empty when every face was meshed). An exact
    /// face's seam (an edge with the face on both sides) is not shown; its other edges are,
    /// along their exact curves. @p cancelled is looked at between exact faces: once it is set,
    /// none is meshed, and the first's reason is "cancelled".
    static geo::MeshData tessellate(const topo::Solid& solid, double tolerance = kDisplayTolerance,
                                    std::string* problem = nullptr,
                                    const std::atomic<bool>* cancelled = nullptr);
};

}  // namespace hz::model
