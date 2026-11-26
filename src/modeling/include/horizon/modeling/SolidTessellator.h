#pragma once

#include <atomic>
#include <string>
#include <vector>

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
    /// The least chord tolerance a part is drawn at, in model units: a part's own is
    /// displayTolerance(), which is this for a part up to about 667 across.
    static constexpr double kDisplayTolerance = 0.1;
    /// A large part is drawn to this much of its size (topo::exactExtent, its diagonal), not to
    /// kDisplayTolerance: the owner's choice. The mesh is cached in the part file and loaded for
    /// every part of an assembly, and at 0.1 a part metres across was hundreds of thousands of
    /// triangles, whatever the angle (the Moon buggy, 4.49 m across: 329k, its wheels' spheres
    /// and tori alone 302k; 49k at 1.5e-4 of its diagonal).
    static constexpr double kDisplayRelativeTolerance = 1.5e-4;
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
    static geo::MeshData tessellate(const topo::Solid& solid, double tolerance,
                                    std::string* problem = nullptr,
                                    const std::atomic<bool>* cancelled = nullptr);

    /// As above, at displayTolerance(@p solid): the part as it is drawn.
    static geo::MeshData tessellate(const topo::Solid& solid);

    /// The chord tolerance @p solid is drawn at: kDisplayRelativeTolerance of its size
    /// (topo::exactExtent, the diagonal of the box round its vertices and exact edges, the size
    /// its model tolerance comes from), or kDisplayTolerance where that is larger.
    static double displayTolerance(const topo::Solid& solid);

    /// The chord tolerance a part of @p bodies is drawn at: displayTolerance of the solid they are
    /// joined into, from the size of them all (topo::exactExtent of them together), found before
    /// they are joined, so that each can be meshed on its own at the part's tolerance.
    static double displayTolerance(const std::vector<const topo::Solid*>& bodies);
};

}  // namespace hz::model
