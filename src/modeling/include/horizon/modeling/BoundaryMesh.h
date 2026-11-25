#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/topology/TopologyID.h"

namespace hz::topo {
class Solid;
struct Wire;
}  // namespace hz::topo

namespace hz::geo {
class NurbsSurface;
class Surface;
}  // namespace hz::geo

namespace hz::model {

/// One boundary polygon extracted from a B-Rep face: its outer loop, with
/// any holes bridged in (see BoundaryMesh::keyholePolygon).
struct BoundaryPolygon {
    std::vector<math::Vec3> points;  ///< Ordered loop (no closing duplicate).
    /// A face with holes: its outer loop and its holes apart, each a loop of
    /// the face's own vertices (holes wound against the outer loop), for a
    /// caller that keeps the face as it is rather than triangulating it.
    /// Empty when the face has no holes (`points` is then its outer loop).
    std::vector<math::Vec3> outer;
    std::vector<std::vector<math::Vec3>> holes;
    topo::TopologyID topoId;                     ///< Provenance: source face's topology ID.
    std::shared_ptr<geo::NurbsSurface> surface;  ///< Source face surface (may be null).
    /// Source face's ideal surface (topo::Face::analyticSurface; may be null).
    std::shared_ptr<geo::NurbsSurface> analyticSurface;
    /// Source face's exact surface (topo::Face::exactSurface), set on the triangles an exact
    /// face reads as; null for every other polygon.
    std::shared_ptr<const geo::Surface> exactSurface;
    /// An exact face's triangle: the surface's outward normal at each of `points`, as the mesher
    /// found it (FaceMesh::normals), what a display shades it by. Empty for every other polygon.
    std::vector<math::Vec3> normals;
};

/// Faithful boundary evaluation of a B-Rep solid from its face loops.
///
/// Planar face surfaces are stored as bounding-rectangle patches that
/// over-cover the trimmed region (see Extrude), so the face vertex loops —
/// not surface tessellations — are the correct boundary for Boolean
/// classification, ray casting, and mass properties.  Curved faces are
/// approximated by their loop polygon, matching the MassProperties
/// convention.
class BoundaryMesh {
public:
    /// Extract one polygon per face (outer loop, ordered; holes bridged in).  The set is
    /// oriented so it encloses positive volume (outward normals); a solid
    /// whose loops are consistently wound inward is flipped globally.
    /// Faces with fewer than 3 loop vertices are skipped.
    ///
    /// A face with an exact surface (topo::Face::exactSurface) is meshed by
    /// TrimmedFaceMesher and read as one polygon per triangle, each with its face's topoId
    /// and exactSurface and wound outward (a mesh is never inverted, whatever the solid's
    /// loops do); a face without one is read from its loop as above. Faces keep the solid's
    /// order, an exact face's triangles standing where the face stands. This overload meshes
    /// at a chord tolerance of 1e-4 times the diagonal of the box around the solid's vertices and
    /// points along its exact edges, and throws std::runtime_error if an exact face cannot be
    /// meshed (never a silent hole).
    static std::vector<BoundaryPolygon> extractFacePolygons(const topo::Solid& solid);

    /// As above, at the given chord tolerance (absolute: scale it with the part). A face that
    /// cannot be meshed is never a silent hole. With @p problem given, it is set to the first
    /// such face's topoId tag and the mesher's reason ("<tag>: <reason>"; empty when every
    /// exact face meshed) and the faces that could be read still are, the failed face
    /// contributing nothing. With @p problem null, std::runtime_error is thrown with that text.
    /// @p cancelled is looked at between faces (MeshOptions::cancelled): once set, no exact face
    /// is meshed, and the first's reason is "cancelled".
    static std::vector<BoundaryPolygon> extractFacePolygons(
        const topo::Solid& solid, double chordTolerance, std::string* problem = nullptr,
        const std::atomic<bool>* cancelled = nullptr);

    /// Triangulate a planar (or near-planar) polygon by ear clipping in its
    /// dominant plane, preserving winding.  Handles non-convex polygons.
    /// Falls back to a fan when ear clipping cannot proceed.
    static std::vector<std::array<math::Vec3, 3>> triangulatePolygon(
        const std::vector<math::Vec3>& points);
    /// The same triangles, each as the indices of its corners in @p points:
    /// for a caller that joins triangles back together along their shared
    /// sides, which positions alone cannot tell where a polygon touches
    /// itself.
    static std::vector<std::array<size_t, 3>> triangulatePolygonCorners(
        const std::vector<math::Vec3>& points);

    /// A face loop's points: its vertices in loop order and, with
    /// @p alongCurves, points along each curved edge between them (a round
    /// hole bounded by one or two edges has too few vertices to be a
    /// polygon).
    static std::vector<math::Vec3> loopPoints(const topo::Wire& wire, bool alongCurves);

    /// A face with holes as one polygon: each hole, wound against @p outer,
    /// joined to it by a bridge that runs out to the hole and back. The
    /// polygon is weakly simple (the bridge is traversed twice) and
    /// triangulatePolygon, a fan, and signedVolume all read it as the face
    /// minus its holes. A hole with fewer than 3 points, or one no bridge
    /// reaches without crossing an edge, is left out.
    static std::vector<math::Vec3> keyholePolygon(const std::vector<math::Vec3>& outer,
                                                  std::vector<std::vector<math::Vec3>> holes);

    /// Signed volume enclosed by the polygon set (divergence theorem over
    /// fan triangles).  Positive means outward-oriented boundary.
    static double signedVolume(const std::vector<BoundaryPolygon>& polygons);
};

}  // namespace hz::model
