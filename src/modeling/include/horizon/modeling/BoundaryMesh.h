#pragma once

#include <array>
#include <memory>
#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/topology/TopologyID.h"

namespace hz::topo {
class Solid;
struct Wire;
}  // namespace hz::topo

namespace hz::geo {
class NurbsSurface;
}  // namespace hz::geo

namespace hz::model {

/// One boundary polygon extracted from a B-Rep face: its outer loop, with
/// any holes bridged in (see BoundaryMesh::keyholePolygon).
struct BoundaryPolygon {
    std::vector<math::Vec3> points;              ///< Ordered loop (no closing duplicate).
    topo::TopologyID topoId;                     ///< Provenance: source face's topology ID.
    std::shared_ptr<geo::NurbsSurface> surface;  ///< Source face surface (may be null).
    /// Source face's ideal surface (topo::Face::analyticSurface; may be null).
    std::shared_ptr<geo::NurbsSurface> analyticSurface;
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
    static std::vector<BoundaryPolygon> extractFacePolygons(const topo::Solid& solid);

    /// Triangulate a planar (or near-planar) polygon by ear clipping in its
    /// dominant plane, preserving winding.  Handles non-convex polygons.
    /// Falls back to a fan when ear clipping cannot proceed.
    static std::vector<std::array<math::Vec3, 3>> triangulatePolygon(
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
