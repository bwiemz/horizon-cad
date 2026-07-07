#pragma once

#include <array>
#include <memory>
#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/topology/TopologyID.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::geo {
class NurbsSurface;
}  // namespace hz::geo

namespace hz::model {

/// One boundary polygon extracted from a B-Rep face outer loop.
struct BoundaryPolygon {
    std::vector<math::Vec3> points;              ///< Ordered loop (no closing duplicate).
    topo::TopologyID topoId;                     ///< Provenance: source face's topology ID.
    std::shared_ptr<geo::NurbsSurface> surface;  ///< Source face surface (may be null).
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
    /// Extract one polygon per face (outer loop, ordered).  The set is
    /// oriented so it encloses positive volume (outward normals); a solid
    /// whose loops are consistently wound inward is flipped globally.
    /// Faces with fewer than 3 loop vertices are skipped.
    static std::vector<BoundaryPolygon> extractFacePolygons(const topo::Solid& solid);

    /// Triangulate a planar (or near-planar) polygon by ear clipping in its
    /// dominant plane, preserving winding.  Handles non-convex polygons.
    /// Falls back to a fan when ear clipping cannot proceed.
    static std::vector<std::array<math::Vec3, 3>> triangulatePolygon(
        const std::vector<math::Vec3>& points);

    /// Signed volume enclosed by the polygon set (divergence theorem over
    /// fan triangles).  Positive means outward-oriented boundary.
    static double signedVolume(const std::vector<BoundaryPolygon>& polygons);
};

}  // namespace hz::model
