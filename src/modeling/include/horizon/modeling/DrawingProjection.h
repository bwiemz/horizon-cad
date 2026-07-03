#pragma once

#include <vector>

#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/topology/TopologyID.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::model {

/// Orthographic camera for 2D drawing generation.
///
/// `dir` points into the screen (the view direction); `up` orients the vertical
/// axis. The projected 2D basis is right = normalize(cross(dir, up)) and
/// up' = normalize(cross(right, dir)); a world point p maps to
/// (dot(p - origin, right), dot(p - origin, up')).
struct ViewProjection {
    math::Vec3 origin{0.0, 0.0, 0.0};  ///< A point on the projection plane.
    math::Vec3 dir{0.0, -1.0, 0.0};    ///< View direction (into the screen).
    math::Vec3 up{0.0, 0.0, 1.0};      ///< Up hint (need not be exactly perpendicular).
};

/// A model edge (or edge segment) projected to the 2D view plane, tagged with
/// its visibility and the genealogy of the source edge for model association.
struct ProjectedEdge {
    enum class Visibility { Visible, Hidden };

    math::Vec2 a;                 ///< 2D start (u, v).
    math::Vec2 b;                 ///< 2D end (u, v).
    topo::TopologyID sourceEdge;  ///< TopologyID of the originating model edge.
    Visibility visibility = Visibility::Visible;
};

/// Canonical orthographic views.
enum class StandardView { Front, Top, Right, Isometric };

/// Hidden-line removal for 2D drawing generation.
///
/// Projects every edge of a solid onto a view plane and classifies it as visible
/// or hidden by ray-casting toward the viewer against the solid's own
/// tessellation. Each edge is sampled into sub-segments, so a partly occluded
/// edge splits into separate visible and hidden runs; edges parallel to the view
/// direction collapse to a point and are dropped. This is the Phase 53 core; a
/// 2D R*-tree acceleration and tangent-edge classification are follow-ups.
class DrawingProjection {
public:
    /// Project and classify all edges of @p solid as seen through @p view.
    /// Returns one ProjectedEdge per maximal same-visibility run of each edge
    /// (a fully visible or fully hidden edge yields a single segment).
    static std::vector<ProjectedEdge> project(const topo::Solid& solid, const ViewProjection& view);

    /// A canonical orthographic view direction/up. `origin` is the world origin;
    /// framing and scale are the caller's concern.
    static ViewProjection standardView(StandardView view);
};

}  // namespace hz::model
