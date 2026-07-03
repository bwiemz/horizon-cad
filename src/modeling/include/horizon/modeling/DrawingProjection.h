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
/// Projects every edge of a solid onto a view plane and classifies each as
/// visible or hidden by ray-casting from the edge midpoint toward the viewer
/// against the solid's own tessellation. This is the Phase 53 core; a 2D R*-tree
/// acceleration, tangent-edge classification, and per-edge partial-visibility
/// splitting are follow-ups.
class DrawingProjection {
public:
    /// Project and classify all edges of @p solid as seen through @p view.
    /// Straight edges yield one segment; curved edges are sampled into several.
    static std::vector<ProjectedEdge> project(const topo::Solid& solid, const ViewProjection& view);

    /// A canonical orthographic view direction/up. `origin` is the world origin;
    /// framing and scale are the caller's concern.
    static ViewProjection standardView(StandardView view);
};

}  // namespace hz::model
