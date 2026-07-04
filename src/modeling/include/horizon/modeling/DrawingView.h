#pragma once

#include <vector>

#include "horizon/math/Vec2.h"
#include "horizon/modeling/DrawingDimension.h"
#include "horizon/modeling/DrawingProjection.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::model {

/// One placed view in a 2D drawing: the projected/classified geometry, its 2D
/// bounding box in view space, and where it sits on the sheet.
///
/// A view-space point `p` renders on the sheet at `(p - boundsMin) + placement`,
/// so `placement` is the sheet position of the view's lower-left corner and each
/// view occupies `[placement, placement + (boundsMax - boundsMin)]`.
struct DrawingView {
    StandardView kind = StandardView::Front;  ///< label; meaningful for standard views
    ViewProjection projection;                ///< the camera this view was projected through
    std::vector<ProjectedEdge> edges;
    std::vector<LinearDimension> dimensions;  ///< dimensions anchored to this view's edges
    math::Vec2 boundsMin{0.0, 0.0};
    math::Vec2 boundsMax{0.0, 0.0};
    math::Vec2 placement{0.0, 0.0};

    double width() const { return boundsMax.x - boundsMin.x; }
    double height() const { return boundsMax.y - boundsMin.y; }
};

/// A 2D drawing: a set of placed orthographic/isometric views of one solid.
struct Drawing {
    std::vector<DrawingView> views;
};

/// Builds standard multi-view drawings from a solid.
class DrawingGenerator {
public:
    /// Project @p solid through a standard @p view, compute its 2D bounds, and
    /// return the (unplaced) DrawingView.
    static DrawingView makeView(const topo::Solid& solid, StandardView view);

    /// Project @p solid through an arbitrary camera — the basis for auxiliary
    /// views (e.g. looking square at an angled face). `kind` is left at its
    /// default; `projection` records the camera used.
    static DrawingView makeView(const topo::Solid& solid, const ViewProjection& projection);

    /// An auxiliary view looking straight at a face with the given outward
    /// @p faceNormal (the view direction is the negated normal, so the face
    /// faces the viewer). @p up orients the vertical axis.
    static DrawingView auxiliaryView(const topo::Solid& solid, const math::Vec3& faceNormal,
                                     const math::Vec3& up = math::Vec3(0.0, 0.0, 1.0));

    /// The classic engineering layout: front (lower-left), top (above front),
    /// right (right of front), and isometric (upper-right), placed without
    /// overlap and separated by @p gap.
    static Drawing standardViews(const topo::Solid& solid, double gap = 10.0);

    /// A detail view: crop @p source's geometry to the circle (@p center,
    /// @p radius) in view space and enlarge it by @p scale about that center.
    /// Edges crossing the circle are clipped to it; each kept edge preserves its
    /// visibility and source TopologyID. The returned view carries @p source's
    /// projection and its own recomputed bounds.
    static DrawingView detailView(const DrawingView& source, const math::Vec2& center,
                                  double radius, double scale);
};

}  // namespace hz::model
