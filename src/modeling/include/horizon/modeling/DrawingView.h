#pragma once

#include <vector>

#include "horizon/math/Vec2.h"
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
    StandardView kind = StandardView::Front;
    std::vector<ProjectedEdge> edges;
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
    /// Project @p solid through @p view, compute its 2D bounds, and return the
    /// (unplaced) DrawingView.
    static DrawingView makeView(const topo::Solid& solid, StandardView view);

    /// The classic engineering layout: front (lower-left), top (above front),
    /// right (right of front), and isometric (upper-right), placed without
    /// overlap and separated by @p gap.
    static Drawing standardViews(const topo::Solid& solid, double gap = 10.0);
};

}  // namespace hz::model
