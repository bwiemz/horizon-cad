#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "horizon/math/Vec2.h"
#include "horizon/modeling/DrawingBalloon.h"
#include "horizon/modeling/DrawingDimension.h"
#include "horizon/modeling/DrawingProjection.h"
#include "horizon/modeling/GeometricTolerance.h"
#include "horizon/modeling/Sheet.h"
#include "horizon/modeling/TitleBlock.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::model {

/// What a view shows (Phase 149): the part as seen, the part cut, or a part
/// of another view drawn larger.
enum class ViewRole { Projection, Section, Detail };

/// One placed view in a 2D drawing: the projected/classified geometry, its 2D
/// bounding box in view space, and where it sits on the sheet.
///
/// A view-space point `p` renders on the sheet at `(p - boundsMin) * scale +
/// placement` (toSheet), so `placement` is the sheet position of the view's
/// lower-left corner and each view occupies `[placement, placement +
/// (boundsMax - boundsMin) * scale]`.
struct DrawingView {
    StandardView kind = StandardView::Front;  ///< label; meaningful for standard views
    ViewProjection projection;                ///< the camera this view was projected through
    /// A section looks at its cut: the plane through `projection.origin`,
    /// facing back along `projection.dir`.
    ViewRole role = ViewRole::Projection;
    /// "A" names section A-A or detail A, in the caption under the view and
    /// in its mark on its source. Empty: neither is drawn.
    std::string label;
    /// The view it was taken from (an index into the drawing's views), where
    /// its mark is drawn: a section's cut, a detail's circle. -1: none.
    int source = -1;
    /// A detail's circle, in its source's view space (model millimetres).
    math::Vec2 detailCenter{0.0, 0.0};
    double detailRadius = 0.0;
    std::vector<ProjectedEdge> edges;
    std::vector<LinearDimension> dimensions;        ///< dimensions anchored to this view's edges
    std::vector<RadialDimension> radialDimensions;  ///< R/⌀ dimensions on circular edges
    std::vector<FeatureControlFrame> tolerances;    ///< GD&T frames anchored to this view's edges
    std::vector<DatumFeature> datums;               ///< datum feature symbols anchored to edges
    std::vector<DrawingBalloon> balloons;           ///< BOM balloons anchored to this view's edges

    // Section views (Phase 61): the cut profile and its hatching, in view
    // space. Empty for ordinary projection views.
    std::vector<std::vector<math::Vec2>> sectionLoops;            ///< closed cut boundaries
    std::vector<std::pair<math::Vec2, math::Vec2>> sectionHatch;  ///< 45° hatch segments
    /// Centre lines (Phase 149), in view space, to the outline: a cross on
    /// each hole or boss seen end-on, an axis along each seen side-on. The
    /// export runs them a little past it.
    std::vector<std::pair<math::Vec2, math::Vec2>> centreLines;
    math::Vec2 boundsMin{0.0, 0.0};
    math::Vec2 boundsMax{0.0, 0.0};
    math::Vec2 placement{0.0, 0.0};
    /// Sheet millimetres per model millimetre: 0.5 for 1:2, 2 for 2:1.
    double scale = 1.0;
    /// Whether the view draws its hidden edges, and its tangent edges (where
    /// a fillet meets a face). A new drawing sheet leaves tangent edges out.
    bool showHidden = true;
    bool showTangentEdges = true;
    bool showCentreLines = true;

    /// Where a view-space point lands on the sheet. Every renderer maps with
    /// this, so a view's scale reaches all it draws.
    math::Vec2 toSheet(const math::Vec2& p) const {
        return {(p.x - boundsMin.x) * scale + placement.x,
                (p.y - boundsMin.y) * scale + placement.y};
    }

    /// Model-space extents.
    double width() const { return boundsMax.x - boundsMin.x; }
    double height() const { return boundsMax.y - boundsMin.y; }
    /// Extents on the sheet.
    double sheetWidth() const { return width() * scale; }
    double sheetHeight() const { return height() * scale; }

    /// The sheet room under a view that its caption takes.
    static constexpr double kCaptionRoom = 8.0;
    /// The view's rectangle on the sheet (lower-left, upper-right), with the
    /// room below it for its caption when it has a label.
    std::pair<math::Vec2, math::Vec2> sheetFootprint() const;
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

    /// The standard scales, largest first: 10:1 down to 1:100 (ISO 5455).
    static const std::vector<double>& standardScales();

    /// A sheet of the four standard views in third-angle projection: Front
    /// at the lower left, Top above it, Right to its right, Isometric above
    /// that. They are centred in the space above the title block, @p gap
    /// apart, at the largest standard scale at which they fit, or at
    /// @p fixedScale when it is positive (the user chose it; it may not fit).
    /// @p chosenScale receives the scale used. Tangent edges are left out, as
    /// drawings show them.
    static Drawing sheetLayout(const topo::Solid& solid, const Sheet& sheet,
                               const TitleBlock& titleBlock, double gap = 10.0,
                               double* chosenScale = nullptr, double fixedScale = 0.0);

    /// A scale as a drawing states it: "1:2", "1:1", "5:1".
    static std::string scaleName(double scale);

    /// A balloon numbered @p item on @p view, for the component whose names
    /// start with @p namePrefix (Phase 150): on the longest edge of it the
    /// view shows, its circle outside the view, away from the view's centre.
    /// None when the view shows none of it.
    static std::optional<DrawingBalloon> balloonFor(const DrawingView& view,
                                                    const std::string& namePrefix, int item);

    /// Where @p view can go on @p sheet (its placement): inside the border,
    /// clear of the title block, and @p gap from every view of @p drawing
    /// and its caption. Candidates are the sheet's corner and the sides of
    /// the views, top first. None: beside the sheet, and @p fits false.
    static math::Vec2 freePlacement(const Drawing& drawing, const DrawingView& view,
                                    const Sheet& sheet, const TitleBlock& titleBlock,
                                    double gap = 10.0, bool* fits = nullptr);

    /// A detail view: crop @p source's geometry to the circle (@p center,
    /// @p radius) in view space and enlarge it by @p scale about that center.
    /// Edges crossing the circle are clipped to it; each kept edge preserves its
    /// visibility and source TopologyID. The returned view carries @p source's
    /// projection and its own recomputed bounds.
    static DrawingView detailView(const DrawingView& source, const math::Vec2& center,
                                  double radius, double scale);
};

}  // namespace hz::model
