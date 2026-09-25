#pragma once

#include <memory>

#include "horizon/drafting/DimensionStyle.h"

namespace hz::draft {
class DraftLinearDimension;
}  // namespace hz::draft

namespace hz::model {
struct DrawingView;
struct LinearDimension;
}  // namespace hz::model

namespace hz::io {

/// Renders a model-driven dimension onto a drawing view as a draftable dimension
/// entity, which the DXF writer decomposes into lines + text on export.
///
/// The dimension's model edge (identified by TopologyID) is located in the view's
/// projected geometry; the resulting dimension spans that edge's 2D projection.
/// For an edge lying in the view plane (the usual case for a dimensioned edge)
/// the drafted length equals the true model length.
class DrawingDimensionRenderer {
public:
    /// Build an aligned linear dimension for @p dim's edge as it appears in
    /// @p view, with the dimension line offset from the edge by @p offset (in
    /// sheet units). Returns nullptr if the dimension's edge does not appear in
    /// the view.
    ///
    /// On a view drawn at a scale, the dimension states the length at 1:1, in
    /// @p style: the document's, whose units and precision the drawing shows.
    static std::shared_ptr<draft::DraftLinearDimension> render(
        const model::DrawingView& view, const model::LinearDimension& dim, double offset,
        const draft::DimensionStyle& style = draft::DimensionStyle{});
};

}  // namespace hz::io
