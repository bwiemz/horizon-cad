#pragma once

#include <memory>
#include <vector>

namespace hz::draft {
class DraftEntity;
}  // namespace hz::draft

namespace hz::model {
struct DrawingView;
struct DrawingBalloon;
}  // namespace hz::model

namespace hz::io {

/// Renders a BOM balloon onto a drawing view: a numbered circle with a leader
/// pointing at the anchored feature's projected edge.
///
/// The balloon's feature (a model edge, by TopologyID) is located in the view's
/// projected geometry; the leader tip lands on that edge's midpoint and the
/// circle sits at the configured offset. The DXF writer emits the circle, the
/// leader (as lines), and the item number (as text).
class BalloonRenderer {
public:
    /// Build the drafted entities for @p balloon as it appears in @p view: a
    /// leader (arrow at the edge), a circle, and the item-number text. Returns an
    /// empty vector if the balloon's feature does not appear in the view.
    static std::vector<std::shared_ptr<draft::DraftEntity>> render(
        const model::DrawingView& view, const model::DrawingBalloon& balloon);
};

}  // namespace hz::io
