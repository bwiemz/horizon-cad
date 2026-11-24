#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "horizon/math/Vec2.h"

namespace hz::draft {
class DraftEntity;
}  // namespace hz::draft

namespace hz::model {
struct DrawingView;
}  // namespace hz::model

namespace hz::io {

/// Draws a view's GD&T as a drawing shows it (ASME Y14.5, ISO 1101): each
/// feature control frame a row of boxes (the characteristic's symbol, the
/// tolerance with its ⌀ and Ⓜ/Ⓛ, the datums) on a leader to the toleranced
/// edge, and each datum feature symbol a boxed letter on a leader from a
/// triangle on the edge.
///
/// The characteristic symbols and the circled modifiers are drawn in lines,
/// circles and arcs, not written as characters: fonts, and DXF readers,
/// rarely have ⌖ or ⏥. Everything is drawn as lines, arcs, circles and
/// text, which the DXF writer emits as they are.
///
/// Each annotation is anchored to a model edge by TopologyID: every piece of
/// the edge the view shows by that edge's logical name (a circle is cut into
/// pieces and chords), so it tracks the edge as the model is rebuilt.
class GeometricToleranceRenderer {
public:
    /// Where a hole's callout leaves room under its text for the frames and
    /// datum of the same edge: the left of the text and the top of the room,
    /// on the sheet, by the edge's logical name without its piece. Frames on
    /// such an edge hang there, one under another, rather than on a leader
    /// of their own, as a hole's position is given under its size.
    using Hangers = std::map<std::string, math::Vec2>;

    /// The entities that draw @p view's frames and datum feature symbols;
    /// none for one whose edge the view does not show. @p value writes a
    /// tolerance, given in millimetres, as the sheet states lengths.
    static std::vector<std::shared_ptr<draft::DraftEntity>> render(
        const model::DrawingView& view, const std::function<std::string(double)>& value,
        const Hangers& hangers = {});

    /// The height of a frame's text, and of a frame: twice its text, as the
    /// standards draw it.
    static constexpr double kTextHeight = 3.0;
    static constexpr double kFrameHeight = 2.0 * kTextHeight;
    /// How far a datum feature symbol reaches from its edge: its triangle,
    /// its leader and its box.
    static constexpr double kDatumDepth = 2.6 + 4.0 + kFrameHeight;
};

}  // namespace hz::io
